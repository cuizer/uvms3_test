#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "app/app_motion_mode_coordinator.hpp"
#include "app/app_motion_mode_protocol.hpp"

namespace app::motion {

namespace {

std::string nodeLabel(NodeKey node)
{
    switch (node) {
        case NodeKey::Keyboard: return "keyboard";
        case NodeKey::Target: return "target";
        case NodeKey::Controller: return "controller";
        default: return "unknown";
    }
}

ResultCode resultCodeFor(ErrorCode error)
{
    switch (error) {
        case ErrorCode::None: return ResultCode::Ok;
        case ErrorCode::LifecycleUnavailable: return ResultCode::Timeout;
        case ErrorCode::TransitionFailed:
        case ErrorCode::VerificationFailed:
        case ErrorCode::RollbackFailed:
            return ResultCode::TransitionFailed;
        default: return ResultCode::PreconditionFailed;
    }
}

}  // namespace

class RosLifecycleGateway final : public LifecycleGateway
{
public:
    RosLifecycleGateway(
        rclcpp::Node &node,
        std::chrono::milliseconds timeout,
        const std::map<NodeKey, std::string> &node_names)
    : timeout_(timeout)
    {
        for (const auto &[key, name] : node_names) {
            Clients clients;
            clients.get_state = node.create_client<lifecycle_msgs::srv::GetState>(
                name + "/get_state");
            clients.change_state = node.create_client<lifecycle_msgs::srv::ChangeState>(
                name + "/change_state");
            clients_[key] = std::move(clients);
        }
    }

    bool getState(NodeKey node, LifecycleState &state, std::string &error) override
    {
        const auto found = clients_.find(node);
        if (found == clients_.end()) {
            error = "no lifecycle clients for " + nodeLabel(node);
            return false;
        }
        auto &client = found->second.get_state;
        if (!client->wait_for_service(timeout_)) {
            error = nodeLabel(node) + " get_state service unavailable";
            return false;
        }
        auto future = client->async_send_request(
            std::make_shared<lifecycle_msgs::srv::GetState::Request>());
        if (future.wait_for(timeout_) != std::future_status::ready) {
            //client->remove_pending_request(future);
            error = nodeLabel(node) + " get_state timed out";
            return false;
        }
        try {
            const auto response = future.get();
            state = static_cast<LifecycleState>(response->current_state.id);
        } catch (const std::exception &exception) {
            error = nodeLabel(node) + " get_state failed: " + exception.what();
            return false;
        }
        return true;
    }

    bool changeState(NodeKey node, Transition transition, std::string &error) override
    {
        const auto found = clients_.find(node);
        if (found == clients_.end()) {
            error = "no lifecycle clients for " + nodeLabel(node);
            return false;
        }
        auto &client = found->second.change_state;
        if (!client->wait_for_service(timeout_)) {
            error = nodeLabel(node) + " change_state service unavailable";
            return false;
        }
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = static_cast<std::uint8_t>(transition);
        auto future = client->async_send_request(request);
        if (future.wait_for(timeout_) != std::future_status::ready) {
            //client->remove_pending_request(future);
            error = nodeLabel(node) + " change_state timed out";
            return false;
        }
        try {
            const auto response = future.get();
            if (!response->success) {
                error = nodeLabel(node) + " rejected lifecycle transition";
                return false;
            }
        } catch (const std::exception &exception) {
            error = nodeLabel(node) + " change_state failed: " + exception.what();
            return false;
        }
        return true;
    }

private:
    struct Clients {
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state;
    };

    std::chrono::milliseconds timeout_;
    std::map<NodeKey, Clients> clients_;
};

class AppMotionModeManagerNode final : public rclcpp::Node
{
public:
    AppMotionModeManagerNode()
    : rclcpp::Node("app_motion_mode_manager_node")
    {
        const auto port = declare_parameter<int>("manager_port", 5004);
        const auto service_timeout_ms = declare_parameter<int>("service_timeout_ms", 1000);
        cache_limit_ = declare_parameter<int>("request_cache_size", 256);
        cache_ttl_ = std::chrono::milliseconds(
            declare_parameter<int>("request_cache_ttl_ms", 30000));

        if (port < 1 || port > 65535) {
            throw std::invalid_argument("manager_port must be in [1, 65535]");
        }
        if (service_timeout_ms < 100 || service_timeout_ms > 10000) {
            throw std::invalid_argument("service_timeout_ms must be in [100, 10000]");
        }
        if (cache_limit_ < 1 || cache_limit_ > 4096) {
            throw std::invalid_argument("request_cache_size must be in [1, 4096]");
        }
        if (cache_ttl_.count() < 1000 || cache_ttl_.count() > 300000) {
            throw std::invalid_argument("request_cache_ttl_ms must be in [1000, 300000]");
        }

        const std::map<NodeKey, std::string> node_names = {
            {NodeKey::Keyboard, declare_parameter<std::string>(
                "keyboard_node", "/app_keyboard_control_node")},
            {NodeKey::Target, declare_parameter<std::string>(
                "target_node", "/app_motion_target_node")},
            {NodeKey::Controller, declare_parameter<std::string>(
                "controller_node", "/bsp_motioncontrol_node")},
        };
        gateway_ = std::make_unique<RosLifecycleGateway>(
            *this, std::chrono::milliseconds(service_timeout_ms), node_names);
        coordinator_ = std::make_unique<Coordinator>(*gateway_);

        openSocket(static_cast<std::uint16_t>(port));
        running_ = true;
        worker_ = std::thread(&AppMotionModeManagerNode::run, this);
        RCLCPP_INFO(get_logger(), "Motion mode manager listening on UDP :%d", port);
    }

    ~AppMotionModeManagerNode() override
    {
        running_ = false;
        if (socket_fd_ >= 0) {
            shutdown(socket_fd_, SHUT_RDWR);
            close(socket_fd_);
            socket_fd_ = -1;
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    struct CacheKey {
        std::uint32_t client_id;
        std::uint32_t request_id;

        bool operator==(const CacheKey &other) const
        {
            return client_id == other.client_id && request_id == other.request_id;
        }
    };

    struct CacheKeyHash {
        std::size_t operator()(const CacheKey &key) const
        {
            return (static_cast<std::size_t>(key.client_id) << 1U) ^ key.request_id;
        }
    };

    struct CacheEntry {
        std::array<std::uint8_t, 18U> fingerprint{};
        std::vector<std::uint8_t> response;
        Mode completed_mode = Mode::Degraded;
        std::chrono::steady_clock::time_point created;
    };

    void openSocket(std::uint16_t port)
    {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            throw std::runtime_error("motion manager socket() failed: " +
                std::string(std::strerror(errno)));
        }
        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        if (setsockopt(
                socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            const auto message = std::string(std::strerror(errno));
            close(socket_fd_);
            socket_fd_ = -1;
            throw std::runtime_error("motion manager SO_RCVTIMEO failed: " + message);
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = INADDR_ANY;
        if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
            const auto message = std::string(std::strerror(errno));
            close(socket_fd_);
            socket_fd_ = -1;
            throw std::runtime_error("motion manager bind() failed: " + message);
        }
    }

    void run()
    {
        convergeToStoppedOnStartup();
        std::array<std::uint8_t, 512U> buffer{};
        while (running_ && rclcpp::ok()) {
            sockaddr_in peer{};
            socklen_t peer_length = sizeof(peer);
            const auto received = recvfrom(
                socket_fd_, buffer.data(), buffer.size(), 0,
                reinterpret_cast<sockaddr *>(&peer), &peer_length);
            if (received < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && running_) {
                    RCLCPP_ERROR(get_logger(), "motion manager recvfrom failed: %s",
                        std::strerror(errno));
                }
                checkLeaseExpiry();
                checkFailsafeStop();
                pruneCache();
                continue;
            }
            // Expiration is processed before any queued command can renew or
            // replace the expired lease.
            checkLeaseExpiry();
            handlePacket(buffer.data(), static_cast<std::size_t>(received), peer);
            checkFailsafeStop();
            pruneCache();
        }
    }

    void convergeToStoppedOnStartup()
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            last_result_ = coordinator_->setMode(Mode::Stopped);
            if (last_result_.success) {
                RCLCPP_INFO(get_logger(), "Startup convergence reached STOPPED");
                return;
            }
            RCLCPP_WARN(get_logger(), "Startup STOPPED convergence pending: %s",
                last_result_.message.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
        RCLCPP_ERROR(get_logger(), "Unable to verify STOPPED during startup: %s",
            last_result_.message.c_str());
        scheduleFailsafeStop();
    }

    void handlePacket(
        const std::uint8_t *data,
        std::size_t size,
        const sockaddr_in &peer)
    {
        const auto decoded = decodeRequest(data, size);
        if (!decoded.ok) {
            RCLCPP_WARN(get_logger(), "Rejected management packet: %s",
                decoded.message.c_str());
            sendResponse(makeInvalidResponse(decoded), peer);
            return;
        }

        const CacheKey key{decoded.request.client_id, decoded.request.request_id};
        std::array<std::uint8_t, 18U> fingerprint{};
        std::copy_n(data, fingerprint.size(), fingerprint.begin());
        const auto cached = cache_.find(key);
        if (cached != cache_.end()) {
            if (cached->second.fingerprint != fingerprint) {
                auto response = responseFrom(last_result_, decoded.request);
                response.result = ResultCode::Invalid;
                response.error_code = static_cast<std::uint16_t>(ErrorCode::InvalidCommand);
                sendResponse(response, peer);
                return;
            }
            const bool cached_active_control =
                decoded.request.command != Command::GetStatus &&
                (cached->second.completed_mode == Mode::Keyboard ||
                cached->second.completed_mode == Mode::Pid);
            if (cached_active_control &&
                lease_guard_.canHeartbeat(
                    decoded.request.client_id,
                    cached->second.completed_mode,
                    LeaseGuard::Clock::now()) != LeaseDecision::Allowed) {
                last_result_ = coordinator_->query();
                auto response = responseFrom(last_result_, decoded.request);
                response.result = decoded.request.command == Command::SetMode
                    ? ResultCode::Busy
                    : ResultCode::PreconditionFailed;
                response.error_code = static_cast<std::uint16_t>(
                    ErrorCode::LeaseOwnerMismatch);
                sendResponse(response, peer);
                return;
            }
            const auto actual = coordinator_->query();
            if (actual.success && actual.current_mode == cached->second.completed_mode) {
                sendPacket(cached->second.response, peer);
            } else {
                auto response = cached_active_control
                    ? failClosedLeaseViolation(
                        decoded.request, ErrorCode::VerificationFailed,
                        "cached active response no longer matches lifecycle state")
                    : responseFrom(actual, decoded.request);
                if (!cached_active_control) {
                    response.result = ResultCode::PreconditionFailed;
                    response.error_code = static_cast<std::uint16_t>(
                        ErrorCode::VerificationFailed);
                }
                sendResponse(response, peer);
            }
            return;
        }

        if (decoded.request.command == Command::SetMode) {
            auto accepted = responseFrom(last_result_, decoded.request);
            accepted.message_type = MessageType::Accepted;
            accepted.result = ResultCode::Accepted;
            accepted.error_code = 0U;
            sendResponse(accepted, peer);
        }

        const auto final_response = execute(decoded.request);
        const auto packet = encodeResponse(final_response);
        sendPacket(packet, peer);
        cacheResult(key, fingerprint, packet, final_response.current_mode);
    }

    Response execute(const Request &request)
    {
        if (request.command == Command::GetStatus) {
            last_result_ = coordinator_->query();
            return responseFrom(last_result_, request);
        }
        if (request.command == Command::Heartbeat) {
            auto lease_decision = lease_guard_.canHeartbeat(
                request.client_id, request.mode, LeaseGuard::Clock::now());
            if (lease_decision != LeaseDecision::Allowed) {
                checkLeaseExpiry();
                last_result_ = coordinator_->query();
                auto response = responseFrom(last_result_, request);
                response.result = ResultCode::PreconditionFailed;
                response.error_code = static_cast<std::uint16_t>(
                    ErrorCode::LeaseOwnerMismatch);
                return response;
            }
            last_result_ = coordinator_->query();
            const auto observed = lease_guard_.validateObservedMode(
                last_result_.success,
                last_result_.current_mode,
                LeaseGuard::Clock::now());
            if (observed != LeaseDecision::Allowed) {
                return failClosedLeaseViolation(
                    request,
                    last_result_.success
                        ? ErrorCode::VerificationFailed
                        : ErrorCode::LifecycleUnavailable,
                    "heartbeat lifecycle observation does not match active lease");
            }
            auto response = responseFrom(last_result_, request);
            lease_decision = lease_guard_.renew(
                request.client_id,
                request.mode,
                std::chrono::milliseconds(request.lease_ms),
                LeaseGuard::Clock::now());
            if (lease_decision != LeaseDecision::Allowed) {
                checkLeaseExpiry();
                response = responseFrom(last_result_, request);
                response.result = ResultCode::PreconditionFailed;
                response.error_code = static_cast<std::uint16_t>(
                    ErrorCode::LeaseOwnerMismatch);
            }
            return response;
        }

        if (request.mode != Mode::Stopped) {
            auto lease_decision = lease_guard_.canSetMode(
                request.client_id, request.mode, LeaseGuard::Clock::now());
            if (lease_decision == LeaseDecision::Expired) {
                checkLeaseExpiry();
                lease_decision = lease_guard_.canSetMode(
                    request.client_id, request.mode, LeaseGuard::Clock::now());
            }
            if (lease_decision != LeaseDecision::Allowed) {
                last_result_ = coordinator_->query();
                auto response = responseFrom(last_result_, request);
                response.result = ResultCode::Busy;
                response.error_code = static_cast<std::uint16_t>(
                    ErrorCode::LeaseOwnerMismatch);
                return response;
            }
        }

        if (stop_retry_required_ && request.mode != Mode::Stopped) {
            last_result_ = coordinator_->setMode(Mode::Stopped);
            if (!last_result_.success) {
                auto response = responseFrom(last_result_, request);
                response.result = ResultCode::PreconditionFailed;
                response.error_code = static_cast<std::uint16_t>(
                    ErrorCode::RollbackFailed);
                return response;
            }
            stop_retry_required_ = false;
        }

        last_result_ = coordinator_->setMode(request.mode);
        if (last_result_.success && request.mode != Mode::Stopped) {
            stop_retry_required_ = false;
            lease_guard_.acquire(
                request.client_id,
                request.mode,
                std::chrono::milliseconds(request.lease_ms),
                LeaseGuard::Clock::now());
        } else if (last_result_.success) {
            stop_retry_required_ = false;
            clearLease();
        }
        if (!last_result_.success) {
            clearLease();
            scheduleFailsafeStop();
            RCLCPP_ERROR(get_logger(), "Mode transition failed: %s",
                last_result_.message.c_str());
        }
        return responseFrom(last_result_, request);
    }

    Response responseFrom(const CoordinatorResult &result, const Request &request) const
    {
        Response response;
        response.success = result.success;
        response.message_type = MessageType::Final;
        response.result = result.success ? ResultCode::Ok : resultCodeFor(result.error);
        response.current_mode = result.current_mode;
        response.desired_mode = request.command == Command::GetStatus
            ? result.current_mode
            : request.mode;
        if (response.desired_mode == Mode::Degraded) {
            response.desired_mode = Mode::Stopped;
        }
        response.keyboard_state = result.keyboard_state;
        response.target_state = result.target_state;
        response.controller_state = result.controller_state;
        response.client_id = request.client_id;
        response.request_id = request.request_id;
        response.error_code = static_cast<std::uint16_t>(result.error);
        return response;
    }

    Response makeInvalidResponse(const RequestDecodeResult &decoded) const
    {
        Request request = decoded.request;
        auto response = responseFrom(last_result_, request);
        response.result = ResultCode::Invalid;
        response.desired_mode = Mode::Stopped;
        response.error_code = static_cast<std::uint16_t>(decoded.error);
        return response;
    }

    void sendResponse(const Response &response, const sockaddr_in &peer)
    {
        sendPacket(encodeResponse(response), peer);
    }

    void sendPacket(const std::vector<std::uint8_t> &packet, const sockaddr_in &peer)
    {
        const auto sent = sendto(
            socket_fd_, packet.data(), packet.size(), 0,
            reinterpret_cast<const sockaddr *>(&peer), sizeof(peer));
        if (sent != static_cast<ssize_t>(packet.size())) {
            RCLCPP_ERROR(get_logger(), "Failed to send complete management response: %s",
                sent < 0 ? std::strerror(errno) : "short UDP send");
        }
    }

    void checkLeaseExpiry()
    {
        if (!lease_guard_.isExpired(LeaseGuard::Clock::now())) {
            return;
        }
        RCLCPP_ERROR(get_logger(), "Motion control lease expired; converging to STOPPED");
        last_result_ = coordinator_->setMode(Mode::Stopped);
        if (!last_result_.success) {
            RCLCPP_ERROR(get_logger(), "Lease-expiry STOPPED transition failed: %s",
                last_result_.message.c_str());
            scheduleFailsafeStop();
        } else {
            stop_retry_required_ = false;
        }
        clearLease();
        cache_.clear();
        cache_order_.clear();
    }

    Response failClosedLeaseViolation(
        const Request &request,
        ErrorCode error,
        const char *reason)
    {
        RCLCPP_ERROR(get_logger(), "%s; converging to STOPPED", reason);
        clearLease();
        cache_.clear();
        cache_order_.clear();
        last_result_ = coordinator_->setMode(Mode::Stopped);
        if (!last_result_.success) {
            scheduleFailsafeStop();
            RCLCPP_ERROR(get_logger(), "Lease-violation STOPPED transition failed: %s",
                last_result_.message.c_str());
        } else {
            stop_retry_required_ = false;
        }
        auto response = responseFrom(last_result_, request);
        response.result = ResultCode::PreconditionFailed;
        response.error_code = static_cast<std::uint16_t>(error);
        return response;
    }

    void scheduleFailsafeStop()
    {
        stop_retry_required_ = true;
        next_stop_retry_ = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(500);
    }

    void checkFailsafeStop()
    {
        if (!stop_retry_required_ ||
            std::chrono::steady_clock::now() < next_stop_retry_) {
            return;
        }
        last_result_ = coordinator_->setMode(Mode::Stopped);
        if (last_result_.success) {
            stop_retry_required_ = false;
            RCLCPP_INFO(get_logger(), "Fail-closed retry reached STOPPED");
            return;
        }
        RCLCPP_ERROR(get_logger(), "Fail-closed STOPPED retry failed: %s",
            last_result_.message.c_str());
        next_stop_retry_ = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(500);
    }

    void clearLease()
    {
        lease_guard_.clear();
    }

    void cacheResult(
        const CacheKey &key,
        const std::array<std::uint8_t, 18U> &fingerprint,
        const std::vector<std::uint8_t> &response,
        Mode completed_mode)
    {
        cache_[key] = CacheEntry{
            fingerprint, response, completed_mode, std::chrono::steady_clock::now()};
        cache_order_.push_back(key);
        while (static_cast<int>(cache_.size()) > cache_limit_) {
            cache_.erase(cache_order_.front());
            cache_order_.pop_front();
        }
    }

    void pruneCache()
    {
        const auto cutoff = std::chrono::steady_clock::now() - cache_ttl_;
        while (!cache_order_.empty()) {
            const auto found = cache_.find(cache_order_.front());
            if (found == cache_.end()) {
                cache_order_.pop_front();
                continue;
            }
            if (found->second.created >= cutoff) {
                break;
            }
            cache_.erase(found);
            cache_order_.pop_front();
        }
    }

    int socket_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::unique_ptr<RosLifecycleGateway> gateway_;
    std::unique_ptr<Coordinator> coordinator_;
    CoordinatorResult last_result_{};

    int cache_limit_ = 256;
    std::chrono::milliseconds cache_ttl_{30000};
    std::unordered_map<CacheKey, CacheEntry, CacheKeyHash> cache_;
    std::deque<CacheKey> cache_order_;

    LeaseGuard lease_guard_;
    bool stop_retry_required_ = false;
    std::chrono::steady_clock::time_point next_stop_retry_{};
};

}  // namespace app::motion

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<app::motion::AppMotionModeManagerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2U);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


