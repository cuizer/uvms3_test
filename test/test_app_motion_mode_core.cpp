#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "app/app_motion_mode_coordinator.hpp"
#include "app/app_motion_mode_protocol.hpp"
#include "hal/motion_control_validation.hpp"

namespace {

using app::motion::Command;
using app::motion::Coordinator;
using app::motion::ErrorCode;
using app::motion::LifecycleGateway;
using app::motion::LifecycleState;
using app::motion::LeaseDecision;
using app::motion::LeaseGuard;
using app::motion::Mode;
using app::motion::NodeKey;
using app::motion::Request;
using app::motion::Response;
using app::motion::Transition;

int failure_count = 0;

std::string toHex(const std::vector<std::uint8_t> &packet)
{
    static constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(packet.size() * 2U);
    for (const auto value : packet) {
        result.push_back(digits[(value >> 4U) & 0x0FU]);
        result.push_back(digits[value & 0x0FU]);
    }
    return result;
}

void expect(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failure_count;
    }
}

void reseal(std::vector<std::uint8_t> &packet, std::size_t crc_offset)
{
    const auto crc = app::motion::crc16(packet.data(), crc_offset);
    packet[crc_offset] = static_cast<std::uint8_t>(crc & 0xFFU);
    packet[crc_offset + 1U] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
}

class FakeGateway final : public LifecycleGateway
{
public:
    FakeGateway()
    {
        states = {
            {NodeKey::Keyboard, LifecycleState::Unconfigured},
            {NodeKey::Target, LifecycleState::Unconfigured},
            {NodeKey::Controller, LifecycleState::Unconfigured},
        };
    }

    bool getState(NodeKey node, LifecycleState &state, std::string &error) override
    {
        if (unavailable_node == node) {
            error = "service unavailable";
            return false;
        }
        state = states.at(node);
        return true;
    }

    bool changeState(NodeKey node, Transition transition, std::string &error) override
    {
        operations.emplace_back(node, transition);
        if (failed_operation == std::make_pair(node, transition)) {
            error = "injected transition failure";
            return false;
        }

        if (ignored_operation == std::make_pair(node, transition)) {
            return true;
        }

        auto &state = states.at(node);
        if (transition == Transition::Configure && state == LifecycleState::Unconfigured) {
            state = LifecycleState::Inactive;
            return true;
        }
        if (transition == Transition::Activate && state == LifecycleState::Inactive) {
            state = LifecycleState::Active;
            return true;
        }
        if (transition == Transition::Deactivate && state == LifecycleState::Active) {
            state = LifecycleState::Inactive;
            return true;
        }
        error = "invalid transition";
        return false;
    }

    std::map<NodeKey, LifecycleState> states;
    std::vector<std::pair<NodeKey, Transition>> operations;
    NodeKey unavailable_node = NodeKey::None;
    std::pair<NodeKey, Transition> failed_operation{
        NodeKey::None, Transition::None};
    std::pair<NodeKey, Transition> ignored_operation{
        NodeKey::None, Transition::None};
};

void test_request_protocol()
{
    Request request;
    request.command = Command::SetMode;
    request.mode = Mode::Pid;
    request.client_id = 0x11223344U;
    request.request_id = 0x55667788U;
    request.lease_ms = 3000U;

    const auto packet = app::motion::encodeRequest(request);
    expect(packet.size() == 20U, "request packet must be 20 bytes");
    expect(std::string(packet.begin(), packet.begin() + 4) == "UVMC",
        "request magic must be UVMC");
    expect(toHex(packet) == "55564d43010102004433221188776655b80bf4b7",
        "request must match the shared Python/C++ golden vector");

    const auto decoded = app::motion::decodeRequest(packet.data(), packet.size());
    expect(decoded.ok, "valid request must decode");
    expect(decoded.request.command == Command::SetMode, "command must round-trip");
    expect(decoded.request.mode == Mode::Pid, "mode must round-trip");
    expect(decoded.request.client_id == request.client_id, "client id must round-trip");
    expect(decoded.request.request_id == request.request_id, "request id must round-trip");
    expect(decoded.request.lease_ms == 3000U, "lease must round-trip");

    auto corrupt = packet;
    corrupt[7] = 1U;
    expect(!app::motion::decodeRequest(corrupt.data(), corrupt.size()).ok,
        "non-zero reserved flags must be rejected");
    corrupt = packet;
    corrupt[10] ^= 1U;
    expect(!app::motion::decodeRequest(corrupt.data(), corrupt.size()).ok,
        "CRC mismatch must be rejected");
    expect(!app::motion::decodeRequest(packet.data(), packet.size() - 1U).ok,
        "short request must be rejected");

    auto invalid = packet;
    invalid[0] = 'X';
    reseal(invalid, 18U);
    expect(app::motion::decodeRequest(invalid.data(), invalid.size()).error ==
            ErrorCode::InvalidMagic,
        "invalid request magic must be rejected precisely");

    invalid = packet;
    invalid[4] = 2U;
    reseal(invalid, 18U);
    expect(app::motion::decodeRequest(invalid.data(), invalid.size()).error ==
            ErrorCode::UnsupportedVersion,
        "unsupported request version must be rejected precisely");

    invalid = packet;
    invalid[5] = 0U;
    reseal(invalid, 18U);
    expect(app::motion::decodeRequest(invalid.data(), invalid.size()).error ==
            ErrorCode::InvalidCommand,
        "invalid request command must be rejected precisely");

    invalid = packet;
    invalid[6] = static_cast<std::uint8_t>(Mode::Degraded);
    reseal(invalid, 18U);
    expect(app::motion::decodeRequest(invalid.data(), invalid.size()).error ==
            ErrorCode::InvalidMode,
        "invalid requested mode must be rejected precisely");

    Request get_status;
    get_status.command = Command::GetStatus;
    auto get_status_packet = app::motion::encodeRequest(get_status);
    expect(app::motion::decodeRequest(
            get_status_packet.data(), get_status_packet.size()).ok,
        "GET_STATUS with STOPPED and zero lease must decode");
    get_status_packet[6] = static_cast<std::uint8_t>(Mode::Keyboard);
    reseal(get_status_packet, 18U);
    expect(app::motion::decodeRequest(
            get_status_packet.data(), get_status_packet.size()).error ==
            ErrorCode::InvalidLease,
        "GET_STATUS with an active mode must be rejected");

    Request heartbeat;
    heartbeat.command = Command::Heartbeat;
    auto heartbeat_packet = app::motion::encodeRequest(heartbeat);
    expect(app::motion::decodeRequest(
            heartbeat_packet.data(), heartbeat_packet.size()).error ==
            ErrorCode::InvalidMode,
        "HEARTBEAT with STOPPED must be rejected");

    heartbeat.mode = Mode::Keyboard;
    heartbeat.lease_ms = app::motion::MIN_LEASE_MS - 1U;
    heartbeat_packet = app::motion::encodeRequest(heartbeat);
    expect(app::motion::decodeRequest(
            heartbeat_packet.data(), heartbeat_packet.size()).error ==
            ErrorCode::InvalidLease,
        "active lease below minimum must be rejected");

    Request stopped_with_lease;
    stopped_with_lease.command = Command::SetMode;
    stopped_with_lease.lease_ms = app::motion::MIN_LEASE_MS;
    const auto stopped_packet = app::motion::encodeRequest(stopped_with_lease);
    expect(app::motion::decodeRequest(
            stopped_packet.data(), stopped_packet.size()).error ==
            ErrorCode::InvalidLease,
        "STOPPED request with a non-zero lease must be rejected");
}

std::vector<std::uint8_t> makeDataPlanePacket(
    std::uint16_t flags = 0U,
    std::uint16_t reserved = 0U)
{
    std::vector<std::uint8_t> packet = {
        0xA5U,
        0x34U, 0x12U,
        0xE8U, 0x03U,
        0x18U, 0xFCU,
        0xF4U, 0x01U,
        0x84U, 0x03U,
        static_cast<std::uint8_t>(flags & 0xFFU),
        static_cast<std::uint8_t>((flags >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(reserved & 0xFFU),
        static_cast<std::uint8_t>((reserved >> 8U) & 0xFFU),
    };
    app::motion::appendU16(
        packet, app::motion::crc16(packet.data(), packet.size()));
    return packet;
}

void test_data_plane_protocol_rejects_reserved_and_unknown_flags()
{
    auto packet = makeDataPlanePacket(
        app::motion::DATA_FLAG_ESTOP | app::motion::DATA_FLAG_DEPTH_HOLD);
    auto decoded = app::motion::decodeDataPlaneCommand(packet.data(), packet.size());
    expect(decoded.ok, "valid 17-byte data-plane command must decode");
    expect(decoded.command.sequence == 0x1234U, "data-plane sequence must decode");
    expect(decoded.command.surge == 1000, "data-plane signed surge must decode");
    expect(decoded.command.sway == -1000, "data-plane signed sway must decode");
    expect(decoded.command.flags == 0x0003U, "known data-plane flags must decode");
    expect(
        app::motion::decodeBoundedVelocity(decoded.command.surge, 1.0) == 1.0,
        "surge wire value must decode directly as metres per second");
    expect(
        app::motion::decodeBoundedVelocity(decoded.command.sway, 0.5) == -0.5,
        "configured velocity limit must clamp the wire value");

    packet = makeDataPlanePacket(0U, 1U);
    decoded = app::motion::decodeDataPlaneCommand(packet.data(), packet.size());
    expect(!decoded.ok, "non-zero data-plane reserved field must be rejected");

    packet = makeDataPlanePacket(0x0008U, 0U);
    decoded = app::motion::decodeDataPlaneCommand(packet.data(), packet.size());
    expect(!decoded.ok, "unknown data-plane flags must be rejected");

    packet = makeDataPlanePacket();
    decoded = app::motion::decodeDataPlaneCommand(packet.data(), packet.size() - 1U);
    expect(!decoded.ok, "short data-plane command must be rejected");
}

void test_lease_guard_enforces_owner_stop_and_deadline()
{
    using Clock = LeaseGuard::Clock;
    const auto start = Clock::time_point{};
    LeaseGuard lease;

    expect(
        lease.canSetMode(11U, Mode::Keyboard, start) == LeaseDecision::Allowed,
        "first active client must be admitted");
    lease.acquire(11U, Mode::Keyboard, std::chrono::milliseconds(3000), start);
    expect(
        lease.canSetMode(22U, Mode::Pid, start + std::chrono::milliseconds(1)) ==
            LeaseDecision::OwnerMismatch,
        "a live active lease must reject a different owner");
    expect(
        lease.canSetMode(22U, Mode::Stopped, start + std::chrono::milliseconds(1)) ==
            LeaseDecision::Allowed,
        "STOPPED must be admitted from any client");
    expect(
        lease.canHeartbeat(11U, Mode::Keyboard, start + std::chrono::milliseconds(2999)) ==
            LeaseDecision::Allowed,
        "owner heartbeat before the deadline must be admitted");
    expect(
        lease.validateObservedMode(
            true, Mode::Keyboard, start + std::chrono::milliseconds(100)) ==
            LeaseDecision::Allowed,
        "an active lease must accept its observed lifecycle mode");
    expect(
        lease.validateObservedMode(
            true, Mode::Pid, start + std::chrono::milliseconds(100)) ==
            LeaseDecision::ModeMismatch,
        "observed mode drift must require fail-closed STOPPED");
    expect(
        lease.validateObservedMode(
            false, Mode::Degraded, start + std::chrono::milliseconds(100)) ==
            LeaseDecision::StateUnavailable,
        "unobservable active lifecycle state must require fail-closed STOPPED");
    expect(
        lease.renew(
            22U, Mode::Keyboard, std::chrono::milliseconds(3000),
            start + std::chrono::milliseconds(1000)) == LeaseDecision::OwnerMismatch,
        "a different client must not renew the owner's lease");
    expect(
        lease.renew(
            11U, Mode::Pid, std::chrono::milliseconds(3000),
            start + std::chrono::milliseconds(1000)) == LeaseDecision::ModeMismatch,
        "heartbeat mode must match the leased mode");
    expect(
        lease.canHeartbeat(11U, Mode::Keyboard, start + std::chrono::milliseconds(3000)) ==
            LeaseDecision::Expired,
        "heartbeat at the deadline must not resurrect an expired lease");
    expect(
        lease.renew(
            11U, Mode::Keyboard, std::chrono::milliseconds(3000),
            start + std::chrono::milliseconds(3000)) == LeaseDecision::Expired,
        "renew itself must reject an expired heartbeat");
    expect(
        lease.canSetMode(22U, Mode::Pid, start + std::chrono::milliseconds(3000)) ==
            LeaseDecision::Expired,
        "active SET_MODE must first converge an expired lease to STOPPED");

    lease.clear();
    expect(
        lease.canHeartbeat(11U, Mode::Keyboard, start) == LeaseDecision::Missing,
        "heartbeat without an active lease must be rejected");
}

hal::motion::MotionControlConfig makeValidMotionControlConfig()
{
    hal::motion::MotionControlConfig config;
    config.control_rate_hz = 50.0;
    config.command_timeout_s = 1.0;
    config.thrust_min_pct = -100.0;
    config.thrust_max_pct = 100.0;
    config.servo_max_deg = 45.0;
    for (auto &pid : config.pid) {
        pid = {1.0, 0.1, 0.01, 10.0, 100.0};
    }
    config.feedforward.fill(0.0);
    config.allocation.assign(36U, 0.0);
    for (std::size_t index = 0; index < 6U; ++index) {
        config.allocation[index * 6U + index] = 1.0;
    }
    return config;
}

void test_motion_control_parameter_validation()
{
    auto config = makeValidMotionControlConfig();
    expect(hal::motion::validateMotionControlConfig(config).ok,
        "finite bounded full-rank controller configuration must be accepted");

    config.control_rate_hz = 0.0;
    expect(!hal::motion::validateMotionControlConfig(config).ok,
        "zero control rate must be rejected");
    config = makeValidMotionControlConfig();
    config.command_timeout_s = 0.0;
    expect(!hal::motion::validateMotionControlConfig(config).ok,
        "zero command timeout must be rejected");
    config = makeValidMotionControlConfig();
    config.thrust_min_pct = config.thrust_max_pct;
    expect(!hal::motion::validateMotionControlConfig(config).ok,
        "thrust minimum must be below maximum");
    config = makeValidMotionControlConfig();
    config.pid[0].kp = std::numeric_limits<double>::quiet_NaN();
    expect(!hal::motion::validateMotionControlConfig(config).ok,
        "non-finite PID gains must be rejected");
    config = makeValidMotionControlConfig();
    config.allocation.assign(35U, 0.0);
    expect(!hal::motion::validateMotionControlConfig(config).ok,
        "allocation matrix must contain exactly 36 elements");
    config = makeValidMotionControlConfig();
    config.allocation.assign(36U, 0.0);
    expect(!hal::motion::validateMotionControlConfig(config).ok,
        "rank-deficient allocation matrix must be rejected");
}

void test_response_protocol()
{
    Response response;
    response.success = true;
    response.result = app::motion::ResultCode::Ok;
    response.current_mode = Mode::Keyboard;
    response.desired_mode = Mode::Keyboard;
    response.keyboard_state = LifecycleState::Active;
    response.target_state = LifecycleState::Inactive;
    response.controller_state = LifecycleState::Inactive;
    response.client_id = 0x10203040U;
    response.request_id = 0x50607080U;

    const auto packet = app::motion::encodeResponse(response);
    expect(packet.size() == 24U, "response packet must be 24 bytes");
    const auto decoded = app::motion::decodeResponse(packet.data(), packet.size());
    expect(decoded.ok, "valid response must decode");
    expect(decoded.response.current_mode == Mode::Keyboard,
        "response mode must round-trip");
    expect(decoded.response.keyboard_state == LifecycleState::Active,
        "response lifecycle state must round-trip");

    expect(!app::motion::decodeResponse(packet.data(), packet.size() - 1U).ok,
        "short response must be rejected");

    auto invalid = packet;
    invalid[0] = 'X';
    reseal(invalid, 22U);
    expect(app::motion::decodeResponse(invalid.data(), invalid.size()).error ==
            ErrorCode::InvalidMagic,
        "invalid response magic must be rejected precisely");

    invalid = packet;
    invalid[4] = 2U;
    reseal(invalid, 22U);
    expect(app::motion::decodeResponse(invalid.data(), invalid.size()).error ==
            ErrorCode::UnsupportedVersion,
        "unsupported response version must be rejected precisely");

    invalid = packet;
    invalid[12] ^= 1U;
    expect(app::motion::decodeResponse(invalid.data(), invalid.size()).error ==
            ErrorCode::CrcMismatch,
        "response CRC mismatch must be rejected precisely");

    invalid = packet;
    invalid[5] = 0U;
    reseal(invalid, 22U);
    expect(app::motion::decodeResponse(invalid.data(), invalid.size()).error ==
            ErrorCode::InvalidMode,
        "invalid response enum must be rejected");

    response.message_type = app::motion::MessageType::Accepted;
    response.result = app::motion::ResultCode::Accepted;
    const auto accepted_packet = app::motion::encodeResponse(response);
    const auto accepted = app::motion::decodeResponse(
        accepted_packet.data(), accepted_packet.size());
    expect(accepted.ok && !accepted.response.success,
        "ACCEPTED response must decode but must not be final success");
}

void test_keyboard_mode_is_idempotent_and_exclusive()
{
    FakeGateway gateway;
    Coordinator coordinator(gateway);

    auto result = coordinator.setMode(Mode::Keyboard);
    expect(result.success, "STOPPED to KEYBOARD must succeed");
    expect(result.current_mode == Mode::Keyboard, "actual mode must be KEYBOARD");
    expect(gateway.states[NodeKey::Keyboard] == LifecycleState::Active,
        "keyboard must be active");
    expect(gateway.states[NodeKey::Target] != LifecycleState::Active,
        "target must not be active");
    expect(gateway.states[NodeKey::Controller] != LifecycleState::Active,
        "controller must not be active");

    const auto count = gateway.operations.size();
    result = coordinator.setMode(Mode::Keyboard);
    expect(result.success, "repeating KEYBOARD must succeed");
    expect(gateway.operations.size() == count,
        "repeating current mode must not issue transitions");
}

void test_pid_start_and_stop_order()
{
    FakeGateway gateway;
    gateway.states[NodeKey::Keyboard] = LifecycleState::Active;
    Coordinator coordinator(gateway);

    auto result = coordinator.setMode(Mode::Pid);
    expect(result.success, "KEYBOARD to PID must succeed");
    const std::vector<std::pair<NodeKey, Transition>> expected_start = {
        {NodeKey::Keyboard, Transition::Deactivate},
        {NodeKey::Controller, Transition::Configure},
        {NodeKey::Target, Transition::Configure},
        {NodeKey::Controller, Transition::Activate},
        {NodeKey::Target, Transition::Activate},
    };
    expect(gateway.operations == expected_start,
        "PID must stop keyboard, configure, then activate controller before target");

    gateway.operations.clear();
    result = coordinator.setMode(Mode::Stopped);
    expect(result.success, "PID to STOPPED must succeed");
    const std::vector<std::pair<NodeKey, Transition>> expected_stop = {
        {NodeKey::Target, Transition::Deactivate},
        {NodeKey::Controller, Transition::Deactivate},
    };
    expect(gateway.operations == expected_stop,
        "PID stop must deactivate target before controller");
}

void test_partial_pid_failure_rolls_back_to_stopped()
{
    FakeGateway gateway;
    gateway.failed_operation = {NodeKey::Target, Transition::Activate};
    Coordinator coordinator(gateway);

    const auto result = coordinator.setMode(Mode::Pid);
    expect(!result.success, "target activation failure must fail the request");
    expect(result.error == ErrorCode::TransitionFailed,
        "transition failure must have a precise error code");
    expect(gateway.states[NodeKey::Keyboard] != LifecycleState::Active,
        "rollback must keep keyboard stopped");
    expect(gateway.states[NodeKey::Target] != LifecycleState::Active,
        "rollback must keep target stopped");
    expect(gateway.states[NodeKey::Controller] != LifecycleState::Active,
        "rollback must deactivate the already active controller");
}

void test_unavailable_state_service_fails_closed()
{
    FakeGateway gateway;
    gateway.states[NodeKey::Keyboard] = LifecycleState::Active;
    gateway.unavailable_node = NodeKey::Controller;
    Coordinator coordinator(gateway);

    const auto result = coordinator.setMode(Mode::Pid);
    expect(!result.success, "unavailable state service must fail the request");
    expect(result.current_mode == Mode::Degraded,
        "unobservable lifecycle state must be reported as degraded");
    expect(gateway.states[NodeKey::Keyboard] != LifecycleState::Active,
        "failed request must not activate keyboard");
    expect(gateway.states[NodeKey::Target] != LifecycleState::Active,
        "failed request must not activate target");
}

void test_query_reports_complete_and_unavailable_snapshots()
{
    FakeGateway gateway;
    Coordinator coordinator(gateway);

    auto result = coordinator.query();
    expect(result.success, "query must succeed when every lifecycle service responds");
    expect(result.current_mode == Mode::Stopped,
        "three inactive lifecycle nodes must classify as STOPPED");

    gateway.unavailable_node = NodeKey::Keyboard;
    result = coordinator.query();
    expect(!result.success, "keyboard get_state failure must fail query");
    expect(result.error == ErrorCode::LifecycleUnavailable,
        "query failure must report lifecycle unavailable");

    gateway.unavailable_node = NodeKey::Target;
    result = coordinator.query();
    expect(!result.success, "target get_state failure must fail query");
    expect(result.current_mode == Mode::Degraded,
        "incomplete query must classify as DEGRADED");
}

void test_degraded_is_observed_only_and_conflicts_are_reported()
{
    FakeGateway gateway;
    Coordinator coordinator(gateway);

    auto result = coordinator.setMode(Mode::Degraded);
    expect(!result.success, "DEGRADED must not be accepted as a requested mode");
    expect(result.error == ErrorCode::InvalidMode,
        "DEGRADED request must return invalid mode");

    gateway.states[NodeKey::Keyboard] = LifecycleState::Active;
    gateway.states[NodeKey::Target] = LifecycleState::Active;
    result = coordinator.query();
    expect(result.success, "conflicting but observable states must still be queryable");
    expect(result.current_mode == Mode::Degraded,
        "simultaneously active command sources must classify as DEGRADED");
}

void test_transition_verification_failure_rolls_back()
{
    FakeGateway gateway;
    gateway.ignored_operation = {NodeKey::Controller, Transition::Configure};
    Coordinator coordinator(gateway);

    const auto result = coordinator.setMode(Mode::Pid);
    expect(!result.success,
        "a transition response without the expected state change must fail");
    expect(result.error == ErrorCode::TransitionFailed,
        "state verification failure during a transition must be precise");
    expect(result.current_mode == Mode::Stopped,
        "verification failure must roll back to STOPPED");
}

void test_rollback_failure_is_reported()
{
    FakeGateway gateway;
    gateway.states[NodeKey::Target] = LifecycleState::Active;
    gateway.failed_operation = {NodeKey::Target, Transition::Deactivate};
    Coordinator coordinator(gateway);

    const auto result = coordinator.setMode(Mode::Keyboard);
    expect(!result.success, "a failed exclusive-source deactivation must fail");
    expect(result.error == ErrorCode::RollbackFailed,
        "a second failure during fail-closed rollback must be visible");
    expect(result.message.find("rollback failed") != std::string::npos,
        "rollback diagnostics must be preserved in the result message");
}

void test_unstable_lifecycle_state_is_rejected()
{
    FakeGateway gateway;
    gateway.states[NodeKey::Target] = LifecycleState::Unknown;
    Coordinator coordinator(gateway);

    const auto result = coordinator.setMode(Mode::Stopped);
    expect(!result.success, "transitional or unknown lifecycle state must be rejected");
    expect(result.error == ErrorCode::RollbackFailed,
        "an unstable node that cannot be stopped must fail rollback explicitly");
}

}  // namespace

int main()
{
    test_request_protocol();
    test_data_plane_protocol_rejects_reserved_and_unknown_flags();
    test_lease_guard_enforces_owner_stop_and_deadline();
    test_motion_control_parameter_validation();
    test_response_protocol();
    test_keyboard_mode_is_idempotent_and_exclusive();
    test_pid_start_and_stop_order();
    test_partial_pid_failure_rolls_back_to_stopped();
    test_unavailable_state_service_fails_closed();
    test_query_reports_complete_and_unavailable_snapshots();
    test_degraded_is_observed_only_and_conflicts_are_reported();
    test_transition_verification_failure_rolls_back();
    test_rollback_failure_is_reported();
    test_unstable_lifecycle_state_is_rejected();

    if (failure_count != 0) {
        std::cerr << failure_count << " test assertion(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "All motion mode core tests passed\n";
    return EXIT_SUCCESS;
}


