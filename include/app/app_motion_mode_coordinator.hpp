#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "app/app_motion_mode_protocol.hpp"

namespace app::motion {

enum class NodeKey : std::uint8_t {
    None = 0U,
    Keyboard = 1U,
    Target = 2U,
    Controller = 3U,
};

enum class Transition : std::uint8_t {
    None = 0U,
    Configure = 1U,
    Activate = 3U,
    Deactivate = 4U,
};

enum class LeaseDecision : std::uint8_t {
    Allowed = 0U,
    Missing = 1U,
    OwnerMismatch = 2U,
    ModeMismatch = 3U,
    Expired = 4U,
    StateUnavailable = 5U,
};

class LeaseGuard
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    LeaseDecision canSetMode(
        std::uint32_t client_id, Mode mode, TimePoint now) const;
    LeaseDecision canHeartbeat(
        std::uint32_t client_id, Mode mode, TimePoint now) const;
    LeaseDecision validateObservedMode(
        bool observation_succeeded, Mode observed_mode, TimePoint now) const;
    void acquire(
        std::uint32_t client_id,
        Mode mode,
        std::chrono::milliseconds duration,
        TimePoint now);
    LeaseDecision renew(
        std::uint32_t client_id,
        Mode mode,
        std::chrono::milliseconds duration,
        TimePoint now);
    void clear();
    bool hasActiveLease() const;
    bool isExpired(TimePoint now) const;
    std::optional<std::uint32_t> owner() const;
    Mode mode() const;

private:
    std::optional<std::uint32_t> owner_;
    Mode mode_ = Mode::Stopped;
    TimePoint deadline_{};
};

class LifecycleGateway
{
public:
    virtual ~LifecycleGateway() = default;
    virtual bool getState(NodeKey node, LifecycleState &state, std::string &error) = 0;
    virtual bool changeState(NodeKey node, Transition transition, std::string &error) = 0;
};

struct CoordinatorResult {
    bool success = false;
    ErrorCode error = ErrorCode::None;
    std::string message;
    Mode current_mode = Mode::Degraded;
    LifecycleState keyboard_state = LifecycleState::Unknown;
    LifecycleState target_state = LifecycleState::Unknown;
    LifecycleState controller_state = LifecycleState::Unknown;
};

class Coordinator
{
public:
    explicit Coordinator(LifecycleGateway &gateway);
    CoordinatorResult query();
    CoordinatorResult setMode(Mode desired_mode);

private:
    struct Snapshot {
        bool complete = false;
        LifecycleState keyboard = LifecycleState::Unknown;
        LifecycleState target = LifecycleState::Unknown;
        LifecycleState controller = LifecycleState::Unknown;
        std::string error;
    };

    Snapshot snapshot();
    CoordinatorResult makeResult(const Snapshot &states) const;
    bool ensureInactive(NodeKey node, std::string &error);
    bool ensureActive(NodeKey node, std::string &error);
    bool ensureConfigured(NodeKey node, std::string &error);
    bool ensureActivated(NodeKey node, std::string &error);
    bool transitionAndVerify(
        NodeKey node,
        Transition transition,
        LifecycleState expected,
        std::string &error);
    bool rollback(std::string &error);
    bool matches(const Snapshot &states, Mode desired_mode) const;
    static Mode classify(const Snapshot &states);

    LifecycleGateway &gateway_;
};

}  // namespace app::motion


