#include "app/app_motion_mode_coordinator.hpp"

#include <array>
#include <sstream>

namespace app::motion {

namespace {

bool isStoppedState(LifecycleState state)
{
    return state == LifecycleState::Unconfigured ||
        state == LifecycleState::Inactive;
}

}  // namespace

LeaseDecision LeaseGuard::canSetMode(
    std::uint32_t client_id,
    Mode mode,
    TimePoint now) const
{
    if (mode == Mode::Stopped) {
        return LeaseDecision::Allowed;
    }
    if (mode != Mode::Keyboard && mode != Mode::Pid) {
        return LeaseDecision::ModeMismatch;
    }
    if (!owner_.has_value()) {
        return LeaseDecision::Allowed;
    }
    if (isExpired(now)) {
        return LeaseDecision::Expired;
    }
    return *owner_ == client_id
        ? LeaseDecision::Allowed
        : LeaseDecision::OwnerMismatch;
}

LeaseDecision LeaseGuard::canHeartbeat(
    std::uint32_t client_id,
    Mode mode,
    TimePoint now) const
{
    if (!owner_.has_value()) {
        return LeaseDecision::Missing;
    }
    if (isExpired(now)) {
        return LeaseDecision::Expired;
    }
    if (*owner_ != client_id) {
        return LeaseDecision::OwnerMismatch;
    }
    return mode_ == mode
        ? LeaseDecision::Allowed
        : LeaseDecision::ModeMismatch;
}

LeaseDecision LeaseGuard::validateObservedMode(
    bool observation_succeeded,
    Mode observed_mode,
    TimePoint now) const
{
    if (!owner_.has_value()) {
        return LeaseDecision::Missing;
    }
    if (isExpired(now)) {
        return LeaseDecision::Expired;
    }
    if (!observation_succeeded) {
        return LeaseDecision::StateUnavailable;
    }
    return observed_mode == mode_
        ? LeaseDecision::Allowed
        : LeaseDecision::ModeMismatch;
}

void LeaseGuard::acquire(
    std::uint32_t client_id,
    Mode mode,
    std::chrono::milliseconds duration,
    TimePoint now)
{
    if ((mode != Mode::Keyboard && mode != Mode::Pid) || duration.count() <= 0) {
        clear();
        return;
    }
    owner_ = client_id;
    mode_ = mode;
    deadline_ = now + duration;
}

LeaseDecision LeaseGuard::renew(
    std::uint32_t client_id,
    Mode mode,
    std::chrono::milliseconds duration,
    TimePoint now)
{
    const auto decision = canHeartbeat(client_id, mode, now);
    if (decision == LeaseDecision::Allowed && duration.count() > 0) {
        deadline_ = now + duration;
        return LeaseDecision::Allowed;
    }
    return decision == LeaseDecision::Allowed
        ? LeaseDecision::ModeMismatch
        : decision;
}

void LeaseGuard::clear()
{
    owner_.reset();
    mode_ = Mode::Stopped;
    deadline_ = TimePoint{};
}

bool LeaseGuard::hasActiveLease() const
{
    return owner_.has_value();
}

bool LeaseGuard::isExpired(TimePoint now) const
{
    return owner_.has_value() && now >= deadline_;
}

std::optional<std::uint32_t> LeaseGuard::owner() const
{
    return owner_;
}

Mode LeaseGuard::mode() const
{
    return mode_;
}

Coordinator::Coordinator(LifecycleGateway &gateway)
: gateway_(gateway)
{
}

CoordinatorResult Coordinator::query()
{
    const auto states = snapshot();
    auto result = makeResult(states);
    result.success = states.complete;
    if (!states.complete) {
        result.error = ErrorCode::LifecycleUnavailable;
        result.message = states.error;
    } else {
        result.message = "lifecycle state query completed";
    }
    return result;
}

CoordinatorResult Coordinator::setMode(Mode desired_mode)
{
    if (desired_mode == Mode::Degraded) {
        auto result = query();
        result.success = false;
        result.error = ErrorCode::InvalidMode;
        result.message = "DEGRADED is an observed state, not a requested mode";
        return result;
    }

    auto before = snapshot();
    if (!before.complete) {
        const auto observation_error = before.error;
        std::string rollback_error;
        rollback(rollback_error);
        auto result = makeResult(snapshot());
        result.success = false;
        result.error = ErrorCode::LifecycleUnavailable;
        result.message = observation_error;
        if (!rollback_error.empty()) {
            result.message += "; fail-closed deactivation: " + rollback_error;
        }
        return result;
    }
    if (matches(before, desired_mode)) {
        auto result = makeResult(before);
        result.success = true;
        result.message = "requested mode is already active";
        return result;
    }

    std::string error;
    bool changed = true;
    if (desired_mode == Mode::Pid) {
        changed = ensureInactive(NodeKey::Target, error) &&
            ensureInactive(NodeKey::Controller, error) &&
            ensureInactive(NodeKey::Keyboard, error) &&
            ensureConfigured(NodeKey::Controller, error) &&
            ensureConfigured(NodeKey::Target, error) &&
            ensureActivated(NodeKey::Controller, error) &&
            ensureActivated(NodeKey::Target, error);
    } else if (desired_mode == Mode::Keyboard) {
        changed = ensureInactive(NodeKey::Target, error) &&
            ensureInactive(NodeKey::Controller, error) &&
            ensureInactive(NodeKey::Keyboard, error) &&
            ensureActive(NodeKey::Keyboard, error);
    } else {
        changed = ensureInactive(NodeKey::Target, error) &&
            ensureInactive(NodeKey::Controller, error) &&
            ensureInactive(NodeKey::Keyboard, error);
    }

    if (!changed) {
        const std::string transition_error = error;
        std::string rollback_error;
        const bool rolled_back = rollback(rollback_error);
        auto result = makeResult(snapshot());
        result.success = false;
        result.error = rolled_back
            ? ErrorCode::TransitionFailed
            : ErrorCode::RollbackFailed;
        result.message = "transition failed: " + transition_error;
        if (!rolled_back) {
            result.message += "; rollback failed: " + rollback_error;
        }
        return result;
    }

    const auto after = snapshot();
    if (!after.complete || !matches(after, desired_mode)) {
        std::string rollback_error;
        const bool rolled_back = rollback(rollback_error);
        auto result = makeResult(snapshot());
        result.success = false;
        result.error = rolled_back
            ? ErrorCode::VerificationFailed
            : ErrorCode::RollbackFailed;
        result.message = "post-transition state did not match requested mode";
        if (!rolled_back) {
            result.message += "; rollback failed: " + rollback_error;
        }
        return result;
    }

    auto result = makeResult(after);
    result.success = true;
    result.message = "mode transition completed and verified";
    return result;
}

Coordinator::Snapshot Coordinator::snapshot()
{
    Snapshot states;
    std::string error;
    if (!gateway_.getState(NodeKey::Keyboard, states.keyboard, error)) {
        states.error = "keyboard get_state failed: " + error;
        return states;
    }
    if (!gateway_.getState(NodeKey::Target, states.target, error)) {
        states.error = "target get_state failed: " + error;
        return states;
    }
    if (!gateway_.getState(NodeKey::Controller, states.controller, error)) {
        states.error = "controller get_state failed: " + error;
        return states;
    }
    states.complete = true;
    return states;
}

CoordinatorResult Coordinator::makeResult(const Snapshot &states) const
{
    CoordinatorResult result;
    result.current_mode = classify(states);
    result.keyboard_state = states.keyboard;
    result.target_state = states.target;
    result.controller_state = states.controller;
    return result;
}

bool Coordinator::ensureInactive(NodeKey node, std::string &error)
{
    LifecycleState state = LifecycleState::Unknown;
    if (!gateway_.getState(node, state, error)) {
        return false;
    }
    if (state == LifecycleState::Unconfigured || state == LifecycleState::Inactive) {
        return true;
    }
    if (state != LifecycleState::Active) {
        error = "node is not in a stable state that can be deactivated";
        return false;
    }
    return transitionAndVerify(node, Transition::Deactivate, LifecycleState::Inactive, error);
}

bool Coordinator::ensureActive(NodeKey node, std::string &error)
{
    return ensureConfigured(node, error) && ensureActivated(node, error);
}

bool Coordinator::ensureConfigured(NodeKey node, std::string &error)
{
    LifecycleState state = LifecycleState::Unknown;
    if (!gateway_.getState(node, state, error)) {
        return false;
    }
    if (state == LifecycleState::Active || state == LifecycleState::Inactive) {
        return true;
    }
    if (state == LifecycleState::Unconfigured) {
        return transitionAndVerify(
            node, Transition::Configure, LifecycleState::Inactive, error);
    }
    error = "node is not in a stable state that can be configured";
    return false;
}

bool Coordinator::ensureActivated(NodeKey node, std::string &error)
{
    LifecycleState state = LifecycleState::Unknown;
    if (!gateway_.getState(node, state, error)) {
        return false;
    }
    if (state == LifecycleState::Active) {
        return true;
    }
    if (state != LifecycleState::Inactive) {
        error = "node is not in a stable state that can be activated";
        return false;
    }
    return transitionAndVerify(node, Transition::Activate, LifecycleState::Active, error);
}

bool Coordinator::transitionAndVerify(
    NodeKey node,
    Transition transition,
    LifecycleState expected,
    std::string &error)
{
    if (!gateway_.changeState(node, transition, error)) {
        return false;
    }
    LifecycleState actual = LifecycleState::Unknown;
    if (!gateway_.getState(node, actual, error)) {
        return false;
    }
    if (actual != expected) {
        std::ostringstream stream;
        stream << "transition returned but state is " << static_cast<int>(actual)
               << ", expected " << static_cast<int>(expected);
        error = stream.str();
        return false;
    }
    return true;
}

bool Coordinator::rollback(std::string &error)
{
    bool success = true;
    std::ostringstream errors;
    for (const auto node : {
            NodeKey::Target, NodeKey::Controller, NodeKey::Keyboard}) {
        std::string node_error;
        if (!ensureInactive(node, node_error)) {
            success = false;
            errors << static_cast<int>(node) << ": " << node_error << "; ";
        }
    }
    error = errors.str();
    return success;
}

bool Coordinator::matches(const Snapshot &states, Mode desired_mode) const
{
    if (!states.complete) {
        return false;
    }
    const bool keyboard_active = states.keyboard == LifecycleState::Active;
    const bool target_active = states.target == LifecycleState::Active;
    const bool controller_active = states.controller == LifecycleState::Active;
    if (desired_mode == Mode::Keyboard) {
        return keyboard_active && isStoppedState(states.target) &&
            isStoppedState(states.controller);
    }
    if (desired_mode == Mode::Pid) {
        return isStoppedState(states.keyboard) && target_active && controller_active;
    }
    return isStoppedState(states.keyboard) && isStoppedState(states.target) &&
        isStoppedState(states.controller);
}

Mode Coordinator::classify(const Snapshot &states)
{
    if (!states.complete) {
        return Mode::Degraded;
    }
    const bool keyboard_active = states.keyboard == LifecycleState::Active;
    const bool target_active = states.target == LifecycleState::Active;
    const bool controller_active = states.controller == LifecycleState::Active;
    if (keyboard_active && isStoppedState(states.target) &&
        isStoppedState(states.controller)) {
        return Mode::Keyboard;
    }
    if (isStoppedState(states.keyboard) && target_active && controller_active) {
        return Mode::Pid;
    }
    if (isStoppedState(states.keyboard) && isStoppedState(states.target) &&
        isStoppedState(states.controller)) {
        return Mode::Stopped;
    }
    return Mode::Degraded;
}

}  // namespace app::motion


