#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace app::motion {

constexpr std::size_t REQUEST_SIZE = 20U;
constexpr std::size_t RESPONSE_SIZE = 24U;
constexpr std::uint8_t PROTOCOL_VERSION = 1U;
constexpr std::uint16_t MIN_LEASE_MS = 1000U;
constexpr std::uint16_t MAX_LEASE_MS = 10000U;
constexpr std::size_t DATA_PLANE_SIZE = 17U;
constexpr std::uint8_t DATA_PLANE_HEADER = 0xA5U;
constexpr std::uint16_t DATA_FLAG_ESTOP = 0x0001U;
constexpr std::uint16_t DATA_FLAG_DEPTH_HOLD = 0x0002U;
constexpr std::uint16_t DATA_FLAG_YAW_HOLD = 0x0004U;
constexpr std::uint16_t DATA_FLAG_MASK =
    DATA_FLAG_ESTOP | DATA_FLAG_DEPTH_HOLD | DATA_FLAG_YAW_HOLD;

enum class Command : std::uint8_t {
    SetMode = 1U,
    GetStatus = 2U,
    Heartbeat = 3U,
};

enum class Mode : std::uint8_t {
    Stopped = 0U,
    Keyboard = 1U,
    Pid = 2U,
    Degraded = 3U,
};

enum class MessageType : std::uint8_t {
    Accepted = 1U,
    Final = 2U,
};

enum class ResultCode : std::uint8_t {
    Ok = 0U,
    Accepted = 1U,
    Busy = 2U,
    Invalid = 3U,
    Timeout = 4U,
    TransitionFailed = 5U,
    PreconditionFailed = 6U,
    FaultLocked = 7U,
};

enum class ErrorCode : std::uint16_t {
    None = 0U,
    InvalidLength = 1U,
    InvalidMagic = 2U,
    UnsupportedVersion = 3U,
    InvalidCommand = 4U,
    InvalidMode = 5U,
    InvalidFlags = 6U,
    InvalidLease = 7U,
    CrcMismatch = 8U,
    LifecycleUnavailable = 100U,
    TransitionFailed = 101U,
    VerificationFailed = 102U,
    RollbackFailed = 103U,
    LeaseOwnerMismatch = 104U,
};

enum class LifecycleState : std::uint8_t {
    Unknown = 0U,
    Unconfigured = 1U,
    Inactive = 2U,
    Active = 3U,
    Finalized = 4U,
    Configuring = 10U,
    CleaningUp = 11U,
    ShuttingDown = 12U,
    Activating = 13U,
    Deactivating = 14U,
    ErrorProcessing = 15U,
};

struct Request {
    Command command = Command::GetStatus;
    Mode mode = Mode::Stopped;
    std::uint32_t client_id = 0U;
    std::uint32_t request_id = 0U;
    std::uint16_t lease_ms = 0U;
};

struct Response {
    bool success = false;
    MessageType message_type = MessageType::Final;
    ResultCode result = ResultCode::Invalid;
    Mode current_mode = Mode::Degraded;
    Mode desired_mode = Mode::Stopped;
    LifecycleState keyboard_state = LifecycleState::Unknown;
    LifecycleState target_state = LifecycleState::Unknown;
    LifecycleState controller_state = LifecycleState::Unknown;
    std::uint32_t client_id = 0U;
    std::uint32_t request_id = 0U;
    std::uint16_t error_code = 0U;
};

struct RequestDecodeResult {
    bool ok = false;
    Request request{};
    ErrorCode error = ErrorCode::None;
    std::string message;
};

struct ResponseDecodeResult {
    bool ok = false;
    Response response{};
    ErrorCode error = ErrorCode::None;
    std::string message;
};

struct DataPlaneCommand {
    std::uint16_t sequence = 0U;
    std::int16_t surge = 0;
    std::int16_t sway = 0;
    std::int16_t heave = 0;
    std::int16_t yaw = 0;
    std::uint16_t flags = 0U;
};

struct DataPlaneDecodeResult {
    bool ok = false;
    DataPlaneCommand command{};
    ErrorCode error = ErrorCode::None;
    std::string message;
};

inline std::uint16_t crc16(const std::uint8_t *data, std::size_t size)
{
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U)
                : static_cast<std::uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

inline void appendU16(std::vector<std::uint8_t> &packet, std::uint16_t value)
{
    packet.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    packet.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

inline void appendU32(std::vector<std::uint8_t> &packet, std::uint32_t value)
{
    for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
        packet.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
    }
}

inline std::uint16_t readU16(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

inline std::uint32_t readU32(const std::uint8_t *data)
{
    return static_cast<std::uint32_t>(data[0]) |
        (static_cast<std::uint32_t>(data[1]) << 8U) |
        (static_cast<std::uint32_t>(data[2]) << 16U) |
        (static_cast<std::uint32_t>(data[3]) << 24U);
}

inline DataPlaneDecodeResult decodeDataPlaneCommand(
    const std::uint8_t *data,
    std::size_t size)
{
    DataPlaneDecodeResult decoded;
    if (size != DATA_PLANE_SIZE) {
        decoded.error = ErrorCode::InvalidLength;
        decoded.message = "data-plane command must be exactly 17 bytes";
        return decoded;
    }
    if (data[0] != DATA_PLANE_HEADER) {
        decoded.error = ErrorCode::InvalidMagic;
        decoded.message = "invalid data-plane header";
        return decoded;
    }
    if (readU16(data + 15U) != crc16(data, 15U)) {
        decoded.error = ErrorCode::CrcMismatch;
        decoded.message = "data-plane CRC mismatch";
        return decoded;
    }
    const auto flags = readU16(data + 11U);
    if (readU16(data + 13U) != 0U || (flags & ~DATA_FLAG_MASK) != 0U) {
        decoded.error = ErrorCode::InvalidFlags;
        decoded.message = "data-plane reserved field or flags are invalid";
        return decoded;
    }

    decoded.command.sequence = readU16(data + 1U);
    decoded.command.surge = static_cast<std::int16_t>(readU16(data + 3U));
    decoded.command.sway = static_cast<std::int16_t>(readU16(data + 5U));
    decoded.command.heave = static_cast<std::int16_t>(readU16(data + 7U));
    decoded.command.yaw = static_cast<std::int16_t>(readU16(data + 9U));
    decoded.command.flags = flags;
    decoded.ok = true;
    return decoded;
}

inline double decodeBoundedVelocity(std::int16_t raw_millimetres_per_second, double max_abs)
{
    const double velocity = raw_millimetres_per_second / 1000.0;
    if (velocity > max_abs) return max_abs;
    if (velocity < -max_abs) return -max_abs;
    return velocity;
}

inline bool isLifecycleState(std::uint8_t value)
{
    return value <= 4U || (value >= 10U && value <= 15U);
}

inline std::vector<std::uint8_t> encodeRequest(const Request &request)
{
    std::vector<std::uint8_t> packet;
    packet.reserve(REQUEST_SIZE);
    packet.insert(packet.end(), {'U', 'V', 'M', 'C'});
    packet.push_back(PROTOCOL_VERSION);
    packet.push_back(static_cast<std::uint8_t>(request.command));
    packet.push_back(static_cast<std::uint8_t>(request.mode));
    packet.push_back(0U);
    appendU32(packet, request.client_id);
    appendU32(packet, request.request_id);
    appendU16(packet, request.lease_ms);
    appendU16(packet, crc16(packet.data(), packet.size()));
    return packet;
}

inline RequestDecodeResult decodeRequest(const std::uint8_t *data, std::size_t size)
{
    RequestDecodeResult decoded;
    if (size != REQUEST_SIZE) {
        decoded.error = ErrorCode::InvalidLength;
        decoded.message = "request must be exactly 20 bytes";
        return decoded;
    }
    decoded.request.client_id = readU32(data + 8U);
    decoded.request.request_id = readU32(data + 12U);
    if (data[0] != 'U' || data[1] != 'V' || data[2] != 'M' || data[3] != 'C') {
        decoded.error = ErrorCode::InvalidMagic;
        decoded.message = "invalid request magic";
        return decoded;
    }
    if (data[4] != PROTOCOL_VERSION) {
        decoded.error = ErrorCode::UnsupportedVersion;
        decoded.message = "unsupported request version";
        return decoded;
    }
    if (data[7] != 0U) {
        decoded.error = ErrorCode::InvalidFlags;
        decoded.message = "reserved request flags must be zero";
        return decoded;
    }
    if (readU16(data + 18U) != crc16(data, 18U)) {
        decoded.error = ErrorCode::CrcMismatch;
        decoded.message = "request CRC mismatch";
        return decoded;
    }

    if (data[5] < static_cast<std::uint8_t>(Command::SetMode) ||
        data[5] > static_cast<std::uint8_t>(Command::Heartbeat)) {
        decoded.error = ErrorCode::InvalidCommand;
        decoded.message = "invalid request command";
        return decoded;
    }
    if (data[6] > static_cast<std::uint8_t>(Mode::Pid)) {
        decoded.error = ErrorCode::InvalidMode;
        decoded.message = "invalid requested mode";
        return decoded;
    }

    decoded.request.command = static_cast<Command>(data[5]);
    decoded.request.mode = static_cast<Mode>(data[6]);
    decoded.request.lease_ms = readU16(data + 16U);

    const bool active_lease = decoded.request.mode == Mode::Keyboard ||
        decoded.request.mode == Mode::Pid;
    if (decoded.request.command == Command::GetStatus) {
        if (decoded.request.mode != Mode::Stopped || decoded.request.lease_ms != 0U) {
            decoded.error = ErrorCode::InvalidLease;
            decoded.message = "GET_STATUS requires STOPPED and a zero lease";
            return decoded;
        }
    } else if (decoded.request.command == Command::Heartbeat) {
        if (!active_lease) {
            decoded.error = ErrorCode::InvalidMode;
            decoded.message = "HEARTBEAT requires an active mode";
            return decoded;
        }
    }
    if (active_lease) {
        if (decoded.request.lease_ms < MIN_LEASE_MS ||
            decoded.request.lease_ms > MAX_LEASE_MS) {
            decoded.error = ErrorCode::InvalidLease;
            decoded.message = "active-mode lease is outside the allowed range";
            return decoded;
        }
    } else if (decoded.request.lease_ms != 0U) {
        decoded.error = ErrorCode::InvalidLease;
        decoded.message = "stopped mode requires a zero lease";
        return decoded;
    }

    decoded.ok = true;
    return decoded;
}

inline std::vector<std::uint8_t> encodeResponse(const Response &response)
{
    std::vector<std::uint8_t> packet;
    packet.reserve(RESPONSE_SIZE);
    packet.insert(packet.end(), {'U', 'V', 'M', 'C'});
    packet.push_back(PROTOCOL_VERSION);
    packet.push_back(static_cast<std::uint8_t>(response.message_type));
    packet.push_back(static_cast<std::uint8_t>(response.result));
    packet.push_back(static_cast<std::uint8_t>(response.current_mode));
    packet.push_back(static_cast<std::uint8_t>(response.desired_mode));
    packet.push_back(static_cast<std::uint8_t>(response.keyboard_state));
    packet.push_back(static_cast<std::uint8_t>(response.target_state));
    packet.push_back(static_cast<std::uint8_t>(response.controller_state));
    appendU32(packet, response.client_id);
    appendU32(packet, response.request_id);
    appendU16(packet, response.error_code);
    appendU16(packet, crc16(packet.data(), packet.size()));
    return packet;
}

inline ResponseDecodeResult decodeResponse(const std::uint8_t *data, std::size_t size)
{
    ResponseDecodeResult decoded;
    if (size != RESPONSE_SIZE) {
        decoded.error = ErrorCode::InvalidLength;
        decoded.message = "response must be exactly 24 bytes";
        return decoded;
    }
    if (data[0] != 'U' || data[1] != 'V' || data[2] != 'M' || data[3] != 'C') {
        decoded.error = ErrorCode::InvalidMagic;
        decoded.message = "invalid response magic";
        return decoded;
    }
    if (data[4] != PROTOCOL_VERSION) {
        decoded.error = ErrorCode::UnsupportedVersion;
        decoded.message = "unsupported response version";
        return decoded;
    }
    if (readU16(data + 22U) != crc16(data, 22U)) {
        decoded.error = ErrorCode::CrcMismatch;
        decoded.message = "response CRC mismatch";
        return decoded;
    }
    if (data[5] < static_cast<std::uint8_t>(MessageType::Accepted) ||
        data[5] > static_cast<std::uint8_t>(MessageType::Final) ||
        data[6] > static_cast<std::uint8_t>(ResultCode::FaultLocked) ||
        data[7] > static_cast<std::uint8_t>(Mode::Degraded) ||
        data[8] > static_cast<std::uint8_t>(Mode::Pid) ||
        !isLifecycleState(data[9]) || !isLifecycleState(data[10]) ||
        !isLifecycleState(data[11])) {
        decoded.error = ErrorCode::InvalidMode;
        decoded.message = "response contains an invalid enum or lifecycle state";
        return decoded;
    }

    decoded.response.message_type = static_cast<MessageType>(data[5]);
    decoded.response.result = static_cast<ResultCode>(data[6]);
    decoded.response.success = decoded.response.message_type == MessageType::Final &&
        decoded.response.result == ResultCode::Ok;
    decoded.response.current_mode = static_cast<Mode>(data[7]);
    decoded.response.desired_mode = static_cast<Mode>(data[8]);
    decoded.response.keyboard_state = static_cast<LifecycleState>(data[9]);
    decoded.response.target_state = static_cast<LifecycleState>(data[10]);
    decoded.response.controller_state = static_cast<LifecycleState>(data[11]);
    decoded.response.client_id = readU32(data + 12U);
    decoded.response.request_id = readU32(data + 16U);
    decoded.response.error_code = readU16(data + 20U);
    decoded.ok = true;
    return decoded;
}

}  // namespace app::motion


