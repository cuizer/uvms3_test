#ifndef UVMS_HAL_MANIPULATOR_PROTOCOL_PARSER_HPP
#define UVMS_HAL_MANIPULATOR_PROTOCOL_PARSER_HPP

#include <optional>
#include <map>
#include "types.hpp"

namespace uvms_hal_manipulator
{

class ProtocolParser
{
public:
    ProtocolParser() = default;
    ~ProtocolParser() = default;

    CanFrame pack_arm_control_command(ArmControlCommand cmd) const;
    std::optional<CompleteMessage> process_can_frame(const CanFrame& frame);

    bool get_armcabin_motor_state(ArmCabinMotorState& state) const;
    bool get_arm_motor_state(ArmMotorState& state) const;
    bool get_arm_controller_state(ArmControllerState& state) const;

private:
    std::optional<CompleteMessage> process_fragment(
        uint8_t app_msg_id,
        uint8_t total_frames,
        uint8_t frame_index,
        const std::vector<uint8_t>& fragment_payload);

    bool is_complete(const FragmentBuffer& buffer) const;
    std::vector<uint8_t> assemble_payload(const FragmentBuffer& buffer) const;

    bool parse_armcabin_motor_payload(const std::vector<uint8_t>& payload, ArmCabinMotorState& state) const;
    bool parse_arm_motor_payload(const std::vector<uint8_t>& payload, ArmMotorState& state) const;
    bool parse_arm_controller_payload(const std::vector<uint8_t>& payload, ArmControllerState& state) const;

    int16_t read_int16_le(const std::vector<uint8_t>& data, size_t offset) const;
    uint16_t read_uint16_le(const std::vector<uint8_t>& data, size_t offset) const;

private:
    std::map<uint8_t, FragmentBuffer> fragment_buffers_;
    ArmCabinMotorState armcabin_motor_state_{};
    ArmMotorState arm_motor_state_{};
    ArmControllerState arm_controller_state_{};
    bool has_armcabin_motor_state_{false};
    bool has_arm_motor_state_{false};
    bool has_arm_controller_state_{false};
};

}  // namespace uvms_hal_manipulator

#endif