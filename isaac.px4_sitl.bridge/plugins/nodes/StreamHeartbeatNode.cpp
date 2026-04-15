#include <StreamHeartbeatNodeDatabase.h>
#include "MavlinkServerManager.hpp"


using omni::graph::core::Type;
using omni::graph::core::BaseDataType;


namespace isaac::px4_sitl::bridge{

class StreamHeartbeatNode{
public:
    static void initialize(const GraphContextObj &context, const NodeObj &nodeObj){
        constexpr u_char kDefaultSystemId = 1;

        AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, state::systemId.token());
        attr.iAttribute->setDefaultValue(attr, omni::fabric::BaseDataType::eUChar, &kDefaultSystemId, 0);
    }

    static bool compute(const StreamHeartbeatNodeDatabase &db){
        const auto &px4_instance = db.inputs.px4Instance();
        auto &server = MavlinkServerManager::getInstance();
        if(!server.isConnected(px4_instance)){
            return false;
        }

        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(
            db.state.systemId(), 
            MAV_COMP_ID_ONBOARD_COMPUTER, 
            &msg,
            MAV_TYPE_GENERIC, // Type
            MAV_AUTOPILOT_GENERIC,       // Autopilot type
            MAV_MODE_AUTO_ARMED,         // System mode
            0,                           // Custom mode
            MAV_STATE_ACTIVE          // System state
        );

        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        int len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
        return server.send(px4_instance, buffer.data(), len);
    } 
};
// Following allow visibility of node to omnigraph
REGISTER_OGN_NODE()  
}
