/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros/ControlSetpoint.h>

#include <geometry_msgs/Pose.h>


namespace mavplugin {
/**
 * @brief Setpoint control plugin
 *
 * Send setpoint to FCU for specified controller.
 */
class ForceSetpointPubPlugin : public MavRosPlugin {
public:
	ForceSetpointPubPlugin() :
		sp_nh("~force_setpoint_pub"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		force_setpoint_pub = sp_nh.advertise<mavros::ControlSetpoint>("force_setpoint_debug", 10);
	}

    const message_map get_rx_handlers() {
    	return
    	{
    		MESSAGE_HANDLER(MAVLINK_MSG_ID_SET_CONTROL_TARGET_LOCAL_NED, &ForceSetpointPubPlugin::handle_force_setpoint)
    	};
	}


private:
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Publisher force_setpoint_pub;

	void handle_force_setpoint(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

		mavlink_set_control_target_local_ned_t force_setpoint;
		mavlink_msg_set_control_target_local_ned_decode(msg, &force_setpoint);

		float x, y, z, yaw;
		x = force_setpoint.x;
		y = force_setpoint.y;
		z = force_setpoint.z;
		yaw = force_setpoint.yaw;

		auto force_msg = boost::make_shared<mavros::ControlSetpoint>();

		force_msg->x = x;
		force_msg->y = y;
		force_msg->z = z;
		force_msg->yaw = yaw;

		force_setpoint_pub.publish(force_msg);
	}

};
}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ForceSetpointPubPlugin, mavplugin::MavRosPlugin)
