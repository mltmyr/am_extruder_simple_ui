#include "am_extruder_simpleUI/simple_extruder_ui.hpp"

#include <pluginlib/class_list_macros.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/Float64MultiArray"

namespace am_extruder_simpleUI
{

SimpleExtruderUIPlugin::SimpleExtruderUIPlugin()
	: rqt_gui_cpp::Plugin(), rclcpp::Node("simple_extruder_ui"), widget_(0)
{
	setObjectName("SimpleExtruderUI");
}

void SimpleExtruderUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
	this->targetTempPublisher_  = this->create_publisher<std_msgs::msg::Float64MultiArray>(
		"filament_heater_controller/commands", 1);
	this->targetSpeedPublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
		"filament_mover_controller/commands", 1);
	
	this->jointStateSubscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		"joint_states", 1, std::bind(&SimpleExtruderUIPlugin::joint_states_callback, this, _1));

	this->widget_ = new SimpleExtruderUIFrame(
		std::bind(&SimpleExtruderUIPlugin::temp_target_changed_callback, this, _1), 
		std::bind(&SimpleExtruderUIPlugin::speed_target_changed_callback, this, _1));

}

void SimpleExtruderUIPlugin::shutdownPlugin()
{

}

void SimpleExtruderUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{

}

void SimpleExtruderUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{

}

void temp_target_changed_callback(double temp)
{
	auto msg = std_msgs::msg::Float64MultiArray();
	msg.data.emplace_back(temp);
	this->targetTempPublisher_->publish(msg);
	return;
}

void speed_target_changed_callback(double speed)
{
	auto msg = std_msgs::msg::Float64MultiArray();
	msg.data.emplace_back(speed);
	this->targetSpeedPublisher_->publish(msg);
	return;
}

void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
	for (int i = 0; i < msg->name.size(); i++)
	{
		if (msg->name[i].compare("filament_heater"))
		{
			this->widget_->setCurrentTemp(msg->position[i]);
		}
		else if (msg->name[i].compare("filament_mover"))
		{
			this->widget_->setCurrentSpeed(msg->velocity[i]);
		}
	}
	return;
}


} /* namespace am_extruder_simpleUI */

PLUGINLIB_DECLARE_CLASS(am_extruder_simpleUI, SimpleExtruderUIPlugin, am_extruder_simpleUI::SimpleExtruderUIPlugin, rqt_gui_cpp::Plugin)