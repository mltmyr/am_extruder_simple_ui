#ifndef SIMPLE_EXTRUDER_UI_HPP__
#define SIMPLE_EXTRUDER_UI_HPP__

#include <QWidget>
#include <thread>

#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ui_simple_extruder_ui.hpp"

namespace am_extruder_simple_ui
{

class SimpleExtruderUIPlugin : public rqt_gui_cpp::Plugin
{
	Q_OBJECT
public:
	SimpleExtruderUIPlugin();
	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


private:
	void temp_target_changed_callback(double temp);
	void speed_target_changed_callback(double speed);

	void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const;

	SimpleExtruderUIFrame* widget_;

	std::thread* spinner_;
	rclcpp::executors::SingleThreadedExecutor spin_executor_;
	rclcpp::Node::SharedPtr node_;

	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr targetTempPublisher_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr targetSpeedPublisher_;

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscription_;

	
};

} /* namespace am_extruder_simpleUI */

#endif /* SIMPLE_EXTRUDER_UI_HPP__ */
