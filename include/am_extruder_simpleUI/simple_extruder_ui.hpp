#ifndef SIMPLE_EXTRUDER_UI_HPP__
#define SIMPLE_EXTRUDER_UI_HPP__

#include <rqt_gui_cpp/plugin.h>
#include "ui_simple_extruder_ui.hpp"

#include <QWidget>

namespace am_extruder_simpleUI
{

class SimpleExtruderUIPlugin : public rqt_gui_cpp::Plugin, public rclcpp::Node
{
	Q_OBJECT
public:
	SimpleExtruderUIPlugin();
	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


private:
	void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

	SimpleExtruderUIFrame* widget_;



	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr targetTempPublisher_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr targetSpeedPublisher_;
	//These should be called when the gui is updated, or with a periodic interval of say x Hz, with a timer.

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscription_;
	//Want to bind the subscriptions to the gui label text displays. The rclcpp::spin function calls these
	// when the topic us updated.
};

} /* namespace am_extruder_simpleUI */

#endif /* SIMPLE_EXTRUDER_UI_HPP__ */
