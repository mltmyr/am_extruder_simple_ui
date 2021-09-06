#include "am_extruder_simple_ui/simple_extruder_ui.hpp"

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace am_extruder_simple_ui
{

SimpleExtruderUIPlugin::SimpleExtruderUIPlugin()
	: rqt_gui_cpp::Plugin(), widget_(0)
{
	setObjectName("SimpleExtruderUI");
}

void SimpleExtruderUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
	std::string node_name = std::string("simple_extruder_ui");
	QString window_title_postfix = "";
	if (context.serialNumber() > 1)
	{
		node_name.append( std::string("_")+std::to_string(context.serialNumber()) );
		window_title_postfix.append( " (" + QString::number(context.serialNumber()) + ")" );
	}
	this->node_ = rclcpp::Node::make_shared(node_name);
	
	this->spin_executor_.add_node(this->node_);

	this->targetTempPublisher_  = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(
		"filament_heater_controller/commands", 1);
	this->targetSpeedPublisher_ = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(
		"filament_mover_controller/commands", 1);
	
	this->jointStateSubscription_ = this->node_->create_subscription<sensor_msgs::msg::JointState>(
		"joint_states", 1, std::bind(&SimpleExtruderUIPlugin::joint_states_callback, this, std::placeholders::_1));

	this->widget_ = new SimpleExtruderUIFrame(
		std::bind(&SimpleExtruderUIPlugin::temp_target_changed_callback, this, std::placeholders::_1), 
		std::bind(&SimpleExtruderUIPlugin::speed_target_changed_callback, this, std::placeholders::_1));

	this->widget_->setWindowTitle(this->widget_->windowTitle() + window_title_postfix);
	context.addWidget(this->widget_);

	this->spinner_ = new std::thread([this]() { this->spin_executor_.spin(); });
	this->spinner_->detach();

	 RCLCPP_INFO(this->node_->get_logger(), "%s plugin initialized.", this->node_->get_name());

	 return;
}

void SimpleExtruderUIPlugin::shutdownPlugin()
{
	RCLCPP_INFO(this->node_->get_logger(), "Shuting down %s", this->node_->get_name());
	this->spin_executor_.cancel();
	//this->spin_executor_.remove_node(this->node_);
	delete this->spinner_;
	//this->jointStateSubscription_.reset();
	//this->targetTempPublisher_.reset();
	//this->targetSpeedPublisher_.reset();
	//this->node_.reset();
}

void SimpleExtruderUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
	(void)plugin_settings;
	(void)instance_settings;
	return;
}

void SimpleExtruderUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
	(void)plugin_settings;
	(void)instance_settings;
	return;
}

void SimpleExtruderUIPlugin::temp_target_changed_callback(double temp)
{
	auto msg = std_msgs::msg::Float64MultiArray();
	msg.data.emplace_back(temp);
	this->targetTempPublisher_->publish(msg);

	/* RCLCPP_INFO(this->node_->get_logger(), "temperature target changed to %f celsius.", temp); */
	return;
}

void SimpleExtruderUIPlugin::speed_target_changed_callback(double speed)
{
	auto msg = std_msgs::msg::Float64MultiArray();
	msg.data.emplace_back(speed);
	this->targetSpeedPublisher_->publish(msg);

	/* RCLCPP_INFO(this->node_->get_logger(), "filament speed target changed to %f mm/s.", speed); */
	return;
}

void SimpleExtruderUIPlugin::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
{
	unsigned int n = msg->name.size();
	for (unsigned int i = 0; i < n; i++)
	{
		if (msg->name[(n-1)-i].compare("filament_heater"))
		{
			this->widget_->setCurrentTemp(msg->position[i]);
		}
		else if (msg->name[(n-1)-i].compare("filament_mover"))
		{
			this->widget_->setCurrentSpeed(msg->velocity[i]);
		}
	}

	/* RCLCPP_INFO(this->node_->get_logger(), "received JointState message: [%s, %s], [%f, %f], [%f, %f]",
		msg->name[1].c_str(), msg->name[0].c_str(),
		msg->position[0],     msg->position[1],
		msg->velocity[0],     msg->velocity[1]
	); */

	return;
}

} /* namespace am_extruder_simple_ui */

PLUGINLIB_EXPORT_CLASS(am_extruder_simple_ui::SimpleExtruderUIPlugin, rqt_gui_cpp::Plugin)
