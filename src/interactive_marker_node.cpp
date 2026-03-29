#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2/LinearMath/Transform.h>

#include <cmath>

using namespace visualization_msgs::msg;
using namespace interactive_markers;

class MenuMarkerNode : public rclcpp::Node
{
public:
	MenuMarkerNode() : Node("menu"), marker_pos_(0)
	{
	}

	void init()
	{
		server_ = std::make_shared<InteractiveMarkerServer>(
			"menu", this->shared_from_this());

		initMenu();

		makeMenuMarker("marker1");
		makeMenuMarker("marker2");

		menu_handler_.apply(*server_, "marker1");
		menu_handler_.apply(*server_, "marker2");
		server_->applyChanges();
	}

private:
	std::shared_ptr<InteractiveMarkerServer> server_;
	MenuHandler menu_handler_;
	MenuHandler::EntryHandle h_first_entry_;
	MenuHandler::EntryHandle h_mode_last_;
	float marker_pos_;

	void enableCb(const InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		MenuHandler::EntryHandle handle = feedback->menu_entry_id;
		MenuHandler::CheckState state;
		menu_handler_.getCheckState(handle, state);

		if (state == MenuHandler::CHECKED)
		{
			menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
			RCLCPP_INFO(this->get_logger(), "Hiding first menu entry");
			menu_handler_.setVisible(h_first_entry_, false);
		}
		else
		{
			menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
			RCLCPP_INFO(this->get_logger(), "Showing first menu entry");
			menu_handler_.setVisible(h_first_entry_, true);
		}
		menu_handler_.reApply(*server_);
		RCLCPP_INFO(this->get_logger(), "update");
		server_->applyChanges();
	}

	void modeCb(const InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		menu_handler_.setCheckState(h_mode_last_, MenuHandler::UNCHECKED);
		h_mode_last_ = feedback->menu_entry_id;
		menu_handler_.setCheckState(h_mode_last_, MenuHandler::CHECKED);

		RCLCPP_INFO(this->get_logger(), "Switching to menu entry #%d", h_mode_last_);

		menu_handler_.reApply(*server_);
		server_->applyChanges();
	}

	void deepCb(const InteractiveMarkerFeedback::ConstSharedPtr &)
	{
		RCLCPP_INFO(this->get_logger(), "The deep sub-menu has been found.");
	}

	Marker makeBox(InteractiveMarker &msg)
	{
		Marker marker;
		marker.type = Marker::CUBE;
		marker.scale.x = msg.scale * 0.45;
		marker.scale.y = msg.scale * 0.45;
		marker.scale.z = msg.scale * 0.45;
		marker.color.r = 0.5;
		marker.color.g = 0.5;
		marker.color.b = 0.5;
		marker.color.a = 1.0;
		return marker;
	}

	InteractiveMarker makeEmptyMarker()
	{
		InteractiveMarker int_marker;
		int_marker.header.frame_id = "map";
		int_marker.pose.position.y = -3.0 * marker_pos_++;
		int_marker.scale = 1;
		return int_marker;
	}

	void makeMenuMarker(const std::string &name)
	{
		InteractiveMarker int_marker = makeEmptyMarker();
		int_marker.name = name;

		InteractiveMarkerControl control;
		control.interaction_mode = InteractiveMarkerControl::BUTTON;
		control.always_visible = true;
		control.markers.push_back(makeBox(int_marker));
		int_marker.controls.push_back(control);

		server_->insert(int_marker);
	}

	void initMenu()
	{
		h_first_entry_ = menu_handler_.insert("First Entry");
		MenuHandler::EntryHandle entry = menu_handler_.insert(h_first_entry_, "deep");
		entry = menu_handler_.insert(entry, "sub");
		entry = menu_handler_.insert(entry, "menu",
			std::bind(&MenuMarkerNode::deepCb, this, std::placeholders::_1));

		menu_handler_.setCheckState(
			menu_handler_.insert("Show First Entry",
				std::bind(&MenuMarkerNode::enableCb, this, std::placeholders::_1)),
			MenuHandler::CHECKED);

		MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Switch");

		for (int i = 0; i < 5; i++)
		{
			std::ostringstream s;
			s << "Mode " << i;
			h_mode_last_ = menu_handler_.insert(sub_menu_handle, s.str(),
				std::bind(&MenuMarkerNode::modeCb, this, std::placeholders::_1));
			menu_handler_.setCheckState(h_mode_last_, MenuHandler::UNCHECKED);
		}
		menu_handler_.setCheckState(h_mode_last_, MenuHandler::CHECKED);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MenuMarkerNode>();
	node->init();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
