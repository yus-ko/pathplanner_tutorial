#include <rclcpp/rclcpp.hpp>
#include <potbot_lib/artificial_potential_field.hpp>
#include <potbot_lib/apf_path_planner.hpp>
#include <potbot_lib/utility.hpp>
#include <potbot_ros/utility.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <tf2/utils.h>

void toPointVec(const visualization_msgs::msg::Marker& obs, std::vector<geometry_msgs::msg::Point>& points)
{
	double origin_x = obs.pose.position.x;
	double origin_y = obs.pose.position.y;
	double origin_th = tf2::getYaw(obs.pose.orientation);
	double res = 0.05;

	Eigen::MatrixXd vertexes;
	if (obs.type == visualization_msgs::msg::Marker::CUBE)
	{
		double width = obs.scale.x;
		double height = obs.scale.y;

		Eigen::Matrix2d rotation = potbot_lib::utility::get_rotate_matrix(origin_th);
		Eigen::MatrixXd translation(4,2);
		translation <<  origin_x, origin_y,
						origin_x, origin_y,
						origin_x, origin_y,
						origin_x, origin_y;
		Eigen::MatrixXd origin_vertexes(4,2);
		origin_vertexes <<  -width/2,   -height/2,
							-width/2,   height/2,
							width/2,    height/2,
							width/2,    -height/2;

		vertexes = rotation*origin_vertexes.transpose() + translation.transpose();
	}
	else if (obs.type == visualization_msgs::msg::Marker::SPHERE)
	{
		double width = obs.scale.x;
		double height = obs.scale.y;

		Eigen::Matrix2d rotation = potbot_lib::utility::get_rotate_matrix(origin_th);
		Eigen::Vector2d translation;
		translation <<  origin_x, origin_y;

		size_t vertex_num = 2*M_PI/res;
		vertexes.resize(2,vertex_num);
		for (size_t i = 0; i < vertex_num; i++)
		{
			double t = 2 * M_PI * i / vertex_num;
			double x = width/2 * cos(t);
			double y = height/2 * sin(t);
			Eigen::Vector2d p;
			p << x, y;
			vertexes.col(i) = rotation*p + translation;
		}
	}

	for (size_t i = 0; i < vertexes.cols(); i++)
	{
		points.push_back(potbot_lib::utility::get_point(vertexes(0,i), vertexes(1,i)));
	}
}

void toPointVec(const std::vector<visualization_msgs::msg::Marker>& obs, std::vector<geometry_msgs::msg::Point>& points)
{
	for (const auto& o : obs)
	{
		toPointVec(o, points);
	}
}

class MarkerPathPlanner : public rclcpp::Node
{
private:
	geometry_msgs::msg::PoseWithCovarianceStamped robot_;
	geometry_msgs::msg::PoseStamped goal_;
	std::vector<geometry_msgs::msg::Point> obstacles_;

	std::string planning_method_ = "dijkstra";

	// APF パラメータ
	double field_resolution_ = 0.05;
	int field_rows_ = 240;
	int field_cols_ = 240;
	double weight_attraction_ = 0.1;
	double weight_repulsion_ = 0.1;
	double distance_threshold_repulsion_ = 0.3;
	double max_path_length_ = 6.0;
	int path_search_range_ = 1;
	double weight_potential_ = 0.0;
	double weight_pose_ = 1.0;

	// Interactive markers
	size_t interactive_marker_num_ = 3;
	std::vector<visualization_msgs::msg::Marker> interactive_markers_;

	// APF
	std::unique_ptr<potbot_lib::ArtificialPotentialField> apf_;

	// ROS 2
	std::shared_ptr<interactive_markers::InteractiveMarkerServer> imsrv_;
	std::shared_ptr<interactive_markers::MenuHandler> menu_handler_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_field_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_inipose_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;
	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	void inipose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		robot_ = *msg;
	}

	void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		goal_ = *msg;
	}

	void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
	{
		obstacles_.push_back(msg->point);
	}

	rcl_interfaces::msg::SetParametersResult param_callback(
		const std::vector<rclcpp::Parameter> &parameters)
	{
		auto result = rcl_interfaces::msg::SetParametersResult();
		result.successful = true;
		for (const auto &param : parameters)
		{
			if (param.get_name() == "planning_method")
				planning_method_ = param.as_string();
			else if (param.get_name() == "field_resolution")
				field_resolution_ = param.as_double();
			else if (param.get_name() == "field_rows")
				field_rows_ = param.as_int();
			else if (param.get_name() == "field_cols")
				field_cols_ = param.as_int();
			else if (param.get_name() == "weight_attraction")
				weight_attraction_ = param.as_double();
			else if (param.get_name() == "weight_repulsion")
				weight_repulsion_ = param.as_double();
			else if (param.get_name() == "distance_threshold_repulsion")
				distance_threshold_repulsion_ = param.as_double();
			else if (param.get_name() == "max_path_length")
				max_path_length_ = param.as_double();
			else if (param.get_name() == "path_search_range")
				path_search_range_ = param.as_int();
			else if (param.get_name() == "weight_potential")
				weight_potential_ = param.as_double();
			else if (param.get_name() == "weight_pose")
				weight_pose_ = param.as_double();
		}
		return result;
	}

	void marker_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		std::stringstream ss(feedback->marker_name);
		std::string segment;
		std::vector<std::string> segments;

		while (std::getline(ss, segment, '_'))
		{
			segments.push_back(segment);
		}

		if (segments.size() > 1)
		{
			if (segments[0] == "obstacle")
			{
				int id = std::stoi(segments[1]);
				interactive_markers_[id].pose = feedback->pose;

				visualization_msgs::msg::InteractiveMarker int_marker;
				if (imsrv_->get(feedback->marker_name, int_marker))
				{
					size_t eid = feedback->menu_entry_id;
					if (eid == 4)
						int_marker.controls[0].markers[0].scale.x *= 2;
					else if (eid == 5)
						int_marker.controls[0].markers[0].scale.x *= 0.5;
					else if (eid == 6)
						int_marker.controls[0].markers[0].scale.y *= 2;
					else if (eid == 7)
						int_marker.controls[0].markers[0].scale.y *= 0.5;
					else if (eid == 8)
					{
						int_marker.controls[0].markers[0].scale.x *= 2;
						int_marker.controls[0].markers[0].scale.y *= 2;
					}
					else if (eid == 9)
					{
						int_marker.controls[0].markers[0].scale.x *= 0.5;
						int_marker.controls[0].markers[0].scale.y *= 0.5;
					}
					else if (eid == 11)
						int_marker.controls[0].markers[0].type = visualization_msgs::msg::Marker::CUBE;
					else if (eid == 12)
						int_marker.controls[0].markers[0].type = visualization_msgs::msg::Marker::SPHERE;

					interactive_markers_[id].scale = int_marker.controls[0].markers[0].scale;
					interactive_markers_[id].type = int_marker.controls[0].markers[0].type;

					imsrv_->insert(int_marker,
						std::bind(&MarkerPathPlanner::marker_feedback, this, std::placeholders::_1));
					imsrv_->applyChanges();
				}
			}
		}
	}

	void init_interactive_markers()
	{
		interactive_markers_.resize(interactive_marker_num_);
		visualization_msgs::msg::Marker init_marker;
		init_marker.type = visualization_msgs::msg::Marker::CUBE;
		init_marker.scale.x = 0.5;
		init_marker.scale.y = 0.5;
		init_marker.scale.z = 0.2;
		init_marker.color.r = 0.5;
		init_marker.color.g = 0.5;
		init_marker.color.b = 0.5;
		init_marker.color.a = 1.0;
		init_marker.pose = potbot_lib::utility::get_pose();
		std::fill(interactive_markers_.begin(), interactive_markers_.end(), init_marker);
	}

	void init_interactive_marker_server()
	{
		init_interactive_markers();

		imsrv_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
			"simple_marker", this->shared_from_this());
		menu_handler_ = std::make_shared<interactive_markers::MenuHandler>();

		interactive_markers::MenuHandler::EntryHandle x_entry = menu_handler_->insert("scale x");
		interactive_markers::MenuHandler::EntryHandle y_entry = menu_handler_->insert("scale y");
		interactive_markers::MenuHandler::EntryHandle xy_entry = menu_handler_->insert("scale xy");

		auto fb = std::bind(&MarkerPathPlanner::marker_feedback, this, std::placeholders::_1);

		menu_handler_->insert(x_entry, "x2", fb);
		menu_handler_->insert(x_entry, "x0.5", fb);
		menu_handler_->insert(y_entry, "x2", fb);
		menu_handler_->insert(y_entry, "x0.5", fb);
		menu_handler_->insert(xy_entry, "x2", fb);
		menu_handler_->insert(xy_entry, "x0.5", fb);

		interactive_markers::MenuHandler::EntryHandle type_entry = menu_handler_->insert("marker type");
		menu_handler_->insert(type_entry, "cube", fb);
		menu_handler_->insert(type_entry, "sphere", fb);

		visualization_msgs::msg::Marker move_marker;
		move_marker.type = visualization_msgs::msg::Marker::SPHERE;
		move_marker.scale.x = 0.2;
		move_marker.scale.y = 0.2;
		move_marker.scale.z = 0.2;
		move_marker.color.r = 0.0;
		move_marker.color.g = 0.0;
		move_marker.color.b = 0.7;
		move_marker.color.a = 1.0;
		move_marker.pose = potbot_lib::utility::get_pose(0, 0.5, 1, 0, 0, 0);

		visualization_msgs::msg::InteractiveMarkerControl move_control;
		move_control.name = "move_plane";
		move_control.orientation = potbot_lib::utility::get_quat(0, -M_PI_2, 0);
		move_control.always_visible = true;
		move_control.markers.push_back(move_marker);
		move_control.interaction_mode =
			visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

		visualization_msgs::msg::InteractiveMarkerControl rotate_control;
		rotate_control.name = "rotate_yaw";
		rotate_control.orientation = potbot_lib::utility::get_quat(0, -M_PI_2, 0);
		rotate_control.interaction_mode =
			visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

		for (size_t i = 0; i < interactive_marker_num_; i++)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			int_marker.header.frame_id = "map";
			int_marker.header.stamp = this->now();
			int_marker.name = "obstacle_" + std::to_string(i);
			int_marker.description = int_marker.name;
			int_marker.pose = potbot_lib::utility::get_pose(6, 0, 1, 0, 0, 0);

			move_control.markers[0] = interactive_markers_[i];

			int_marker.controls.push_back(move_control);
			int_marker.controls.push_back(rotate_control);

			imsrv_->insert(int_marker, fb);

			interactive_markers_[i].pose = int_marker.pose;
			menu_handler_->apply(*imsrv_, int_marker.name);
		}

		imsrv_->applyChanges();
	}

	void timer_callback()
	{
		double robot_x = robot_.pose.pose.position.x;
		double robot_y = robot_.pose.pose.position.y;
		double goal_x = goal_.pose.position.x;
		double goal_y = goal_.pose.position.y;

		apf_->initPotentialField(
			field_rows_, field_cols_, field_resolution_, robot_x, robot_y);
		apf_->setParams(weight_attraction_, weight_repulsion_, distance_threshold_repulsion_);
		apf_->setRobot(robot_x, robot_y);
		apf_->setGoal(goal_x, goal_y);

		apf_->clearObstacles();
		for (const auto& obs : obstacles_)
		{
			apf_->setObstacle(obs.x, obs.y);
		}

		std::vector<geometry_msgs::msg::Point> marker_obs;
		toPointVec(interactive_markers_, marker_obs);
		for (const auto& obs : marker_obs)
		{
			apf_->setObstacle(obs.x, obs.y);
		}

		apf_->createPotentialField();

		// ポテンシャル場の可視化
		sensor_msgs::msg::PointCloud2 field_msg;
		potbot_lib::utility::field_to_pcl2(*(apf_->getValues()), field_msg);
		field_msg.header.frame_id = "map";
		field_msg.header.stamp = this->now();
		pub_field_->publish(field_msg);

		// 経路計画
		potbot_lib::path_planner::APFPathPlanner planner(apf_.get());
		planner.setParams(
			max_path_length_,
			static_cast<size_t>(path_search_range_),
			weight_potential_,
			weight_pose_);

		if (planning_method_ == "astar")
			planner.createPathAStar();
		else if (planning_method_ == "dijkstra")
			planner.createPathDijkstra();
		else
			planner.createPath();

		planner.bezier();

		// 経路をパブリッシュ
		std::vector<potbot_lib::Pose> path;
		planner.getPath(path);
		nav_msgs::msg::Path path_msg;
		potbot_lib::utility::to_msg(path, path_msg);
		path_msg.header.frame_id = "map";
		path_msg.header.stamp = this->now();
		pub_path_->publish(path_msg);
	}

public:
	MarkerPathPlanner() : Node("pathplanner_tutorial")
	{
		// パラメータ宣言
		this->declare_parameter("planning_method", planning_method_);
		this->declare_parameter("field_resolution", field_resolution_);
		this->declare_parameter("field_rows", field_rows_);
		this->declare_parameter("field_cols", field_cols_);
		this->declare_parameter("weight_attraction", weight_attraction_);
		this->declare_parameter("weight_repulsion", weight_repulsion_);
		this->declare_parameter("distance_threshold_repulsion", distance_threshold_repulsion_);
		this->declare_parameter("max_path_length", max_path_length_);
		this->declare_parameter("path_search_range", path_search_range_);
		this->declare_parameter("weight_potential", weight_potential_);
		this->declare_parameter("weight_pose", weight_pose_);

		planning_method_ = this->get_parameter("planning_method").as_string();
		field_resolution_ = this->get_parameter("field_resolution").as_double();
		field_rows_ = this->get_parameter("field_rows").as_int();
		field_cols_ = this->get_parameter("field_cols").as_int();
		weight_attraction_ = this->get_parameter("weight_attraction").as_double();
		weight_repulsion_ = this->get_parameter("weight_repulsion").as_double();
		distance_threshold_repulsion_ = this->get_parameter("distance_threshold_repulsion").as_double();
		max_path_length_ = this->get_parameter("max_path_length").as_double();
		path_search_range_ = this->get_parameter("path_search_range").as_int();
		weight_potential_ = this->get_parameter("weight_potential").as_double();
		weight_pose_ = this->get_parameter("weight_pose").as_double();

		param_callback_handle_ = this->add_on_set_parameters_callback(
			std::bind(&MarkerPathPlanner::param_callback, this, std::placeholders::_1));

		// APF ���期化
		apf_ = std::make_unique<potbot_lib::ArtificialPotentialField>(
			field_rows_, field_cols_, field_resolution_,
			weight_attraction_, weight_repulsion_, distance_threshold_repulsion_);

		// サブスクリプション
		sub_inipose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", 1,
			std::bind(&MarkerPathPlanner::inipose_callback, this, std::placeholders::_1));
		sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"goal_pose", 1,
			std::bind(&MarkerPathPlanner::goal_callback, this, std::placeholders::_1));
		sub_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
			"clicked_point", 1,
			std::bind(&MarkerPathPlanner::point_callback, this, std::placeholders::_1));

		// パブリッシャー
		pub_path_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);
		pub_field_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("apf/field/potential", 1);

		// メインループ用タイマー (10Hz)
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100),
			std::bind(&MarkerPathPlanner::timer_callback, this));
	}

	void init()
	{
		init_interactive_marker_server();
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MarkerPathPlanner>();
	node->init();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
