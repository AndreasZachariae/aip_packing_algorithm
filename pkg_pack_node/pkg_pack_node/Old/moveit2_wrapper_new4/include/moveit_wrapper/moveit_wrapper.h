#ifndef MOVEIT_WRAPPER
#define MOVEIT_WRAPPER

#include <geometry_msgs/msg/pose.hpp>
#include <iras_srvs/srv/pose.hpp>
#include "iras_srvs/srv/attach_object.hpp"
#include "iras_srvs/srv/detach_object.hpp"
#include <map>
#include "rclcpp/rclcpp.hpp"
#include <stdlib.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_wrapper");

namespace moveit_wrapper
{
    class MoveitWrapper : public rclcpp::Node
    {
        public:
            MoveitWrapper(const rclcpp::NodeOptions &options);
            ~MoveitWrapper() {};
            void init_move_group();
        private:
            double _epsilon = 0.001;
            std::string _planning_group;
            bool _i_move_group_initialized;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _move_group;
            rclcpp::Service<iras_srvs::srv::Pose>::SharedPtr _move_to_pose;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subscription;
            void move_to_pose(const std::shared_ptr<iras_srvs::srv::Pose::Request> request,
                std::shared_ptr<iras_srvs::srv::Pose::Response> response);
            void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
            rclcpp::callback_group::CallbackGroup::SharedPtr _callback_group_1;
            rclcpp::callback_group::CallbackGroup::SharedPtr _callback_group_2;

            std::vector<double> _current_joint_states;

            rclcpp::Service<iras_srvs::srv::AttachObject>::SharedPtr _attach_object;
            void attach_object(const std::shared_ptr<iras_srvs::srv::AttachObject::Request> request,
                std::shared_ptr<iras_srvs::srv::AttachObject::Response> response);

            rclcpp::Service<iras_srvs::srv::DetachObject>::SharedPtr _detach_object;
            void detach_object(const std::shared_ptr<iras_srvs::srv::DetachObject::Request> request,
                std::shared_ptr<iras_srvs::srv::DetachObject::Response> response);

            rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr _planning_scene_diff_publisher;
            rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr _planning_scene_diff_client;
            moveit_msgs::msg::PlanningScene _planning_scene;
            moveit_msgs::msg::CollisionObject _remove_object;
            std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> attached_object_stored;
    };
}


#endif
