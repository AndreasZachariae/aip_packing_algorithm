#include <moveit_wrapper/moveit_wrapper.h>
#include "rclcpp/rclcpp.hpp"
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <typeinfo>
#include <chrono>
#include <cstdlib>
#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> move_to_a_pose_node = rclcpp::Node::make_shared("move_to_a_pose_node");
  // this file incudes 3 service clients that call services, attach_object, move_to_a_pose and detach_object
 
  // -------------------------------------------------------------------------------------
  // service client to call the service "attach_object" 

  rclcpp::Client<iras_srvs::srv::AttachObject>::SharedPtr client_attach =
  move_to_a_pose_node->create_client<iras_srvs::srv::AttachObject>("attach_object");
  auto request_attach = std::make_shared<iras_srvs::srv::AttachObject::Request>();

  // Define the attached object message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "cylinder_back_right";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "cylinder_back_right";
  /* The id of the object */
  attached_object.object.id = "package1";
  /* A default pose */
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation.
  attached_object.object.operation = attached_object.object.ADD;

  // Since we are attaching the object to the robot hand to simulate picking up the object,
  // we want the collision checker to ignore collisions between the object and the robot hand.
  attached_object.touch_links = std::vector<std::string>{ "tool0"};

  request_attach->attached_object_details = attached_object;

  while (!client_attach->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  auto result_attach = client_attach->async_send_request(request_attach);
  
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(move_to_a_pose_node, result_attach) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status: %ld", result_attach.get()->attach_success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service attach_object");
  }

// --------------------------------------------------------------------
// service client to call the service "move_to_pose" 
  rclcpp::Client<iras_srvs::srv::Pose>::SharedPtr client_move_1 =
    move_to_a_pose_node->create_client<iras_srvs::srv::Pose>("move_to_pose");

  auto request_move_1 = std::make_shared<iras_srvs::srv::Pose::Request>();

  // Set a target Pose
    geometry_msgs::msg::Pose msg_1;
    msg_1.orientation.w = -0.056;
    msg_1.orientation.x = -0.38;
    msg_1.orientation.y = 0.65;
    msg_1.orientation.z = -0.64;
    msg_1.position.x = 1.5113;
    msg_1.position.y = -2.2602;
    msg_1.position.z = 2.1683;

    // msg.orientation.w = 0.0803554;
    // msg.orientation.x = 0.374312;
    // msg.orientation.y = 0.900488;
    // msg.orientation.z = 0.206286;
    // msg.position.x = -0.648971;
    // msg.position.y = 0.726718;
    // msg.position.z = -0.183858;
  request_move_1->pose = msg_1;
  request_move_1->cart = false;

  while (!client_move_1->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  auto result_move_1 = client_move_1->async_send_request(request_move_1);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(move_to_a_pose_node, result_move_1) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status: %ld", result_move_1.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service move_to_pose");
  }

// ----------------------------------------------------------------------
// service client to call the service "detach_object" 

  rclcpp::Client<iras_srvs::srv::DetachObject>::SharedPtr client_detach =
    move_to_a_pose_node->create_client<iras_srvs::srv::DetachObject>("detach_object");
  auto request_detach = std::make_shared<iras_srvs::srv::DetachObject::Request>();

  /* First, define the DETACH object message*/
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "tool0";
  request_detach->detached_object_details = detach_object;

  while (!client_detach->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result_detach = client_detach->async_send_request(request_detach);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(move_to_a_pose_node, result_detach) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status: %ld", result_detach.get()->detach_success);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service detach_object");
    }


  rclcpp::shutdown();
  return 0;
}

