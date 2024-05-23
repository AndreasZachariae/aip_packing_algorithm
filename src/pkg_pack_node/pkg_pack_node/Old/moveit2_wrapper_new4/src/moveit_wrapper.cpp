#include <moveit_wrapper/moveit_wrapper.h>

using std::placeholders::_1;
using std::placeholders::_2;



namespace moveit_wrapper
{
    MoveitWrapper::MoveitWrapper(const rclcpp::NodeOptions &options) : Node("moveit_wrapper", options)
    {
        _i_move_group_initialized = false;
        _callback_group_1 = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        _callback_group_2 = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);


//        this->declare_parameter("planning_group", "manipulator");
        this->get_parameter("planning_group", _planning_group);
        _move_to_pose = this->create_service<iras_srvs::srv::Pose>("move_to_pose", std::bind(&MoveitWrapper::move_to_pose, this, _1, _2), rmw_qos_profile_services_default,
                                                                             _callback_group_1);
        _attach_object = this->create_service<iras_srvs::srv::AttachObject>("attach_object", std::bind(&MoveitWrapper::attach_object, this, _1, _2), rmw_qos_profile_services_default,
                                                                             _callback_group_1);
        _detach_object = this->create_service<iras_srvs::srv::DetachObject>("detach_object", std::bind(&MoveitWrapper::detach_object, this, _1, _2), rmw_qos_profile_services_default,
                                                                             _callback_group_1);
        
        _planning_scene_diff_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
        rclcpp::Rate loop_rate(1);

        _planning_scene_diff_client = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
        loop_rate.sleep();
        rclcpp::SubscriptionOptions s_options;
        s_options.callback_group = _callback_group_2;
        _subscription = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MoveitWrapper::joint_state_callback, this, _1),
                                                                   s_options);

        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Initialized.");
    }

    void MoveitWrapper::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if(_i_move_group_initialized) {
            _current_joint_states = msg->position;
        }
        // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f current joint positions.", 
        //     msg->position[0], 
        //     msg->position[1], 
        //     msg->position[2], 
        //     msg->position[3], 
        //     msg->position[4], 
        //     msg->position[5]);
    }


    void MoveitWrapper::init_move_group()
    {
        static const std::string PLANNING_GROUP = "manipulator";
        _move_group.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), PLANNING_GROUP));
        _move_group->setPlannerId("BFMT");


        _i_move_group_initialized = true;
        // TODO initialize number of joints properly
        _current_joint_states.resize(6);
        rclcpp::Rate loop_rate(1000);
        loop_rate.sleep();
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Ready to receive commands.");
    }

    void MoveitWrapper::move_to_pose(const std::shared_ptr<iras_srvs::srv::Pose::Request> request,
                std::shared_ptr<iras_srvs::srv::Pose::Response> response)
    {
        std::cout << "message received" << '\n';
        bool success = false;
        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();
            if(!request->cart) {
                _move_group->setPoseTarget(request->pose);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                success = (_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if(success) {
                    _move_group->move();
                }
            } else {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(request->pose);

                moveit_msgs::msg::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;
                const double eef_step = 0.001;
                double fraction = _move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

                if(fraction > 0.0) {
                    success = true;
                    _move_group->execute(trajectory);

                }
                int len_n = trajectory.joint_trajectory.joint_names.size();
                int len_j = trajectory.joint_trajectory.points.size();
                int len_p = trajectory.joint_trajectory.points[0].positions.size();
                // trajectory_msgs::msg::JointTrajectoryPoint jtp = trajectory.joint_trajectory.points[len_j-1];

                
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "n names %d", len_n);
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "n points %d", len_j);
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "n joints? %d", len_p);


                // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %s %s %s %s %s %s %s %s joint names.", 
                //     trajectory.joint_trajectory.joint_names[0].c_str(),
                //     trajectory.joint_trajectory.joint_names[1].c_str(),
                //     trajectory.joint_trajectory.joint_names[2].c_str(),
                //     trajectory.joint_trajectory.joint_names[3].c_str(),
                //     trajectory.joint_trajectory.joint_names[4].c_str(),
                //     trajectory.joint_trajectory.joint_names[5].c_str(),
                //     trajectory.joint_trajectory.joint_names[6].c_str(),
                //     trajectory.joint_trajectory.joint_names[7].c_str()
                //     );

                // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f joint trajectory.", 
                //     trajectory.joint_trajectory.points[0].positions[0], 
                //     trajectory.joint_trajectory.points[1].positions[0], 
                //     trajectory.joint_trajectory.points[2].positions[0], 
                //     trajectory.joint_trajectory.points[3].positions[0], 
                //     trajectory.joint_trajectory.points[4].positions[0], 
                //     trajectory.joint_trajectory.points[5].positions[0]);

                
                // std::vector<double> diff;
                // diff.resize(_current_joint_states.size());

                // bool target_reached = false;
                
                // while(! target_reached){
                //     target_reached = true;
                //     for(long unsigned int i = 0; i < _current_joint_states.size(); i++) {
                //         double joint_diff = trajectory.joint_trajectory.points[len_j-1].positions[i] - _current_joint_states[i];
                //         joint_diff = std::abs(joint_diff);
                //         target_reached = target_reached && (joint_diff < _epsilon);
                //         // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "%d, %f, %d", i, joint_diff, int(target_reached));
                //         diff[i] = joint_diff;
                //     }
                // }
                
                

                // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f joint trajectory.", 
                //     trajectory.joint_trajectory.points[len_j-1].positions[0], 
                //     trajectory.joint_trajectory.points[len_j-1].positions[1], 
                //     trajectory.joint_trajectory.points[len_j-1].positions[2], 
                //     trajectory.joint_trajectory.points[len_j-1].positions[3], 
                //     trajectory.joint_trajectory.points[len_j-1].positions[4], 
                //     trajectory.joint_trajectory.points[len_j-1].positions[5]);

                // // std::vector<double> c_j = _move_group->getCurrentJointValues();


                // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f current joint values.", 
                //     _current_joint_states[0], 
                //     _current_joint_states[1], 
                //     _current_joint_states[2], 
                //     _current_joint_states[3], 
                //     _current_joint_states[4], 
                //     _current_joint_states[5]);


                // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f current joint DIFF.", 
                //     diff[0], 
                //     diff[1], 
                //     diff[2], 
                //     diff[3], 
                //     diff[4], 
                //     diff[5]);

            }
        }
        response->success = success;
    }

    // for more detailed info on attach, detach operations and planning scene please look at the link below
    // https://moveit.picknik.ai/foxy/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html 
    void MoveitWrapper::attach_object(const std::shared_ptr<iras_srvs::srv::AttachObject::Request> request,
                std::shared_ptr<iras_srvs::srv::AttachObject::Response> response)
    {
        bool success = false;
        attached_object_stored[request->attached_object_details.object.id] = request->attached_object_details;
        // // Adding an object into the environment
        RCLCPP_INFO(LOGGER, "Adding the object into the world at the location of the hand.");
        _planning_scene.world.collision_objects.push_back(request->attached_object_details.object);
        _planning_scene.is_diff = true;
        _planning_scene_diff_publisher->publish(_planning_scene);
        
        _planning_scene_diff_client->wait_for_service();
        // and send the diffs to the planning scene via a service call:
        auto request_planning_scene = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request_planning_scene->scene = _planning_scene;
        std::shared_future<std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>> response_future;
        response_future = _planning_scene_diff_client->async_send_request(request_planning_scene);
        
        std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response> planning_response;
        planning_response = response_future.get();
        if (planning_response->success)
        {
          RCLCPP_INFO(LOGGER, "Service successfully added object.");
          success = true;
        }
        else
        {
          RCLCPP_ERROR(LOGGER, "Service failed to add object.");
        }

        /* First, define the REMOVE object message*/
        _remove_object.id = request->attached_object_details.object.id;
        _remove_object.header.frame_id = request->attached_object_details.object.header.frame_id;
        _remove_object.operation = _remove_object.REMOVE;

        // /* Carry out the REMOVE + ATTACH operation */
        RCLCPP_INFO(LOGGER, "Attaching the object to the hand and removing it from the world.");
        _planning_scene.world.collision_objects.clear();
        _planning_scene.world.collision_objects.push_back(_remove_object);
        _planning_scene.robot_state.attached_collision_objects.push_back(request->attached_object_details);
        _planning_scene.robot_state.is_diff = true;
        _planning_scene_diff_publisher->publish(_planning_scene);

        response->attach_success = success;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response...");
    }

    void MoveitWrapper::detach_object(const std::shared_ptr<iras_srvs::srv::DetachObject::Request> request,
                std::shared_ptr<iras_srvs::srv::DetachObject::Response> response)
    {
        bool success = false;

        request->detached_object_details.object.operation = attached_object_stored[request->detached_object_details.object.id].object.REMOVE;

        /* Carry out the DETACH + ADD operation */
        RCLCPP_INFO(LOGGER, "Detaching the object from the robot and returning it to the world.");

        _planning_scene.robot_state.attached_collision_objects.clear();
        _planning_scene.robot_state.attached_collision_objects.push_back(request->detached_object_details);
        _planning_scene.robot_state.is_diff = true;
        _planning_scene.world.collision_objects.clear();
        _planning_scene.world.collision_objects.push_back(attached_object_stored[request->detached_object_details.object.id].object);
        _planning_scene.is_diff = true;
        _planning_scene_diff_publisher->publish(_planning_scene);

        /* First, define the REMOVE object message*/
        _remove_object.id = attached_object_stored[request->detached_object_details.object.id].object.id;
        _remove_object.header.frame_id = attached_object_stored[request->detached_object_details.object.id].object.header.frame_id;
        _remove_object.operation = _remove_object.REMOVE;
        attached_object_stored.erase(request->detached_object_details.object.id);

        RCLCPP_INFO(LOGGER, "Removing the object from the world.");
        _planning_scene.robot_state.attached_collision_objects.clear();
        _planning_scene.world.collision_objects.clear();
        _planning_scene.world.collision_objects.push_back(_remove_object);
        _planning_scene_diff_publisher->publish(_planning_scene);
        success = true;
        response->detach_success = success;

    }
}





//_________________________________________________________________________________________________________________________________


// #include <moveit_wrapper/moveit_wrapper.h>

// using std::placeholders::_1;
// using std::placeholders::_2;



// namespace moveit_wrapper
// {
//     MoveitWrapper::MoveitWrapper(const rclcpp::NodeOptions &options) : Node("moveit_wrapper", options)
//     {
//         _i_move_group_initialized = false;
//         _callback_group_1 = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
//         _callback_group_2 = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);


// //        this->declare_parameter("planning_group", "manipulator");
//         this->get_parameter("planning_group", _planning_group);
//         _move_to_pose = this->create_service<iras_srvs::srv::Pose>("move_to_pose", std::bind(&MoveitWrapper::move_to_pose, this, _1, _2), rmw_qos_profile_services_default,
//                                                                              _callback_group_1);
//         _attach_object = this->create_service<iras_srvs::srv::AttachObject>("attach_object", std::bind(&MoveitWrapper::attach_object, this, _1, _2), rmw_qos_profile_services_default,
//                                                                              _callback_group_1);
//         _detach_object = this->create_service<iras_srvs::srv::DetachObject>("detach_object", std::bind(&MoveitWrapper::detach_object, this, _1, _2), rmw_qos_profile_services_default,
//                                                                              _callback_group_1);
        
//         _planning_scene_diff_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
//         rclcpp::Rate loop_rate(1);

//         _planning_scene_diff_client = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
//         loop_rate.sleep();
//         rclcpp::SubscriptionOptions s_options;
//         s_options.callback_group = _callback_group_2;
//         _subscription = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MoveitWrapper::joint_state_callback, this, _1),
//                                                                    s_options);

//         RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Initialized.");
//     }

//     void MoveitWrapper::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
//     {
//         if(_i_move_group_initialized) {
//             _current_joint_states = msg;
//         }
//         // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f current joint positions.", 
//         //     msg->position[0], 
//         //     msg->position[1], 
//         //     msg->position[2], 
//         //     msg->position[3], 
//         //     msg->position[4], 
//         //     msg->position[5]);
//     }


//     void MoveitWrapper::init_move_group()
//     {
//         static const std::string PLANNING_GROUP = "manipulator";
//         _move_group.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), PLANNING_GROUP));
//         _move_group->setPlannerId("BFMT");

//         _i_move_group_initialized = true;
//         rclcpp::Rate loop_rate(1000);
//         loop_rate.sleep();
//         RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Ready to receive commands.");
//     }

//     void MoveitWrapper::move_to_pose(const std::shared_ptr<iras_srvs::srv::Pose::Request> request,
//                 std::shared_ptr<iras_srvs::srv::Pose::Response> response)
//     {
//         std::cout << "message received" << '\n';
//         bool success = false;
//         if(_i_move_group_initialized)
//         {
//             _move_group->stop();
//             _move_group->clearPoseTargets();
//             if(!request->cart) {
//                 _move_group->setPoseTarget(request->pose);
//                 moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//                 success = (_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//                 if(success) {
//                     _move_group->move();
//                 }
//             } else {
//                 std::vector<geometry_msgs::msg::Pose> waypoints;
//                 waypoints.push_back(request->pose);

//                 moveit_msgs::msg::RobotTrajectory trajectory;
//                 const double jump_threshold = 0.0;
//                 const double eef_step = 0.001;
//                 double fraction = _move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

//                 if(fraction > 0.0) {
//                     success = true;
//                     _move_group->execute(trajectory);

//                 }
//                 int len_n = trajectory.joint_trajectory.joint_names.size();
//                 int len_j = trajectory.joint_trajectory.points.size();
//                 int len_p = trajectory.joint_trajectory.points[0].positions.size();
//                 // trajectory_msgs::msg::JointTrajectoryPoint jtp = trajectory.joint_trajectory.points[len_j-1];

                
//                 RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "n names %d", len_n);
//                 RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "n points %d", len_j);
//                 RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "n joints? %d", len_p);


//                 // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %s %s %s %s %s %s %s %s joint names.", 
//                 //     trajectory.joint_trajectory.joint_names[0].c_str(),
//                 //     trajectory.joint_trajectory.joint_names[1].c_str(),
//                 //     trajectory.joint_trajectory.joint_names[2].c_str(),
//                 //     trajectory.joint_trajectory.joint_names[3].c_str(),
//                 //     trajectory.joint_trajectory.joint_names[4].c_str(),
//                 //     trajectory.joint_trajectory.joint_names[5].c_str(),
//                 //     trajectory.joint_trajectory.joint_names[6].c_str(),
//                 //     trajectory.joint_trajectory.joint_names[7].c_str()
//                 //     );

//                 // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f joint trajectory.", 
//                 //     trajectory.joint_trajectory.points[0].positions[0], 
//                 //     trajectory.joint_trajectory.points[1].positions[0], 
//                 //     trajectory.joint_trajectory.points[2].positions[0], 
//                 //     trajectory.joint_trajectory.points[3].positions[0], 
//                 //     trajectory.joint_trajectory.points[4].positions[0], 
//                 //     trajectory.joint_trajectory.points[5].positions[0]);


//                 std::vector<double> diff;
//                 diff.resize(_current_joint_states->position.size());

//                 bool target_reached = false;
                
//                 while (!target_reached)
//                 {
//                     target_reached = true;
//                     for(size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i)
//                     {   
//                         auto itr = std::find_if(_current_joint_states->name.begin(), _current_joint_states->name.end(), [&]( auto & name){ return name == trajectory.joint_trajectory.joint_names[i];});
//                         if (itr != _current_joint_states->name.end())
//                         {
//                           auto j = std::distance(_current_joint_states->name.begin(), itr);
//                           double joint_diff = trajectory.joint_trajectory.points[len_j - 1].positions[i] - _current_joint_states->position[j];
//                           joint_diff = std::abs(joint_diff);
//                           target_reached = target_reached && (joint_diff < _epsilon);
//                           // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "%d, %f, %d", i, joint_diff, int(target_reached));
//                           diff[i] = joint_diff;
//                         }
//                         else
//                         {
//                           RCLCPP_WARN(rclcpp::get_logger("moveit_wrapper"), "Joint name '%s' not found in the JointState message. This should not happen, at least not repeatedly. If so, check your programming logic.", trajectory.joint_trajectory.joint_names[i].c_str());
//                           target_reached = false;
//                           break;
//                         } 
//                     }
//                 }

//                 // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f joint trajectory.", 
//                 //     trajectory.joint_trajectory.points[len_j-1].positions[0], 
//                 //     trajectory.joint_trajectory.points[len_j-1].positions[1], 
//                 //     trajectory.joint_trajectory.points[len_j-1].positions[2], 
//                 //     trajectory.joint_trajectory.points[len_j-1].positions[3], 
//                 //     trajectory.joint_trajectory.points[len_j-1].positions[4], 
//                 //     trajectory.joint_trajectory.points[len_j-1].positions[5]);

//                 // // std::vector<double> c_j = _move_group->getCurrentJointValues();


//                 // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f current joint values.", 
//                 //     _current_joint_states[0], 
//                 //     _current_joint_states[1], 
//                 //     _current_joint_states[2], 
//                 //     _current_joint_states[3], 
//                 //     _current_joint_states[4], 
//                 //     _current_joint_states[5]);


//                 // RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "---------------- %f %f %f %f %f %f current joint DIFF.", 
//                 //     diff[0], 
//                 //     diff[1], 
//                 //     diff[2], 
//                 //     diff[3], 
//                 //     diff[4], 
//                 //     diff[5]);

//             }
//         }
//         response->success = success;
//     }

//     // for more detailed info on attach, detach operations and planning scene please look at the link below
//     // https://moveit.picknik.ai/foxy/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html 
//     void MoveitWrapper::attach_object(const std::shared_ptr<iras_srvs::srv::AttachObject::Request> request,
//                 std::shared_ptr<iras_srvs::srv::AttachObject::Response> response)
//     {
//         bool success = false;
//         attached_object_stored[request->attached_object_details.object.id] = request->attached_object_details;
//         // // Adding an object into the environment
//         RCLCPP_INFO(LOGGER, "Adding the object into the world at the location of the hand.");
//         _planning_scene.world.collision_objects.push_back(request->attached_object_details.object);
//         _planning_scene.is_diff = true;
//         _planning_scene_diff_publisher->publish(_planning_scene);
        
//         _planning_scene_diff_client->wait_for_service();
//         // and send the diffs to the planning scene via a service call:
//         auto request_planning_scene = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
//         request_planning_scene->scene = _planning_scene;
//         std::shared_future<std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>> response_future;
//         response_future = _planning_scene_diff_client->async_send_request(request_planning_scene);
        
//         std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response> planning_response;
//         planning_response = response_future.get();
//         if (planning_response->success)
//         {
//           RCLCPP_INFO(LOGGER, "Service successfully added object.");
//           success = true;
//         }
//         else
//         {
//           RCLCPP_ERROR(LOGGER, "Service failed to add object.");
//         }

//         /* First, define the REMOVE object message*/
//         _remove_object.id = request->attached_object_details.object.id;
//         _remove_object.header.frame_id = request->attached_object_details.object.header.frame_id;
//         _remove_object.operation = _remove_object.REMOVE;

//         // /* Carry out the REMOVE + ATTACH operation */
//         RCLCPP_INFO(LOGGER, "Attaching the object to the hand and removing it from the world.");
//         _planning_scene.world.collision_objects.clear();
//         _planning_scene.world.collision_objects.push_back(_remove_object);
//         _planning_scene.robot_state.attached_collision_objects.push_back(request->attached_object_details);
//         _planning_scene.robot_state.is_diff = true;
//         _planning_scene_diff_publisher->publish(_planning_scene);

//         response->attach_success = success;
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response...");
//     }

//     void MoveitWrapper::detach_object(const std::shared_ptr<iras_srvs::srv::DetachObject::Request> request,
//                 std::shared_ptr<iras_srvs::srv::DetachObject::Response> response)
//     {
//         bool success = false;

//         request->detached_object_details.object.operation = attached_object_stored[request->detached_object_details.object.id].object.REMOVE;

//         /* Carry out the DETACH + ADD operation */
//         RCLCPP_INFO(LOGGER, "Detaching the object from the robot and returning it to the world.");

//         _planning_scene.robot_state.attached_collision_objects.clear();
//         _planning_scene.robot_state.attached_collision_objects.push_back(request->detached_object_details);
//         _planning_scene.robot_state.is_diff = true;
//         _planning_scene.world.collision_objects.clear();
//         _planning_scene.world.collision_objects.push_back(attached_object_stored[request->detached_object_details.object.id].object);
//         _planning_scene.is_diff = true;
//         _planning_scene_diff_publisher->publish(_planning_scene);

//         /* First, define the REMOVE object message*/
//         _remove_object.id = attached_object_stored[request->detached_object_details.object.id].object.id;
//         _remove_object.header.frame_id = attached_object_stored[request->detached_object_details.object.id].object.header.frame_id;
//         _remove_object.operation = _remove_object.REMOVE;
//         attached_object_stored.erase(request->detached_object_details.object.id);

//         RCLCPP_INFO(LOGGER, "Removing the object from the world.");
//         _planning_scene.robot_state.attached_collision_objects.clear();
//         _planning_scene.world.collision_objects.clear();
//         _planning_scene.world.collision_objects.push_back(_remove_object);
//         _planning_scene_diff_publisher->publish(_planning_scene);
//         success = true;
//         response->detach_success = success;

//     }
// }

