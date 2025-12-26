#include "ahpc_planning_node.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ahpc_planning");

static const std::string PLANNING_GROUP = "jaka_arm";
static const double PLANNING_TIME_S = 30.0;
static const double PLANNING_ATTEMPTS = 5.0;

robot_driver_status::robot_driver_status()
{
	this->ts_.sec = 0;
	this->ts_.nanosec = 0;
	this->connect_status_ = false;                 /* 连接状态 */
	this->enable_status_ = false;                 /* 使能状态 */
	this->is_robot_stop_ = true;                  /* 机器人是否停止 */
	this->in_collision_ = false;                   /* 处在碰撞状态 */
	this->in_emergency_ = false;                   /* 处在急停状态 */
	this->is_free_drive_ = false;                  /* 是否处在自由模式 */
	this->errcode_ = 0;                         /* 错误码 */
	this->collision_level_ = 0;                 /* 碰撞等级 */
	this->tcp_pose_.clear();        /* 工具TCP姿态 */
	this->joint_pose_.clear();      /* 关节姿态 */
	this->tool_id_ = 0;                         /* 工具编号 */
	this->tool_tcp_.clear();        /* 工具TCP */
	this->mass_ = 0;                         /* 负载重量 */
	this->center_of_mass_.clear();  /* 负载重心 */
	this->force_control_enable_ = false;           /* 是否处在力控模式 */
	this->torque_adjust_enable_ = false; /* 力控调整 */
	this->torque_.clear();
	this->act_torque_.clear();
	this->torque_extract_adjust_enable_ = false; /* 力控调整 */
	this->admit_extract_enable_ = false;
	this->admit_extract_seq_ = 0;
	return;
}
AHPCPlanning::AHPCPlanning(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("ahpc_planning_node", options) }
{
	single_task_update_status_ = false;
	loc_type_ = 0;
	robot_driver_status_sub_ = node_->create_subscription<shared_message::msg::RobotDriverStatus>(
      "/jaka_zu5_driver/robot_states", 10, std::bind(&AHPCPlanning::robotDriverStatusHandle, this, std::placeholders::_1));
	move_to_location_pos_srv_ = node_->create_service<shared_message::srv::MoveToLocationPosition>("move_to_location_position", std::bind(&AHPCPlanning::moveToLocationPosition, this, std::placeholders::_1,std::placeholders::_2));
    p_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    p_move_group_interface_->setPlanningTime(PLANNING_TIME_S);
    p_move_group_interface_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr AHPCPlanning::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}
AHPCPlanning::~AHPCPlanning()
{
    
}
void AHPCPlanning::moveToLocationPosition(const std::shared_ptr<shared_message::srv::MoveToLocationPosition::Request> request,
                                          std::shared_ptr<shared_message::srv::MoveToLocationPosition::Response> response)
{
	RCLCPP_INFO(LOGGER,"input loc_type = %d",request->loc_type);
	if(request->pose.size() != 6)
		return;
	RCLCPP_INFO(LOGGER,"input pose: x = %f,y = %f, z =%f,yaw = %f,pitch =%f,roll =%f",request->pose[0],request->pose[1],request->pose[2]
		        ,request->pose[3],request->pose[4],request->pose[5]);
	loc_type_ = request->loc_type;
	vec_loc_pose_.clear();
	for(int i=0;i<6;i++)
	{
		vec_loc_pose_.push_back(request->pose[i]);
	}
	single_task_update_status_ = true;
	response->result = true;
}
void AHPCPlanning::robotDriverStatusHandle(const shared_message::msg::RobotDriverStatus::ConstSharedPtr driver_status)
{
    this->driver_status_.ts_ = driver_status->header.stamp;
	this->driver_status_.connect_status_ = driver_status->connect_status;
	this->driver_status_.enable_status_ = driver_status->enable;
	this->driver_status_.is_robot_stop_ = driver_status->is_robot_stop;
	this->driver_status_.is_free_drive_ = driver_status->free_drive;
	this->driver_status_.force_control_enable_ = driver_status->force_mode;
	this->driver_status_.in_collision_ = driver_status->collision;
	this->driver_status_.in_emergency_ = driver_status->emergency;
	this->driver_status_.torque_adjust_enable_ = driver_status->force_freedrive;
	this->driver_status_.torque_extract_adjust_enable_ =
	    driver_status->torque_extract_adjust;
	this->driver_status_.admit_extract_enable_ =
	    driver_status->admit_extract;
	this->driver_status_.admit_extract_success_ =
	    driver_status->admit_extract_success;
	this->driver_status_.in_pos_ = driver_status->in_pos;

	this->driver_status_.tcp_pose_.resize(6);
	this->driver_status_.tcp_pose_.at(0) = driver_status->tcp_pose.x;
	this->driver_status_.tcp_pose_.at(1) = driver_status->tcp_pose.y;
	this->driver_status_.tcp_pose_.at(2) = driver_status->tcp_pose.z;
	this->driver_status_.tcp_pose_.at(3) = driver_status->tcp_pose.yaw;
	this->driver_status_.tcp_pose_.at(4) = driver_status->tcp_pose.pitch;
	this->driver_status_.tcp_pose_.at(5) = driver_status->tcp_pose.roll;

	this->driver_status_.joint_base_position_.resize(
	    driver_status->joint_base_position.size());

	for (size_t i = 0; i < this->driver_status_.joint_base_position_.size(); i++)
	{
		this->driver_status_.joint_base_position_.at(i).resize(3);
		this->driver_status_.joint_base_position_.at(i).at(0) =
		    driver_status->joint_base_position.at(i).x;
		this->driver_status_.joint_base_position_.at(i).at(1) =
		    driver_status->joint_base_position.at(i).y;
		this->driver_status_.joint_base_position_.at(i).at(2) =
		    driver_status->joint_base_position.at(i).z;
	}

	this->driver_status_.joint_pose_ = driver_status->joint_pose.pose;

	this->driver_status_.tool_id_ = driver_status->tool_id;
	this->driver_status_.tool_tcp_.resize(6);
	this->driver_status_.tool_tcp_.at(0) = driver_status->tool_tcp.x;
	this->driver_status_.tool_tcp_.at(1) = driver_status->tool_tcp.y;
	this->driver_status_.tool_tcp_.at(2) = driver_status->tool_tcp.z;
	this->driver_status_.tool_tcp_.at(3) = driver_status->tool_tcp.yaw;
	this->driver_status_.tool_tcp_.at(4) = driver_status->tool_tcp.pitch;
	this->driver_status_.tool_tcp_.at(5) = driver_status->tool_tcp.roll;

	this->driver_status_.mass_ = driver_status->mass;
	this->driver_status_.center_of_mass_.resize(3);
	this->driver_status_.center_of_mass_.at(0) = driver_status->cog.x;
	this->driver_status_.center_of_mass_.at(1) = driver_status->cog.y;
	this->driver_status_.center_of_mass_.at(2) = driver_status->cog.z;
	this->driver_status_.errcode_ = driver_status->errcode;
	this->driver_status_.collision_level_ = driver_status->collision_level;

	this->driver_status_.force_adjust_seq_ = driver_status->force_adjust_seq;
	this->driver_status_.force_control_seq_ = driver_status->force_control_seq;
	this->driver_status_.force_extract_adjust_seq_ =
	    driver_status->force_extract_adjust_seq;
	this->driver_status_.admit_extract_seq_ = driver_status->admit_extract_seq;

	this->driver_status_.torque_.resize(driver_status->torque.size());

	for (size_t i = 0; i < this->driver_status_.torque_.size(); i++)
	{
		this->driver_status_.torque_.at(i) = driver_status->torque.at(i);
	}

	this->driver_status_.act_torque_.resize(driver_status->act_torque.size());

	for (size_t i = 0; i < this->driver_status_.act_torque_.size(); i++)
	{
		this->driver_status_.act_torque_.at(i) = driver_status->act_torque.at(i);
	}

	this->driver_status_.j_torque_.resize(driver_status->j_torque.size());

	for (size_t i = 0; i < this->driver_status_.j_torque_.size(); i++)
	{
		this->driver_status_.j_torque_.at(i) = driver_status->j_torque.at(i);
	}

	this->driver_status_.j_current_.resize(driver_status->j_current.size());

	for (size_t i = 0; i < this->driver_status_.j_current_.size(); i++)
	{
		this->driver_status_.j_current_.at(i) = driver_status->j_current.at(i);
	}

	static uint32_t print_data_cnt = 0;

	if (1)
	{
		if (0 == (print_data_cnt % 500))
		{
        std::cout<< " act_torque " << vec2str(this->driver_status_.act_torque_) <<
        " raw_torque " << vec2str(this->driver_status_.torque_) <<
        " tcp " << vec2str(this->driver_status_.tcp_pose_) <<
        " joint " << vec2str(this->driver_status_.joint_pose_) <<
        " jtorque " << vec2str(this->driver_status_.j_torque_) <<
        " jcurrent " << vec2str(this->driver_status_.j_current_)<<std::endl;
		}
	}

	print_data_cnt ++;

	return;
}
void AHPCPlanning::baseToTcp(geometry_msgs::msg::Pose& cur_pose,std::vector<float> delta_pose,geometry_msgs::msg::Pose& tar_pose)
{
	//另外一种表达方式
	tf2::Quaternion q;
	q.setRPY(vec_loc_pose_[3], vec_loc_pose_[4], vec_loc_pose_[5]);
    RCLCPP_INFO(LOGGER,"q : %f,%f,%f,%f",q.x(),q.y(),q.z(),q.w());
	q = q.normalize();
	tf2::Transform T_current, T_delta;
	tf2::fromMsg(cur_pose, T_current);

	T_delta.setOrigin(tf2::Vector3(vec_loc_pose_[0] ,vec_loc_pose_[1], vec_loc_pose_[2]));
	T_delta.setRotation(q);
	
	// 计算目标位姿（base_link 下）
	tf2::Transform T_target = T_current * T_delta;
	
	tf2::toMsg(T_target, tar_pose);
}
void AHPCPlanning::doTask()
{
	//RCLCPP_INFO(LOGGER,"Hello ahpc!");
    if(single_task_update_status_)
	{
		single_task_update_status_ = false;
		const moveit::core::JointModelGroup* p_joint_model_group = p_move_group_interface_->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
		// Now, we call the planner to compute the plan and visualize it.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		// When done with the path constraint be sure to clear it.
		p_move_group_interface_->clearPoseTargets();
		p_move_group_interface_->clearPathConstraints();
		moveit::core::RobotStatePtr p_cur_state = p_move_group_interface_->getCurrentState(10);
		if(!p_cur_state)
		{
			return;
		}
		//参考坐标系
		std::string frame = p_move_group_interface_->getPlanningFrame();
		RCLCPP_INFO(LOGGER,"reference frame: %s", frame.c_str());
		//末端Link
		RCLCPP_INFO(LOGGER, "End effector link: %s",p_move_group_interface_->getEndEffectorLink().c_str());
		std::vector<std::string> vec_link_names = p_move_group_interface_->getLinkNames();
		for(size_t i=0;i<vec_link_names.size();i++)
		{
			RCLCPP_INFO(LOGGER, "move group link %ld: %s",i,vec_link_names[i].c_str());
		}
		const std::vector<std::string>& vec_joints_names = p_move_group_interface_->getActiveJoints();
		// 当前关节值
		std::vector<double> joint_group_positions;
		p_cur_state->copyJointGroupPositions(p_joint_model_group, joint_group_positions);
		int i=0;
		for (auto joint_name : vec_joints_names)
		{
			RCLCPP_INFO(LOGGER, "IK joint: %s  : %f", joint_name.c_str(),joint_group_positions[i]);
			i++;
		}
		if(loc_type_ == 0)// Cartesian Paths 
		{
			p_move_group_interface_->setStartStateToCurrentState();//设置当前状态为开始位置（表示1）
			// Cartesian Paths
			geometry_msgs::msg::Pose start_pose = p_move_group_interface_->getCurrentPose().pose;
			std::vector<geometry_msgs::msg::Pose> waypoints;
			waypoints.push_back(start_pose); //?

			geometry_msgs::msg::Pose target_pose2 = start_pose;
			baseToTcp(start_pose,vec_loc_pose_,target_pose2);
			waypoints.push_back(target_pose2);
			
			// We want the Cartesian path to be interpolated at a resolution of 1 cm
			// which is why we will specify 0.01 as the max step in Cartesian
			// translation.  We will specify the jump threshold as 0.0, effectively disabling it.
			// Warning - disabling the jump threshold while operating real hardware can cause
			// large unpredictable motions of redundant joints and could be a safety issue
			moveit_msgs::msg::RobotTrajectory trajectory;
			const double jump_threshold = 0.0;
			const double eef_step = 0.008;
			double fraction = p_move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			RCLCPP_INFO(LOGGER, "plan 0 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
			if(fraction < 1.0)
			{
				return;
			}
			// You can execute a trajectory like this.
			p_move_group_interface_->execute(trajectory);
			
		}
		else if(loc_type_ == 1) //send pose goal
		{

			//******************基于base_link坐标系的规划****************** */
			geometry_msgs::msg::Pose current_pose = p_move_group_interface_->getCurrentPose().pose;
			geometry_msgs::msg::Pose target_pose;
            baseToTcp(current_pose,vec_loc_pose_,target_pose);
			//******************************end********************************************** */
			//先用 IK 求 joint target，再用 RRT 规划
			p_move_group_interface_->setStartStateToCurrentState(); //设置当前状态为开始位置（表示2）
		    bool found_ik = p_cur_state->setFromIK(p_joint_model_group,target_pose,0.1);

			if (found_ik)
			{
				RCLCPP_INFO(LOGGER, "IK solution found");
				std::vector<double> joint_values;
				p_cur_state->copyJointGroupPositions(
					p_joint_model_group, joint_values);

				const auto& joint_names = p_joint_model_group->getVariableNames();

				for (size_t i = 0; i < joint_values.size(); ++i)
				{
					RCLCPP_INFO(LOGGER,
					"%s = %f",
					joint_names[i].c_str(),
					joint_values[i]);
				}
			}
			else
			{
				RCLCPP_ERROR(LOGGER, "No IK solution for target pose");
				return;
			}
			p_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
			// //RRT 对 精确姿态目标 非常不友好。
			// p_move_group_interface_->setGoalPositionTolerance(0.01);
			// p_move_group_interface_->setGoalOrientationTolerance(0.15);


			bool success = (p_move_group_interface_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

			RCLCPP_INFO(LOGGER, "Plan 1 (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
			if(!success)
			{
				return;
			}
			// Moving to a pose goal
			p_move_group_interface_->move();
		}
		else if(loc_type_ == 2) //space joint goal
		{
			// Planning to a joint-space goal
			// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
			joint_group_positions[0] = vec_loc_pose_[0];  // radians
			joint_group_positions[1] = vec_loc_pose_[1];  // radians
			joint_group_positions[2] = vec_loc_pose_[2];  // radians
			joint_group_positions[3] = vec_loc_pose_[3];  // radians
			joint_group_positions[4] = vec_loc_pose_[4];  // radians
			joint_group_positions[5] = vec_loc_pose_[5];  // radians

			p_move_group_interface_->setStartStateToCurrentState();
			p_move_group_interface_->setJointValueTarget(joint_group_positions);
			
			p_move_group_interface_->setMaxVelocityScalingFactor(0.05);
			p_move_group_interface_->setMaxAccelerationScalingFactor(0.05);

			bool success = (p_move_group_interface_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			RCLCPP_INFO(LOGGER, "Plan 2 (joint space goal) %s", success ? "SUCCEEDED" : "FAILED");
			if(!success)
			{
				return;
			}
			// Moving to a joint-space goal
			p_move_group_interface_->move();


		}
	}
}
int main(int argc, char** argv)
{
  
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto planning_node = std::make_shared<AHPCPlanning>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(planning_node->getNodeBaseInterface());
  auto spin_thread = std::make_unique<std::thread>([&executor, &planning_node]() {
    executor.spin();
    //executor.remove_node(planning_node->getNodeBaseInterface());
  });

  // ===== 主线程循环做任务 =====
  rclcpp::Rate rate(10);   // 10 Hz
  while (rclcpp::ok())
  {
    planning_node->doTask();
    rate.sleep();
  }

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}