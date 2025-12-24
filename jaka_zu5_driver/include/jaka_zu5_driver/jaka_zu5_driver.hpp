#ifndef JAKA_ZU5_DRIVER_
#define JAKA_ZU5_DRIVER_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "shared_message/msg/robot_driver_status.hpp"

#include "shared_message/srv/clear_error.hpp"
#include "shared_message/srv/get_fk.hpp"
#include "shared_message/srv/get_ik.hpp"
#include "shared_message/srv/get_io.hpp"
#include "shared_message/srv/move_j.hpp"
#include "shared_message/srv/move_l.hpp"
#include "shared_message/srv/move_jog.hpp"
#include "shared_message/srv/move_path.hpp"
#include "shared_message/srv/servo_move.hpp"
#include "shared_message/srv/servo_move_enable.hpp"
#include "shared_message/srv/servo_step_move.hpp"
#include "shared_message/srv/set_collision.hpp"
#include "shared_message/srv/set_io.hpp"
#include "shared_message/srv/set_payload.hpp"
#include "shared_message/srv/set_tcp_frame.hpp"
#include "shared_message/srv/set_tcp.hpp"
#include "shared_message/srv/set_user_frame.hpp"
#include "shared_message/srv/move_l_adjust.hpp"
#include "shared_message/srv/set_force_mode.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "shared_message/srv/set_torque_adjust.hpp"
#include "shared_message/srv/set_torque_extract_adjust.hpp"
#include "shared_message/srv/set_admit_extract.hpp"
#include "shared_message/srv/set_robot_param.hpp"

#include "jaka_api/JAKAZuRobot.h"
#include "jaka_api/jkerr.h"
#include "jaka_api/jktypes.h"

#include "soft_inserter_interface.hpp"

namespace jaka_zu5_driver
{

class torque_control
{
public:
	explicit torque_control()
	{
		this->sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
		this->enabled_ = false;
		this->force_ = 0;
		this->vel_ = 30.0;
		this->acc_ = 30.0;
		this->depth_ = 50.0;
		this->seq_ = 0; // sequence number
		this->require_frame_cnt_ = 5;
		this->frame_cnt_ = 0;
		this->last_tcp_force_.clear();
	}

	typedef enum _torque_control_state_t_
	{
		TORQUE_CONTROL_IDLE = 0,
		TORQUE_CONTROL_INIT = 1,
		TORQUE_CONTROL_RUN_MOVE = 2,
		TORQUE_CONTROL_STOP_MOVE = 3,
		TORQUE_CONTROL_FREE_DRIVE = 4,
		TORQUE_CONTROL_DEEP_PLAN_MOVE = 5,        /* 插入深度较深的调整 */
		TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT = 6,   /* 等待第一次插入深度较深的调整完成, 同时收集数据 */
		TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT = 7,   /* 根据收集到的数据, 进行再一次调整 */
		TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE = 8,   /* 等待第二步完成 */
	} torque_control_state_t;

	torque_control_state_t sm_state_; // state
	bool enabled_; // enabled
	bool deep_planed_ = false;            /* 是否已经调用过deepplan方法 */
	uint32_t seq_; // sequence number
	double force_;
	double vel_;
	double acc_;
	double depth_;
	double first_depth_thres_; /* 第一段插枪阈值 */
	int inpos_count_ = 0;
	int frame_cnt_; /* 达到力控值的帧数 */
	int require_frame_cnt_;
	int require_deep_plan_frame_cnt_ =
	    5;            /* 第一次深度调整的期望力数量 */
	double depth_thres_ =
	    10.0; /* 深度调整的插入深度阈值，低于此阈值开启调整 */
	double force_second_ = 150.0; /* 深度算法调整的力阈值 */
	double cur_insert_depth_ = 0.0; /* 当前插入深度 */
	double joint_vel_ = 0.5;          /* 角速度 */
	double joint_acc_ = 0.5;          /* 角加速度 */
	double inpos_thres_ = 10.0;        /* 到位阈值 */
	double depth_second_ = 10.0;        /* final insert 的深度 */
	// int32_t inpos_count_ = 0;          /* 空闲计数 到达一定时间进入下一阶段 */
	std::vector<double> ref_base_tcp_; /* 参考插入点 */
	std::vector<double> last_tcp_force_;
	std::vector<std::vector<double>>
	                              torque_array_;  /* 深度调整的数据队列 */
	std::vector<std::vector<double>>
	                              decoupling_torque_array_;  /* 深度调整的数据队列 */
	std::vector<std::vector<double>>
	                              decoupling_tcp_array_;  /* 深度调整的数据队列 */
	void euler_ang_to_Mat3d(const std::vector<double> &euler_ang,
	                        Eigen::Matrix3d &R);
	bool is_rotation_matrix(const Eigen::Matrix3d &R);
	bool mat3d_to_euler_angles(const Eigen::Matrix3d &R,
	                           Eigen::Vector3d &euler_angle);
	bool trans_tcp_to_base(
	    const std::vector<double> &vec_tcp_move,
	    const std::vector<double> &vec_base_tcp,
	    std::vector<double> &base_move);

	inline void clear()
	{
		this->frame_cnt_ = 0;
		this->torque_array_.clear();
	}
};

class torque_adjustment
{
public:
	explicit torque_adjustment()
	{
		this->enabled_ = false;
		this->set_payload_ = false;
		this->vel_ = 0.5;
		this->acc_ = 0.5;
		this->min_angle_ = 0.001;
		this->seq_ = 0; // sequence number
	}

	bool enabled_; // enabled
	uint32_t seq_; // sequence number
	bool set_payload_; // 是否已经进行过清零
	double vel_;
	double acc_;
	double min_angle_; // 最小调整角
	std::vector<std::vector<double>> force_array_;  /* 输入数据大小 */
};

class admit_extract
{
public:
	explicit admit_extract()
	{
		this->enabled_ = false;
		this->seq_ = 0; // sequence number
	}

	typedef enum _admit_extract_state_t_
	{
		ADMIT_EXTRACT_IDLE = 0,
		ADMIT_EXTRACT_INIT = 1,
		ADMIT_EXTRACT_ADJUST = 2, /* 调整 */
		ADMIT_EXTRACT_WAIT_ADJUST = 3,
		ADMIT_EXTRACT_BACK_TO_ORIGIN = 4, /* 回原点 */
		ADMIT_EXTRACT_WAIT_BACK_TO_ORIGIN = 5, /* 等待回原点 */
		ADMIT_EXTRACT_FINISH = 6,        /* 完成状态 调整力控等 */
	} admit_extract_state_t;

	/**
	 * @brief
	 *
	 */
	typedef struct _adjustment_
	{
		uint32_t times_;
		std::vector<double> coord_;
		double joint_acc_;
		double joint_vel_;
	} adjustment;

	/**
	 * @brief
	 *
	 */
	typedef struct _admit_extract_params_
	{
		bool enabled_;
		uint32_t seq_; // sequence number
		/* 恒力模式配置参数 */
		double ft_damping_;
		double ft_constant_;
		double max_extraction_;
		bool loop_;           /* 是否循环进行 */
		double ft_increment_; /* 每次循环的力增加值 */
		std::vector<admit_extract::adjustment> adjustment_;
		std::vector<admit_extract::adjustment> adjustment_backup_;
	} admit_extract_params;

	/**
	 * @brief
	 *
	 */
	typedef struct _admit_extract_sm_
	{
		bool enabled_;
		uint32_t seq_; // sequence number
		uint32_t adjust_count_; /* 调整次数 */
		double last_depth_ = 0.0; /* 记录上次的深度 */
		admit_extract_state_t state_;
		bool success_; /* 成功标志 */
		std::vector<double> target_;
		std::vector<double> origin_;
		uint32_t wait_count_ = 0;
		bool restart_; /* 重试标志 */
	} admit_extract_sm;

	bool enabled_; // enabled
	uint32_t seq_; // sequence number
	double depth_thres_;
	double force_thres_ =
	    -10.0; /* 深度调整的反馈力阈值，合力少于这个值开始调整 */
	bool restart_admit_after_adjust_;
	bool admit_adjust_after_inpos_;


	/* 最大运动 */
	double max_move_;
	admit_extract_sm sm_;
	admit_extract_params param_;
	double depth_calculate(const std::vector<double> &cur_tcp,
	                       const std::vector<double> &tar_tcp);

	using AdmitReq = shared_message::srv::SetAdmitExtract::Request;
	/**
	 * @brief 设置调整量
	 *
	 * @param rx_move rx方向总调整量
	 * @param ry_move ry方向总调整量
	 * @param div 细分次数
	 */
	inline bool set_admit_adjustment(const std::shared_ptr<AdmitReq> request)
	{
		if (this->param_.enabled_)
		{
			return false;
		}

		if (request->total_adjustment.empty())
		{
			return false;
		}

		/* TODO: 这里添加其他的参数校验, 比如恒力配置参数 */

		this->param_.ft_damping_ = request->ft_damping;
		this->param_.ft_constant_ = request->ft_constant;
		this->param_.max_extraction_ = request->max_extraction;

		if (this->param_.max_extraction_ < 10.0)
		{
			this->param_.max_extraction_ = 10.0;
		}

		this->param_.ft_increment_ = request->ft_increment;
		this->param_.loop_ = request->loop;

		/* 首先清除调整量 */
		this->param_.adjustment_.clear();
		this->param_.adjustment_backup_.clear();

		/* 所有调整量记录下来 */
		for (auto &it : request->total_adjustment)
		{
			admit_extract::adjustment tmp_adjustment;
			tmp_adjustment.coord_.clear();
			/* coord_.at(0) is rx */
			tmp_adjustment.coord_.push_back(it.adjustment.x);
			/* coord_.at(1) is ry */
			tmp_adjustment.coord_.push_back(it.adjustment.y);
			/* coord_.at(2) is rz */
			tmp_adjustment.coord_.push_back(it.adjustment.z);
			tmp_adjustment.times_ = it.times;
			tmp_adjustment.joint_acc_ = it.joint_acc;
			tmp_adjustment.joint_vel_ = it.joint_vel;
			this->param_.adjustment_.push_back(tmp_adjustment);
		}

		this->param_.adjustment_backup_ = this->param_.adjustment_;
		return true;
	}

	/**
	 * @brief 获取rx方向的进给量
	 *
	 * @return double
	 */
	inline double get_rx_inc()
	{
		if (this->param_.adjustment_.empty())
		{
			return static_cast<double>(0);
		}

		return this->param_.adjustment_.begin()->coord_.at(0);
	}

	/**
	 * @brief 获取调整次数
	 *
	 * @return uint32_t
	 */
	inline uint32_t get_adjust_times()
	{
		if (this->param_.adjustment_.empty())
		{
			return static_cast<uint32_t>(0);
		}

		return this->param_.adjustment_.begin()->times_;
	}

	/**
	 * @brief 获取ry方向的进给量
	 *
	 * @return double
	 */
	inline double get_ry_inc()
	{
		if (this->param_.adjustment_.empty())
		{
			return static_cast<double>(0);
		}

		return this->param_.adjustment_.begin()->coord_.at(1);
	}

	/**
	 * @brief 获取rz方向的进给量
	 *
	 * @return double
	 */
	inline double get_rz_inc()
	{
		if (this->param_.adjustment_.empty())
		{
			return static_cast<double>(0);
		}

		return this->param_.adjustment_.begin()->coord_.at(2);
	}

private:




	double rx_inc_; /* x方向上的进给量 */
	double ry_inc_; /* x方向上的进给量 */

};

/* 统一的状态管理 */
class jaka_status
{
public:
	explicit jaka_status()
	{
		this->program_state_ = PROGRAM_IDLE;
		/* 清零机械臂状态变量 */
		memset(&this->cache_robot_status_, 0, sizeof(RobotStatus));
		memset(&this->payload_, 0, sizeof(PayLoad));
		memset(&this->tool_tcp_, 0, sizeof(CartesianPose));

		this->joint_base_position_.resize(6);

		for (size_t i = 0; i < this->joint_base_position_.size(); i++)
		{
			this->joint_base_position_.at(i).resize(6);

			for (size_t j = 0; j < this->joint_base_position_.at(i).size(); j++)
			{
				this->joint_base_position_.at(i).at(j) = 0.0;
			}
		}
	}
	std::mutex robot_status_lock_; /* 机器人状态的互斥锁 */
	bool conn_status_ = false; /* 连接状态, 仅代表能否正常调用接口 */
	bool in_collision_ = false; /* 碰撞状态 */
	int collision_level_ = 5;
	PayLoad payload_;
	CartesianPose tool_tcp_;
	ProgramState program_state_;
	builtin_interfaces::msg::Time ts_; /* 上次成功更新的时间戳 */
	RobotStatus cache_robot_status_; /* 缓存的机械臂状态 */
	std::vector<std::vector<double>> joint_base_position_; /* 关节基座坐标 */
};

/**
 * @brief 机器人的类是SDK提供的，这里封装一个ROS2节点类
 *
 */
class jaka_robot_driver 
{

public:
	explicit jaka_robot_driver(const rclcpp::NodeOptions
	                           &node_options); // 构造函数
	JAKAZuRobot jaka_robot_; // 机器人对象
	~jaka_robot_driver(); // 析构函数
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
private:

	// 错误消息
	const std::map<int, std::string> mapErr_ =
	{
		{ERR_SUCC, "ERR_SUCC"},
		{ERR_FUCTION_CALL_ERROR, "ERR_FUCTION_CALL_ERROR"},
		{ERR_INVALID_HANDLER, "ERR_INVALID_HANDLER"},
		{ERR_INVALID_PARAMETER, "ERR_INVALID_PARAMETER"},
		{ERR_COMMUNICATION_ERR, "ERR_COMMUNICATION_ERR"},
		{ERR_KINE_INVERSE_ERR, "ERR_KINE_INVERSE_ERR"},
		{ERR_EMERGENCY_PRESSED, "ERR_EMERGENCY_PRESSED"},
		{ERR_NOT_POWERED, "ERR_NOT_POWERED"},
		{ERR_NOT_ENABLED, "ERR_NOT_ENABLED"},
		{ERR_DISABLE_SERVOMODE, "ERR_DISABLE_SERVOMODE"},
		{ERR_NOT_OFF_ENABLE, "ERR_NOT_OFF_ENABLE"},
		{ERR_PROGRAM_IS_RUNNING, "ERR_PROGRAM_IS_RUNNING"},
		{ERR_CANNOT_OPEN_FILE, "ERR_CANNOT_OPEN_FILE"},
		{ERR_MOTION_ABNORMAL, "ERR_MOTION_ABNORMAL"},
		{ERR_FTP_PREFROM, "ERR_FTP_PREFROM"}
	};

	// 常量
	const double PI_ = 3.1415926;
	int index_initial_ = 100;
	bool is_block_ = false;
	double protect_torque_ = 0.0;

	// 定义变量：上次调用jog时下发的方向
	int jog_index_last_ = -1;
	// 定义变量：调用jog的次数
	int jog_count_ = 0;
	// 定义变量：保存jog调用的次数
	int jog_count_temp_ = 0;

	using JakaMoveL = shared_message::srv::MoveL;
	// 直线运动
	bool linear_move_callback(const std::shared_ptr<JakaMoveL::Request> request,
	                          std::shared_ptr<JakaMoveL::Response> response);


	using JakaMoveLAdjust = shared_message::srv::MoveLAdjust;
	// 直线运动
	bool move_l_adjust_callback(const std::shared_ptr<JakaMoveLAdjust::Request>
	                            request, std::shared_ptr<JakaMoveLAdjust::Response> response);

	using JakaMoveJ = shared_message::srv::MoveJ;
	// 关节运动
	bool joint_move_callback(const std::shared_ptr<JakaMoveJ::Request> request,
	                         std::shared_ptr<JakaMoveJ::Response> response);

	using JakaMovePath = shared_message::srv::MovePath;
	// 关节运动
	bool move_path_callback(const std::shared_ptr<JakaMovePath::Request> request,
	                        std::shared_ptr<JakaMovePath::Response> response);

	// 手动模式移动
	bool jog_callback(const std::shared_ptr<shared_message::srv::MoveJog::Request>
	                  request,
	                  std::shared_ptr<shared_message::srv::MoveJog::Response> response);

	// 机器人SERVO MOVE模式使能
	bool servo_move_enable_callback(const
	                                std::shared_ptr<shared_message::srv::ServoMoveEnable::Request> request,
	                                std::shared_ptr<shared_message::srv::ServoMoveEnable::Response> response);

	// 机器人笛卡尔空间位置控制模式
	bool servo_p_callback(const
	                      std::shared_ptr<shared_message::srv::ServoMove::Request> request,
	                      const std::shared_ptr<shared_message::srv::ServoMove::Response> response);

	// 机器人关节空间位置控制模式
	bool servo_j_callback(const
	                      std::shared_ptr<shared_message::srv::ServoMove::Request> request,
	                      std::shared_ptr<shared_message::srv::ServoMove::Response> response);

	// 机器人步进关节空间位置控制模式
	bool servo_step_j_callback(const
	                           std::shared_ptr<shared_message::srv::ServoStepMove::Request> request,
	                           std::shared_ptr<shared_message::srv::ServoStepMove::Response> response);

	// 停止机器人
	bool stop_move_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
	                        std::shared_ptr<std_srvs::srv::Empty::Response>);

	// 设置相对于法兰盘坐标系下的工具位置姿态数据
	bool set_tool_frame_callback(const
	                             std::shared_ptr<shared_message::srv::SetTcpFrame::Request> request,
	                             std::shared_ptr<shared_message::srv::SetTcpFrame::Response> response);

	// 设置相对于法兰盘坐标系下的工具位置姿态数据
	bool set_tcp_callback(const
	                      std::shared_ptr<shared_message::srv::SetTcp::Request> request,
	                      std::shared_ptr<shared_message::srv::SetTcp::Response> response);

	// 设置用户坐标系参数
	bool set_user_frame_callback(const
	                             std::shared_ptr<shared_message::srv::SetUserFrame::Request> request,
	                             std::shared_ptr<shared_message::srv::SetUserFrame::Response> response);

	// 机器人负载设置
	bool set_payload_callback(const
	                          std::shared_ptr<shared_message::srv::SetPayload::Request> request,
	                          std::shared_ptr<shared_message::srv::SetPayload::Response> response);

	// 控制机器人进入或退出拖拽模式
	bool drag_mode_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request>
	                        request,
	                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

	// 设置机器人碰撞等级
	bool set_collisionLevel_callback(const
	                                 std::shared_ptr<shared_message::srv::SetCollision::Request> request,
	                                 std::shared_ptr<shared_message::srv::SetCollision::Response> response);

	// 设置输出变量的值
	bool set_io_callback(const std::shared_ptr<shared_message::srv::SetIO::Request>
	                     request,
	                     std::shared_ptr<shared_message::srv::SetIO::Response> response);

	// 获取输入输出状态
	bool get_io_callback(const std::shared_ptr<shared_message::srv::GetIO::Request>
	                     request,
	                     std::shared_ptr<shared_message::srv::GetIO::Response> response);

	// 计算指定关节位置在当前工具、当前安装角度以及当前用户坐标系设置下的位姿值
	bool get_fk_callback(const std::shared_ptr<shared_message::srv::GetFK::Request>
	                     request,
	                     std::shared_ptr<shared_message::srv::GetFK::Response> response);

	// 计算指定位姿在当前工具、当前安装角度以及当前用户坐标系设置下的逆解
	bool get_ik_callback(const std::shared_ptr<shared_message::srv::GetIK::Request>
	                     request,
	                     std::shared_ptr<shared_message::srv::GetIK::Response> response);

	bool recover_callback(const
	                      std::shared_ptr<std_srvs::srv::Trigger::Request>,
	                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

	using JakaSetParam = shared_message::srv::SetRobotParam;
	/* 设置参数接口, 可以一次性设置TCP payload 碰撞等级 */
	bool set_robot_param_callback(const std::shared_ptr<JakaSetParam::Request>
	                              request, std::shared_ptr<JakaSetParam::Response> response);

	// 上报tool position
	void tool_position_callback();

	// 上报 joint position
	void joint_position_callback();

	// 上报 robot states
	void robot_states_callback();

	// 获取负载
	void payload_status_callback();

	// 获取当前工具tcp
	void tool_tcp_status_callback();

	// 获取当前碰撞等级
	void collision_level_callback();

	// 定时停止机器人
	void stop_jog_callback();

	// 初始化jog参数
	void init_jog_param();

	/* 检查链接状态, 并保持连接 */
	void check_conn_socket_state();

	// 判断robot 是否运行到目标位置
	bool robot_joint_in_position(RobotStatus *robotstatus,
	                             trajectory_msgs::msg::JointTrajectory joint_trajectory);
	// 设置伺服控制模式启停
	bool servo_move_enable(bool enable);
	// FollowJointTrajectory Action handle_accepted
	void handle_accepted(const
	                     std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
	                     goal_handle);
	
	// 目标结果
	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
	                                        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
	rclcpp_action::CancelResponse handle_cancel(const
	        std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
	        goal_handle);
	rclcpp::Node::SharedPtr node_;
	
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
	recover_srv_;                     // 恢复服务
	rclcpp::Service<shared_message::srv::MoveL>::SharedPtr
	linear_move_srv_;                     // 直线运动服务
	rclcpp::Service<shared_message::srv::MoveLAdjust>::SharedPtr
	move_l_adjust_srv_;                     // 直线运动服务
	rclcpp::Service<shared_message::srv::MoveJ>::SharedPtr
	joint_move_srv_;                      // 关节运动服务
	rclcpp::Service<shared_message::srv::MovePath>::SharedPtr
	move_path_srv_;                      // 路径移动
	rclcpp::Service<shared_message::srv::MoveJog>::SharedPtr
	jog_srv_;                             // 手动模式移动服务
	rclcpp::Service<shared_message::srv::ServoMoveEnable>::SharedPtr
	servo_move_enable_srv_;    // 机器人SERVO MOVE模式使能服务
	rclcpp::Service<shared_message::srv::ServoMove>::SharedPtr
	servo_p_srv_;                    // 机器人笛卡尔空间位置控制模式
	rclcpp::Service<shared_message::srv::ServoMove>::SharedPtr
	servo_j_srv_;                    // 机器人关节空间位置控制模式服务
	rclcpp::Service<shared_message::srv::ServoStepMove>::SharedPtr
	servo_step_j_srv_;           // 机器人步进关节空间位置控制模式服务
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr
	stop_move_srv_;                             // 停止机器人服务
	rclcpp::Service<shared_message::srv::SetTcpFrame>::SharedPtr
	set_toolFrame_srv_;            // 设置相对于法兰盘坐标系下的工具位置姿态数据服务
	rclcpp::Service<shared_message::srv::SetTcp>::SharedPtr
	set_tcp_srv_;            // 设置相对于法兰盘坐标系下的工具位置姿态数据服务
	rclcpp::Service<shared_message::srv::SetUserFrame>::SharedPtr
	set_user_frame_srv_;          // 设置用户坐标系参数服务
	rclcpp::Service<shared_message::srv::SetPayload>::SharedPtr
	set_payload_srv_;               // 机器人负载设置服务
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
	drag_mode_srv_;                           // 控制机器人进入或退出拖拽模式服务
	rclcpp::Service<shared_message::srv::SetCollision>::SharedPtr
	set_collisionLevel_srv_;      // 设置机器人碰撞等级服务
	rclcpp::Service<shared_message::srv::SetIO>::SharedPtr
	set_io_srv_;                         // 设置输出变量的值服务
	rclcpp::Service<shared_message::srv::GetIO>::SharedPtr
	get_io_srv_;                         // 获取输入输出状态服务
	rclcpp::Service<shared_message::srv::GetFK>::SharedPtr
	get_fk_srv_;                         // 计算指定关节位置在当前工具、当前安装角度以及当前用户坐标系设置下的位姿值服务
	rclcpp::Service<shared_message::srv::GetIK>::SharedPtr
	get_ik_srv_;                         // 计算指定位姿在当前工具、当前安装角度以及当前用户坐标系设置下的逆解服务
	rclcpp::Service<JakaSetParam>::SharedPtr
	set_robot_param_srv_; /* 同一个接口设置参数, 包含tcp payload 碰撞等级 */
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
	tool_position_pub_;           // publish tool position
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
	joint_position_pub_;              // publish joint position
	rclcpp::Publisher<shared_message::msg::RobotDriverStatus>::SharedPtr
	robot_states_pub_;              // publish robot state
	rclcpp::TimerBase::SharedPtr
	tool_position_timer_;                                           // periodic publish timer 125Hz
	rclcpp::TimerBase::SharedPtr
	joint_position_timer_;                                          // periodic publish timer 125Hz
	rclcpp::TimerBase::SharedPtr
	robot_states_timer_;                                            // periodic publish timer 125Hz
	rclcpp::TimerBase::SharedPtr
	payload_status_timer_;                                            // periodic publish timer 125Hz
	rclcpp::TimerBase::SharedPtr
	tool_data_status_timer_;                                            // periodic publish timer 125Hz
	rclcpp::TimerBase::SharedPtr
	collision_level_timer_;                                            // periodic publish timer 125Hz
	rclcpp::TimerBase::SharedPtr
	stop_jog_timer_;                                                // periodic publish timer every 3 seconds
	rclcpp::TimerBase::SharedPtr
	check_conn_socket_state_timer_;                                   // periodic publish timer 10Hz
	rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr moveit_server_; // moveit follow joint trajectory action server

	// 控制句柄
	rclcpp::TimerBase::SharedPtr torque_control_timer_; // 状态机运行线程
	torque_control torque_control_;
	// 状态机
	void torque_control_callback();
	rclcpp::Service<shared_message::srv::SetForceMode>::SharedPtr
	force_mode_srv_;                           // 控制机器人进入力控插枪模式
	bool force_control_callback(const
	                            std::shared_ptr<shared_message::srv::SetForceMode::Request> request,
	                            std::shared_ptr<shared_message::srv::SetForceMode::Response> response);
	torque_adjustment torque_adjust_;
	rclcpp::TimerBase::SharedPtr torque_adjust_timer_; // 状态机运行线程
	void torque_adjust_callback();
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
	torque_reset_srv_; // 力控负载清零服务
	bool torque_reset_callback(const
	                           std::shared_ptr<std_srvs::srv::Trigger::Request>,
	                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
	rclcpp::Service<shared_message::srv::SetTorqueAdjust>::SharedPtr
	torque_adjust_srv_; // 力控负载清零服务
	bool torque_adjust_control_callback(const
	                                    std::shared_ptr<shared_message::srv::SetTorqueAdjust::Request> request,
	                                    std::shared_ptr<shared_message::srv::SetTorqueAdjust::Response> response);
	SoftInserterInterface::Ptr_ soft_charger_interface_; // 插枪调整

	torque_adjustment torque_extract_adjust_; /* 拔枪 */
	rclcpp::TimerBase::SharedPtr
	torque_extract_adjust_timer_; // 状态机运行线程
	void torque_extract_adjust_callback();
	rclcpp::Service<shared_message::srv::SetTorqueExtractAdjust>::SharedPtr
	torque_extract_adjust_srv_; // 力控负载清零服务
	bool torque_adjust_extract_callback(const
	                                    std::shared_ptr<shared_message::srv::SetTorqueExtractAdjust::Request> request,
	                                    std::shared_ptr<shared_message::srv::SetTorqueExtractAdjust::Response>
	                                    response);

	admit_extract admit_extract_; /* 恒力拔枪 */
	rclcpp::TimerBase::SharedPtr admit_extract_timer_; // 状态机运行线程
	void admit_extract_callback();
	rclcpp::Service<shared_message::srv::SetAdmitExtract>::SharedPtr
	admit_extract_srv_; // 恒力拔枪服务
	bool admit_extract_service_callback(const
	                                    std::shared_ptr<shared_message::srv::SetAdmitExtract::Request> request,
	                                    std::shared_ptr<shared_message::srv::SetAdmitExtract::Response> response);
	bool is_admit_adjust_in_pos(const std::vector<double> &current_tcp_pose,
	                            const std::vector<double> &target_tcp_pose);

	bool kine_inverse_adjustment(RobotStatus &status,
	                             std::vector<double> &adjustment, std::vector<double> &target_tcp,
	                             double joint_acc = 0.2, double joint_vel = 0.2);

	rclcpp::TimerBase::SharedPtr torque_protect_timer_;   // 力矩保护定时器
	void torque_protect_callback();                       /* 力矩保护定时器回调 */
	void config_torque_sensor();                            /* 配置力控传感器 */
	void reset_torque_sensor();                           /* 重新初始化力控传感器 */
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
	torque_protect_switch_srv_; // 力控保护开关
	void torque_protect_switch_callback(const
	                                    std::shared_ptr<std_srvs::srv::SetBool::Request> request,
	                                    std::shared_ptr<std_srvs::srv::SetBool::Response>
	                                    response); /* 力控保护开关回调 */
	bool torque_protect_switch_ = false;           /* 力控保护开关 */
	rclcpp::CallbackGroup::SharedPtr srv_cb_group_;     /* 服务回调组 */
	rclcpp::CallbackGroup::SharedPtr timer_cb_group_;   /* 定时器回调组 */
	rclcpp::CallbackGroup::SharedPtr
	torque_timer_cb_group_;   /* 定时器回调组 */
	rclcpp::CallbackGroup::SharedPtr
	admit_timer_cb_group_;   /* 定时器回调组 */
	rclcpp::CallbackGroup::SharedPtr
	status_timer_cb_group_;   /* 定时器回调组 */
	rclcpp::CallbackGroup::SharedPtr
	payload_timer_cb_group_;   /* 查询当前负载定时器回调组 */
	rclcpp::CallbackGroup::SharedPtr
	tool_data_timer_cb_group_;   /* 查询当前工具tcp定时器回调组 */
	rclcpp::CallbackGroup::SharedPtr
	coli_lvl_timer_cb_group_;   /* 查询碰撞等级的callback定时器回调组 */
	std::string algo_log_path_;
	std::string controller_yaml_path_;
	jaka_status jaka_status_; /* 节卡机器人状态管理 */
	bool is_status_fresh(); /* 状态是否是新的 */
	void publish_robot_status(); /* 发布机器人状态 */
	std::string robot_ip_; /* 控制器IP */

	bool first_boot_ = true; /* 初次启动 */
	bool set_config_ = false; /* 设置一些参数 */
	rclcpp::TimerBase::SharedPtr device_manage_timer_; /* 状态管理定时器 */
	void device_manage(); /* 机械臂的连接状态管理 */
	std::string log_path_; /* 日志路径变量 */
	void change_log_file(void);
	void robot_state_handler();
	std::thread device_manage_thread_; /* 状态管理线程 */
	bool in_loop_ = false;
	bool init_soft_inserter_interface(DHParam
	                                  *dh_table); /* 初始化力控算法 */
	bool get_cur_base_tcp(std::vector<double>
	                      &tcp); /* 获取当前基座TCP坐标 */
	int device_manage_set_servo();
};

template <typename Type>
std::string vec2str(const std::vector<Type> &vec)
{
	std::stringstream vec_str;
	vec_str << "[";

	for (size_t i = 0; i < vec.size(); ++i)
	{
		vec_str << vec.at(i);

		if (i < vec.size() - 1)
		{
			vec_str << ", ";
		}
	}

	vec_str << "]";
	return vec_str.str();
}
}  // namespace jaka_zu5_driver

#endif  // JAKA_ZU5_DRIVER_