#include "jaka_zu5_driver/jaka_zu5_driver.hpp"

#include <string>

#include <pthread.h>
#include <unistd.h>
#include <sstream>

#include <chrono>
#include <vector>
#include <cinttypes>

#include "jaka_api/conversion.h"
#include "time.h"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "console/console.hpp"

#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>

#define TS_STR (\
[] () -> std::string {\
	auto now = std::chrono::system_clock::now();\
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);\
	auto time = std::localtime(&now_c);\
	std::ostringstream oss;\
	oss << std::setfill('0')\
	    << (1900 + time->tm_year)\
	    << std::setw(2) << (1 + time->tm_mon)\
	    << std::setw(2) << time->tm_mday\
	    << "_"\
	    << std::setw(2) << time->tm_hour\
	    << std::setw(2) << time->tm_min\
	    << std::setw(2) << time->tm_sec;\
	return oss.str();\
}()\
               )

static const rclcpp::Logger LOGGER = rclcpp::get_logger("jaka_zu5_driver");
namespace jaka_zu5_driver
{
/**
* @brief Construct a new jaka ros2 driver node::jaka ros2 driver node object
*
* 构造函数
*
* @param options
*/
jaka_robot_driver::jaka_robot_driver(const rclcpp::NodeOptions &options)
	: node_{std::make_shared<rclcpp::Node>("jaka_zu5_driver_node", options)}
{
	// 设置编码
	setlocale(LC_ALL, "");

	// 获取力控时达到所需力矩之后保持的数据帧数
	node_->declare_parameter<int>("frame_cnt", 5); // 5帧数据
	node_->get_parameter("frame_cnt",
	                    this->torque_control_.require_frame_cnt_); // 读取参数
	RCLCPP_INFO(LOGGER,"Set frame cnt %d\n", this->torque_control_.require_frame_cnt_);

	// admit thres depth
	node_->get_parameter("admit_depth_thres",
	                    this->admit_extract_.depth_thres_); // 读取参数
	RCLCPP_INFO(LOGGER,"Set admit_depth_thres %f\n", this->admit_extract_.depth_thres_);

	// admit thres depth
	node_->get_parameter("admit_force_thres", this->admit_extract_.force_thres_);
	RCLCPP_INFO(LOGGER,"Set admit_force_thres %f\n", this->admit_extract_.force_thres_);

	// restart admit after adjust
	node_->get_parameter("restart_admit_after_adjust",
	                    this->admit_extract_.restart_admit_after_adjust_);
	RCLCPP_INFO(LOGGER,"Set restart_admit_after_adjust %d\n",
	           this->admit_extract_.restart_admit_after_adjust_);

	// admit
	node_->get_parameter("admit_adjust_after_inpos",
	                    this->admit_extract_.admit_adjust_after_inpos_);
	RCLCPP_INFO(LOGGER,"Set admit_adjust_after_inpos %d\n",
	           this->admit_extract_.admit_adjust_after_inpos_);

	// get robot ip setting
	node_->get_parameter("ip", this->robot_ip_); // 读取机器人IP参数
	RCLCPP_INFO(LOGGER,"Set robot ip %s\n", this->robot_ip_.c_str());

	// get algo_log_path setting
	node_->get_parameter("algo_log_path",
	                    this->algo_log_path_); // 读取算法日志路径
	RCLCPP_INFO(LOGGER,"Set algo_log_path %s\n", this->algo_log_path_.c_str());

	// get controller path
	std::string package_share_directory =
	    ament_index_cpp::get_package_share_directory("jaka_zu5_driver");
	auto default_algo_config = package_share_directory + "/config/controller.yaml";
	node_->get_parameter("controller_yaml_path",
	                    this->controller_yaml_path_); // 读取算法日志路径
	RCLCPP_INFO(LOGGER,"Set controller_yaml_path %s\n",
	           this->controller_yaml_path_.c_str());

	/* 保护力矩 */
	node_->get_parameter("protect_torque",
	                    this->protect_torque_); // 读取参数中的保护力矩
	RCLCPP_INFO(LOGGER,"Load parameter protect_torque %f\n", this->protect_torque_);

	// get robot model
	std::string robot_model; // 读取机械臂型号，如jaka_zu5
	node_->get_parameter("model", robot_model); // 读取参数中的机械臂型号
	RCLCPP_INFO(LOGGER,"Load parameter robot_model %s\n", robot_model.c_str());

	srv_cb_group_ = node_->create_callback_group(
	                    rclcpp::CallbackGroupType::MutuallyExclusive);

	timer_cb_group_ = node_->create_callback_group(
	                      rclcpp::CallbackGroupType::MutuallyExclusive);

	status_timer_cb_group_ = node_->create_callback_group(
	                             rclcpp::CallbackGroupType::MutuallyExclusive);

	payload_timer_cb_group_ = node_->create_callback_group(
	                              rclcpp::CallbackGroupType::MutuallyExclusive);

	tool_data_timer_cb_group_ = node_->create_callback_group(
	                                rclcpp::CallbackGroupType::MutuallyExclusive);

	coli_lvl_timer_cb_group_ = node_->create_callback_group(
	                               rclcpp::CallbackGroupType::MutuallyExclusive);

	torque_timer_cb_group_ = node_->create_callback_group(
	                             rclcpp::CallbackGroupType::MutuallyExclusive);

	admit_timer_cb_group_ = node_->create_callback_group(
	                            rclcpp::CallbackGroupType::MutuallyExclusive);

	this->device_manage_thread_ = std::thread(&jaka_robot_driver::device_manage,
	                              this);

	// 1.1 直线运动（自定义用户坐标系下）
	linear_move_srv_ =
	    node_->create_service<shared_message::srv::MoveL>("/jaka_zu5_driver/linear_move",
	            std::bind(&jaka_robot_driver::linear_move_callback, this, std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);

	move_l_adjust_srv_ =
	    node_->create_service<shared_message::srv::MoveLAdjust>("/jaka_zu5_driver/move_l_adjust",
	            std::bind(&jaka_robot_driver::move_l_adjust_callback, this,
	                      std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);

	// 1.2 关节运动
	joint_move_srv_ =
	    node_->create_service<shared_message::srv::MoveJ>("/jaka_zu5_driver/joint_move",
	            std::bind(&jaka_robot_driver::joint_move_callback, this, std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);

	// 1.2 关节运动
	move_path_srv_ =
	    node_->create_service<shared_message::srv::MovePath>("/jaka_zu5_driver/move_path",
	            std::bind(&jaka_robot_driver::move_path_callback, this, std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 1.3 步进运动
	jog_srv_ = node_->create_service<shared_message::srv::MoveJog>("/jaka_zu5_driver/jog",
	           std::bind(&jaka_robot_driver::jog_callback, this, std::placeholders::_1,
	                     std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 1.4 伺服位置控制模式使能
	servo_move_enable_srv_ =
	    node_->create_service<shared_message::srv::ServoMoveEnable>("/jaka_zu5_driver/servo_move_enable",
	            std::bind(&jaka_robot_driver::servo_move_enable_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);
	// 1.5 笛卡尔空间伺服模式运动
	servo_p_srv_ =
	    node_->create_service<shared_message::srv::ServoMove>("/jaka_zu5_driver/servo_p",
	            std::bind(&jaka_robot_driver::servo_p_callback, this, std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 1.6 关节空间伺服模式运动
	servo_j_srv_ =
	    node_->create_service<shared_message::srv::ServoMove>("/jaka_zu5_driver/servo_j",
	            std::bind(&jaka_robot_driver::servo_j_callback, this, std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 1.6.1 步进关节空间伺服模式运动
	servo_step_j_srv_ =
	    node_->create_service<shared_message::srv::ServoStepMove>("/jaka_zu5_driver/servo_step_j",
	            std::bind(&jaka_robot_driver::servo_step_j_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);
	// 1.7 停止运动
	stop_move_srv_ = node_->create_service<std_srvs::srv::Empty>("/jaka_zu5_driver/stop_move",
	                 std::bind(&jaka_robot_driver::stop_move_callback, this, std::placeholders::_1,
	                           std::placeholders::_2));
	recover_srv_ = node_->create_service<std_srvs::srv::Trigger>("/jaka_zu5_driver/recover",
	               std::bind(&jaka_robot_driver::recover_callback, this, std::placeholders::_1,
	                         std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 2.1 设置tcp参数
	set_toolFrame_srv_ =
	    node_->create_service<shared_message::srv::SetTcpFrame>("/jaka_zu5_driver/set_toolframe",
	            std::bind(&jaka_robot_driver::set_tool_frame_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);

	// 2.1 设置tcp参数
	set_tcp_srv_ =
	    node_->create_service<shared_message::srv::SetTcp>("/jaka_zu5_driver/set_tcp",
	            std::bind(&jaka_robot_driver::set_tcp_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);

	// 2.2 设置用户坐标系参数
	set_user_frame_srv_ =
	    node_->create_service<shared_message::srv::SetUserFrame>("/jaka_zu5_driver/set_userframe",
	            std::bind(&jaka_robot_driver::set_user_frame_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);
	// 2.3 设置机械臂负载重心参数
	set_payload_srv_ =
	    node_->create_service<shared_message::srv::SetPayload>("/jaka_zu5_driver/set_payload",
	            std::bind(&jaka_robot_driver::set_payload_callback, this, std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 2.4 设置自由驱动模式
	drag_mode_srv_ =
	    node_->create_service<std_srvs::srv::SetBool>("/jaka_zu5_driver/drag_move",
	            std::bind(&jaka_robot_driver::drag_mode_callback, this, std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 2.5 设置碰撞灵敏度
	set_collisionLevel_srv_ =
	    node_->create_service<shared_message::srv::SetCollision>("/jaka_zu5_driver/set_collisionlevel",
	            std::bind(&jaka_robot_driver::set_collisionLevel_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);
	// 2.6 设置IO
	set_io_srv_ = node_->create_service<shared_message::srv::SetIO>("/jaka_zu5_driver/set_io",
	              std::bind(&jaka_robot_driver::set_io_callback, this, std::placeholders::_1,
	                        std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	// 2.7 获取IO
	get_io_srv_ = node_->create_service<shared_message::srv::GetIO>("/jaka_zu5_driver/get_io",
	              std::bind(&jaka_robot_driver::get_io_callback, this, std::placeholders::_1,
	                        std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	//2.8 求解正解
	get_fk_srv_ = node_->create_service<shared_message::srv::GetFK>("/jaka_zu5_driver/get_fk",
	              std::bind(&jaka_robot_driver::get_fk_callback, this, std::placeholders::_1,
	                        std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	//2.9 求解逆解
	get_ik_srv_ = node_->create_service<shared_message::srv::GetIK>("/jaka_zu5_driver/get_ik",
	              std::bind(&jaka_robot_driver::get_ik_callback, this, std::placeholders::_1,
	                        std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);

	/* 修改配置, TCP Payload 碰撞等级 */
	set_robot_param_srv_ =
	    node_->create_service<shared_message::srv::SetRobotParam>("/jaka_zu5_driver/set_robot_param",
	            std::bind(&jaka_robot_driver::set_robot_param_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);

	// 3.1 末端位置位姿状态信息上报
	tool_position_pub_ =
	    node_->create_publisher<geometry_msgs::msg::TwistStamped>("/jaka_zu5_driver/tool_position",
	            10);
	tool_position_timer_ = node_->create_wall_timer(std::chrono::milliseconds(8),
	                       std::bind(&jaka_robot_driver::tool_position_callback, this), timer_cb_group_);
	// 3.2 关节状态信息上报
	joint_position_pub_ =
	    node_->create_publisher<sensor_msgs::msg::JointState>("/jaka_zu5_driver/joint_position",
	            10);
	joint_position_timer_ = node_->create_wall_timer(std::chrono::milliseconds(8),
	                        std::bind(&jaka_robot_driver::joint_position_callback, this), timer_cb_group_);
	// 3.3 机器人事件状态信息上报
	robot_states_pub_ =
	    node_->create_publisher<shared_message::msg::RobotDriverStatus>("/jaka_zu5_driver/robot_states",
	            10);
	robot_states_timer_ = node_->create_wall_timer(std::chrono::milliseconds(8),
	                                        std::bind(&jaka_robot_driver::robot_states_callback, this),
	                                        status_timer_cb_group_);
	// payload_status_timer_ = create_wall_timer(std::chrono::milliseconds(250),
	//                         std::bind(&jaka_robot_driver::payload_status_callback, this),
	//                         payload_timer_cb_group_);
	// tool_data_status_timer_ = create_wall_timer(std::chrono::milliseconds(250),
	//                           std::bind(&jaka_robot_driver::tool_tcp_status_callback, this),
	//                           tool_data_timer_cb_group_);
	// collision_level_timer_ = create_wall_timer(std::chrono::milliseconds(250),
	//                          std::bind(&jaka_robot_driver::collision_level_callback, this),
	//                          coli_lvl_timer_cb_group_);

	// 自动停止机器人jog运动
	stop_jog_timer_ = node_->create_wall_timer(std::chrono::seconds(3),
	                                    std::bind(&jaka_robot_driver::stop_jog_callback, this), timer_cb_group_);

	// 监控网络连接状态
	// device_manage_timer_ = create_wall_timer(std::chrono::milliseconds(500),
	//                        std::bind(&jaka_robot_driver::device_manage, this), timer_cb_group_);

	// 建立和机器人型号一致的action，如/jaka_arm_controller/follow_joint_trajectory
	this->moveit_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
	        node_,
	        "/" + robot_model + "_controller/follow_joint_trajectory",
	        std::bind(&jaka_robot_driver::handle_goal, this, std::placeholders::_1,
	                  std::placeholders::_2),
	        std::bind(&jaka_robot_driver::handle_cancel, this, std::placeholders::_1),
	        std::bind(&jaka_robot_driver::handle_accepted, this, std::placeholders::_1));

	this->torque_control_timer_ = node_->create_wall_timer(std::chrono::milliseconds(20),
	                              std::bind(&jaka_robot_driver::torque_control_callback, this));
	force_mode_srv_ =
	    node_->create_service<shared_message::srv::SetForceMode>("/jaka_zu5_driver/set_force_mode",
	            std::bind(&jaka_robot_driver::force_control_callback, this,
	                      std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);

	this->torque_adjust_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
	                             std::bind(&jaka_robot_driver::torque_adjust_callback, this),
	                             torque_timer_cb_group_);
	this->torque_extract_adjust_timer_ = node_->create_wall_timer(
	        std::chrono::milliseconds(100),
	        std::bind(&jaka_robot_driver::torque_extract_adjust_callback, this),
	        torque_timer_cb_group_);

	this->admit_extract_timer_ = node_->create_wall_timer(
	                                 std::chrono::milliseconds(100),
	                                 std::bind(&jaka_robot_driver::admit_extract_callback, this),
	                                 admit_timer_cb_group_);
	torque_reset_srv_ =
	    node_->create_service<std_srvs::srv::Trigger>("/jaka_zu5_driver/torque_reset",
	            std::bind(&jaka_robot_driver::torque_reset_callback, this,
	                      std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	torque_adjust_srv_ =
	    node_->create_service<shared_message::srv::SetTorqueAdjust>("/jaka_zu5_driver/torque_adjust",
	            std::bind(&jaka_robot_driver::torque_adjust_control_callback, this,
	                      std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);
	torque_extract_adjust_srv_ =
	    node_->create_service<shared_message::srv::SetTorqueExtractAdjust>("/jaka_zu5_driver/torque_extract_adjust",
	            std::bind(&jaka_robot_driver::torque_adjust_extract_callback, this,
	                      std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);

	admit_extract_srv_ =
	    node_->create_service<shared_message::srv::SetAdmitExtract>("/jaka_zu5_driver/admit_extract",
	            std::bind(&jaka_robot_driver::admit_extract_service_callback, this,
	                      std::placeholders::_1,
	                      std::placeholders::_2), rmw_qos_profile_services_default, srv_cb_group_);

	this->torque_protect_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
	                              std::bind(&jaka_robot_driver::torque_protect_callback, this),
	                              torque_timer_cb_group_);

	this->torque_protect_switch_srv_ =
	    node_->create_service<std_srvs::srv::SetBool>("/jaka_zu5_driver/torque_protect_switch",
	            std::bind(&jaka_robot_driver::torque_protect_switch_callback, this,
	                      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
	            srv_cb_group_);

	RCLCPP_INFO(LOGGER,"robot start\n");
}

// 析构函数
jaka_robot_driver::~jaka_robot_driver()
{
	if (in_loop_)
	{
		this->in_loop_ = false;
		this->device_manage_thread_.join();
	}

	this->soft_charger_interface_->deinit();
	this->jaka_robot_.set_torque_sensor_mode(0);
	this->jaka_robot_.motion_abort();
	// this->jaka_robot_.login_out();

	RCLCPP_INFO(LOGGER,"robot stop\n");
	return;
}
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr jaka_robot_driver::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}
/**
 * @brief
 *
 * @param euler_ang
 * @param R
 */
void torque_control::euler_ang_to_Mat3d(const std::vector<double> &euler_ang,
                                        Eigen::Matrix3d &R)
{
	R = Eigen::AngleAxisd(euler_ang[2], Eigen::Vector3d::UnitZ()) *
	    Eigen::AngleAxisd(euler_ang[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler_ang[0], Eigen::Vector3d::UnitX());
}

/**
 * @brief
 *
 * @param R
 * @return true
 * @return false
 */
bool torque_control::is_rotation_matrix(const Eigen::Matrix3d &R)
{
	Eigen::Matrix3d Rt = R.transpose();
	Eigen::Matrix3d shouldBeIdentity = Rt * R;

	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

	return (shouldBeIdentity - I).norm() < 1e-6;
}

bool torque_control::mat3d_to_euler_angles(const Eigen::Matrix3d &R,
        Eigen::Vector3d &euler_angle)
{
	if (false == is_rotation_matrix(R))
	{
		return false;
	}

	double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
	bool singular = sy < 1e-6; // If
	double x, y, z;

	if (!singular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));
	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;
	}

	euler_angle.x() = x;
	euler_angle.y() = y;
	euler_angle.z() = z;
	return true;
}

/**
 * @brief
 *
 * @param vec_tcp_move m
 * @param vec_base_tcp mm
 * @param base_move
 */
bool torque_control::trans_tcp_to_base(
    const std::vector<double> &vec_tcp_move,
    const std::vector<double> &vec_base_tcp,
    std::vector<double> &base_move)
{
	Eigen::Matrix3d R_base_tcp = Eigen::Matrix3d::Identity();
	std::vector<double> euler = {vec_base_tcp[3], vec_base_tcp[4], vec_base_tcp[5]};
	euler_ang_to_Mat3d(euler, R_base_tcp);

	Eigen::Matrix3d R_tcp_tar = Eigen::Matrix3d::Identity();
	euler = {vec_tcp_move[3], vec_tcp_move[4], vec_tcp_move[5]};
	euler_ang_to_Mat3d(euler, R_tcp_tar);

	Eigen::Vector3d t_base_tcp = Eigen::Vector3d::Zero();
	t_base_tcp << vec_base_tcp[0] / 1000.0, vec_base_tcp[1] / 1000.0,
	           vec_base_tcp[2] / 1000.0;
	Eigen::Vector3d t_tcp_tar = Eigen::Vector3d::Zero();
	t_tcp_tar << vec_tcp_move[0], vec_tcp_move[1], vec_tcp_move[2];

	Eigen::Vector3d t_base_tar = Eigen::Vector3d::Zero();
	t_base_tar = R_base_tcp * t_tcp_tar + t_base_tcp;
	base_move[0] = t_base_tar.x() * 1000.0;
	base_move[1] = t_base_tar.y() * 1000.0;
	base_move[2] = t_base_tar.z() * 1000.0;

	Eigen::Matrix3d R_base_tar = R_base_tcp * R_tcp_tar;
	Eigen::Vector3d euler_ang =  Eigen::Vector3d::Zero();

	if (false == mat3d_to_euler_angles(R_base_tar, euler_ang))
	{
		return false;
	}

	base_move[3] = euler_ang.x();
	base_move[4] = euler_ang.y();
	base_move[5] = euler_ang.z();
	return true;
}

/**
 * @brief 初始化算法
 *
 */
bool jaka_robot_driver::init_soft_inserter_interface(DHParam *dh_table)
{
	// 将dh_table数据存储到vector内
	std::vector<double> dh_alpha(6, 0.0);
	std::vector<double> dh_a(6, 0.0);
	std::vector<double> dh_d(6, 0.0);
	std::vector<double> dh_joint_homeoff(6, 0.0);

	for (int i = 0; i < 6; ++i)
	{
		double alpha_i_1 = dh_table->alpha[i] / 180.0 * this->PI_; // 角度单位rad
		double a_i_1 =  dh_table->a[i] / 1000.0; // 连杆长度单位m
		double d_i = dh_table->d[i] / 1000.0; // 偏移长度单位m
		double theta_i = dh_table->joint_homeoff[i] / 180.0 *
		                 this->PI_; // 角度单位rad

		dh_alpha[i] = alpha_i_1;
		dh_a[i] = a_i_1;
		dh_d[i] = d_i;
		dh_joint_homeoff[i] = theta_i;
	}

	this->soft_charger_interface_ = SoftInserterInterface::Ptr_
	                                (new SoftInserterInterface(dh_alpha, dh_a, dh_d, dh_joint_homeoff));

	if (NULL == soft_charger_interface_)
	{
		RCLCPP_INFO(LOGGER,"create soft inserter interface failed\n");
		return false;
	}

	std::string log_path = algo_log_path_;
	auto timestamp = TS_STR;
	log_path = log_path + "/" + "SOFT-CHARGER-INSERTER_" + timestamp + "_";
	soft_charger_interface_->init(this->controller_yaml_path_, log_path);
	RCLCPP_INFO(LOGGER,"init algo done, pth %s", log_path.c_str());

	return true;
}

/**
 * @brief 获取当前TCP vector格式
 *
 * @param[out] tcp 当前TCP
 * @return true
 * @return false 获取失败
 */
bool jaka_robot_driver::get_cur_base_tcp(std::vector<double> &tcp)
{
	CartesianPose cur_tcp_pose;
	this->jaka_robot_.get_tcp_position(&cur_tcp_pose);
	tcp.clear();
	tcp.resize(6);
	tcp.at(0) = cur_tcp_pose.tran.x;
	tcp.at(1) = cur_tcp_pose.tran.y;
	tcp.at(2) = cur_tcp_pose.tran.z;
	tcp.at(3) = cur_tcp_pose.rpy.rx;
	tcp.at(4) = cur_tcp_pose.rpy.ry;
	tcp.at(5) = cur_tcp_pose.rpy.rz;
	return true;
}

/**
 * @brief 更换日志文件
 *
 * @param log_path
 */
void jaka_robot_driver::change_log_file(void)
{
	struct stat filestat;

	if (stat(this->log_path_.c_str(), &filestat) != 0)
	{
		RCLCPP_ERROR_STREAM(LOGGER,"Error occurred while checking the file size");
	}
	else
	{
		int file_size = filestat.st_size; // 获取文件大小
        //RCLCPP_INFO(LOGGER,"file_size :%d",file_size);
		//if (file_size > 10 * 1024 * 1024) /* 10M大小 */
		if (file_size > 1 * 1024 * 1024) /* 1M大小 */
		{
			// 执行切换日志文件操作
			// 在这里填入切换日志文件的代码
			std::string tmp_log_path = "/root/RCC_WS/log/ROBOT_ARM";
			std::string temp_str = "rm -rf " + tmp_log_path + "/" + "JAKA-SDK_*.log";
			//auto tmp_timestamp = TS_STR;
			auto tmp_timestamp = "20250205";
			tmp_log_path = tmp_log_path + "/" + "JAKA-SDK_" + tmp_timestamp + ".log";
		    system(temp_str.c_str()); //删除正则表达式文件
			auto ret = this->jaka_robot_.set_SDK_filepath(tmp_log_path.c_str());
			RCLCPP_INFO_STREAM(LOGGER,"Switching log file to " << tmp_log_path << " " << ret);

			if (ERR_SUCC == ret)
			{
				this->log_path_ = tmp_log_path;
			}
		}
	}

	return;
}

/**
 * @brief 设备管理
 *
 * 负责启动后第一次机械臂上电
 * 如果连接丢失, 重新连接
 *
 */
void jaka_robot_driver::device_manage()
{
	rclcpp::WallRate loop(2); /* 500ms一次 */
	this->in_loop_ = true;
	this->log_path_ = "/root/RCC_WS/log/ROBOT_ARM";
	//auto timestamp = TS_STR;
	auto timestamp = "20250205";
	log_path_ = log_path_ + "/" + "JAKA-SDK_" + timestamp + ".log";
	JAKAZuRobot::static_Set_SDK_filepath(log_path_.c_str());
	RCLCPP_INFO(LOGGER,"static set jaka sdk log path, pth %s", log_path_.c_str());

	while (this->in_loop_)
	{
		/* 判断是否需要登录 */
		if ((false == this->jaka_status_.conn_status_)
		        || ((0 == this->jaka_status_.cache_robot_status_.is_socket_connect)
		            && (false == this->first_boot_)))
		{
			int login_ret = this->jaka_robot_.login_in(this->robot_ip_.c_str());

			static int last_login_ret = 0;

			if (ERR_SUCC != login_ret)
			{
				if (last_login_ret != login_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage connect robot %s\n",
					            this->mapErr_.at(login_ret).c_str());
					last_login_ret = login_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_login_ret = 0;
			}
		}

		/* 初次启动 */
		if (this->first_boot_)
		{
			/* 初次启动, 清除错误 */
			int clear_error_ret = this->jaka_robot_.clear_error();
			static int last_first_boot_clear_error_ret = 0;

			if (ERR_SUCC != clear_error_ret)
			{
				if (last_first_boot_clear_error_ret != clear_error_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot clear error %s\n",
					            this->mapErr_.at(clear_error_ret).c_str());
					last_first_boot_clear_error_ret = clear_error_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_clear_error_ret = 0;
			}

			/* 初次启动, 清除碰撞 */
			int col_rec_ret = this->jaka_robot_.collision_recover();

			static int last_first_boot_collision_recover_ret = 0;

			if (ERR_SUCC != col_rec_ret)
			{
				if (last_first_boot_collision_recover_ret != col_rec_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot collision recover %s\n",
					            this->mapErr_.at(col_rec_ret).c_str());
					last_first_boot_collision_recover_ret = col_rec_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_collision_recover_ret = 0;
			}

			/* 如果没有上电, 上电 */
			int power_on_ret = this->jaka_robot_.power_on();

			static int last_first_boot_power_on_ret = 0;

			if (ERR_SUCC != power_on_ret)
			{
				if (last_first_boot_power_on_ret != power_on_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot power on robot %s\n",
					            this->mapErr_.at(power_on_ret).c_str());
					last_first_boot_power_on_ret = power_on_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_power_on_ret = 0;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}

			/* 如果没有上使能, 上使能 */
			int enable_ret = this->jaka_robot_.enable_robot();
			static int last_first_boot_enable_robot_ret = 0;

			if (ERR_SUCC != enable_ret)
			{
				if (last_first_boot_enable_robot_ret != enable_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot enable robot %s\n",
					            this->mapErr_.at(enable_ret).c_str());
					last_first_boot_enable_robot_ret = enable_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_enable_robot_ret = 0;
			}


			/* 机器人伺服模式下关节空间一阶低通滤波 */
			int servo_ret = this->jaka_robot_.servo_speed_foresight(15, 0.03);
			static int last_first_boot_set_foresight_ret = 0;

			if (ERR_SUCC != servo_ret)
			{
				if (last_first_boot_set_foresight_ret != servo_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot set robot servo speed foresight %s\n",
					            this->mapErr_.at(servo_ret).c_str());
					last_first_boot_set_foresight_ret = servo_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_set_foresight_ret = 0;
			}

			/* 关闭伺服模式使能 */
			int servo_enable_ret = this->jaka_robot_.servo_move_enable(false);
			static int last_first_boot_servo_move_ret = 0;

			if (ERR_SUCC != servo_enable_ret)
			{
				if (last_first_boot_servo_move_ret != servo_enable_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot set robot servo move enable error %s",
					            this->mapErr_.at(servo_enable_ret).c_str());
					last_first_boot_servo_move_ret = servo_enable_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_servo_move_ret = 0;
			}

			/* 设置滤波参数 */
			int lpf_ret = this->jaka_robot_.servo_move_use_joint_LPF(0.5);
			static int last_first_boot_set_lpf_ret = 0;

			if (ERR_SUCC != lpf_ret)
			{
				if (last_first_boot_set_lpf_ret != lpf_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot set robot lpf error %s",
					            this->mapErr_.at(lpf_ret).c_str());
					last_first_boot_set_lpf_ret = lpf_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_set_lpf_ret = 0;
			}

			/* 设置滤波参数 */
			DHParam dh_table;
			int dh_table_ret = this->jaka_robot_.get_dh_param(&dh_table);
			static int last_first_boot_dh_table_ret = 0;

			if (ERR_SUCC != dh_table_ret)
			{
				if (last_first_boot_dh_table_ret != dh_table_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot get dh table error %s",
					            this->mapErr_.at(dh_table_ret).c_str());
					last_first_boot_dh_table_ret = dh_table_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_dh_table_ret = 0;
			}

			int rapidrate_ret = this->jaka_robot_.set_rapidrate(1.0);
			static int last_first_boot_rapidrate_ret = 0;
			if (ERR_SUCC != rapidrate_ret)
			{
				if (last_first_boot_rapidrate_ret != rapidrate_ret)
				{
					RCLCPP_ERROR(LOGGER,"device_manage first_boot set rapidrate_ret error %s",
					            this->mapErr_.at(rapidrate_ret).c_str());
					last_first_boot_rapidrate_ret = rapidrate_ret;
				}

				loop.sleep();
				continue;
			}
			else
			{
				last_first_boot_rapidrate_ret = 0;
			}

			/* 初始化力控传感器 */
			this->config_torque_sensor();
			this->init_soft_inserter_interface(&dh_table);
			this->first_boot_ = false; /* 初始化完成 */
			this->set_config_ = true;
		}

		if (true == this->jaka_status_.conn_status_)
		{
			/* 急停状态, 不进行后续步骤 */
			if (this->jaka_status_.cache_robot_status_.emergency_stop)
			{
				static bool last_in_emergency = false;

				if (last_in_emergency != this->jaka_status_.cache_robot_status_.emergency_stop)
				{
					RCLCPP_ERROR(LOGGER,"device_manage robot in emergency stop\n");
					last_in_emergency = this->jaka_status_.cache_robot_status_.emergency_stop;
				}

				this->set_config_ = false;
				loop.sleep();
				continue;
			}

			/* 碰撞状态 不进行后续步骤 */
			if (this->jaka_status_.in_collision_)
			{
				static bool last_in_collision = 0;

				if (last_in_collision != this->jaka_status_.in_collision_)
				{
					RCLCPP_ERROR(LOGGER,"device_manage in collision \n");
					last_in_collision = this->jaka_status_.in_collision_;
				}

				loop.sleep();
				continue;
			}

			/* 如果没有上电, 上电 */
			if (0 == this->jaka_status_.cache_robot_status_.powered_on)
			{
				this->set_config_ = false;
				int power_on_ret = this->jaka_robot_.power_on();
				static int last_power_on_robot_ret = 0;

				if (ERR_SUCC != power_on_ret)
				{
					if (last_power_on_robot_ret != power_on_ret)
					{
						RCLCPP_ERROR(LOGGER,"device_manage power on robot %s\n",
						            this->mapErr_.at(power_on_ret).c_str());
						last_power_on_robot_ret = power_on_ret;
					}

					loop.sleep();
					continue;
				}
				else
				{
					last_power_on_robot_ret = 0;
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
			}

			/* 如果没有上使能, 上使能 */
			if (0 == this->jaka_status_.cache_robot_status_.enabled)
			{
				this->set_config_ = false;
				int enable_ret = this->jaka_robot_.enable_robot();
				static int last_enable_robot_ret = 0;

				if (ERR_SUCC != enable_ret)
				{
					if (last_enable_robot_ret != enable_ret)
					{
						RCLCPP_ERROR(LOGGER,"device_manage enable robot %s\n",
						            this->mapErr_.at(enable_ret).c_str());
						last_enable_robot_ret = enable_ret;
					}

					loop.sleep();
					continue;
				}
				else
				{
					last_enable_robot_ret = 0;
				}
			}

			if (false == this->set_config_)
			{
				static uint32_t set_config_count = 0;

				if (0 == (set_config_count % 10))
				{
					RCLCPP_INFO(LOGGER,"%s set config", __func__);
				}

				set_config_count ++;

				/* 机器人伺服模式下关节空间一阶低通滤波 */
				int servo_ret = this->jaka_robot_.servo_speed_foresight(15, 0.03);
				static int last_set_robot_foresight_ret = 0;

				if (ERR_SUCC != servo_ret)
				{
					if (last_set_robot_foresight_ret != servo_ret)
					{
						RCLCPP_ERROR(LOGGER,"device_manage set robot servo speed foresight %s\n",
						            this->mapErr_.at(servo_ret).c_str());
						last_set_robot_foresight_ret = servo_ret;
					}

					this->set_config_ = false;
					loop.sleep();
					continue;
				}
				else
				{
					last_set_robot_foresight_ret = 0;
				}

				/* 关闭伺服模式使能 */
				int servo_enable_ret = this->jaka_robot_.servo_move_enable(false);
				static int last_servo_move_disable_ret = 0;

				if (ERR_SUCC != servo_enable_ret)
				{
					if (last_servo_move_disable_ret != servo_enable_ret)
					{
						RCLCPP_ERROR(LOGGER,"device_manage first_boot set robot servo move enable error %s",
						            this->mapErr_.at(servo_enable_ret).c_str());
						last_servo_move_disable_ret = servo_enable_ret;
					}

					this->set_config_ = false;
					loop.sleep();
					continue;
				}
				else
				{
					last_servo_move_disable_ret = 0;
				}

				/* 设置滤波参数 */
				int lpf_ret = this->jaka_robot_.servo_move_use_joint_LPF(0.5);
				static int last_set_robot_lpf_ret = 0;

				if (ERR_SUCC != lpf_ret)
				{
					if (last_set_robot_lpf_ret != lpf_ret)
					{
						RCLCPP_ERROR(LOGGER,"device_manage first_boot set robot lpf error %s",
						            this->mapErr_.at(lpf_ret).c_str());
						last_set_robot_lpf_ret = lpf_ret;
					}

					this->set_config_ = false;
					loop.sleep();
					continue;
				}
				else
				{
					last_set_robot_lpf_ret = 0;
				}

				/* 初始化力控传感器 */
				this->config_torque_sensor();
				this->set_config_ = true;
			}

			/* 判断当前日志文件的大小, 并适当时间切换日志文件 */
			this->change_log_file();
		}

		loop.sleep(); // 休眠, 整个循环以2Hz速度运行
	}

	return;
}

/**
 * @brief 初始化力控传感器
 *
 */
void jaka_robot_driver::config_torque_sensor(void)
{
	/* 设置力控品牌 */
	this->jaka_robot_.set_torsenosr_brand(6);
	/* 开启力控传感器 */
	this->jaka_robot_.set_torque_sensor_mode(1);
	/* 设置力控传感器输出方式 */
	this->jaka_robot_.set_ft_ctrl_frame(0);
	return;
}

/**
 * @brief 重新初始化力控传感器
 *
 */
void jaka_robot_driver::reset_torque_sensor(void)
{
	/* 设置力控传感器初始化 */
	this->jaka_robot_.set_compliant_type(1, 0); // 1.初始化; 1.恒力控制
	return;
}

/**
 * @brief 服务回调函数
 * @param request 请求
 * @param response 响应
 */
void jaka_robot_driver::torque_protect_switch_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
	// 打印请求中的bool数据
	RCLCPP_INFO(LOGGER,"torque protect switch: %s", request->data ? "true" : "false");

	this->torque_protect_switch_ = request->data;

	// 设置返回的响应
	response->success = true;
	response->message = "Successfully set bool";
	return;
}

/**
 * @brief 定时力控数据保护函数，防止出现很大的力
 *
 * TODO: 增加告警上报
 *
 */
void jaka_robot_driver::torque_protect_callback(void)
{
	bool over_limit = false;
	double force = 0.0;
	size_t force_idx = 0;
	BOOL in_pos = true;
	RobotStatus robot_status;

	if (false == this->torque_protect_switch_)
	{
		return;
	}

	this->jaka_robot_.is_in_pos(&in_pos);
	this->jaka_robot_.get_robot_status(&robot_status);

	for (size_t i = 0; i < 6; i++)
	{
		if (abs(robot_status.torq_sensor_monitor_data.torque[i]) >
		        this->protect_torque_)
		{
			force_idx = i;
			force = robot_status.torq_sensor_monitor_data.torque[i];
			over_limit = true;
			break;
		}
	}

	if ((over_limit) && (0 == in_pos))
	{
		this->jaka_robot_.motion_abort();
		RCLCPP_ERROR(LOGGER,"torque over limit %lu, %f", force_idx, force);
	}

	return;
}

/**
 * @brief 插枪状态机
*/
void jaka_robot_driver::torque_control_callback()
{
	switch (this->torque_control_.sm_state_)
	{
		case torque_control::TORQUE_CONTROL_IDLE: /* 空闲没有任务 */
		{
			if (this->torque_control_.enabled_)
			{
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_INIT;
				// print all params
				RCLCPP_INFO(LOGGER,"torque_control start, "
				           "force %f depth %f "
				           "acc %f vel %f"
				           "require frame cnt %d"
				           "require deep plan frame cnt %d"
				           "deep thres %f"
				           "force second %f"
				           "joint vel %f acc %f"
				           "inpos thres %f "
						   "depth second %f",
				           this->torque_control_.force_,
				           this->torque_control_.depth_,
				           this->torque_control_.acc_,
				           this->torque_control_.vel_,
				           this->torque_control_.require_frame_cnt_,
				           this->torque_control_.require_deep_plan_frame_cnt_,
				           this->torque_control_.depth_thres_,
				           this->torque_control_.force_second_,
				           this->torque_control_.joint_vel_,
				           this->torque_control_.joint_acc_,
				           this->torque_control_.inpos_thres_,
						   this->torque_control_.depth_second_);
			}
			break;
		}

		case torque_control::TORQUE_CONTROL_INIT: /* 进行初始配置 */
		{
			// 获取力控传感器数值
			RobotStatus status;
			memset(&status, 0, sizeof(RobotStatus));
			int ret = ERR_SUCC;
			ret = this->jaka_robot_.get_robot_status(&status);

			if (ERR_SUCC != ret)
			{
				RCLCPP_ERROR(LOGGER,"TORQUE_CONTROL_INIT get status fail %s",
				            this->mapErr_.at(ret).c_str());
				break;
			}

			/* 遇到意外情况退出 */
			if (0 == status.enabled)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot not enable");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.collision_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in collision");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.emergency_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in emergency");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			RCLCPP_INFO(LOGGER,
			    "TORQUE_CONTROL_INIT torque control using tool id %d, "
			    "tcp x=%f, y=%f, z=%f, "
			    "yaw=%f, pitch=%f, roll=%f\n",
			    status.current_tool_id,
			    status.current_tool_offsets[0], status.current_tool_offsets[1],
			    status.current_tool_offsets[2],
			    status.current_tool_offsets[3], status.current_tool_offsets[4],
			    status.current_tool_offsets[5]);
			RCLCPP_INFO(LOGGER,
			    "TORQUE_CONTROL_INIT torque control payload mass %f, "
			    "center of mass x=%f, y=%f, z=%f\n",
			    status.payload.mass,
			    status.payload.centroid.x,
			    status.payload.centroid.y,
			    status.payload.centroid.z);
			this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_RUN_MOVE;
			break;
		}

		case torque_control::TORQUE_CONTROL_RUN_MOVE: /* 发送移动指令 */
		{
			// 获取力控传感器数值
			RobotStatus status;
			memset(&status, 0, sizeof(RobotStatus));
			int ret = ERR_SUCC;
			ret = this->jaka_robot_.get_robot_status(&status);

			if (ERR_SUCC != ret)
			{
				RCLCPP_ERROR(LOGGER,"TORQUE_CONTROL_RUN_MOVE get status fail %s",
				            this->mapErr_.at(ret).c_str());
				break;
			}

			/* 遇到意外情况退出 */
			if (0 == status.enabled)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot not enable");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.collision_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in collision");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.emergency_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in emergency");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			/* 继续调整 */
			std::vector<double> tmp_cur_tcp;
			tmp_cur_tcp.clear();
			tmp_cur_tcp.resize(6);

			for (size_t i = 0; i < 6; i++)
			{
				tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
			}

			this->torque_control_.ref_base_tcp_ = tmp_cur_tcp;
			RCLCPP_INFO(LOGGER,
			    "TORQUE_CONTROL_RUN_MOVE torque_control, cur ref tcp x=%f, y=%f, z=%f, "
			    "yaw=%f, pitch=%f, roll=%f\n",
			    this->torque_control_.ref_base_tcp_.at(0),
			    this->torque_control_.ref_base_tcp_.at(1),
			    this->torque_control_.ref_base_tcp_.at(2),
			    this->torque_control_.ref_base_tcp_.at(3),
			    this->torque_control_.ref_base_tcp_.at(4),
			    this->torque_control_.ref_base_tcp_.at(5));

			this->soft_charger_interface_->setRefBaseTcp(
			    this->torque_control_.ref_base_tcp_);

			std::vector<double> target_offset; // mm, rad
			target_offset.resize(6);
			target_offset.at(2) = this->torque_control_.depth_ / 1000.0;
			std::vector<double> target;
			target.resize(6);
			this->torque_control_.trans_tcp_to_base(target_offset,
			                                        this->torque_control_.ref_base_tcp_, target);
			RCLCPP_INFO(LOGGER,
			    "TORQUE_CONTROL_RUN_MOVE torque_control, tar_tcp x=%f, y=%f, z=%f, "
			    "yaw=%f, pitch=%f, roll=%f\n",
			    target.at(0), target.at(1), target.at(2),
			    target.at(3), target.at(4), target.at(5));

			// run linear move
			CartesianPose base_move;
			base_move.tran.x = target[0];
			base_move.tran.y = target[1];
			base_move.tran.z = target[2];
			base_move.rpy.rx = target[3];
			base_move.rpy.ry = target[4];
			base_move.rpy.rz = target[5];

			int linear_ret = this->jaka_robot_.linear_move(&base_move, MoveMode::ABS, FALSE,
			                 this->torque_control_.vel_, this->torque_control_.acc_, 0.2, NULL);

			RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_RUN_MOVE jaka robot linear ret %d", linear_ret);
			this->torque_control_.inpos_count_ = 0;
			this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_STOP_MOVE;
			break;
		}

		case torque_control::TORQUE_CONTROL_STOP_MOVE:
		{
			// 获取力控传感器数值
			RobotStatus status;
			memset(&status, 0, sizeof(RobotStatus));
			int ret = ERR_SUCC;
			ret = this->jaka_robot_.get_robot_status(&status);

			if (ERR_SUCC != ret)
			{
				RCLCPP_ERROR(LOGGER,"TORQUE_CONTROL_STOP_MOVE get status fail %s",
				            this->mapErr_.at(ret).c_str());
				break;
			}

			/* 遇到意外情况退出 */
			if (0 == status.enabled)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot not enable");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.collision_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in collision");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.emergency_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in emergency");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			/* 继续调整 */
			std::vector<double> tmp_cur_tcp;
			tmp_cur_tcp.clear();
			tmp_cur_tcp.resize(6);

			for (size_t i = 0; i < 6; i++)
			{
				tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
			}

			static uint32_t log_cur_tcp_count = 0;

			if (0 == (log_cur_tcp_count % 20))
			{
				RCLCPP_INFO_STREAM(LOGGER,"current tcp" << " " << jaka_zu5_driver::vec2str(tmp_cur_tcp));
			}

			log_cur_tcp_count ++;

			/* 增加插入深度的计算 */
			this->torque_control_.cur_insert_depth_ =
			    this->soft_charger_interface_->calculateInsertDepth(tmp_cur_tcp);

			double total_force = 0;
			total_force = sqrt(pow(status.torq_sensor_monitor_data.actTorque[0],
			                       2) + pow(status.torq_sensor_monitor_data.actTorque[1],
			                                2) + pow(status.torq_sensor_monitor_data.actTorque[2], 2));

			// PRINT_DATA;

			/* 2023年10月18日新增, 插入预期深度的80%认为是真实到位 */
			/**
			 * 2024年7月15日 逻辑重新整理
			 *
			 */
			if (status.inpos) /* 到位之后 */
			{
				this->torque_control_.inpos_count_ ++;
				/* 到位阈值 */
				if (this->torque_control_.inpos_thres_ < fabs(
				            this->torque_control_.cur_insert_depth_))
				{
					RCLCPP_INFO(LOGGER,
					    "%s "
					    "frame required %d "
					    "frame cnt %d "
					    "force required %f, "
					    "current total force %f ,"
					    "insert depth %f "
					    "inpos thres %f "
						"depth second %f",
					    __func__,
					    this->torque_control_.require_frame_cnt_,
					    this->torque_control_.frame_cnt_,
					    this->torque_control_.force_,
					    total_force,
					    this->torque_control_.cur_insert_depth_,
					    this->torque_control_.inpos_thres_,
						this->torque_control_.depth_second_);
					// stop linear move
					this->jaka_robot_.motion_abort();
					this->torque_control_.last_tcp_force_.resize(6);

					for (size_t i = 0; i < this->torque_control_.last_tcp_force_.size(); i++)
					{
						this->torque_control_.last_tcp_force_.at(i) =
						    status.torq_sensor_monitor_data.torque[i]; // TODO: 问一下是原始还是真实
					}

					this->soft_charger_interface_->setDeepestBaseTcp(tmp_cur_tcp);
					this->soft_charger_interface_->setDeepestTcpForce(
					    this->torque_control_.last_tcp_force_);
					this->torque_control_.enabled_ = false;
					this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
					break;
				}
				else
				{
					if (5 < this->torque_control_.inpos_count_)
					{
						/* 如果插入深度不满足阈值，开启第二段插枪 */
						if (this->torque_control_.depth_thres_ >
						        this->torque_control_.cur_insert_depth_)
						{
							RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_STOP_MOVE torque control depth %f not satisfy deep thres %f",
							           this->torque_control_.cur_insert_depth_, this->torque_control_.depth_thres_);
							this->torque_control_.inpos_count_ = 0;
							this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_DEEP_PLAN_MOVE;
							break;
						}
					}
				}
			}
			else if (this->torque_control_.force_ < -1 *
			         (status.torq_sensor_monitor_data.actTorque[2]))
			{
				RCLCPP_INFO(LOGGER,
				    "TORQUE_CONTROL_STOP_MOVE Current force is up to %f, "
				    "total force %f, "
				    "frame required %d "
				    "frame cnt %d "
				    "insert depth %f ",
				    this->torque_control_.force_,
				    total_force,
				    this->torque_control_.require_frame_cnt_,
				    this->torque_control_.frame_cnt_,
				    this->torque_control_.cur_insert_depth_);

				if (this->torque_control_.require_frame_cnt_ <=
				        this->torque_control_.frame_cnt_)
				{
					/* 达到了需要的保持次数 */
					// stop linear move
					RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_STOP_MOVE torque_control finish");
					this->jaka_robot_.motion_abort();
					this->torque_control_.last_tcp_force_.resize(6);

					for (size_t i = 0; i < this->torque_control_.last_tcp_force_.size(); i++)
					{
						this->torque_control_.last_tcp_force_.at(i) =
						    status.torq_sensor_monitor_data.actTorque[i];
					}

					this->soft_charger_interface_->setDeepestBaseTcp(tmp_cur_tcp);
					this->soft_charger_interface_->setDeepestTcpForce(
					    this->torque_control_.last_tcp_force_);

					/* 如果插入深度不满足阈值，开启第二段插枪 */
					if (this->torque_control_.depth_thres_ >
					        this->torque_control_.cur_insert_depth_)
					{
						RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_STOP_MOVE torque control depth %f not satisfy deep thres %f",
						           this->torque_control_.cur_insert_depth_, this->torque_control_.depth_thres_);
						this->torque_control_.inpos_count_ = 0;
						this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_DEEP_PLAN_MOVE;
						break;
					}

					this->torque_control_.enabled_ = false;
					this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
					break;
				}
				else
				{
					/* 增加 */
					this->torque_control_.frame_cnt_++;
					RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_STOP_MOVE add frame cnt %d\n",
					           this->torque_control_.frame_cnt_);
				}
			}
			else
			{
				if ((this->torque_control_.frame_cnt_ > 0)
				        && (this->torque_control_.require_frame_cnt_ >
				            this->torque_control_.frame_cnt_))
				{
					/* 如果力小于了规定值, 将计数器清零 */
					RCLCPP_INFO(LOGGER,
					    "TORQUE_CONTROL_STOP_MOVE Current force do not satisfy %f, "
					    "total force %f, "
					    "frame required %d "
					    "frame cnt %d "
					    " clear cnt",
					    this->torque_control_.force_,
					    total_force,
					    this->torque_control_.require_frame_cnt_,
					    this->torque_control_.frame_cnt_);
					this->torque_control_.frame_cnt_ = 0;
				}
				else
				{
					// 其他未被覆盖的状态，这个你需要自己根据实际业务场景进行处理
					//TODO: 超时处理?
				}
			}

			this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_STOP_MOVE;
			break;
		}

		case torque_control::TORQUE_CONTROL_DEEP_PLAN_MOVE:
		{
			/* 第一步的调整, 首先收集足够的数据, 获取5帧的数据 */
			RobotStatus status;
			errno_t ret = ERR_SUCC;
			memset(&status, 0, sizeof(RobotStatus));
			ret = this->jaka_robot_.get_robot_status(&status);

			if (ERR_SUCC != ret)
			{
				RCLCPP_ERROR(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_MOVE get robotstatus failed %d", ret);
				break;
			}

			/* 遇到意外情况退出 */
			if (0 == status.enabled)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot not enable");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.collision_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in collision");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.emergency_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in emergency");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (this->torque_control_.require_deep_plan_frame_cnt_ >
			        this->torque_control_.torque_array_.size())
			{
				/* 把当前力大小获取出来 */
				std::vector<double> tmp_origin_tcp_torque;
				tmp_origin_tcp_torque.clear();
				tmp_origin_tcp_torque.resize(6);

				for (size_t i = 0; i < 6; i++)
				{
					tmp_origin_tcp_torque.at(i) = status.torq_sensor_monitor_data.actTorque[i];
				}

				this->torque_control_.torque_array_.push_back(tmp_origin_tcp_torque);
				RCLCPP_INFO(LOGGER,
				    "TORQUE_CONTROL_DEEP_PLAN_MOVE cur force data size %lu",
				    this->torque_control_.torque_array_.size());
				// PRINT_DATA;
				break;
			}

			/* 继续调整 */
			std::vector<double> tmp_cur_tcp;
			tmp_cur_tcp.clear();
			tmp_cur_tcp.resize(6);

			for (size_t i = 0; i < 6; i++)
			{
				tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
			}

			// PRINT_DATA;

			/* 当数据已经足够多，调用算法接口 */
			/* 增加插入深度的计算 */
			this->torque_control_.cur_insert_depth_ =
			    this->soft_charger_interface_->calculateInsertDepth(tmp_cur_tcp);
			RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_MOVE deepplan cur insert depth %f",
			           this->torque_control_.cur_insert_depth_);
			std::vector<double> adjustment;
			this->soft_charger_interface_->deepPlan(this->torque_control_.cur_insert_depth_,
			                                        this->torque_control_.torque_array_,
			                                        adjustment);

			RCLCPP_INFO(LOGGER,
			    "TORQUE_CONTROL_DEEP_PLAN_MOVE algo adjustment %f %f %f %f %f %f\n",
			    adjustment.at(0), adjustment.at(1), adjustment.at(2),
			    adjustment.at(3), adjustment.at(4), adjustment.at(5));

			adjustment.at(0) = 0.0;
			adjustment.at(1) = 0.0;
			adjustment.at(2) = 0.0;

			// 3. 进行transpose运算 获取基座TCP
			std::vector<double> target_tcp;
			target_tcp.clear();
			target_tcp.resize(6);
			this->torque_control_.trans_tcp_to_base(adjustment, tmp_cur_tcp, target_tcp);

			RCLCPP_INFO_STREAM(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_MOVE trans target tcp pose" <<
			                  jaka_zu5_driver::vec2str(target_tcp));

			CartesianPose target_tcp_pose;
			target_tcp_pose.tran.x = target_tcp.at(0);
			target_tcp_pose.tran.y = target_tcp.at(1);
			target_tcp_pose.tran.z = target_tcp.at(2);
			target_tcp_pose.rpy.rx = target_tcp.at(3);
			target_tcp_pose.rpy.ry = target_tcp.at(4);
			target_tcp_pose.rpy.rz = target_tcp.at(5);

			// 2.1 当前的关节角度
			JointValue ref_pos;

			for (size_t j = 0; j < 6; j++)
			{
				ref_pos.jVal[j] = status.joint_position[j];
			}

			RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_MOVE ref joint pose %f %f %f %f %f %f\n",
			           ref_pos.jVal[0], ref_pos.jVal[1], ref_pos.jVal[2],
			           ref_pos.jVal[3], ref_pos.jVal[4], ref_pos.jVal[5]);

			// 4. 进行关节逆运算, 获取关节角度
			JointValue tar_joint_pos;

			this->jaka_robot_.kine_inverse(&ref_pos, &target_tcp_pose, &tar_joint_pos);

			RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_MOVE tar joint pose %f %f %f %f %f %f\n",
			           tar_joint_pos.jVal[0], tar_joint_pos.jVal[1], tar_joint_pos.jVal[2],
			           tar_joint_pos.jVal[3], tar_joint_pos.jVal[4], tar_joint_pos.jVal[5]);

			// 5. 进行关节运动
			errno_t joint_move_ret = ERR_SUCC;
			double tol = 0.2;
			OptionalCond *option_cond = nullptr;
			joint_move_ret = jaka_robot_.joint_move(&tar_joint_pos, MoveMode::ABS, FALSE,
			                                        this->torque_control_.joint_vel_,
			                                        this->torque_control_.joint_acc_, tol, option_cond);
			RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_MOVE torque control second joint move %d\n",
			           joint_move_ret);
			this->torque_control_.torque_array_.clear();
			this->torque_control_.decoupling_torque_array_.clear();
			this->torque_control_.decoupling_tcp_array_.clear();
			this->torque_control_.inpos_count_ = 0;
			this->torque_control_.sm_state_ =
			    torque_control::TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT;
			break;
		}

		case torque_control::TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT:
		{
			// 获取力控传感器数值
			RobotStatus status;
			memset(&status, 0, sizeof(RobotStatus));
			errno_t ret = ERR_SUCC;
			ret = this->jaka_robot_.get_robot_status(&status);

			if (ERR_SUCC != ret)
			{
				RCLCPP_ERROR(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT get status fail %s",
				            this->mapErr_.at(ret).c_str());
				break;
			}

			/* 遇到意外情况退出 */
			if (0 == status.enabled)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot not enable");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.collision_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in collision");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.emergency_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in emergency");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (status.inpos)
			{
				this->torque_control_.inpos_count_ ++;
			}
			else
			{
				this->torque_control_.inpos_count_ = 0;
			}

			/* 2023年12月21日新增 */
			if (3 < this->torque_control_.inpos_count_) /* 位置到达 */
			{
				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT torque control deep plan target arrive");
				std::vector<double> adjustment;
				this->soft_charger_interface_->decouplingSearch(
				    this->torque_control_.decoupling_torque_array_,
				    this->torque_control_.decoupling_tcp_array_, adjustment);

				RCLCPP_INFO_STREAM(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT algo adjustment " <<
				                  jaka_zu5_driver::vec2str(adjustment));

				adjustment.at(0) = 0.0;
				adjustment.at(1) = 0.0;
				adjustment.at(2) = 0.0;

				/* 继续调整 */
				std::vector<double> tmp_cur_tcp;
				tmp_cur_tcp.clear();
				tmp_cur_tcp.resize(6);

				for (size_t i = 0; i < 6; i++)
				{
					tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
				}

				RCLCPP_INFO_STREAM(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT cur tcp pose");
				// PRINT_DATA;

				// 3. 进行transpose运算 获取基座TCP
				std::vector<double> target_tcp;
				target_tcp.clear();
				target_tcp.resize(6);
				this->torque_control_.trans_tcp_to_base(adjustment, tmp_cur_tcp, target_tcp);
				RCLCPP_INFO_STREAM(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT trans target tcp pose "
				                  << jaka_zu5_driver::vec2str(target_tcp));

				CartesianPose target_tcp_pose;
				target_tcp_pose.tran.x = target_tcp.at(0);
				target_tcp_pose.tran.y = target_tcp.at(1);
				target_tcp_pose.tran.z = target_tcp.at(2);
				target_tcp_pose.rpy.rx = target_tcp.at(3);
				target_tcp_pose.rpy.ry = target_tcp.at(4);
				target_tcp_pose.rpy.rz = target_tcp.at(5);

				// 2.1 当前的关节角度
				JointValue ref_pos;

				for (size_t j = 0; j < 6; j++)
				{
					ref_pos.jVal[j] = status.joint_position[j];
				}

				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT ref joint pose %f %f %f %f %f %f\n",
				           ref_pos.jVal[0], ref_pos.jVal[1], ref_pos.jVal[2],
				           ref_pos.jVal[3], ref_pos.jVal[4], ref_pos.jVal[5]);

				// 4. 进行关节逆运算, 获取关节角度
				JointValue tar_joint_pos;

				this->jaka_robot_.kine_inverse(&ref_pos, &target_tcp_pose, &tar_joint_pos);

				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT tar joint pose %f %f %f %f %f %f\n",
				           tar_joint_pos.jVal[0], tar_joint_pos.jVal[1], tar_joint_pos.jVal[2],
				           tar_joint_pos.jVal[3], tar_joint_pos.jVal[4], tar_joint_pos.jVal[5]);

				// 5. 进行关节运动
				errno_t joint_move_ret = ERR_SUCC;
				double tol = 0.2;
				OptionalCond *option_cond = nullptr;
				joint_move_ret = jaka_robot_.joint_move(&tar_joint_pos, MoveMode::ABS, FALSE,
				                                        this->torque_control_.joint_vel_,
				                                        this->torque_control_.joint_acc_, tol, option_cond);
				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT torque control joint move %d\n",
				           joint_move_ret);
				this->torque_control_.decoupling_torque_array_.clear();
				this->torque_control_.decoupling_tcp_array_.clear();
				this->torque_control_.inpos_count_ = 0;
				this->torque_control_.sm_state_ =
				    torque_control::TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT;
				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT deep plan finish");
				break;
			}
			else /* 没有到位的过程中 */
			{
				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT collect data for further adjust");
				/* 运动过程中收集 actual_torque 和 tcp_pose */
				std::vector<double> tmp_actual_torque;
				std::vector<double> tmp_tcp_position;
				tmp_actual_torque.resize(6);
				tmp_tcp_position.resize(6);

				for (size_t i = 0; i < 6; i++)
				{
					tmp_actual_torque.at(i) = status.torq_sensor_monitor_data.actTorque[i];
					tmp_tcp_position.at(i) = status.cartesiantran_position[i];
				}

				this->torque_control_.decoupling_torque_array_.push_back(tmp_actual_torque);
				this->torque_control_.decoupling_tcp_array_.push_back(tmp_tcp_position);
				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT collect data frame %lu",
				           this->torque_control_.decoupling_torque_array_.size());
				// PRINT_DATA;
				this->torque_control_.sm_state_ =
				    torque_control::TORQUE_CONTROL_DEEP_PLAN_DATA_COLLECT;
				break;
			}
		}

		case torque_control::TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT:
		{
			// 获取力控传感器数值
			RobotStatus status;
			memset(&status, 0, sizeof(RobotStatus));
			errno_t ret = ERR_SUCC;
			ret = this->jaka_robot_.get_robot_status(&status);

			if (ERR_SUCC != ret)
			{
				RCLCPP_ERROR(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT get status fail %s",
				            this->mapErr_.at(ret).c_str());
				break;
			}

			/* 遇到意外情况退出 */
			if (0 == status.enabled)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot not enable");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.collision_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in collision");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.emergency_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in emergency");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (status.inpos)
			{
				this->torque_control_.inpos_count_ ++;
			}
			else
			{
				this->torque_control_.inpos_count_ = 0;
			}

			/* 2023年12月21日新增 */
			if (3 < this->torque_control_.inpos_count_) /* 位置到达 */
			{
				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT torque control deep plan target arrive");
				/* 位置到达之后先插一次枪 */
				std::vector<double> target_offset; // mm, rad
				target_offset.resize(6);
				target_offset.at(2) = this->torque_control_.depth_second_ / 1000.0;

				/* 继续调整 */
				std::vector<double> tmp_cur_tcp;
				tmp_cur_tcp.clear();
				tmp_cur_tcp.resize(6);

				for (size_t i = 0; i < 6; i++)
				{
					tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
				}

				// PRINT_DATA;
				RCLCPP_INFO(LOGGER,"set ref base tcp");
				this->soft_charger_interface_->setRefBaseTcp(tmp_cur_tcp);

				std::vector<double> target;
				target.resize(6);
				this->torque_control_.trans_tcp_to_base(target_offset, tmp_cur_tcp, target);
				RCLCPP_INFO_STREAM(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT torque_control, tar_tcp "
				                  << jaka_zu5_driver::vec2str(target));

				// run linear move
				CartesianPose base_move;
				base_move.tran.x = target.at(0);
				base_move.tran.y = target.at(1);
				base_move.tran.z = target.at(2);
				base_move.rpy.rx = target.at(3);
				base_move.rpy.ry = target.at(4);
				base_move.rpy.rz = target.at(5);

				this->jaka_robot_.linear_move(&base_move, MoveMode::ABS, FALSE,
				                              this->torque_control_.vel_, this->torque_control_.acc_, 0.2, NULL);
				this->torque_control_.frame_cnt_ = 0;
				this->torque_control_.inpos_count_ = 0;
				this->torque_control_.sm_state_ =
				    torque_control::TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE;
				RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT deep plan final insert finish");
				break;
			}
			else /* 没有到位的过程中 */
			{
				static uint32_t not_inpos_count = 0;

				if (0 == (not_inpos_count % 100))
				{
					RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_FINAL_INSERT wait deep plan final insert finish...");
				}

				// PRINT_DATA;

				not_inpos_count ++;
				break;
			}
		}

		case torque_control::TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE:
		{
			// 获取力控传感器数值
			RobotStatus status;
			memset(&status, 0, sizeof(RobotStatus));
			int ret = ERR_SUCC;
			ret = this->jaka_robot_.get_robot_status(&status);

			if (ERR_SUCC != ret)
			{
				RCLCPP_ERROR(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE get robot status fail %s",
				            this->mapErr_.at(ret).c_str());
				break;
			}

			/* 遇到意外情况退出 */
			if (0 == status.enabled)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot not enable");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.collision_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in collision");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			if (0 != status.emergency_stop)
			{
				RCLCPP_ERROR(LOGGER,"encounter error, robot in emergency");
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}

			/* 继续调整 */
			std::vector<double> tmp_cur_tcp;
			tmp_cur_tcp.clear();
			tmp_cur_tcp.resize(6);

			for (size_t i = 0; i < 6; i++)
			{
				tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
			}

			/* 增加插入深度的计算 */
			this->torque_control_.cur_insert_depth_ =
			    this->soft_charger_interface_->calculateInsertDepth(tmp_cur_tcp);

			double total_force = 0;
			total_force = sqrt(pow(status.torq_sensor_monitor_data.actTorque[0],
			                       2) + pow(status.torq_sensor_monitor_data.actTorque[1],
			                                2) + pow(status.torq_sensor_monitor_data.actTorque[2], 2));

			/* 2023年10月18日新增, 插入预期深度的80%认为是真实到位 */
			if (status.inpos)
			{
				this->torque_control_.inpos_count_ ++;
			}
			else
			{
				this->torque_control_.inpos_count_ = 0;
			}

			if (3 < this->torque_control_.inpos_count_) /* 位置到达 */
			{
				this->torque_control_.inpos_count_ = 0;
				RCLCPP_INFO(LOGGER,
				    "TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE Torque control target arrive %d, "
				    "frame required %d "
				    "frame cnt %d "
				    "force required %f, "
				    "current total force %f ,"
				    "insert depth %f ",
				    status.inpos,
				    this->torque_control_.require_frame_cnt_,
				    this->torque_control_.frame_cnt_,
				    this->torque_control_.force_second_,
				    total_force,
				    this->torque_control_.cur_insert_depth_);
				// PRINT_DATA;
				// stop linear move
				this->jaka_robot_.motion_abort();
				this->torque_control_.last_tcp_force_.resize(6);

				for (size_t i = 0; i < this->torque_control_.last_tcp_force_.size(); i++)
				{
					this->torque_control_.last_tcp_force_.at(i) =
					    status.torq_sensor_monitor_data.torque[i]; // TODO: 问一下是原始还是真实
				}
				this->soft_charger_interface_->setDeepestBaseTcp(tmp_cur_tcp);
				this->soft_charger_interface_->setDeepestTcpForce(
				    this->torque_control_.last_tcp_force_);
				this->torque_control_.enabled_ = false;
				this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
				break;
			}
			else if (-1 * (status.torq_sensor_monitor_data.actTorque[2]) >
			         this->torque_control_.force_second_)
			{
				RCLCPP_INFO(LOGGER,
				    "TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE Current force is up to %f, "
				    "total force %f, "
				    "frame required %d "
				    "frame cnt %d "
				    "insert depth %f ",
				    this->torque_control_.force_second_,
				    total_force,
				    this->torque_control_.require_frame_cnt_,
				    this->torque_control_.frame_cnt_,
				    this->torque_control_.cur_insert_depth_);
				// PRINT_DATA;

				if (this->torque_control_.require_frame_cnt_ <=
				        this->torque_control_.frame_cnt_)
				{
					/* 达到了需要的保持次数 */
					// stop linear move
					RCLCPP_INFO(LOGGER,
					    "TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE torque_control finish");
					this->jaka_robot_.motion_abort();
					this->torque_control_.last_tcp_force_.resize(6);

					for (size_t i = 0; i < this->torque_control_.last_tcp_force_.size(); i++)
					{
						this->torque_control_.last_tcp_force_.at(i) =
						    status.torq_sensor_monitor_data.actTorque[i];
					}

					this->soft_charger_interface_->setDeepestBaseTcp(tmp_cur_tcp);
					this->soft_charger_interface_->setDeepestTcpForce(
					    this->torque_control_.last_tcp_force_);
					this->torque_control_.enabled_ = false;
					this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;
					break;
				}
				else
				{
					/* 增加 */
					this->torque_control_.frame_cnt_++;
					RCLCPP_INFO(LOGGER,"TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE add second frame cnt %d\n",
					           this->torque_control_.frame_cnt_);
					// PRINT_DATA;
				}
			}
			else
			{
				// PRINT_DATA;
				if ((this->torque_control_.require_frame_cnt_ >
				        this->torque_control_.frame_cnt_)
				        && (0 != this->torque_control_.frame_cnt_))
				{
					/* 如果力小于了规定值, 将计数器清零 */
					RCLCPP_INFO(LOGGER,
					    "TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE Current force do not satisfy %f, "
					    "total force %f, "
					    "frame required %d "
					    "frame cnt %d "
					    " clear cnt",
					    this->torque_control_.force_second_,
					    total_force,
					    this->torque_control_.require_frame_cnt_,
					    this->torque_control_.frame_cnt_);
					this->torque_control_.frame_cnt_ = 0;
				}
			}

			this->torque_control_.sm_state_ =
			    torque_control::TORQUE_CONTROL_DEEP_PLAN_STOP_MOVE;
			break;
		}

		default:
		{
			static uint32_t status_error_count = 0;

			if (0 == (status_error_count % 10))
			{
				RCLCPP_ERROR(LOGGER,"torque_control_callback status error %d\n",
				            this->torque_control_.sm_state_);
			}

			status_error_count ++;
		}
	}

	return;
}

/**
 * @brief 进行调整
 *
 */
void jaka_robot_driver::torque_adjust_callback()
{
	// 已经结束退出
	if (false == (this->torque_adjust_.enabled_))
	{
		return;
	}

	// 未设置初始力 直接退出
	if (false == this->torque_adjust_.set_payload_)
	{
		RCLCPP_ERROR(LOGGER,"torque_adjust payload not set");
		this->torque_adjust_.enabled_ = false;
		return;
	}

	RobotStatus status;
	this->jaka_robot_.get_robot_status(&status);

	// 检测到异常退出
	if ((0 != status.errcode)
	        || (0 != status.emergency_stop)
	        || (0 == status.enabled))
	{
		RCLCPP_ERROR(LOGGER,"error detected, torque adjust exit");
		this->torque_adjust_.enabled_ = false;
		return;
	}

	// 1. 获取当前力的大小
	std::vector<double> origin_tcp_torque;
	origin_tcp_torque.clear();
	origin_tcp_torque.resize(6);

	for (size_t i = 0; i < 6; i++)
	{
		origin_tcp_torque.at(i) = status.torq_sensor_monitor_data.torque[i];
	}

	// 当前只判断inpos, 是否需要判断 programstate 和 dragstate ?
	if (1 == status.inpos)
	{
		/* 获取5帧的数据, 如果不足5帧数据 */
		if (5 > this->torque_adjust_.force_array_.size())
		{
			this->torque_adjust_.force_array_.push_back(origin_tcp_torque);
			RCLCPP_INFO(LOGGER,
			    "current torque %f %f %f %f %f %f, cur force data size %lu\n",
			    origin_tcp_torque.at(0), origin_tcp_torque.at(1),
			    origin_tcp_torque.at(2), origin_tcp_torque.at(3),
			    origin_tcp_torque.at(4), origin_tcp_torque.at(5),
			    this->torque_adjust_.force_array_.size());
		}
		else
		{
			// 2. 发送给算法 获取调整角度
			std::vector<double> adjustment;
			adjustment.clear();
			adjustment.resize(6);
			this->soft_charger_interface_->plan(this->torque_adjust_.force_array_,
			                                    adjustment);

			RCLCPP_INFO(LOGGER,
			    "algo adjustment %f %f %f %f %f %f\n",
			    adjustment.at(0), adjustment.at(1), adjustment.at(2),
			    adjustment.at(3), adjustment.at(4), adjustment.at(5));
			adjustment.at(0) = 0.0;
			adjustment.at(1) = 0.0;
			adjustment.at(2) = 0.0;

			if (abs(adjustment.at(3)) > this->torque_adjust_.min_angle_
			        || abs(adjustment.at(4)) > this->torque_adjust_.min_angle_)
			{
				/* 继续调整 */
				CartesianPose tcp_position;
				this->jaka_robot_.get_tcp_position(&tcp_position);
				std::vector<double> cur_tcp;
				cur_tcp.clear();
				cur_tcp.resize(6);
				cur_tcp.at(0) = tcp_position.tran.x;
				cur_tcp.at(1) = tcp_position.tran.y;
				cur_tcp.at(2) = tcp_position.tran.z;
				cur_tcp.at(3) = tcp_position.rpy.rx;
				cur_tcp.at(4) = tcp_position.rpy.ry;
				cur_tcp.at(5) = tcp_position.rpy.rz;

				RCLCPP_INFO(LOGGER,"cur tcp pose %f %f %f %f %f %f\n",
				           cur_tcp.at(0), cur_tcp.at(1), cur_tcp.at(2),
				           cur_tcp.at(3), cur_tcp.at(4), cur_tcp.at(5));

				// 3. 进行transpose运算 获取基座TCP
				std::vector<double> target_tcp;
				target_tcp.clear();
				target_tcp.resize(6);
				this->torque_control_.trans_tcp_to_base(adjustment, cur_tcp, target_tcp);

				RCLCPP_INFO(LOGGER,"target tcp pose %f %f %f %f %f %f\n",
				           target_tcp.at(0), target_tcp.at(1), target_tcp.at(2),
				           target_tcp.at(3), target_tcp.at(4), target_tcp.at(5));

				CartesianPose target_tcp_pose;
				target_tcp_pose.tran.x = target_tcp.at(0);
				target_tcp_pose.tran.y = target_tcp.at(1);
				target_tcp_pose.tran.z = target_tcp.at(2);
				target_tcp_pose.rpy.rx = target_tcp.at(3);
				target_tcp_pose.rpy.ry = target_tcp.at(4);
				target_tcp_pose.rpy.rz = target_tcp.at(5);

				// 2.1 当前的关节角度
				JointValue ref_pos;

				for (size_t j = 0; j < 6; j++)
				{
					ref_pos.jVal[j] = status.joint_position[j];
				}

				RCLCPP_INFO(LOGGER,"ref joint pose %f %f %f %f %f %f\n",
				           ref_pos.jVal[0], ref_pos.jVal[1], ref_pos.jVal[2],
				           ref_pos.jVal[3], ref_pos.jVal[4], ref_pos.jVal[5]);

				// 4. 进行关节逆运算, 获取关节角度
				JointValue tar_joint_pos;

				this->jaka_robot_.kine_inverse(&ref_pos, &target_tcp_pose, &tar_joint_pos);

				RCLCPP_INFO(LOGGER,"tar joint pose %f %f %f %f %f %f\n",
				           tar_joint_pos.jVal[0], tar_joint_pos.jVal[1], tar_joint_pos.jVal[2],
				           tar_joint_pos.jVal[3], tar_joint_pos.jVal[4], tar_joint_pos.jVal[5]);

				// 5. 进行关节运动
				int ret = 0;
				double tol = 0.5;
				OptionalCond *option_cond = nullptr;
				ret = jaka_robot_.joint_move(&tar_joint_pos, MoveMode::ABS, FALSE,
				                             this->torque_adjust_.vel_,
				                             this->torque_adjust_.acc_, tol, option_cond);
				RCLCPP_INFO(LOGGER,"torque adjust joint move %d\n", ret);
				this->torque_adjust_.force_array_.clear();
			}
			else
			{
				/* 结束调整 */
				// 如果角度小于阈值, enable, set_payload 置为false, 返回
				RCLCPP_INFO(LOGGER,"torque adjust satisfy min adjust angle %f\n",
				           this->torque_adjust_.min_angle_);
				this->jaka_robot_.motion_abort();
				this->torque_adjust_.enabled_ = false;
			}
		}
	}
	else
	{
		static uint32_t wait_count = 0;

		if (0 == (wait_count % 10))
		{
			RCLCPP_INFO(LOGGER,"wait cur adjustment to finish\n");
		}

		wait_count ++;
	}

	return;
}

/**
 * @brief 拔枪调整
 *
 */
void jaka_robot_driver::torque_extract_adjust_callback()
{
	// 已经结束退出
	if (false == (this->torque_extract_adjust_.enabled_))
	{
		return;
	}

	// 未设置插枪到底力
	if (0 == this->torque_control_.last_tcp_force_.size())
	{
		RCLCPP_ERROR(LOGGER,"torque_extract_adjust last tcp force not set");
		this->torque_extract_adjust_.enabled_ = false;
		return;
	}

	RobotStatus status;
	this->jaka_robot_.get_robot_status(&status);

	// 检测到异常退出
	if ((0 != status.errcode)
	        || (0 != status.emergency_stop)
	        || (0 == status.enabled))
	{
		RCLCPP_ERROR(LOGGER,"error detected, torque extract adjust exit");
		this->torque_extract_adjust_.enabled_ = false;
		return;
	}

	// 1. 获取当前力的大小
	std::vector<double> origin_tcp_torque;
	origin_tcp_torque.clear();
	origin_tcp_torque.resize(6);

	for (size_t i = 0; i < 6; i++)
	{
		origin_tcp_torque.at(i) = status.torq_sensor_monitor_data.torque[i];
	}

	// 当前只判断inpos, 是否需要判断 programstate 和 dragstate ?
	if (1 == status.inpos)
	{
		/* 获取5帧的数据, 如果不足5帧数据 */
		if (5 > this->torque_extract_adjust_.force_array_.size())
		{
			this->torque_extract_adjust_.force_array_.push_back(origin_tcp_torque);
			RCLCPP_INFO(LOGGER,
			    "current torque %f %f %f %f %f %f, cur force data size %lu\n",
			    origin_tcp_torque.at(0), origin_tcp_torque.at(1),
			    origin_tcp_torque.at(2), origin_tcp_torque.at(3),
			    origin_tcp_torque.at(4), origin_tcp_torque.at(5),
			    this->torque_extract_adjust_.force_array_.size());
		}
		else
		{
			// 2. 发送给算法 获取调整角度
			CartesianPose tcp_position;
			this->jaka_robot_.get_tcp_position(&tcp_position);
			std::vector<double> cur_tcp;
			cur_tcp.clear();
			cur_tcp.resize(6);
			cur_tcp.at(0) = tcp_position.tran.x;
			cur_tcp.at(1) = tcp_position.tran.y;
			cur_tcp.at(2) = tcp_position.tran.z;
			cur_tcp.at(3) = tcp_position.rpy.rx;
			cur_tcp.at(4) = tcp_position.rpy.ry;
			cur_tcp.at(5) = tcp_position.rpy.rz;

			std::vector<double> adjustment;
			adjustment.clear();
			adjustment.resize(6);
			this->soft_charger_interface_->planExtraction(
			    this->torque_extract_adjust_.force_array_, cur_tcp, adjustment);

			RCLCPP_INFO(LOGGER,
			    "algo adjustment %f %f %f %f %f %f\n",
			    adjustment.at(0), adjustment.at(1), adjustment.at(2),
			    adjustment.at(3), adjustment.at(4), adjustment.at(5));

			adjustment.at(0) = 0.0;
			adjustment.at(1) = 0.0;
			adjustment.at(2) = 0.0;

			if (abs(adjustment.at(3)) > this->torque_extract_adjust_.min_angle_
			        || abs(adjustment.at(4)) > this->torque_extract_adjust_.min_angle_)
			{
				/* 继续调整 */
				RCLCPP_INFO(LOGGER,"cur tcp pose %f %f %f %f %f %f\n",
				           cur_tcp.at(0), cur_tcp.at(1), cur_tcp.at(2),
				           cur_tcp.at(3), cur_tcp.at(4), cur_tcp.at(5));

				// 3. 进行transpose运算 获取基座TCP
				std::vector<double> target_tcp;
				target_tcp.clear();
				target_tcp.resize(6);
				this->torque_control_.trans_tcp_to_base(adjustment, cur_tcp, target_tcp);

				RCLCPP_INFO(LOGGER,"target tcp pose %f %f %f %f %f %f\n",
				           target_tcp.at(0), target_tcp.at(1), target_tcp.at(2),
				           target_tcp.at(3), target_tcp.at(4), target_tcp.at(5));

				CartesianPose target_tcp_pose;
				target_tcp_pose.tran.x = target_tcp.at(0);
				target_tcp_pose.tran.y = target_tcp.at(1);
				target_tcp_pose.tran.z = target_tcp.at(2);
				target_tcp_pose.rpy.rx = target_tcp.at(3);
				target_tcp_pose.rpy.ry = target_tcp.at(4);
				target_tcp_pose.rpy.rz = target_tcp.at(5);

				// 2.1 当前的关节角度
				JointValue ref_pos;

				for (size_t j = 0; j < 6; j++)
				{
					ref_pos.jVal[j] = status.joint_position[j];
				}

				RCLCPP_INFO(LOGGER,"ref joint pose %f %f %f %f %f %f\n",
				           ref_pos.jVal[0], ref_pos.jVal[1], ref_pos.jVal[2],
				           ref_pos.jVal[3], ref_pos.jVal[4], ref_pos.jVal[5]);

				// 4. 进行关节逆运算, 获取关节角度
				JointValue tar_joint_pos;

				this->jaka_robot_.kine_inverse(&ref_pos, &target_tcp_pose, &tar_joint_pos);

				RCLCPP_INFO(LOGGER,"tar joint pose %f %f %f %f %f %f\n",
				           tar_joint_pos.jVal[0], tar_joint_pos.jVal[1], tar_joint_pos.jVal[2],
				           tar_joint_pos.jVal[3], tar_joint_pos.jVal[4], tar_joint_pos.jVal[5]);

				// 5. 进行关节运动
				int ret = 0;
				double tol = 0.5;
				OptionalCond *option_cond = nullptr;
				ret = jaka_robot_.joint_move(&tar_joint_pos, MoveMode::ABS, FALSE,
				                             this->torque_extract_adjust_.vel_,
				                             this->torque_extract_adjust_.acc_, tol, option_cond);
				RCLCPP_INFO(LOGGER,"torque extract adjust joint move %d\n", ret);
				this->torque_extract_adjust_.force_array_.clear();
			}
			else
			{
				/* 结束调整 */
				// 如果角度小于阈值, enable, set_payload 置为false, 返回
				RCLCPP_INFO(LOGGER,"torque adjust satisfy min adjust angle %f\n",
				           this->torque_extract_adjust_.min_angle_);
				this->jaka_robot_.motion_abort();
				this->torque_extract_adjust_.enabled_ = false;
			}
		}
	}
	else
	{
		static uint32_t wait_adjust_finish_count = 0;

		if (0 == (wait_adjust_finish_count % 10))
		{
			RCLCPP_ERROR(LOGGER,"extract wait cur adjustment to finish\n");
		}

		wait_adjust_finish_count ++;
	}
	return;
}

/**
 * @brief 控制机器人进入力矩调整模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::torque_adjust_control_callback(const
        std::shared_ptr<shared_message::srv::SetTorqueAdjust::Request> request,
        std::shared_ptr<shared_message::srv::SetTorqueAdjust::Response> response)
{

	this->torque_adjust_.vel_ = request->joint_vel;
	this->torque_adjust_.acc_ = request->joint_acc;
	this->torque_adjust_.min_angle_ = request->angle_min;
	this->torque_adjust_.seq_ = request->seq;
	this->torque_adjust_.enabled_ = true;
	this->torque_adjust_.force_array_.clear();

	RCLCPP_INFO(LOGGER,"torque_adjust seq %u, car_type %d\n",
	           this->torque_adjust_.seq_, request->car_type);
	this->soft_charger_interface_->reset();
	this->soft_charger_interface_->setCarType(request->car_type);

	response->ret = 1;
	response->message = "torque adjust has been set";

	return true;
}

/**
 * @brief 控制机器人进入力矩调整模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::torque_adjust_extract_callback(const
        std::shared_ptr<shared_message::srv::SetTorqueExtractAdjust::Request> request,
        std::shared_ptr<shared_message::srv::SetTorqueExtractAdjust::Response> response)
{

	this->torque_extract_adjust_.vel_ = request->joint_vel;
	this->torque_extract_adjust_.acc_ = request->joint_acc;
	this->torque_extract_adjust_.min_angle_ = request->angle_min;
	this->torque_extract_adjust_.seq_ = request->seq;
	this->torque_extract_adjust_.enabled_ = true;
	this->torque_extract_adjust_.force_array_.clear();

	RCLCPP_INFO(LOGGER,"torque_extract_adjust seq %u, car_type %d\n",
	           this->torque_extract_adjust_.seq_, request->car_type);
	this->soft_charger_interface_->extraReset(); // TODO:跟算法确认是不是要reset
	this->soft_charger_interface_->setCarType(request->car_type);

	response->ret = 1;
	response->message = "torque extract adjust has been set";

	return true;
}

/**
 * @brief 重置力控力矩
 *
 * @param req 请求
 * @param res 回复
 * @return true 固定返回true
 */
bool jaka_robot_driver::torque_reset_callback(const
        std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	bool ret = false;
	RobotStatus robot_status;

	while (false == ret)
	{
		memset(&robot_status, 0, sizeof(RobotStatus));
		this->jaka_robot_.get_robot_status(&robot_status);
		// 将力矩给到算法接口
		// 1. 获取当前力的大小
		std::vector<double> cur_torque;
		cur_torque.clear();
		cur_torque.resize(6);

		for (size_t i = 0; i < 6; i++)
		{
			cur_torque.at(i) = robot_status.torq_sensor_monitor_data.torque[i];
		}

		static uint32_t torque_print_count = 0;

		if (0 == (torque_print_count % 10))
		{
			RCLCPP_INFO(LOGGER,"current torque %f %f %f %f %f %f, zero force sensor\n",
			           cur_torque.at(0), cur_torque.at(1), cur_torque.at(2),
			           cur_torque.at(3), cur_torque.at(4), cur_torque.at(5));
		}

		torque_print_count ++;

		ret = this->soft_charger_interface_->zeroForceSensor(cur_torque);

		static uint32_t reset_result_count = 0;

		if (0 == (reset_result_count % 10))
		{
			RCLCPP_INFO(LOGGER,"torque reset executed %d\n", ret);
		}

		reset_result_count ++;
		if (true == ret)
		{
			this->torque_adjust_.set_payload_ = true;
			RCLCPP_INFO(LOGGER,"reset torque adjust payload end\n");
			break;
		}
	}
	/* 初始化力控, act_torque清零 */
	errno_t set_compliant_type_ret = ERR_SUCC;
	set_compliant_type_ret = this->jaka_robot_.set_compliant_type(1, 0);
	RCLCPP_INFO(LOGGER,"reinit torque sensor, act torque should be zero\n");
	memset(&robot_status, 0, sizeof(RobotStatus));
	this->jaka_robot_.get_robot_status(&robot_status);
	RCLCPP_INFO(LOGGER,
	    "Torque reset current act torque ret %d "
	    "[%f %f %f | %f %f %f]\n",
	    set_compliant_type_ret,
	    robot_status.torq_sensor_monitor_data.actTorque[0],
	    robot_status.torq_sensor_monitor_data.actTorque[1],
	    robot_status.torq_sensor_monitor_data.actTorque[2],
	    robot_status.torq_sensor_monitor_data.actTorque[3],
	    robot_status.torq_sensor_monitor_data.actTorque[4],
	    robot_status.torq_sensor_monitor_data.actTorque[5]);
	response->success = ret;
	return true;
}

/**
 * @brief 直线运动
 *
 * 机器人末端直线运动
 * 欧拉角
 *
 * @param request
 * @param response
 * @return true 成功
 * @return false 失败
 */
bool jaka_robot_driver::linear_move_callback(const
        std::shared_ptr<shared_message::srv::MoveL::Request> request,
        std::shared_ptr<shared_message::srv::MoveL::Response> response)
{
	CartesianPose end_pose;
	double speed = request->vel;
	double accel = request->acc;
	double tol = 0.2;
	OptionalCond *option_cond = nullptr;
	int ret = 0;
	end_pose.tran.x = request->target.x;
	end_pose.tran.y = request->target.y;
	end_pose.tran.z = request->target.z;
	end_pose.rpy.rx = request->target.yaw;
	end_pose.rpy.ry	= request->target.pitch;
	end_pose.rpy.rz	= request->target.roll;
	/*
	norm()是 Frobenius 范数, 所有元素平方之和的平方根
	normalize() 会将向量本身改变进行归一化
	normalized() 会返回一个归一化向量，但不会改变向量本身
	*/
	// Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
	// auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
	// end_pose.rpy.rx = rpy.x();
	// end_pose.rpy.ry = rpy.y();
	// end_pose.rpy.rz = rpy.z();
	RCLCPP_INFO(LOGGER,
	    "linear move speed %f acc %f, tar %f %f %f %f %f %f\n",
	    speed, accel, end_pose.tran.x, end_pose.tran.y, end_pose.tran.z,
	    end_pose.rpy.rx, end_pose.rpy.ry, end_pose.rpy.rz);

	ret = jaka_robot_.linear_move(&end_pose, MoveMode::ABS, FALSE, speed, accel,
	                              tol, option_cond);
	RCLCPP_INFO(LOGGER,"linear move ret %d\n", ret);
	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "linear_move has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 直线调整运动
 *
 * 机器人末端直线运动, 但是通过逆运动学完成
 *
 * @param request
 * @param response
 * @return true 成功
 * @return false 失败
 */
bool jaka_robot_driver::move_l_adjust_callback(const
        std::shared_ptr<shared_message::srv::MoveLAdjust::Request> request,
        std::shared_ptr<shared_message::srv::MoveLAdjust::Response> response)
{
	errno_t ret = ERR_SUCC;
	CartesianPose end_pose;
	double speed = request->vel;
	double accel = request->acc;
	end_pose.tran.x = request->target.x;
	end_pose.tran.y = request->target.y;
	end_pose.tran.z = request->target.z;
	end_pose.rpy.rx = request->target.yaw;
	end_pose.rpy.ry	= request->target.pitch;
	end_pose.rpy.rz	= request->target.roll;
	/*
	norm()是 Frobenius 范数, 所有元素平方之和的平方根
	normalize() 会将向量本身改变进行归一化
	normalized() 会返回一个归一化向量，但不会改变向量本身
	*/
	// Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
	// auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
	// end_pose.rpy.rx = rpy.x();
	// end_pose.rpy.ry = rpy.y();
	// end_pose.rpy.rz = rpy.z();

	CartesianPose cur_tcp_position;
	ret = this->jaka_robot_.get_tcp_position(&cur_tcp_position);

	if (ERR_SUCC != ret)
	{
		response->ret = 0;
		response->message = "error occurred:" + this->mapErr_.at(ret);
		return false;
	}

	// 2.1 当前的关节角度
	JointValue ref_pos;
	RobotStatus robotstatus;
	ret = this->jaka_robot_.get_robot_status(&robotstatus);

	if (ERR_SUCC != ret)
	{
		response->ret = 0;
		response->message = "error occurred:" + this->mapErr_.at(ret);
		return false;
	}

	for (size_t j = 0; j < 6; j++)
	{
		ref_pos.jVal[j] = robotstatus.joint_position[j];
	}

	RCLCPP_INFO(LOGGER,
	    "move_l_adjust_callback ref joint pose %f %f %f %f %f %f\n",
	    ref_pos.jVal[0], ref_pos.jVal[1], ref_pos.jVal[2],
	    ref_pos.jVal[3], ref_pos.jVal[4], ref_pos.jVal[5]);

	// 4. 进行关节逆运算, 获取关节角度
	JointValue tar_joint_pos;

	ret = this->jaka_robot_.kine_inverse(&ref_pos, &end_pose, &tar_joint_pos);

	RCLCPP_INFO(LOGGER,"move_l_adjust_callback kine inverse ret %d tar joint pose %f %f %f %f %f %f\n",
	           ret,
	           tar_joint_pos.jVal[0], tar_joint_pos.jVal[1], tar_joint_pos.jVal[2],
	           tar_joint_pos.jVal[3], tar_joint_pos.jVal[4], tar_joint_pos.jVal[5]);

	double tmp_val = 0.0;

	for (size_t idx = 0; idx < 6; idx++)
	{
		tmp_val += fabs(tar_joint_pos.jVal[idx]);
	}

	/* 转换失败 或者结果都是0 */
	if ((ERR_SUCC != ret) || (0.1 > tmp_val))
	{
		response->ret = 0;
		response->message = "error occurred:" + this->mapErr_.at(ret);
		return false;
	}

	// RCLCPP_INFO(LOGGER,
	//     "move_l_adjust_callback tar joint pose %f %f %f %f %f %f\n",
	//     tar_joint_pos.jVal[0], tar_joint_pos.jVal[1], tar_joint_pos.jVal[2],
	//     tar_joint_pos.jVal[3], tar_joint_pos.jVal[4], tar_joint_pos.jVal[5]);

	// 5. 进行关节运动
	ret = 0;
	double tol = 0.5;
	OptionalCond *option_cond = nullptr;
	ret = jaka_robot_.joint_move(&tar_joint_pos, MoveMode::ABS, FALSE, speed, accel,
	                             tol, option_cond);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "move_l_adjust has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 关节运动
 *
 * 机器人关节运动
 *
 * @param request
 * @param response
 * @return true 成功
 * @return false 失败
 */
bool jaka_robot_driver::joint_move_callback(const
        std::shared_ptr<shared_message::srv::MoveJ::Request> request,
        std::shared_ptr<shared_message::srv::MoveJ::Response> response)
{
	JointValue joint_pose;
	joint_pose.jVal[0] = request->target.pose.at(0);
	joint_pose.jVal[1] = request->target.pose.at(1);
	joint_pose.jVal[2] = request->target.pose.at(2);
	joint_pose.jVal[3] = request->target.pose.at(3);
	joint_pose.jVal[4] = request->target.pose.at(4);
	joint_pose.jVal[5] = request->target.pose.at(5);
	double speed = (double)request->vel;
	double accel = (double)request->acc;
	double tol = 0.5;
	OptionalCond *option_cond = nullptr;
	int ret = 0;

	RCLCPP_INFO(LOGGER,"speed %f acc %f, joint pose %f %f %f %f %f %f",
	           speed, accel,
	           joint_pose.jVal[0], joint_pose.jVal[1],
	           joint_pose.jVal[2], joint_pose.jVal[3],
	           joint_pose.jVal[4], joint_pose.jVal[5]);

	ret = jaka_robot_.joint_move(&joint_pose, MoveMode::ABS, FALSE, speed,
	                             accel, tol, option_cond);
	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "joint_move has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 路径运动
 *
 * 机器人关节运动
 *
 * @param request
 * @param response
 * @return true 成功
 * @return false 失败
 */
bool jaka_robot_driver::move_path_callback(const
        std::shared_ptr<shared_message::srv::MovePath::Request> request,
        std::shared_ptr<shared_message::srv::MovePath::Response> response)
{
	int ret = 0;
	RCLCPP_INFO(LOGGER,"[%s]request path num %lu, vel %f, acc %f\n",
	           __func__,
	           request->waypoint.size(), request->vel, request->acc);

	for (size_t i = 0; i < request->waypoint.size(); i++)
	{
		RCLCPP_INFO(LOGGER,"[%s]waypoint %ld, pose %f %f %f %f %f %f\n",
		           __func__, i,
		           request->waypoint.at(i).pose.pose.at(0),
		           request->waypoint.at(i).pose.pose.at(1),
		           request->waypoint.at(i).pose.pose.at(2),
		           request->waypoint.at(i).pose.pose.at(3),
		           request->waypoint.at(i).pose.pose.at(4),
		           request->waypoint.at(i).pose.pose.at(5));
		JointValue joint_pose;
		joint_pose.jVal[0] = request->waypoint.at(i).pose.pose.at(0);
		joint_pose.jVal[1] = request->waypoint.at(i).pose.pose.at(1);
		joint_pose.jVal[2] = request->waypoint.at(i).pose.pose.at(2);
		joint_pose.jVal[3] = request->waypoint.at(i).pose.pose.at(3);
		joint_pose.jVal[4] = request->waypoint.at(i).pose.pose.at(4);
		joint_pose.jVal[5] = request->waypoint.at(i).pose.pose.at(5);
		double speed = (double)request->vel;
		double accel = (double)request->acc;
		double tol = 0.5;
		OptionalCond *option_cond = nullptr;

		ret |= jaka_robot_.joint_move(&joint_pose, MoveMode::ABS, FALSE, speed,
		                              accel, tol, option_cond);
	}

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "joint_move has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 手动模式移动机器人
 *
 * @param request
 * @param response
 * @return true 成功
 */
bool jaka_robot_driver::jog_callback(const
                                     std::shared_ptr<shared_message::srv::MoveJog::Request> request,
                                     std::shared_ptr<shared_message::srv::MoveJog::Response> response)
{

	// 1. 初始化参数
	double move_velocity = 0;
	CoordType coord_type = COORD_JOINT;

	// 2. 选择index   mapping the index and velocity
	//     request.index 如果是关节空间   就是 0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
	//     request.index 如果是笛卡尔空间 就是 x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
	float index_temp = static_cast<float>(request->index) / 2 + 0.1;
	int index = static_cast<int>(index_temp);

	// 3. 选择坐标
	switch (request->coord_mode)
	{
		case 0:
		{
			// 关节坐标系
			coord_type = COORD_JOINT;
			// 关节运动速度 (rad/s)
			move_velocity = request->mvradii;
			break;
		}
		case 1:
		{
			// 基坐标系（笛卡尔空间）
			coord_type = COORD_BASE;
			// 运动速度 (mm/s)
			move_velocity = request->mvvelo;
			break;
		}
		case 2:
		{
			// 工具坐标系（笛卡尔空间）
			coord_type = COORD_TOOL;
			// 运动速度 (mm/s)
			move_velocity = request->mvvelo;
			break;
		}
		default:
		{
			RCLCPP_INFO(LOGGER,"坐标系输入错误，请重新输入\n");
			return true;
		}
	}

	// 4. 确定速度方向
	//     判断机器人运动（关节或者笛卡尔）正方向还是负方向
	if (request->index & 1)
	{
		move_velocity = -move_velocity;
	}

	// 5. 进行jog运动
	if (this->jog_index_last_ != request->index)
	{
		int ret = jaka_robot_.motion_abort();

		if (ERR_SUCC == ret)
		{
			int jog_state = jaka_robot_.jog(index, CONTINUE, coord_type, move_velocity, 0);

			switch (jog_state)
			{
				case 0:
				{
					response->ret = 1;
					response->message = "Position is reached";
					break;
				}
				default:
				{
					response->ret = jog_state;
					response->message = "error occurred:" + this->mapErr_.at(jog_state);
					break;
				}
			}
		}
		else
		{
			response->ret = ret;
			response->message = "error occurred:" + this->mapErr_.at(ret);
		}

		this->jog_index_last_ = request->index;
	}
	else
	{
		response->ret = 1;
		response->message = "Robot is jogging";
		RCLCPP_INFO(LOGGER,"Robot is jogging\n");
	}

	this->jog_count_ = this->jog_count_ + 1;
	return true;
}

/**
 * @brief 机器人SERVO MOVE模式使能
 *
 * @param req
 * @param res
 * @return true
 */
bool jaka_robot_driver::servo_move_enable_callback(const
        std::shared_ptr<shared_message::srv::ServoMoveEnable::Request> request,
        std::shared_ptr<shared_message::srv::ServoMoveEnable::Response> response)
{
	bool enable = request->enable;
	int ret = this->jaka_robot_.servo_move_enable(enable);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "servo_move_enable has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 机器人笛卡尔空间位置控制模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::servo_p_callback(const
        std::shared_ptr<shared_message::srv::ServoMove::Request> request,
        std::shared_ptr<shared_message::srv::ServoMove::Response> response)
{
	// speed * 0.008
	CartesianPose cartesian_pose;
	cartesian_pose.tran.x = request->pose[0];
	cartesian_pose.tran.y = request->pose[1];
	cartesian_pose.tran.z = request->pose[2];
	cartesian_pose.rpy.rx = request->pose[3];
	cartesian_pose.rpy.ry = request->pose[4];
	cartesian_pose.rpy.rz = request->pose[5];

	int ret = jaka_robot_.servo_p(&cartesian_pose, MoveMode::INCR);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "Servo_p has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 机器人关节空间位置控制模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::servo_j_callback(const
        std::shared_ptr<shared_message::srv::ServoMove::Request> request,
        std::shared_ptr<shared_message::srv::ServoMove::Response> response)
{
	JointValue joint_pose;
	joint_pose.jVal[0] = request->pose[0];
	joint_pose.jVal[1] = request->pose[1];
	joint_pose.jVal[2] = request->pose[2];
	joint_pose.jVal[3] = request->pose[3];
	joint_pose.jVal[4] = request->pose[4];
	joint_pose.jVal[5] = request->pose[5];

	int ret = this->jaka_robot_.servo_j(&joint_pose, MoveMode::ABS);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "Servo_j has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 机器人步进关节空间位置控制模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::servo_step_j_callback(const
        std::shared_ptr<shared_message::srv::ServoStepMove::Request> request,
        std::shared_ptr<shared_message::srv::ServoStepMove::Response> response)
{
	JointValue joint_pose;
	joint_pose.jVal[0] = request->pose[0];
	joint_pose.jVal[1] = request->pose[1];
	joint_pose.jVal[2] = request->pose[2];
	joint_pose.jVal[3] = request->pose[3];
	joint_pose.jVal[4] = request->pose[4];
	joint_pose.jVal[5] = request->pose[5];
	unsigned int step = request->step_num;

	int ret = this->jaka_robot_.servo_j(&joint_pose, MoveMode::ABS, step);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "Servo_step_j has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 停止机器人
 *
 * @param req 请求
 * @param res 回复
 * @return true 固定返回true
 */
bool jaka_robot_driver::stop_move_callback(const
        std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	// 初始化jog的相关参数
	this->init_jog_param();
	auto ret = this->jaka_robot_.motion_abort();
	this->torque_adjust_.enabled_ = false;
	this->torque_extract_adjust_.enabled_ = false;
	/* 停止力控判断到位功能 */
	this->torque_control_.enabled_ = false;
	this->torque_control_.sm_state_ = torque_control::TORQUE_CONTROL_IDLE;

	if (this->admit_extract_.enabled_)
	{
		this->jaka_robot_.set_compliant_type(0, 0);
		this->jaka_robot_.motion_abort();
		this->admit_extract_.sm_.success_ = false;
		this->admit_extract_.sm_.restart_ = false;
		this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
		this->admit_extract_.enabled_ = false;
	}

	switch (ret)
	{
		case 0:
		{
			RCLCPP_INFO(LOGGER,"stop_move has been executed\n");
			break;
		}
		default:
		{
			RCLCPP_ERROR(LOGGER,"error occurred: %s\n", this->mapErr_.at(ret).c_str());
			break;
		}
	}

	return true;
}

/**
 * @brief 恢复机器人
 *
 * @param req 请求
 * @param res 回复
 * @return true 固定返回true
 */
bool jaka_robot_driver::recover_callback(const
        std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	RobotStatus robot_status;
	this->jaka_robot_.get_robot_status(&robot_status);

	if (0 != robot_status.collision_stop)
	{
		// robot 恢复碰撞
		int rec_ret = ERR_SUCC;
		rec_ret = this->jaka_robot_.collision_recover();

		if (ERR_SUCC != rec_ret)
		{
			RCLCPP_ERROR(LOGGER,"Recover robot collision error %s\n",
			            this->mapErr_.at(rec_ret).c_str());
			response->success = false;
			response->message = this->mapErr_.at(rec_ret);
			return true;
		}
	}

	this->jaka_robot_.get_robot_status(&robot_status);

	if (robot_status.errcode)
	{
		this->jaka_robot_.clear_error();
	}

	this->jaka_robot_.get_robot_status(&robot_status);

	if (0 == robot_status.powered_on)
	{
		// robot 上电
		int power_ret = ERR_SUCC;
		power_ret = this->jaka_robot_.power_on();

		if (ERR_SUCC != power_ret)
		{

			RCLCPP_ERROR(LOGGER,"Power on robot error %s\n", this->mapErr_.at(power_ret).c_str());
			response->success = false;
			response->message = this->mapErr_.at(power_ret);
			return true;
		}

		std::this_thread::sleep_for(std::chrono::seconds(
		                                1)); // TODO: 获取机器人上电状态代替原本驱动里面的sleep
	}

	this->jaka_robot_.get_robot_status(&robot_status);

	if (0 == robot_status.enabled)
	{
		// robot 上使能
		int enable_ret = ERR_SUCC;
		enable_ret = this->jaka_robot_.enable_robot();

		if (ERR_SUCC != enable_ret)
		{
			RCLCPP_ERROR(LOGGER,"Enable robot error %s\n", this->mapErr_.at(enable_ret).c_str());
			response->success = false;
			response->message = this->mapErr_.at(enable_ret);
			return true;
		}
	}

	if (this->admit_extract_.enabled_)
	{
		this->jaka_robot_.set_compliant_type(0, 0);
		this->jaka_robot_.motion_abort();
		this->admit_extract_.sm_.success_ = false;
		this->admit_extract_.sm_.restart_ = false;
		this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
		this->admit_extract_.enabled_ = false;
	}

	this->config_torque_sensor();

	RCLCPP_INFO(LOGGER,"recover has been executed\n");
	response->success = true;
	return true;
}

/**
 * @brief 设置tcp参数
 *
 * 设置相对于法兰盘坐标系下的工具位置姿态数据
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool jaka_robot_driver::set_tool_frame_callback(const
        std::shared_ptr<shared_message::srv::SetTcpFrame::Request> request,
        std::shared_ptr<shared_message::srv::SetTcpFrame::Response> response)
{
	CartesianPose tool_frame;
	int tool_frame_id = request->tool_num;
	tool_frame.tran.x = request->pose[0];
	tool_frame.tran.y = request->pose[1];
	tool_frame.tran.z = request->pose[2];
	Eigen::Vector3d Angaxis = {request->pose[3], request->pose[4], request->pose[5]};
	RotMatrix Rot = Angaxis2Rot(Angaxis);
	jaka_robot_.rot_matrix_to_rpy(&Rot, &(tool_frame.rpy));
	// Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
	// auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
	// tool_frame.rpy.rx = rpy.x();
	// tool_frame.rpy.ry = rpy.y();
	// tool_frame.rpy.rz = rpy.z();

	int ret = jaka_robot_.set_tool_data(tool_frame_id, &tool_frame, "ToolCoord");

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "set_toolFrame has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 设置tcp参数
 *
 * 设置相对于法兰盘坐标系下的工具位置姿态数据
 * 并切换到这一把工具
 * 欧拉角
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool jaka_robot_driver::set_tcp_callback(const
        std::shared_ptr<shared_message::srv::SetTcp::Request> request,
        std::shared_ptr<shared_message::srv::SetTcp::Response> response)
{
	CartesianPose tool_frame;
	int tool_frame_id = request->tool_id;
	tool_frame.tran.x = request->tcp.x;
	tool_frame.tran.y = request->tcp.y;
	tool_frame.tran.z = request->tcp.z;
	tool_frame.rpy.rx = request->tcp.yaw;
	tool_frame.rpy.ry = request->tcp.pitch;
	tool_frame.rpy.rz = request->tcp.roll;
	// Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
	// auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
	// tool_frame.rpy.rx = rpy.x();
	// tool_frame.rpy.ry = rpy.y();
	// tool_frame.rpy.rz = rpy.z();
	RCLCPP_INFO(LOGGER,"set tool %d, %f %f %f %f %f %f",
	           tool_frame_id,
	           tool_frame.tran.x, tool_frame.tran.y, tool_frame.tran.z,
	           tool_frame.rpy.rx, tool_frame.rpy.ry, tool_frame.rpy.rz);
	jaka_robot_.set_tool_id(tool_frame_id);
	int ret = jaka_robot_.set_tool_data(tool_frame_id, &tool_frame, "ToolCoord");

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "set_tcp has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 设置用户坐标系参数
 *
 * 设置相对于base 坐标系下的用户坐标系数据
 *
 * @param req 请求
 * @param res 应答
 * @return true 服务接口固定返回true，具体返回值在res中
 */
bool jaka_robot_driver::set_user_frame_callback(const
        std::shared_ptr<shared_message::srv::SetUserFrame::Request> request,
        std::shared_ptr<shared_message::srv::SetUserFrame::Response> response)
{
	CartesianPose user_frame;
	int user_frame_id = request->user_num;
	user_frame.tran.x = request->pose[0];
	user_frame.tran.y = request->pose[1];
	user_frame.tran.z = request->pose[2];
	Eigen::Vector3d Angaxis = {request->pose[3], request->pose[4], request->pose[5]};
	RotMatrix Rot = Angaxis2Rot(Angaxis);
	jaka_robot_.rot_matrix_to_rpy(&Rot, &(user_frame.rpy));
	// Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
	// auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
	// user_frame.rpy.rx = rpy.x();
	// user_frame.rpy.ry = rpy.y();
	// user_frame.rpy.rz = rpy.z();
	int ret = jaka_robot_.set_user_frame_data(user_frame_id, &user_frame,
	          "BaseCoord");

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "set_userFrame has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 设置机械臂负载
 *
 * @param req
 * @param res
 * @return true
 */
bool jaka_robot_driver::set_payload_callback(const
        std::shared_ptr<shared_message::srv::SetPayload::Request> request,
        std::shared_ptr<shared_message::srv::SetPayload::Response> response)
{
	PayLoad payload;
	int tool_id = request->tool_id;

	payload.centroid.x = request->cog.x;
	payload.centroid.y = request->cog.y;
	payload.centroid.z = request->cog.z;
	payload.mass = request->mass;

	jaka_robot_.set_tool_id(tool_id);

	int ret = jaka_robot_.set_payload(&payload);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "set_payload has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 控制机器人进入或退出拖拽模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::drag_mode_callback(const
        std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
	int ret = jaka_robot_.drag_mode_enable(request->data);

	switch (ret)
	{
		case 0:
		{
			response->success = 1;
			response->message = "drag_mode has been executed";
			break;
		}
		default:
		{
			response->success = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 控制机器人进入或退出拖拽模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::force_control_callback(const
        std::shared_ptr<shared_message::srv::SetForceMode::Request> request,
        std::shared_ptr<shared_message::srv::SetForceMode::Response> response)
{

	this->torque_control_.force_ = request->force;
	this->torque_control_.vel_ = request->vel;
	this->torque_control_.acc_ = request->acc;
	this->torque_control_.depth_ = request->depth;
	// this->torque_control_.first_depth_thres_ = request->first_depth_thres;
	this->torque_control_.seq_ = request->seq;
	/* 2023年12月21日 增加第二段插入的功能参数 */
	this->torque_control_.force_second_ = request->force_second;
	this->torque_control_.depth_thres_ = request->depth_thres;
	this->torque_control_.joint_acc_ = request->joint_acc;
	this->torque_control_.joint_vel_ = request->joint_vel;
	this->torque_control_.inpos_thres_ = request->inpos_thres;
	this->torque_control_.depth_second_ = request->depth_second;

	this->torque_control_.frame_cnt_ = 0; /* 统计清零 */
	this->torque_control_.enabled_ = true;

	RCLCPP_INFO(LOGGER,"force mode seq %u\n",
	           this->torque_control_.seq_);

	response->ret = 1;
	response->message = "force mode has been set";

	return true;
}

/**
 * @brief 设置机器人碰撞等级
 *
 * @param request
 * @param respond
 * @return true
 * @return false
 */
bool jaka_robot_driver::set_collisionLevel_callback(const
        std::shared_ptr<shared_message::srv::SetCollision::Request> request,
        std::shared_ptr<shared_message::srv::SetCollision::Response> response)
{
	int collision_level;

	if (false == request->is_enable)
	{
		collision_level = 0;
	}
	else
	{
		if (request->level <= 1)
		{
			collision_level = 1;
		}
		else if (request->level <= 2)
		{
			collision_level = 2;
		}
		else if (request->level <= 3)
		{
			collision_level = 3;
		}
		else if (request->level <= 4)
		{
			collision_level = 4;
		}
		else
		{
			collision_level = 5;
		}
	}

	int ret = jaka_robot_.set_collision_level(collision_level);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "Collision level" + std::to_string(collision_level) +
			                    " has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 设置数字输出变量(DO)的值
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::set_io_callback(const
                                        std::shared_ptr<shared_message::srv::SetIO::Request> request,
                                        std::shared_ptr<shared_message::srv::SetIO::Response> response)
{
	IOType type;
	int ret = ERR_SUCC;

	switch (request->type)
	{
		case 0:
			type = IO_CABINET;
			break;
		case 1:
			type = IO_TOOL;
			break;
		case 2:
			type = IO_EXTEND;
			break;
	}

	float value = request->value;
	std::string signal = request->signal;
	int index = request->index;

	if (signal == "digital")
	{
		BOOL digital_value;

		if (value)
		{
			digital_value = TRUE;
		}
		else
		{
			digital_value = FALSE;
		}

		ret = jaka_robot_.set_digital_output(type, index, digital_value);
	}
	else if (signal == "analog")
	{
		ret = jaka_robot_.set_analog_output(type, index, value);
	}

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "set IO has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}
	return true;
}

/**
 * @brief 获取输入输出状态
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::get_io_callback(const
                                        std::shared_ptr<shared_message::srv::GetIO::Request> request,
                                        std::shared_ptr<shared_message::srv::GetIO::Response> response)
{
	IOType type;
	int ret;
	BOOL digital_result;
	float analog_result;

	switch (request->type)
	{
		case 0:
		{
			type = IO_CABINET;
			break;
		}
		case 1:
		{
			type = IO_TOOL;
			break;
		}
		case 2:
		{
			type = IO_EXTEND;
			break;
		}
	}

	std::string signal = request->signal;
	int index = request->index;
	int path = request->path;

	if (signal == "digital")
	{
		if (path == 0)
		{
			ret = jaka_robot_.get_digital_input(type, index, &digital_result);
		}
		else if (path == 1)
		{
			ret = jaka_robot_.get_digital_output(type, index, &digital_result);
		}

		switch (ret)
		{
			case 0:
			{
				response->value = float(digital_result);
				response->message = "get IO has been executed";
				break;
			}
			default:
			{
				response->value = -999999;
				response->message = "error occurred:" + this->mapErr_.at(ret);
			}
		}
		return true;
	}
	else if (signal == "analog")
	{
		if (path == 0)
		{
			ret = jaka_robot_.get_analog_input(type, index, &analog_result);
		}
		else if (path == 1)
		{
			ret = jaka_robot_.get_analog_output(type, index, &analog_result);
		}

		switch (ret)
		{
			case 0:
			{
				response->value = analog_result;
				response->message = "get IO has been executed";
				break;
			}
			default:
			{
				response->value = -999999;
				response->message = "error occurred:" + this->mapErr_.at(ret);
			}

		}

		return true;
	}

	return false;
}

/**
 * @brief 计算指定关节位置在当前工具、当前安装角度以及当前用户坐标系设置下的位姿值
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::get_fk_callback(const
                                        std::shared_ptr<shared_message::srv::GetFK::Request> request,
                                        std::shared_ptr<shared_message::srv::GetFK::Response> response)
{
	JointValue joint_pose;
	CartesianPose cartesian_pose;

	for (int i = 0; i < 6; i++)
	{
		joint_pose.jVal[i] = request->joint[i];
	}

	int ret = jaka_robot_.kine_forward(&joint_pose, &cartesian_pose);

	switch (ret)
	{
		case 0:
		{
			response->cartesian_pose.push_back(cartesian_pose.tran.x);
			response->cartesian_pose.push_back(cartesian_pose.tran.y);
			response->cartesian_pose.push_back(cartesian_pose.tran.z);
			response->cartesian_pose.push_back(cartesian_pose.rpy.rx);
			response->cartesian_pose.push_back(cartesian_pose.rpy.ry);
			response->cartesian_pose.push_back(cartesian_pose.rpy.rz);
			response->message = "get FK has been executed";
			break;
		}
		default:
		{
			float pose_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};

			for (int i = 0; i < 6; i++)
			{
				response->cartesian_pose.push_back(pose_init[i]);
			}

			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 计算指定位姿在当前工具、当前安装角度以及当前用户坐标系设置下的逆解
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::get_ik_callback(const
                                        std::shared_ptr<shared_message::srv::GetIK::Request> request,
                                        std::shared_ptr<shared_message::srv::GetIK::Response> response)
{
	JointValue joint_pose;
	JointValue ref_joint;
	CartesianPose cartesian_pose;

	for (int i = 0; i < 6; i++)
	{
		ref_joint.jVal[i] = request->ref_joint[i];

	}

	cartesian_pose.tran.x = request->cartesian_pose[0];
	cartesian_pose.tran.y = request->cartesian_pose[1];
	cartesian_pose.tran.z = request->cartesian_pose[2];
	cartesian_pose.rpy.rx = request->cartesian_pose[3];
	cartesian_pose.rpy.ry = request->cartesian_pose[4];
	cartesian_pose.rpy.rz = request->cartesian_pose[5];

	int ret = jaka_robot_.kine_inverse(&ref_joint, &cartesian_pose, &joint_pose);

	switch (ret)
	{
		case 0:
		{
			for (int i = 0; i < 6; i++)
			{
				response->joint.push_back(joint_pose.jVal[i]);
			}
			response->message = "get IK has been executed";
			break;
		}
		default:
		{
			float joint_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};

			for (int i = 0; i < 6; i++)
			{
				response->joint.push_back(joint_init[i]);
			}

			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 设置机器人参数
 *
 * 包含设置TCP 设置Payload 设置碰撞等级
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool jaka_robot_driver::set_robot_param_callback(const
        std::shared_ptr<shared_message::srv::SetRobotParam::Request> request,
        std::shared_ptr<shared_message::srv::SetRobotParam::Response> response)
{
	int ret = 0;
	int collision_level = 0;
	errno_t set_tool_id_ret = ERR_SUCC, set_tool_data_ret = ERR_SUCC,
	        set_payload_ret = ERR_SUCC, set_coli_lvl_ret = ERR_SUCC;
	int tool_frame_id = 0;
	CartesianPose tool_frame;
	memset(&tool_frame, 0, sizeof(CartesianPose));
	PayLoad payload;
	memset(&payload, 0, sizeof(PayLoad));

	do
	{
		tool_frame_id = request->tool_id;
		set_tool_id_ret = jaka_robot_.set_tool_id(tool_frame_id);

		if (ERR_SUCC != set_tool_id_ret)
		{
			ret = set_tool_id_ret;
			break;
		}

		tool_frame.tran.x = request->tcp.x;
		tool_frame.tran.y = request->tcp.y;
		tool_frame.tran.z = request->tcp.z;
		tool_frame.rpy.rx = request->tcp.yaw;
		tool_frame.rpy.ry = request->tcp.pitch;
		tool_frame.rpy.rz = request->tcp.roll;
		set_tool_data_ret = jaka_robot_.set_tool_data(tool_frame_id,
		                    &tool_frame, "ToolCoord");

		if (ERR_SUCC != set_tool_data_ret)
		{
			ret = set_tool_data_ret;
			break;
		}

		payload.centroid.x = request->cog.x;
		payload.centroid.y = request->cog.y;
		payload.centroid.z = request->cog.z;
		payload.mass = request->mass;

		set_payload_ret = jaka_robot_.set_payload(&payload);

		if (ERR_SUCC != set_payload_ret)
		{
			ret = set_payload_ret;
			break;
		}

		if (false == request->is_enable)
		{
			collision_level = 0;
		}
		else
		{
			if (request->level <= 1)
			{
				collision_level = 1;
			}
			else if (request->level <= 2)
			{
				collision_level = 2;
			}
			else if (request->level <= 3)
			{
				collision_level = 3;
			}
			else if (request->level <= 4)
			{
				collision_level = 4;
			}
			else
			{
				collision_level = 5;
			}
		}

		set_coli_lvl_ret = jaka_robot_.set_collision_level(collision_level);

		if (ERR_SUCC != set_coli_lvl_ret)
		{
			ret = set_coli_lvl_ret;
			break;
		}
	}
	while (0);

	RCLCPP_INFO(LOGGER,"%s tool id %d, "
	           "set_tool_id ret %d "
	           "tcp %f %f %f %f %f %f "
	           "set_tool_data ret %d "
	           "payload center %f %f %f mass %f"
	           "set_payload ret %d "
	           "coli lvl %d, set ret %d",
	           __func__,
	           tool_frame_id,
	           set_tool_id_ret,
	           tool_frame.tran.x, tool_frame.tran.y,
	           tool_frame.tran.z, tool_frame.rpy.rx,
	           tool_frame.rpy.ry, tool_frame.rpy.rz,
	           set_tool_data_ret,
	           payload.centroid.x,
	           payload.centroid.y,
	           payload.centroid.z,
	           payload.mass,
	           set_payload_ret,
	           collision_level,
	           set_coli_lvl_ret);

	switch (ret)
	{
		case 0:
		{
			response->ret = 1;
			response->message = "set_tcp has been executed";
			break;
		}
		default:
		{
			response->ret = 0;
			response->message = "error occurred:" + this->mapErr_.at(ret);
			return false;
		}
	}

	return true;
}

/**
 * @brief 上报 tool position
 *
 */
void jaka_robot_driver::tool_position_callback()
{
	auto tool_position = std::make_unique<geometry_msgs::msg::TwistStamped>();
	RobotStatus robotstatus;
	RotMatrix rot;
	Rpy rpy;

	jaka_robot_.get_robot_status(&robotstatus);
	tool_position->twist.linear.x = robotstatus.cartesiantran_position[0];
	tool_position->twist.linear.y = robotstatus.cartesiantran_position[1];
	tool_position->twist.linear.z = robotstatus.cartesiantran_position[2];
	rpy.rx = robotstatus.cartesiantran_position[3];
	rpy.ry = robotstatus.cartesiantran_position[4];
	rpy.rz = robotstatus.cartesiantran_position[5];
	jaka_robot_.rpy_to_rot_matrix(&rpy, &rot);
	Eigen::Vector3d angaxis = Rot2Angaxis(rot);
	tool_position->twist.angular.x = angaxis[0];
	tool_position->twist.angular.y = angaxis[1];
	tool_position->twist.angular.z = angaxis[2];

	// Eigen::Vector3d eulerAngle(rpy.rx,rpy.ry,rpy.rz);
	// Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
	// Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
	// Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
	// Eigen::AngleAxisd rotation_vector;
	// rotation_vector=yawAngle*pitchAngle*rollAngle;
	// cout << "angle is: " << rotation_vector.angle() << endl;
	// cout << "axis is: " << rotation_vector.axis() << endl;
	// double angle = rotation_vector.angle();
	// Eigen::Vector3d axis = rotation_vector.axis();
	// Eigen::Vector3d v = angle * axis;
	// tool_position.twist.angular.x = v[0];
	// tool_position.twist.angular.y = v[1];
	// tool_position.twist.angular.z = v[2];

	tool_position->header.stamp = node_->get_clock()->now();
	tool_position_pub_->publish(std::move(tool_position));
}

/**
 * @brief 上报 joint position
 *
 */
void jaka_robot_driver::joint_position_callback()
{
	if (false == is_status_fresh())
	{
		static uint32_t status_out_of_date_count = 0;

		if (0 == (status_out_of_date_count % 1000))
		{
			RCLCPP_ERROR(LOGGER,"joint_position_callback status out of date!\n");
		}

		status_out_of_date_count ++;

		return;
	}

	auto joint_position = std::make_unique<sensor_msgs::msg::JointState>();

	for (int i = 0; i < 6; i++)
	{
		joint_position->position.push_back(
		    this->jaka_status_.cache_robot_status_.joint_position[i]);
		int j = i + 1;
		joint_position->name.push_back("joint_" + std::to_string(j));
	}
	joint_position->header.stamp = node_->get_clock()->now();
	joint_position_pub_->publish(std::move(joint_position));
	return;
}

/**
 * @brief 发布机器人状态
 *
 * 该函数根据缓存的机器人的各种状态，包括连接状态、电源状态、
 * 使能状态、碰撞状态、急停状态、机器人状态、
 * 停止状态、自由模式、关节姿态、碰撞等级、
 * TCP坐标、工具ID和工具TCP、负载、力控等，并发布这些状态。
 *
 */
void jaka_robot_driver::publish_robot_status()
{
	auto robot_states = std::make_unique<shared_message::msg::RobotDriverStatus>();
	/* 消息头 */
	std_msgs::msg::Header header;
	header.frame_id = "jaka";
	header.stamp = node_->get_clock()->now();
	robot_states->header = header;

	if ((false == this->jaka_status_.conn_status_)
	        || (0 == this->jaka_status_.cache_robot_status_.is_socket_connect))
	{
		robot_states->connect_status = false;
		static uint32_t connect_status_count = 0;

		if (0 == (connect_status_count % 100))
		{
			RCLCPP_ERROR(LOGGER,"connect status %d, is connect %d", this->jaka_status_.conn_status_,
			            this->jaka_status_.cache_robot_status_.is_socket_connect);
		}

		connect_status_count ++;
	}
	else
	{
		robot_states->connect_status = true;
	}

	static bool last_connect_status = false;

	if (last_connect_status != robot_states->connect_status)
	{
		RCLCPP_INFO(LOGGER,"connect status change to %d", robot_states->connect_status);
		last_connect_status = robot_states->connect_status;
	}

	/* 上电标志 */
	if (this->jaka_status_.cache_robot_status_.powered_on)
	{
		robot_states->power = true;
	}
	else
	{
		robot_states->power = false;
	}

	/* 使能标志 */
	if (this->jaka_status_.cache_robot_status_.enabled)
	{
		robot_states->enable = true;
	}
	else
	{
		robot_states->enable = false;
	}

	/* 碰撞状态 */
	robot_states->collision = this->jaka_status_.in_collision_;

	static uint32_t in_collision_count = 0;
	static int last_in_coli = 0;

	if ((0 == (in_collision_count % 200))
	        || (last_in_coli != this->jaka_status_.in_collision_))
	{
		RCLCPP_INFO(LOGGER,"in coli %d", this->jaka_status_.in_collision_);
		last_in_coli = this->jaka_status_.in_collision_;
	}

	in_collision_count ++;

	/* 急停状态 */
	if (this->jaka_status_.cache_robot_status_.emergency_stop)
	{
		robot_states->emergency = true;
	}
	else
	{
		robot_states->emergency = false;
	}

	static uint32_t in_emergency_count = 0;
	static int last_in_emergency = 0;

	if ((0 == (in_emergency_count % 200))
	        || (last_in_emergency != this->jaka_status_.cache_robot_status_.emergency_stop))
	{
		RCLCPP_INFO(LOGGER,"in emergency %d",
		           this->jaka_status_.cache_robot_status_.emergency_stop);
		last_in_emergency = this->jaka_status_.cache_robot_status_.emergency_stop;
	}

	in_emergency_count ++;

	/* 机器人状态 */
	if (this->jaka_status_.cache_robot_status_.emergency_stop)
	{
		robot_states->robot_state = 2;
	}
	else if (this->jaka_status_.cache_robot_status_.errcode)
	{
		robot_states->robot_state = 4;
	}
	else if ((this->jaka_status_.cache_robot_status_.inpos)
	         && (!this->jaka_status_.cache_robot_status_.drag_status))
	{
		robot_states->robot_state = 0;
	}
	else if ((!this->jaka_status_.cache_robot_status_.inpos)
	         || (this->jaka_status_.cache_robot_status_.drag_status))
	{
		robot_states->robot_state = 3;
	}


	static uint32_t in_pos_count = 0;
	static int last_in_pos = 0;

	if ((0 == (in_pos_count % 200))
	        || (last_in_pos != this->jaka_status_.cache_robot_status_.inpos))
	{
		RCLCPP_INFO(LOGGER,"inpos %d", this->jaka_status_.cache_robot_status_.inpos);
		last_in_pos = this->jaka_status_.cache_robot_status_.inpos;
	}

	in_pos_count ++;

	robot_states->in_pos = this->jaka_status_.cache_robot_status_.inpos;

	/* 是否停止 */
	if (0 == robot_states->robot_state)
	{
		robot_states->is_robot_stop = true;
	}
	else
	{
		robot_states->is_robot_stop = false;
	}

	/* 自由模式 */
	if (this->jaka_status_.cache_robot_status_.drag_status)
	{
		robot_states->free_drive = true;
	}
	else
	{
		robot_states->free_drive = false;
	}

	/* 关节姿态 */
	robot_states->joint_pose.pose.resize(6);

	for (size_t i = 0; i < 6; i++)
	{
		robot_states->joint_pose.pose.at(i) =
		    this->jaka_status_.cache_robot_status_.joint_position[i];
	}

	/* 碰撞等级 */
	robot_states->collision_level = this->jaka_status_.collision_level_;

	/* TCP坐标 */
	robot_states->tcp_pose.x =
	    this->jaka_status_.cache_robot_status_.cartesiantran_position[0];
	robot_states->tcp_pose.y =
	    this->jaka_status_.cache_robot_status_.cartesiantran_position[1];
	robot_states->tcp_pose.z =
	    this->jaka_status_.cache_robot_status_.cartesiantran_position[2];
	robot_states->tcp_pose.yaw =
	    this->jaka_status_.cache_robot_status_.cartesiantran_position[3];
	robot_states->tcp_pose.pitch =
	    this->jaka_status_.cache_robot_status_.cartesiantran_position[4];
	robot_states->tcp_pose.roll =
	    this->jaka_status_.cache_robot_status_.cartesiantran_position[5];

	robot_states->joint_base_position.resize(
	    this->jaka_status_.joint_base_position_.size());

	for (size_t i = 0; i < robot_states->joint_base_position.size(); i++)
	{
		robot_states->joint_base_position.at(i).x =
		    this->jaka_status_.joint_base_position_.at(i).at(0) * 1000;
		robot_states->joint_base_position.at(i).y =
		    this->jaka_status_.joint_base_position_.at(i).at(1) * 1000;
		robot_states->joint_base_position.at(i).z =
		    this->jaka_status_.joint_base_position_.at(i).at(2) * 1000;
	}

	/* 工具ID和工具TCP */
	robot_states->tool_id = this->jaka_status_.cache_robot_status_.current_tool_id;
	robot_states->tool_tcp.x = this->jaka_status_.tool_tcp_.tran.x;
	robot_states->tool_tcp.y = this->jaka_status_.tool_tcp_.tran.y;
	robot_states->tool_tcp.z = this->jaka_status_.tool_tcp_.tran.z;
	robot_states->tool_tcp.yaw = this->jaka_status_.tool_tcp_.rpy.rx;
	robot_states->tool_tcp.pitch = this->jaka_status_.tool_tcp_.rpy.ry;
	robot_states->tool_tcp.roll = this->jaka_status_.tool_tcp_.rpy.rz;

	/* 负载 */
	robot_states->mass = this->jaka_status_.payload_.mass;
	robot_states->cog.x = this->jaka_status_.payload_.centroid.x;
	robot_states->cog.y = this->jaka_status_.payload_.centroid.y;
	robot_states->cog.z = this->jaka_status_.payload_.centroid.z;

	/* 力控 */
	robot_states->force_mode = this->torque_control_.enabled_;
	robot_states->force_control_seq = this->torque_control_.seq_;
	robot_states->insert_depth = this->torque_control_.cur_insert_depth_;
	robot_states->torque.resize(6);
	robot_states->act_torque.resize(6);
	robot_states->j_torque.resize(6);
	robot_states->j_current.resize(6);

	for (size_t i = 0; i < 6; i++)
	{
		robot_states->torque.at(i) =
		    this->jaka_status_.cache_robot_status_.torq_sensor_monitor_data.torque[i];
		robot_states->act_torque.at(i) =
		    this->jaka_status_.cache_robot_status_.torq_sensor_monitor_data.actTorque[i];
		robot_states->j_torque.at(i) =
		    this->jaka_status_.cache_robot_status_.robot_monitor_data.jointMonitorData[i].instTorq;
		robot_states->j_current.at(i) =
		    this->jaka_status_.cache_robot_status_.robot_monitor_data.jointMonitorData[i].instCurrent;
	}

	robot_states->force_freedrive = this->torque_adjust_.enabled_;
	robot_states->force_adjust_seq = this->torque_adjust_.seq_;
	robot_states->torque_extract_adjust = this->torque_extract_adjust_.enabled_;
	robot_states->force_extract_adjust_seq = this->torque_extract_adjust_.seq_;
	robot_states->admit_extract = this->admit_extract_.enabled_;
	robot_states->admit_extract_success = this->admit_extract_.sm_.success_;
	robot_states->admit_extract_seq = this->admit_extract_.seq_;
	robot_states_pub_->publish(std::move(robot_states));
	return;
}

/**
 * @brief 处理机器人的状态
 *
 */
void jaka_robot_driver::robot_state_handler()
{
	/* 一个静态变量，用于存储上次的姿态 */
	static std::vector<double> last_joint_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	/* 静态变量, 用于决定是否打印 */
	static uint32_t joint_base_position_log_count = 0;
	/* 姿态是否发生了变化, 如果变化打印一下新的base position */
	bool pose_change = false;
	bool joint_data_invalid = false;

	if ((0.0001 > abs(0.0 -
	                  this->jaka_status_.cache_robot_status_.joint_position[0]))
	        && (0.0001 > abs(0.0 -
	                         this->jaka_status_.cache_robot_status_.joint_position[1]))
	        && (0.0001 > abs(0.0 -
	                         this->jaka_status_.cache_robot_status_.joint_position[2])))
	{
		joint_data_invalid = true;
	}

	static bool in_crash = false;

	if ((true == this->jaka_status_.conn_status_) && (joint_data_invalid))
	{
		if (false == in_crash)
		{
			RCLCPP_ERROR(LOGGER,"this cause crash");
			in_crash = true;
		}
	}
	else
	{
		in_crash = false;
	}

	static bool last_invalid_status = false;

	if (last_invalid_status != joint_data_invalid)
	{
		RCLCPP_ERROR(LOGGER,"joint data invalid %d", joint_data_invalid);
		last_invalid_status = joint_data_invalid;
	}

	if ((true == this->jaka_status_.conn_status_) && (false == joint_data_invalid)
	        && (0 != this->jaka_status_.cache_robot_status_.enabled))
	{
		std::vector<double> joint_pose;
		joint_pose.resize(6);
		joint_pose.at(0) = this->jaka_status_.cache_robot_status_.joint_position[0];
		joint_pose.at(1) = this->jaka_status_.cache_robot_status_.joint_position[1];
		joint_pose.at(2) = this->jaka_status_.cache_robot_status_.joint_position[2];
		joint_pose.at(3) = this->jaka_status_.cache_robot_status_.joint_position[3];
		joint_pose.at(4) = this->jaka_status_.cache_robot_status_.joint_position[4];
		joint_pose.at(5) = this->jaka_status_.cache_robot_status_.joint_position[5];

		for (size_t i = 0; i < last_joint_pose.size(); i++)
		{
			/* 关节角度大于0.1进行记录 */
			if (0.1 < abs(last_joint_pose.at(i) - joint_pose.at(i)))
			{
				pose_change = true;
				last_joint_pose = joint_pose;
				break;
			}
		}

		if ((pose_change) || (0 == (joint_base_position_log_count % 5000)))
		{
			RCLCPP_INFO_STREAM(LOGGER,"robot_state_handler joint pose " << jaka_zu5_driver::vec2str(
			                      joint_pose));
		}

		this->soft_charger_interface_->forwardKinematics(joint_pose,
		        this->jaka_status_.joint_base_position_);

		if ((pose_change) || (0 == (joint_base_position_log_count % 5000)))
		{
			std::string joint_posision_info = "joint base position: ";

			/* joint base positions */
			for (size_t i = 0; i < 6; ++i)
			{
				joint_posision_info += "\njoint" + std::to_string(i) + " ";

				for (size_t j = 0; j < 3; ++j)
				{
					joint_posision_info += "[" + std::to_string(
					                           this->jaka_status_.joint_base_position_.at(
					                               i).at(j)) + "] ";
				}
			}

			joint_posision_info += "\n";
			RCLCPP_INFO(LOGGER,"%s", joint_posision_info.c_str());
		}

		joint_base_position_log_count ++;
	}

	return;
}

/**
 * @brief 定期查询机器人状态并上报
 *
 */
void jaka_robot_driver::robot_states_callback()
{
	int32_t ret = 0;
	RobotStatus robot_status = this->jaka_status_.cache_robot_status_;
	// CartesianPose tool_tcp = this->jaka_status_.tool_tcp_;
	// PayLoad payload = this->jaka_status_.payload_;
	// int collision_level = this->jaka_status_.collision_level_;
	std::lock_guard<std::mutex> lock(this->jaka_status_.robot_status_lock_);
	errno_t robo_ret = ERR_SUCC;
	double tmp_ts_last = 0.0;
	double tmp_ts_now = 0.0;

	do
	{
		if (this->first_boot_)
		{
			ret = -100;
			break;
		}

		tmp_ts_now = node_->get_clock()->now().seconds();

		if (ERR_SUCC != (robo_ret = this->jaka_robot_.get_robot_status(&robot_status)))
		{
			ret = -1;
			break;
		}

		tmp_ts_last = tmp_ts_now;
		tmp_ts_now = node_->get_clock()->now().seconds();

		if (1.0 < (tmp_ts_now - tmp_ts_last))
		{
			RCLCPP_ERROR(LOGGER,"jaka interface get_robot_status takes %f to %f", tmp_ts_last,
			            tmp_ts_now);
		}
	}
	while (0);

	if (0 > ret)
	{
		this->jaka_status_.conn_status_ = false;
		static uint32_t connect_fail_count = 0;
		static int last_ret = 0;
		static errno_t last_robo_ret = 0;

		if ((0 == (connect_fail_count % 200)) || (last_ret != ret)
		        || (last_robo_ret != robo_ret))
		{
			RCLCPP_INFO(LOGGER,"connect fail %d %d", ret, robo_ret);
			last_ret = ret;
			last_robo_ret = robo_ret;
		}

		connect_fail_count ++;
	}
	else
	{
		static int last_socket_connect = 0;

		if (0 == robot_status.is_socket_connect)
		{
			static uint32_t sock_not_connect_count = 0;

			if ((last_socket_connect != robot_status.is_socket_connect)
			        || (0 == (sock_not_connect_count % 200)))
			{
				RCLCPP_ERROR(LOGGER,"get robot status callback is socket connect %d ",
				            robot_status.is_socket_connect);
				last_socket_connect = robot_status.is_socket_connect;
			}

			sock_not_connect_count ++;

			this->jaka_status_.conn_status_ = false;
			this->jaka_status_.ts_ = node_->get_clock()->now();

			if (0 != robot_status.collision_stop)
			{
				this->jaka_status_.in_collision_ = true;
			}
			else
			{
				this->jaka_status_.in_collision_ = false;
			}

			this->jaka_status_.cache_robot_status_ = robot_status;
			this->jaka_status_.tool_tcp_.tran.x = robot_status.current_tool_offsets[0];
			this->jaka_status_.tool_tcp_.tran.y = robot_status.current_tool_offsets[1];
			this->jaka_status_.tool_tcp_.tran.z = robot_status.current_tool_offsets[2];
			this->jaka_status_.tool_tcp_.rpy.rx = robot_status.current_tool_offsets[3];
			this->jaka_status_.tool_tcp_.rpy.ry = robot_status.current_tool_offsets[4];
			this->jaka_status_.tool_tcp_.rpy.rz = robot_status.current_tool_offsets[5];
			this->jaka_status_.payload_ = robot_status.payload;
			this->jaka_status_.collision_level_ = robot_status.sensitivity_level;
		}
		else
		{
			last_socket_connect = robot_status.is_socket_connect;
			this->jaka_status_.conn_status_ = true;
			this->jaka_status_.ts_ = node_->get_clock()->now();

			if (0 != robot_status.collision_stop)
			{
				this->jaka_status_.in_collision_ = true;
			}
			else
			{
				this->jaka_status_.in_collision_ = false;
			}

			this->jaka_status_.cache_robot_status_ = robot_status;
			this->jaka_status_.tool_tcp_.tran.x = robot_status.current_tool_offsets[0];
			this->jaka_status_.tool_tcp_.tran.y = robot_status.current_tool_offsets[1];
			this->jaka_status_.tool_tcp_.tran.z = robot_status.current_tool_offsets[2];
			this->jaka_status_.tool_tcp_.rpy.rx = robot_status.current_tool_offsets[3];
			this->jaka_status_.tool_tcp_.rpy.ry = robot_status.current_tool_offsets[4];
			this->jaka_status_.tool_tcp_.rpy.rz = robot_status.current_tool_offsets[5];
			this->jaka_status_.payload_ = robot_status.payload;
			this->jaka_status_.collision_level_ = robot_status.sensitivity_level;
		}
	}

	this->robot_state_handler();
	this->publish_robot_status();
	return;
}

/**
 * @brief 定期机器人的负载数据
 *
 * 允许阻塞, 单独线程
 *
 */
void jaka_robot_driver::payload_status_callback()
{
	int32_t ret = 0;
	/* 历史数据 */
	PayLoad payload = this->jaka_status_.payload_;
	errno_t robo_ret = ERR_SUCC;
	double tmp_ts_last = 0.0;
	double tmp_ts_now = 0.0;

	do
	{
		if (this->first_boot_)
		{
			ret = -100;
			break;
		}

		tmp_ts_now = node_->get_clock()->now().seconds();

		if (ERR_SUCC != (robo_ret = this->jaka_robot_.get_payload(&payload)))
		{
			ret = -1;
			break;
		}

		tmp_ts_last = tmp_ts_now;
		tmp_ts_now = node_->get_clock()->now().seconds();

		if (1.0 < (tmp_ts_now - tmp_ts_last))
		{
			RCLCPP_ERROR(LOGGER,"jaka interface get_payload takes %f to %f", tmp_ts_last,
			            tmp_ts_now);
		}
	}
	while (0);

	if (0 > ret)
	{
		static uint32_t get_payload_fail_count = 0;
		static int last_ret = 0;
		static errno_t last_robo_ret = 0;

		if ((0 == (get_payload_fail_count % 10)) || (last_ret != ret)
		        || (last_robo_ret != robo_ret))
		{
			RCLCPP_INFO(LOGGER,"jaka interface get_payload fail %d %d", ret, robo_ret);
			last_ret = ret;
			last_robo_ret = robo_ret;
		}

		get_payload_fail_count ++;
	}
	else
	{
		this->jaka_status_.payload_ = payload;
	}

	return;
}

/**
 * @brief 定期查询当前工具tcp
 *
 */
void jaka_robot_driver::tool_tcp_status_callback()
{
	int32_t ret = 0;
	CartesianPose tool_tcp = this->jaka_status_.tool_tcp_;
	errno_t robo_ret = ERR_SUCC;
	double tmp_ts_last = 0.0;
	double tmp_ts_now = 0.0;

	do
	{
		if (this->first_boot_)
		{
			ret = -100;
			break;
		}

		tmp_ts_now = node_->get_clock()->now().seconds();

		if (ERR_SUCC != (robo_ret = this->jaka_robot_.get_tool_data(
		                                this->jaka_status_.cache_robot_status_.current_tool_id, &tool_tcp)))
		{
			ret = -1;
			break;
		}

		tmp_ts_last = tmp_ts_now;
		tmp_ts_now = node_->get_clock()->now().seconds();

		if (1.0 < (tmp_ts_now - tmp_ts_last))
		{
			RCLCPP_ERROR(LOGGER,"jaka interface get_tool_data takes %f to %f", tmp_ts_last,
			            tmp_ts_now);
		}
	}
	while (0);

	if (0 > ret)
	{
		static uint32_t get_tool_data_fail_count = 0;
		static int last_ret = 0;
		static errno_t last_robo_ret = 0;

		if ((0 == (get_tool_data_fail_count % 10)) || (last_ret != ret)
		        || (last_robo_ret != robo_ret))
		{
			RCLCPP_INFO(LOGGER,"get tool data fail %d %d", ret, robo_ret);
			last_ret = ret;
			last_robo_ret = robo_ret;
		}

		get_tool_data_fail_count ++;
	}
	else
	{
		this->jaka_status_.tool_tcp_ = tool_tcp;
	}

	return;
}

/**
 * @brief 定期查询当前碰撞等级
 *
 */
void jaka_robot_driver::collision_level_callback()
{
	int32_t ret = 0;
	int collision_level = this->jaka_status_.collision_level_;
	errno_t robo_ret = ERR_SUCC;
	double tmp_ts_last = 0.0;
	double tmp_ts_now = 0.0;

	do
	{
		if (this->first_boot_)
		{
			ret = -100;
			break;
		}

		tmp_ts_now = node_->get_clock()->now().seconds();

		if (ERR_SUCC != (robo_ret = this->jaka_robot_.get_collision_level(
		                                &collision_level)))
		{
			ret = -4;
			break;
		}

		tmp_ts_last = tmp_ts_now;
		tmp_ts_now = node_->get_clock()->now().seconds();

		if (1.0 < (tmp_ts_now - tmp_ts_last))
		{
			RCLCPP_ERROR(LOGGER,"jaka interface get_collision_level takes %f to %f", tmp_ts_last,
			            tmp_ts_now);
		}
	}
	while (0);

	if (0 > ret)
	{
		static uint32_t get_collision_level_fail_count = 0;
		static int last_ret = 0;
		static errno_t last_robo_ret = 0;

		if ((0 == (get_collision_level_fail_count % 10)) || (last_ret != ret)
		        || (last_robo_ret != robo_ret))
		{
			RCLCPP_INFO(LOGGER,"jaka interface get collision level fail %d %d", ret, robo_ret);
			last_ret = ret;
			last_robo_ret = robo_ret;
		}

		get_collision_level_fail_count ++;
	}
	else
	{
		this->jaka_status_.collision_level_ = collision_level;
	}

	return;
}

/**
 * @brief 状态是否是最近更新的
 *
 * @return true
 * @return false
 */
bool jaka_robot_driver::is_status_fresh()
{
	if (2.0 < abs(node_->get_clock()->now().seconds() - this->jaka_status_.ts_.sec))
	{
		return false;
	}
	else
	{
		return true;
	}
}

/**
 * @brief 定时停止机器人
 *
 */
void jaka_robot_driver::stop_jog_callback()
{
	if ((jog_count_ >= 1) && (jog_count_temp_ == jog_count_))
	{
		jaka_robot_.jog_stop(-1);
		// 初始化jog的相关参数
		this->init_jog_param();
		RCLCPP_INFO(LOGGER,"jog stop\n");
	}

	jog_count_temp_ = jog_count_;
}

/**
 * @brief 初始化jog相关参数
 *
 */
void jaka_robot_driver::init_jog_param()
{
	// 初始化jog的相关参数
	this->jog_count_ = 0;
	this->jog_count_temp_ = 0;
	this->jog_index_last_ = -1;
}

// 判断robot 是否运行到目标位置
bool jaka_robot_driver::robot_joint_in_position(RobotStatus *robotstatus,
        trajectory_msgs::msg::JointTrajectory joint_trajectory)
{
	bool arrive = true;

	for (int i = 0; i < 6; i++)
	{
		bool ret = ((((joint_trajectory.points.back().positions[i] * 180) / M_PI) - 0.2)
		            < (robotstatus->joint_position[i] * 180 / M_PI))
		           && ((robotstatus->joint_position[i] * 180 / M_PI) < (((
		                       joint_trajectory.points.back().positions[i] * 180) / M_PI) + 0.2));
		arrive = arrive && ret;
	}

	RCLCPP_INFO(LOGGER,
	    "Whether the robot has reached the target position: %d\n", arrive);
	return arrive;
}

// FollowJointTrajectory Action handle_accepted
void jaka_robot_driver::handle_accepted(const
	std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
	goal_handle)
{
	float last_duration = 0.0;
	auto feedback =
	std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
	auto result =
	std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
	JointValue joint_pose;
	int point_num = goal_handle->get_goal()->trajectory.points.size();
	RCLCPP_INFO(LOGGER,"Number of points: %d", point_num);
	this->jaka_robot_.servo_move_enable(true);

	for (int i = 0; i < point_num; i++)
	{
		joint_pose.jVal[0] = goal_handle->get_goal()->trajectory.points[i].positions[0];
		joint_pose.jVal[1] = goal_handle->get_goal()->trajectory.points[i].positions[1];
		joint_pose.jVal[2] = goal_handle->get_goal()->trajectory.points[i].positions[2];
		joint_pose.jVal[3] = goal_handle->get_goal()->trajectory.points[i].positions[3];
		joint_pose.jVal[4] = goal_handle->get_goal()->trajectory.points[i].positions[4];
		joint_pose.jVal[5] = goal_handle->get_goal()->trajectory.points[i].positions[5];
		// 计算 step_num
		float duration =
		goal_handle->get_goal()->trajectory.points[i].time_from_start.sec * 1.0 +
		goal_handle->get_goal()->trajectory.points[i].time_from_start.nanosec / 1e9;
		float delta_t = duration - last_duration;
		last_duration = duration;
		// driver 那边一个周期是8ms
		unsigned int step_num = delta_t / 0.008;
		if(step_num < 1)
		{
			step_num = 1;
		}

		RCLCPP_INFO(LOGGER,"Sending joint move: %f %f %f %f %f %f, "
		"%f %d",
		joint_pose.jVal[0],
		joint_pose.jVal[1],
		joint_pose.jVal[2],
		joint_pose.jVal[3],
		joint_pose.jVal[4],
		joint_pose.jVal[5],
		delta_t, step_num);
		/**
		* @brief 机械臂关节空间位置控制模式，增加了周期的调节性可以将周期调整为 8ms 的倍数
		* @param joint_pos 机械臂关节运动目标位置
		* @move_mode 指定运动模式：增量运动、绝对运动
		* @step_num
		倍分周期，servo_j 运动周期为 step_num*8ms，其中 step_num>=1
		* @return ERR_SUCC 成功 其他失败
		*/
		int servo_j_ret = this->jaka_robot_.servo_j(&joint_pose, MoveMode::ABS,
		step_num);
		sleep(0.020);

		// Wait for the response. At this point, robot should be done blocking and ready to accept new commands
		if (ERR_SUCC != servo_j_ret)
		{
			// 不成功返回0
			RCLCPP_ERROR(LOGGER,"Call servo_step_j fail %s\n",
			this->mapErr_.at(servo_j_ret).c_str());
			result->error_code = servo_j_ret;
			result->error_string = this->mapErr_.at(servo_j_ret);
			goal_handle->abort(result);
			return;
		}

		RCLCPP_INFO(LOGGER,"Accepted joint angle: %f %f %f %f %f %f, "
		"%f %d",
		goal_handle.get()->get_goal()->trajectory.points[i].positions[0],
		goal_handle.get()->get_goal()->trajectory.points[i].positions[1],
		goal_handle.get()->get_goal()->trajectory.points[i].positions[2],
		goal_handle.get()->get_goal()->trajectory.points[i].positions[3],
		goal_handle.get()->get_goal()->trajectory.points[i].positions[4],
		goal_handle.get()->get_goal()->trajectory.points[i].positions[5],
		delta_t, step_num);
	}

	// while (true)
	// {
	// 	RobotStatus robotstatus;
	// 	int get_status_ret = this->jaka_robot_.get_robot_status(&robotstatus);

	// 	if (ERR_SUCC == get_status_ret)
	// 	{
	// 		bool in_pos = robot_joint_in_position(&robotstatus,
	// 											goal_handle->get_goal()->trajectory);

	// 		if (in_pos)
	// 		{
	// 			this->servo_move_enable(false);
	// 			result->error_code = 0;
	// 			goal_handle->succeed(result);
	// 			RCLCPP_INFO(LOGGER,"运动成功, 伺服模式使能关闭 %d\n",
	// 					in_pos);
	// 			break;
	// 		}
	// 	}
	// }

	joint_pose.jVal[0] = goal_handle->get_goal()->trajectory.points[point_num-1].positions[0];
	joint_pose.jVal[1] = goal_handle->get_goal()->trajectory.points[point_num-1].positions[1];
	joint_pose.jVal[2] = goal_handle->get_goal()->trajectory.points[point_num-1].positions[2];
	joint_pose.jVal[3] = goal_handle->get_goal()->trajectory.points[point_num-1].positions[3];
	joint_pose.jVal[4] = goal_handle->get_goal()->trajectory.points[point_num-1].positions[4];
	joint_pose.jVal[5] = goal_handle->get_goal()->trajectory.points[point_num-1].positions[5];
	std::cout <<"final joint status"<< joint_pose.jVal[0] << "," << joint_pose.jVal[1] << "," << joint_pose.jVal[2] <<"," << joint_pose.jVal[3] << "," << joint_pose.jVal[4] << "," << joint_pose.jVal[5]<<"\n";
	bool in_target = false;
	JointValue jot_pos;
	while (!in_target)
	{
		sleep(0.1);
		this->jaka_robot_.get_joint_position(&jot_pos);
		std::cout <<"cur joint status1: "<< jot_pos.jVal[0] << "," << jot_pos.jVal[1] << "," << jot_pos.jVal[2]<<","<< jot_pos.jVal[3] << "," << jot_pos.jVal[4] << "," << jot_pos.jVal[5] << "\n";
		// for(int i=0;i<6;i++)
		// {
		// 	jot_pos.jVal[i] = this->jaka_status_.cache_robot_status_.joint_position[i];
		// }
		// std::cout <<"cur joint status2: "<< jot_pos.jVal[0] << "," << jot_pos.jVal[1] << "," << jot_pos.jVal[2]<<","<< jot_pos.jVal[3] << "," << jot_pos.jVal[4] << "," << jot_pos.jVal[5] << "\n";
		if ( 
		fabs(jot_pos.jVal[0] - joint_pose.jVal[0]) < 0.001 &&
		fabs(jot_pos.jVal[1] - joint_pose.jVal[1]) < 0.001 &&
		fabs(jot_pos.jVal[2] - joint_pose.jVal[2]) < 0.001 &&
		fabs(jot_pos.jVal[3] - joint_pose.jVal[3]) < 0.001 &&
		fabs(jot_pos.jVal[4] - joint_pose.jVal[4]) < 0.001 &&
		fabs(jot_pos.jVal[5] - joint_pose.jVal[5]) < 0.001)
		{
			in_target = true;
			//this->servo_move_enable(false);
			result->error_code = 0;
			goal_handle->succeed(result);
			RCLCPP_INFO(LOGGER,"运动成功, 伺服模式使能关闭\n");
		}
	}
	return;
}

// 目标回调, 过滤
rclcpp_action::GoalResponse jaka_robot_driver::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
	RCLCPP_INFO(LOGGER,"Received goal request with point num %ld\n",
	           goal->trajectory.points.size());
	(void)uuid;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消
rclcpp_action::CancelResponse jaka_robot_driver::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
    goal_handle)
{
	RCLCPP_INFO(LOGGER,"Received request to cancel goal\n");
	(void)goal_handle;
	//servo_move_enable(false);
	this->jaka_robot_.servo_move_enable(false);
	return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief 判断角度移动是否到位
 *
 * @param current_tcp_pose
 * @param target_tcp_pose
 * @return true
 * @return false
 */
bool jaka_robot_driver::is_admit_adjust_in_pos(const std::vector<double>
        &current_tcp_pose, const std::vector<double> &target_tcp_pose)
{
	bool R_in_target = false;

	Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
	std::vector<double> euler = {target_tcp_pose.at(3), target_tcp_pose.at(4), target_tcp_pose.at(5)};
	this->torque_control_.euler_ang_to_Mat3d(euler, R1);

	Eigen::Matrix3d R2 = Eigen::Matrix3d::Identity();
	euler = {current_tcp_pose.at(3), current_tcp_pose.at(4), current_tcp_pose.at(5)};
	this->torque_control_.euler_ang_to_Mat3d(euler, R2);

	if ((R1.transpose()*R2 - Eigen::Matrix3d::Identity()).norm() < 1e-3)
	{
		R_in_target = true;
	}

	return R_in_target;
}

double admit_extract::depth_calculate(const std::vector<double> &cur_tcp,
                                      const std::vector<double> &tar_tcp)
{
	if ((6 != cur_tcp.size()) || (6 != cur_tcp.size()))
	{
		return 0.0;
	}

	double depth = sqrt(pow((cur_tcp.at(0) - tar_tcp.at(0)),
	                        2) + pow((cur_tcp.at(1) - tar_tcp.at(1)),
	                                 2) + pow((cur_tcp.at(2) - tar_tcp.at(2)), 2));
	return depth;
}

/**
 * @brief 使用逆运动学做调整
 *
 * @param[in] status
 * @param[in] adjustment
 * @param[out] target_tcp
 * @return true
 * @return false
 */
bool jaka_robot_driver::kine_inverse_adjustment(RobotStatus &status,
        std::vector<double> &adjustment, std::vector<double> &target_tcp,
        double joint_acc, double joint_vel)
{
	if (6 != adjustment.size())
	{
		RCLCPP_ERROR(LOGGER,"%s adjustment size not correct", __func__);
		return false;
	}

	/* xyz rz为0 */
	adjustment.at(0) = 0.0;
	adjustment.at(1) = 0.0;
	adjustment.at(2) = 0.0;

	/* 继续调整 */
	std::vector<double> tmp_cur_tcp;
	tmp_cur_tcp.clear();
	tmp_cur_tcp.resize(6);

	for (size_t i = 0; i < 6; i++)
	{
		tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
	}

	/* 继续调整 */
	RCLCPP_INFO(LOGGER,"%s cur tcp pose %f %f %f %f %f %f\n",
	           __func__,
	           tmp_cur_tcp.at(0), tmp_cur_tcp.at(1),
	           tmp_cur_tcp.at(2), tmp_cur_tcp.at(3),
	           tmp_cur_tcp.at(4), tmp_cur_tcp.at(5));

	/* 进行transpose运算，获取基座TCP */
	std::vector<double> tmp_target_tcp;
	tmp_target_tcp.clear();
	tmp_target_tcp.resize(6);

	if (false == this->torque_control_.trans_tcp_to_base(adjustment, tmp_cur_tcp,
	        tmp_target_tcp))
	{
		RCLCPP_ERROR(LOGGER,"%s trans tcp to base failed", __func__);
		return false;
	}

	target_tcp = tmp_target_tcp;
	RCLCPP_INFO(LOGGER,"%s target tcp pose [%f %f %f %f %f %f] \n", __func__,
	           target_tcp.at(0), target_tcp.at(1), target_tcp.at(2),
	           target_tcp.at(3), target_tcp.at(4), target_tcp.at(5));

	CartesianPose target_tcp_pose;
	target_tcp_pose.tran.x = tmp_target_tcp.at(0);
	target_tcp_pose.tran.y = tmp_target_tcp.at(1);
	target_tcp_pose.tran.z = tmp_target_tcp.at(2);
	target_tcp_pose.rpy.rx = tmp_target_tcp.at(3);
	target_tcp_pose.rpy.ry = tmp_target_tcp.at(4);
	target_tcp_pose.rpy.rz = tmp_target_tcp.at(5);

	/* 当前的关节角度作为参考 */
	JointValue ref_pos;

	for (size_t j = 0; j < 6; j++)
	{
		ref_pos.jVal[j] = status.joint_position[j];
	}

	RCLCPP_INFO(LOGGER,"%s ref joint pose %f %f %f %f %f %f\n", __func__,
	           ref_pos.jVal[0], ref_pos.jVal[1], ref_pos.jVal[2],
	           ref_pos.jVal[3], ref_pos.jVal[4], ref_pos.jVal[5]);

	/* 进行关节逆运算, 获取关节角度 */
	JointValue tar_joint_pos;
	errno_t kine_inverse_ret = ERR_SUCC;

	kine_inverse_ret = this->jaka_robot_.kine_inverse(&ref_pos, &target_tcp_pose,
	                   &tar_joint_pos);

	if (ERR_SUCC != kine_inverse_ret)
	{
		RCLCPP_ERROR(LOGGER,"%s kine inverse failed %s", __func__,
		            this->mapErr_.at(kine_inverse_ret).c_str());
		return false;
	}

	RCLCPP_INFO(LOGGER,"%s tar joint pose %f %f %f %f %f %f\n", __func__,
	           tar_joint_pos.jVal[0], tar_joint_pos.jVal[1], tar_joint_pos.jVal[2],
	           tar_joint_pos.jVal[3], tar_joint_pos.jVal[4], tar_joint_pos.jVal[5]);

	// 5. 进行关节运动
	errno_t joint_move_ret = ERR_SUCC;
	double tol = 0.5;
	joint_move_ret = this->jaka_robot_.joint_move(&tar_joint_pos, MoveMode::ABS,
	                 TRUE, joint_vel, joint_acc, tol, nullptr);

	if (ERR_SUCC != joint_move_ret)
	{
		RCLCPP_INFO(LOGGER,"%s joint move %s\n", __func__,
		           this->mapErr_.at(joint_move_ret).c_str());
		return false;
	}

	return true;
}

/**
 * @brief 服务调用进入恒力拔枪模式
 *
 * @param request
 * @param response
 * @return true
 * @return false
 */
bool jaka_robot_driver::admit_extract_service_callback(const
        std::shared_ptr<shared_message::srv::SetAdmitExtract::Request> request,
        std::shared_ptr<shared_message::srv::SetAdmitExtract::Response> response)
{
	this->admit_extract_.set_admit_adjustment(request);
	RCLCPP_INFO(LOGGER,"receive array size %lu",
	           this->admit_extract_.param_.adjustment_.size());
	RCLCPP_INFO(LOGGER,"receive max extraction %f",
	           this->admit_extract_.param_.max_extraction_);
	RCLCPP_INFO(LOGGER,"receive loop %d ft increment %f",
	           this->admit_extract_.param_.loop_,
	           this->admit_extract_.param_.ft_increment_);
	this->admit_extract_.seq_ = request->seq;
	RCLCPP_INFO(LOGGER,"%s admit extract seq %u", __func__, this->admit_extract_.seq_);
	this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_IDLE;
	this->admit_extract_.enabled_ = true;
	response->ret = 1;
	response->message = "admit extract start";

	return true;
}

/**
 * @brief 恒力拔枪调整
 *
 */
void jaka_robot_driver::admit_extract_callback()
{
	/*  */
	if (false == this->admit_extract_.enabled_)
	{
		/* 退出当前的循环 */
		return;
	}

	/* 获取当前的机器人状态 */
	RobotStatus status;
	memset(&status, 0, sizeof(RobotStatus));
	int ret = ERR_SUCC;
	ret = this->jaka_robot_.get_robot_status(&status);

	if (ERR_SUCC != ret)
	{
		RCLCPP_ERROR(LOGGER,"%s get_robot_status fail %s", __func__,
		            this->mapErr_.at(ret).c_str());
		this->admit_extract_.enabled_ = false;
		this->admit_extract_.sm_.success_ = false;
		this->admit_extract_.sm_.restart_ = false;
		this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
		return;
	}

	/* 遇到意外情况退出 */
	if (0 == status.enabled)
	{
		RCLCPP_ERROR(LOGGER,"%s encounter error, robot not enable", __func__);
		this->admit_extract_.sm_.success_ = false;
		this->admit_extract_.sm_.restart_ = false;
		this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
		return;
	}

	if (0 != status.collision_stop)
	{
		RCLCPP_ERROR(LOGGER,"%s encounter error, robot in collision", __func__);
		this->admit_extract_.sm_.success_ = false;
		this->admit_extract_.sm_.restart_ = false;
		this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
		return;
	}

	if (0 != status.emergency_stop)
	{
		RCLCPP_ERROR(LOGGER,"%s encounter error, robot in emergency", __func__);
		this->admit_extract_.sm_.success_ = false;
		this->admit_extract_.sm_.restart_ = false;
		this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
		return;
	}

	/* 获取当前的tcp坐标 */
	std::vector<double> tmp_cur_tcp;
	tmp_cur_tcp.clear();
	tmp_cur_tcp.resize(6);

	for (size_t i = 0; i < 6; i++)
	{
		tmp_cur_tcp.at(i) = status.cartesiantran_position[i];
	}

	switch (this->admit_extract_.sm_.state_)
	{
		case admit_extract::ADMIT_EXTRACT_IDLE:
		{
			// 设置恒力参数
			this->admit_extract_.sm_.success_ = false;
			this->admit_extract_.sm_.wait_count_ = 0;
			this->admit_extract_.sm_.last_depth_ = 0.0;
			/* 重新启动, 力控不清零 */
			errno_t set_compliant_type_ret = ERR_SUCC;

			if (this->admit_extract_.sm_.restart_)
			{
				RCLCPP_INFO(LOGGER,"restart admit control with higher ft");
				this->admit_extract_.sm_.restart_ = false;
				this->admit_extract_.param_.ft_constant_ =
				    this->admit_extract_.param_.ft_constant_ +
				    this->admit_extract_.param_.ft_increment_;
				this->admit_extract_.param_.adjustment_ =
				    this->admit_extract_.param_.adjustment_backup_;
				this->jaka_robot_.set_admit_ctrl_config(2, 1,
				                                        this->admit_extract_.param_.ft_damping_,
				                                        this->admit_extract_.param_.ft_constant_, 0, 0);
				RCLCPP_INFO(LOGGER,"%s set ft_damping %f ft_constant %f", __func__,
				           this->admit_extract_.param_.ft_damping_,
				           this->admit_extract_.param_.ft_constant_);
				this->jaka_robot_.set_admit_ctrl_config(0, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(1, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(3, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(4, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(5, 0, 0, 0, 0, 0);
				set_compliant_type_ret = this->jaka_robot_.set_compliant_type(0, 1);
				RCLCPP_INFO(LOGGER,"%s, set_compliant_type(0, 1) ret %d\n", __func__,
				           set_compliant_type_ret);
			}
			else
			{
				/* 第一次设置起点 以后都用这个点去计算 */
				this->admit_extract_.sm_.origin_ = tmp_cur_tcp;
				RCLCPP_INFO(LOGGER,"%s set origin %f %f %f %f %f %f", __func__,
				           this->admit_extract_.sm_.origin_.at(0),
				           this->admit_extract_.sm_.origin_.at(1),
				           this->admit_extract_.sm_.origin_.at(2),
				           this->admit_extract_.sm_.origin_.at(3),
				           this->admit_extract_.sm_.origin_.at(4),
				           this->admit_extract_.sm_.origin_.at(5));
				set_compliant_type_ret = this->jaka_robot_.set_compliant_type(1, 0);
				RCLCPP_INFO(LOGGER,"%s, act torque should be zero, ret %d\n", __func__,
				           set_compliant_type_ret);
				this->jaka_robot_.set_admit_ctrl_config(2, 1,
				                                        this->admit_extract_.param_.ft_damping_,
				                                        this->admit_extract_.param_.ft_constant_, 0, 0);
				RCLCPP_INFO(LOGGER,"%s set ft_damping %f ft_constant %f", __func__,
				           this->admit_extract_.param_.ft_damping_,
				           this->admit_extract_.param_.ft_constant_);
				this->jaka_robot_.set_admit_ctrl_config(0, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(1, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(3, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(4, 0, 0, 0, 0, 0);
				this->jaka_robot_.set_admit_ctrl_config(5, 0, 0, 0, 0, 0);
				set_compliant_type_ret = this->jaka_robot_.set_compliant_type(1, 1);
				RCLCPP_INFO(LOGGER,"%s, set_compliant_type(1, 1) ret %d\n", __func__,
				           set_compliant_type_ret);
			}

			std::vector<double> adjustment;
			adjustment.clear();
			adjustment.resize(6);

			for (size_t i = 0; i < adjustment.size(); i++)
			{
				adjustment.at(i) = 0.0;
			}

			/* 进行调整 */
			if (false == this->kine_inverse_adjustment(status, adjustment,
			        this->admit_extract_.sm_.target_, 0.2, 0.2))
			{
				this->admit_extract_.sm_.success_ = false;
				this->admit_extract_.sm_.restart_ = false;
				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
				break;
			}
			else
			{
				RCLCPP_INFO(LOGGER,"%s send init adjust cmd, adjust times %d", __func__,
				           this->admit_extract_.sm_.adjust_count_);
			}

			this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_ADJUST;
			break;
		}

		case admit_extract::ADMIT_EXTRACT_ADJUST:
		{
			/* 这个状态里面机械臂应该不进行调整 */
			double tmp_depth = this->soft_charger_interface_->depthCalculator(
			                       this->admit_extract_.sm_.origin_, tmp_cur_tcp);
			// PRINT_DATA;

			if ((this->admit_extract_.param_.max_extraction_ < fabs(tmp_depth))
			        && (false == this->admit_extract_.sm_.success_))
			{
				this->admit_extract_.sm_.success_ = true;
				this->admit_extract_.sm_.restart_ = false;
				this->jaka_robot_.set_compliant_type(0, 0);
				this->jaka_robot_.motion_abort();
				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN;
				this->admit_extract_.sm_.last_depth_ = tmp_depth;
				RCLCPP_INFO(LOGGER,"%s depth satisfy, depth %f", __func__, tmp_depth);
				break;
			}

			if (this->admit_extract_.sm_.success_)
			{
				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN;
				this->admit_extract_.sm_.last_depth_ = tmp_depth;
				RCLCPP_INFO(LOGGER,"%s depth satisfy success", __func__);
				break;
			}

			if (this->admit_extract_.admit_adjust_after_inpos_)
			{
				if ((this->admit_extract_.depth_thres_ < fabs(tmp_depth -
				        this->admit_extract_.sm_.last_depth_))
				        || ((this->admit_extract_.force_thres_ >
				             (this->admit_extract_.param_.ft_constant_ +
				              status.torq_sensor_monitor_data.actTorque[2]))
				            && (10.0 < fabs(status.torq_sensor_monitor_data.actTorque[2])))
				        || (0 == status.inpos))
				{
					/* 继续保持当前角度, 不调整 */
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_ADJUST;
					this->admit_extract_.sm_.last_depth_ = tmp_depth;
					RCLCPP_INFO(LOGGER,"%s continue current dir, depth %f, act torque %f", __func__,
					           tmp_depth, status.torq_sensor_monitor_data.actTorque[2]);
					break;
				}
				else
				{
					RCLCPP_INFO(LOGGER,"%s extract not moving, do adjust", __func__);
					RCLCPP_INFO(LOGGER,"%s tmp_depth %f last depth %f", __func__, tmp_depth,
					           this->admit_extract_.sm_.last_depth_);
					this->admit_extract_.sm_.last_depth_ = tmp_depth;
				}
			}
			else
			{
				if ((this->admit_extract_.depth_thres_ < fabs(tmp_depth -
				        this->admit_extract_.sm_.last_depth_))
				        || ((this->admit_extract_.force_thres_ >
				             (this->admit_extract_.param_.ft_constant_ +
				              status.torq_sensor_monitor_data.actTorque[2]))
				            && (10.0 < fabs(status.torq_sensor_monitor_data.actTorque[2]))))
				{
					/* 继续保持当前角度, 不调整 */
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_ADJUST;
					this->admit_extract_.sm_.last_depth_ = tmp_depth;
					RCLCPP_INFO(LOGGER,"%s continue current dir, depth %f, act torque %f", __func__,
					           tmp_depth, status.torq_sensor_monitor_data.actTorque[2]);
					break;
				}
				else
				{
					RCLCPP_INFO(LOGGER,"%s extract not moving, do adjust", __func__);
					RCLCPP_INFO(LOGGER,"%s tmp_depth %f last depth %f", __func__, tmp_depth,
					           this->admit_extract_.sm_.last_depth_);
					this->admit_extract_.sm_.last_depth_ = tmp_depth;
				}
			}

			/* 如果调整次数还没到 */
			if (this->admit_extract_.param_.adjustment_.begin()->times_ >
			        this->admit_extract_.sm_.adjust_count_)
			{
				std::vector<double> adjustment;
				adjustment.clear();
				adjustment.resize(6);
				adjustment.at(3) =
				    this->admit_extract_.param_.adjustment_.begin()->coord_.at(0);
				adjustment.at(4) =
				    this->admit_extract_.param_.adjustment_.begin()->coord_.at(1);
				adjustment.at(5) =
				    this->admit_extract_.param_.adjustment_.begin()->coord_.at(2);
				RCLCPP_INFO(LOGGER,
				    "%s adjustment %f %f %f %f %f %f\n", __func__,
				    adjustment.at(0), adjustment.at(1), adjustment.at(2),
				    adjustment.at(3), adjustment.at(4), adjustment.at(5));

				/* 进行调整 */
				if (false == this->kine_inverse_adjustment(status, adjustment,
				        this->admit_extract_.sm_.target_,
				        this->admit_extract_.param_.adjustment_.begin()->joint_acc_,
				        this->admit_extract_.param_.adjustment_.begin()->joint_vel_))
				{
					this->admit_extract_.sm_.success_ = false;
					this->admit_extract_.sm_.restart_ = false;
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
					break;
				}
				else
				{
					this->admit_extract_.sm_.adjust_count_ ++;
					this->admit_extract_.sm_.wait_count_ = 0;
					RCLCPP_INFO(LOGGER,"%s send adjust cmd, adjust times %d", __func__,
					           this->admit_extract_.sm_.adjust_count_);
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_WAIT_ADJUST;
					this->admit_extract_.sm_.last_depth_ = tmp_depth;
				}
			}
			else
			{
				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN;
				this->admit_extract_.sm_.last_depth_ = tmp_depth;
			}

			break;
		}

		case admit_extract::ADMIT_EXTRACT_WAIT_ADJUST:
		{
			double tmp_depth = this->soft_charger_interface_->depthCalculator(
			                       this->admit_extract_.sm_.origin_, tmp_cur_tcp);
			// PRINT_DATA;

			static uint32_t wait_adjust_count = 0;

			if (0 == (wait_adjust_count % 50))
			{
				RCLCPP_INFO(LOGGER,"%s tmp_depth %f", __func__, tmp_depth);
			}

			wait_adjust_count ++;

			if ((this->admit_extract_.param_.max_extraction_ < fabs(tmp_depth))
			        && (false == this->admit_extract_.sm_.success_))
			{
				this->admit_extract_.sm_.success_ = true;
				this->admit_extract_.sm_.restart_ = false;
				this->jaka_robot_.set_compliant_type(0, 0);
				RCLCPP_INFO(LOGGER,"%s depth %f statisfy", __func__, tmp_depth);
			}

			if ((false == is_admit_adjust_in_pos(tmp_cur_tcp,
			                                     this->admit_extract_.sm_.target_)) && (0 == status.inpos))
			{
				/* 当前调整未完成, 继续等待 */
				static uint32_t wait_admit_adjust_count = 0;

				if (0 == (wait_admit_adjust_count % 10))
				{
					RCLCPP_INFO(LOGGER,"%s wait admit adjust in pos\n", __func__);
				}

				wait_admit_adjust_count ++;
				this->admit_extract_.sm_.wait_count_ ++;

				if (50 < this->admit_extract_.sm_.wait_count_)
				{
					this->admit_extract_.sm_.success_ = false;
					this->admit_extract_.sm_.restart_ = false;
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
				}

				break;
			}
			else
			{
				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_ADJUST;
				RCLCPP_INFO(LOGGER,"%s adjust target arrive", __func__);
			}

			break;
		}

		case admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN:
		{
			/* 这个状态下, 无论是什么情况，都要把所有的调整次数都归零 */
			double tmp_depth = this->soft_charger_interface_->depthCalculator(
			                       this->admit_extract_.sm_.origin_, tmp_cur_tcp);
			// PRINT_DATA;

			if ((this->admit_extract_.param_.max_extraction_ < fabs(tmp_depth))
			        && (false == this->admit_extract_.sm_.success_))
			{
				this->admit_extract_.sm_.success_ = true;
				this->admit_extract_.sm_.restart_ = false;
				this->jaka_robot_.set_compliant_type(0, 0);
				this->jaka_robot_.motion_abort();
				RCLCPP_INFO(LOGGER,"%s depth satisfy %f", __func__, tmp_depth);
			}

			if (this->admit_extract_.sm_.success_)
			{
				this->jaka_robot_.motion_abort();
				this->admit_extract_.sm_.restart_ = false;
				RCLCPP_INFO(LOGGER,"%s depth satisfy motion abort", __func__);
				this->admit_extract_.sm_.last_depth_ = tmp_depth;
			}
			else /* 还没有成功 */
			{
				if (this->admit_extract_.admit_adjust_after_inpos_)
				{
					if ((this->admit_extract_.depth_thres_ < fabs(tmp_depth -
					        this->admit_extract_.sm_.last_depth_))
					        || (this->admit_extract_.force_thres_ >
					            (this->admit_extract_.param_.ft_constant_ +
					             status.torq_sensor_monitor_data.actTorque[2]))
					        || (0 == status.inpos))
					{
						/* 能移动的话继续保持当前角度, 不调整 */
						this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN;
						this->admit_extract_.sm_.last_depth_ = tmp_depth;
						RCLCPP_INFO(LOGGER,"%s continue current dir, depth %f, act torque %f", __func__,
						           tmp_depth, status.torq_sensor_monitor_data.actTorque[2]);
						break;
					}
					else
					{
						RCLCPP_INFO(LOGGER,"%s back to origin not moving, do adjust", __func__);
						RCLCPP_INFO(LOGGER,"%s tmp_depth %f last depth %f", __func__, tmp_depth,
						           this->admit_extract_.sm_.last_depth_);
						this->admit_extract_.sm_.last_depth_ = tmp_depth;
					}
				}
				else
				{
					if ((this->admit_extract_.depth_thres_ < fabs(tmp_depth -
					        this->admit_extract_.sm_.last_depth_))
					        || (this->admit_extract_.force_thres_ >
					            (this->admit_extract_.param_.ft_constant_ +
					             status.torq_sensor_monitor_data.actTorque[2])))
					{
						/* 能移动的话继续保持当前角度, 不调整 */
						this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN;
						this->admit_extract_.sm_.last_depth_ = tmp_depth;
						RCLCPP_INFO(LOGGER,"%s continue current dir, depth %f, act torque %f", __func__,
						           tmp_depth, status.torq_sensor_monitor_data.actTorque[2]);
						break;
					}
					else
					{
						RCLCPP_INFO(LOGGER,"%s back to origin not moving, do adjust", __func__);
						RCLCPP_INFO(LOGGER,"%s tmp_depth %f last depth %f", __func__, tmp_depth,
						           this->admit_extract_.sm_.last_depth_);
						this->admit_extract_.sm_.last_depth_ = tmp_depth;
					}
				}
			}

			/* 往反方向调整, 让调整次数归零 */
			if (0 < this->admit_extract_.sm_.adjust_count_)
			{
				std::vector<double> adjustment;
				adjustment.clear();
				adjustment.resize(6);
				adjustment.at(3) = -this->admit_extract_.get_rx_inc();
				adjustment.at(4) = -this->admit_extract_.get_ry_inc();
				adjustment.at(5) = -this->admit_extract_.get_rz_inc();
				RCLCPP_INFO(LOGGER,
				    "%s adjustment %f %f %f %f %f %f\n", __func__,
				    adjustment.at(0), adjustment.at(1), adjustment.at(2),
				    adjustment.at(3), adjustment.at(4), adjustment.at(5));

				/* 进行调整 */
				if (false == this->kine_inverse_adjustment(status, adjustment,
				        this->admit_extract_.sm_.target_))
				{
					this->admit_extract_.sm_.success_ = false;
					this->admit_extract_.sm_.restart_ = false;
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
					break;
				}
				else
				{
					this->admit_extract_.sm_.adjust_count_ --;
					this->admit_extract_.sm_.wait_count_ = 0;
					RCLCPP_INFO(LOGGER,"%s send back to origin cmd, adjust times %d", __func__,
					           this->admit_extract_.sm_.adjust_count_);
					this->admit_extract_.sm_.state_ =
					    admit_extract::ADMIT_EXTRACT_WAIT_BACK_TO_ORIGIN;
				}
			}
			else
			{
				if (this->admit_extract_.sm_.success_)
				{
					/* 已经调整成功了 后续不需要再调整 */
					this->admit_extract_.param_.adjustment_.clear();
					this->admit_extract_.sm_.restart_ = false;
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
					break;
				}
				else
				{
					/* 已经归零, 当前一个调整完成 */
					if (!this->admit_extract_.param_.adjustment_.empty())
					{
						this->admit_extract_.param_.adjustment_.erase(
						    this->admit_extract_.param_.adjustment_.begin());
						RCLCPP_INFO(LOGGER,"%s current adjust end, goto next, remain size %lu", __func__,
						           this->admit_extract_.param_.adjustment_.size());
					}

					/* 如果所有的调整都完成 */
					if (this->admit_extract_.param_.adjustment_.empty())
					{
						if ((!this->admit_extract_.sm_.success_) && (this->admit_extract_.param_.loop_))
						{
							RCLCPP_INFO(LOGGER,"admit extract fail after all adjustment");
							this->admit_extract_.sm_.restart_ = true;
						}
						else
						{
							this->admit_extract_.sm_.restart_ = false;
						}

						this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
						RCLCPP_INFO(LOGGER,"%s adjust finish", __func__);
						break;
					}
					else
					{
						this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_ADJUST;
						RCLCPP_INFO(LOGGER,"%s goto next adjust", __func__);
						break;
					}
				}

				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN;
			}

			break;
		}

		case admit_extract::ADMIT_EXTRACT_WAIT_BACK_TO_ORIGIN:
		{
			double tmp_depth = this->soft_charger_interface_->depthCalculator(
			                       this->admit_extract_.sm_.origin_, tmp_cur_tcp);
			// PRINT_DATA;

			static uint32_t wait_back_depth_count = 0;

			if (0 == (wait_back_depth_count % 50))
			{
				RCLCPP_INFO(LOGGER,"%s tmp_depth %f", __func__, tmp_depth);
			}

			wait_back_depth_count ++;

			if ((this->admit_extract_.param_.max_extraction_ < fabs(tmp_depth))
			        && (false == this->admit_extract_.sm_.success_))
			{
				this->admit_extract_.sm_.success_ = true;
				this->admit_extract_.sm_.restart_ = false;
				this->jaka_robot_.set_compliant_type(0, 0);
				RCLCPP_INFO(LOGGER,"%s depth %f statisfy", __func__, tmp_depth);
			}

			if ((false == is_admit_adjust_in_pos(tmp_cur_tcp,
			                                     this->admit_extract_.sm_.target_)) && (0 == status.inpos))
			{
				/* 当前调整未完成, 继续等待 */
				static uint32_t wait_admit_back_count = 0;

				if (0 == (wait_admit_back_count % 20))
				{
					RCLCPP_INFO(LOGGER,"%s wait admit adjust back in pos\n", __func__);
				}

				wait_admit_back_count ++;
				this->admit_extract_.sm_.wait_count_ ++;

				if (50 < this->admit_extract_.sm_.wait_count_)
				{
					this->admit_extract_.sm_.success_ = false;
					this->admit_extract_.sm_.restart_ = false;
					this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_FINISH;
				}

				break;
			}
			else
			{
				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_BACK_TO_ORIGIN;
				RCLCPP_INFO(LOGGER,"%s back to origin target arrive", __func__);
			}

			break;
		}

		case admit_extract::ADMIT_EXTRACT_FINISH:
		{
			this->jaka_robot_.set_compliant_type(0, 0);
			this->jaka_robot_.motion_abort();
			RCLCPP_INFO(LOGGER,"%s stop finish", __func__);
			// PRINT_DATA;

			if ((true == this->admit_extract_.sm_.restart_)
			        && (this->admit_extract_.param_.loop_))
			{
				/* TODO: 设定一个循环次数上限, 或者力的上限 */
				RCLCPP_INFO(LOGGER,"unable finish in this round");
				this->admit_extract_.sm_.state_ = admit_extract::ADMIT_EXTRACT_IDLE;
				break;
			}

			this->admit_extract_.enabled_ = false;
			break;
		}

		default:
		{
			this->jaka_robot_.set_compliant_type(0, 0);
			this->jaka_robot_.motion_abort();
			RCLCPP_INFO(LOGGER,"%s stop finish", __func__);
			this->admit_extract_.enabled_ = false;
			break;
		}
	}

	return;
}

}

