#ifndef ROBOT_DRIVER_STATUS_
#define ROBOT_DRIVER_STATUS_
#include <mutex>
#include <vector>
#include "rclcpp/rclcpp.hpp"
class robot_driver_status
{
public:
	robot_driver_status();
	std::mutex
	robot_status_lock_;                   /* 处理机器人状态消息的互斥锁 */
	bool connect_status_;                 /* 连接状态 */
	bool enable_status_;                  /* 使能状态 */
	bool is_robot_stop_;                  /* 机器人是否停止 */
	bool in_collision_;                   /* 处在碰撞状态 */
	bool in_emergency_;                   /* 处在急停状态 */
	bool is_free_drive_;                  /* 是否处在自由模式 */
	bool in_pos_;                         /* 到位 */
	int errcode_;                         /* 错误码 */
	std::vector<double> tcp_pose_;        /* 工具TCP姿态 */
	std::vector<double> joint_pose_;      /* 关节姿态 */
	int tool_id_;                         /* 工具编号 */
	std::vector<double> tool_tcp_;        /* 工具TCP */
	double mass_;                         /* 负载重量 */
	std::vector<double> center_of_mass_;  /* 负载重心 */
	bool force_control_enable_;           /* 是否处在力控模式 */
	uint32_t force_control_seq_;          /* 力控序列号 */
	bool torque_adjust_enable_;           /* 是否正在进行力控调整 */
	uint32_t force_adjust_seq_;           /* 力控调整序列号 */
	bool torque_extract_adjust_enable_;           /* 是否正在进行力控调整 */
	uint32_t force_extract_adjust_seq_;           /* 力控调整序列号 */
	bool admit_extract_enable_;           /* 是否正在进行力控调整 */
	bool admit_extract_success_;           /* 是否正在进行力控调整 */
	uint32_t admit_extract_seq_;           /* 力控调整序列号 */
	int collision_level_;                 /* 碰撞等级 */
	std::vector<double> torque_;          /* 原始力矩 */
	std::vector<double> act_torque_;      /* 归零后的力矩 */
	std::vector<double> j_torque_;        /* 每一个关节的力矩 */
	std::vector<double> j_current_;       /* 每一个关节的电流 */
	builtin_interfaces::msg::Time ts_;    /* 时间戳 */
	std::vector<std::vector<double>>
	                              joint_base_position_; /* 关节的基座坐标 */
	uint8_t arm_pose_;                    /* 姿态范围 */

	/**
	 * @brief 机器人的连接状态
	 *
	 * 判断上报的标志位
	 */
	inline bool connect_status()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->connect_status_;
	}

	/**
	 * @brief 机器人的使能状态
	 *
	 * 判断上报的标志位
	 */
	inline bool enable_status()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->enable_status_;
	}

	/**
	 * @brief 机器人是否已经停止
	 *
	 * 判断上报的标志位
	 */
	inline bool is_robot_stop()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->is_robot_stop_;
	}

	/**
	 * @brief 是否处于碰撞状态
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_in_collision()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->in_collision_;
	}

	/**
	 * @brief 是否处于急停状态
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_in_emergency()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->in_emergency_;
	}

	/**
	 * @brief 是否处于力控模式
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_force_control_enable()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->force_control_enable_;
	}

	/**
	 * @brief 力控模式序列号
	 *
	 * @return seq
	 */
	inline uint32_t force_control_seq()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->force_control_seq_;
	}

	/**
	 * @brief 是否处在自由模式
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_in_free_drive_mode()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->is_free_drive_;
	}

	/**
	 * @brief 是否正在进行力控调整
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_in_torque_adjust()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->torque_adjust_enable_;
	}

	/**
	 * @brief 力控调整序列号
	 *
	 * @return seq
	 */
	inline uint32_t torque_adjust_seq()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->force_adjust_seq_;
	}

	/**
	 * @brief 获取当前的错误码
	 *
	 * @return int
	 */
	inline int robot_error()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->errcode_;
	}

	/**
	 * @brief 获取当前的碰撞等级
	 *
	 * @return int
	 */
	inline int collision_level()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->collision_level_;
	}

	/**
	 * @brief 获取当前的基座TCP坐标
	 *
	 * @return std::vector<double>
	 */
	inline std::vector<double> cur_tcp_pose()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->tcp_pose_;
	}

	/**
	 * @brief 获取当前的关节坐标
	 *
	 * @return std::vector<double>
	 */
	inline std::vector<double> cur_joint_pose()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->joint_pose_;
	}

	/**
	 * @brief 获取当前的力控原始数据
	 *
	 * @return std::vector<double>
	 */
	inline std::vector<double> cur_torque()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->torque_;
	}

	/**
	 * @brief 获取当前的归零后力控数据
	 *
	 * @return std::vector<double>
	 */
	inline std::vector<double> cur_act_torque()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->act_torque_;
	}

	/**
	 * @brief 获取关节坐标值
	 *
	 * @param[in] joint_id
	 *
	 * @return bool
	 */
	inline bool get_joint_base_position(uint8_t joint_id,
	                                    std::vector<double> &joint_base_position)
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);

		if (this->joint_base_position_.empty())
		{
			return false;
		}
		else if (this->joint_base_position_.size() < joint_id)
		{
			return false;
		}
		else
		{
			if (this->joint_base_position_.at(joint_id).empty())
			{
				return false;
			}
			else
			{
				joint_base_position = this->joint_base_position_.at(joint_id);
			}
		}

		return true;
	}

	/**
	 * @brief 是否正在进行力控调整
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_in_torque_extract_adjust()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->torque_extract_adjust_enable_;
	}

	/**
	 * @brief 力控调整序列号
	 *
	 * @return seq
	 */
	inline uint32_t torque_extract_adjust_seq()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->force_extract_adjust_seq_;
	}

	/**
	 * @brief 是否正在进行力控调整
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_in_admit_extract()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->admit_extract_enable_;
	}

	/**
	 * @brief 是否正在进行力控调整
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_admit_extract_success()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->admit_extract_success_;
	}

	/**
	 * @brief 力控调整序列号
	 *
	 * @return seq
	 */
	inline uint32_t admit_extract_seq()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->admit_extract_seq_;
	}

	/**
	 * @brief 是否处于到位状态
	 *
	 * @return true
	 * @return false
	 */
	inline bool is_in_pos()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->in_pos_;
	}

	/**
	 * @brief 获取当前的关节力矩
	 *
	 * @return std::vector<double>
	 */
	inline std::vector<double> cur_j_torque()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->j_torque_;
	}

	/**
	 * @brief 获取当前的关节电流
	 *
	 * @return std::vector<double>
	 */
	inline std::vector<double> cur_j_current()
	{
		std::lock_guard<std::mutex> lock(this->robot_status_lock_);
		return this->j_current_;
	}
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

#endif //ROBOT_DRIVER_STATUS_