std_msgs/Header header
bool connect_status    # 连接标志位
bool power             # 上电标志位
bool enable            # 使能标志位
bool collision         # 碰撞标志位
bool emergency         # 急停标志位
bool is_robot_stop     # 当前运动是否停止
bool free_drive        # 自由模式标志位
bool in_pos            # 到位标志位

int32 errcode          # 错误码 0 为没有错误

# 0 示教模式 1 自动模式 2 远程模式
uint8 robot_mode

# 0 停止状态 
# 1 暂停状态
# 2 急停状态
# 3 运行状态
# 4 报警状态
# 5 碰撞状态
uint8 robot_state

# 速度倍率
float64 rapid_rate     # 运行速度倍率

# 碰撞等级
int32 collision_level  # 碰撞等级

# 机械臂位置
shared_message/JointPose joint_pose 
shared_message/TcpPose tcp_pose
geometry_msgs/Point[] joint_base_position # 各关节相对基座位置

# 工具设置
int32 tool_id
shared_message/TcpPose tool_tcp

# 负载设置
float64 mass       # 重量, 单位kg
geometry_msgs/Point cog  # 重心

# 力控数据
bool force_mode        # 力控模式
uint32 force_control_seq # 判断任务是否执行
float64 insert_depth # 相对于参考点的插入深度
bool force_freedrive   # 力控拖动模式
uint32 force_adjust_seq # 判断任务是否执行
bool torque_extract_adjust   # 力控拖动模式
uint32 force_extract_adjust_seq # 判断任务是否执行
bool admit_extract   # 力控拖动模式
uint32 admit_extract_seq # 判断任务是否执行
bool admit_extract_success
float64[] torque       # 原始数据
float64[] act_torque   # 实际数据
float64[] j_torque     # 关节力矩
float64[] j_current    # 关节电流