#ifndef SOFT_INSERTER_INTERFACE_HPP
#define SOFT_INSERTER_INTERFACE_HPP

#include <memory>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

struct ControllerParams
{
    double KpFx;
    double KpFy;
    double KiFx;
    double KiFy;
    double KdFx;
    double KdFy;

    double ignore_Fx_lower;
    double ignore_Fx_upper;

    double ignore_Fy_lower;
    double ignore_Fy_upper;

    double desired_fx;
    double desired_fy;

    double err_fx;
    double err_fy;

    double last_err_fx;
    double last_err_fy;

    double integral_fx;
    double integral_fy;
};

class SoftInserterInterface
{
public:
    typedef std::shared_ptr<SoftInserterInterface> Ptr_;

    SoftInserterInterface();

    /**
     * @brief 构造函数

     * @param[in] ini_force_tcp 当前基于tcp坐标的力控值
     * @param[in] dh_alpha 连杆扭角alpha
     * @param[in] dh_a 连杆长度a
     * @param[in] dh_d 偏置距离d
     * @param[in] dh_joint_homeoff 关节角theta偏移
     */
    SoftInserterInterface(
        const std::vector<double>& dh_alpha,
        const std::vector<double>& dh_a,
        const std::vector<double>& dh_d, 
        const std::vector<double>& dh_joint_homeoff);

    ~SoftInserterInterface();

    /**
     * @brief 初始化函数
     * 
     * @param[in] config_file_path PID控制器参数路径
     * @param[in] log_file_path 日志文件路径
     */
    void init(const std::string& config_file_path, const std::string& log_file_path);

    /**
     * @brief 反初始化函数，流程结束时调用
     * 
     */
    void deinit();

    /**
     * @brief 重置函数，重置变量
     * 
     */
    void reset();

    /**
     * @brief 拔枪前调整执行的重置函数
     * 
     */
    void extraReset();

    /**
     * @brief 力控归零后设置force_bias
     * 
     * @param[out] bias 原始力控值偏差
     */
    void setForceBias(const std::vector<double>& bias);

    /**
    *  @brief 设置车型
    *  @param[in] id 车型id (1:eletre; 2:emeya)
    */
    void setCarType(const int& id);

    /**
     * @brief 传入插枪前洞口位置处的tcp坐标
     * @param[in] cur_base_tcp 当前base下tcp坐标
     **/
    void setRefBaseTcp(const std::vector<double>& cur_base_tcp);

    /**
     * @brief 传入插枪到底时的tcp坐标
     * @param[in] cur_base_tcp 当前base下tcp坐标
     **/
    void setDeepestBaseTcp(const std::vector<double>& cur_base_tcp);

    /**
     * @brief 传入插枪到底时tcp坐标系下的力控值
     * @param[in] cur_tcp_force 当前tcp坐标系下的力控值
     **/
    void setDeepestTcpForce(const std::vector<double>& cur_tcp_force);

    /**
     * @brief 力控清零函数，传入N个原始力控值，计算出力控值偏差
     * 
     * @param[in] cur_tcp_force 通过sdk获取到的6维力控值
     * @return true 已经完成清零操作
     * @return false 还未完成清零操作
     */
    bool zeroForceSensor(const std::vector<double>& cur_tcp_force);

    /**
     * @brief 获取减去负载后的6维力控值
     * 
     * @param[in] cur_tcp_force 当前通过sdk获取到的6维力控值
     * @param[out] actual_tcp_force 减去负载后的6维力控值
     */
    void getActualTcpForce(const std::vector<double>& cur_tcp_force, std::vector<double>& actual_tcp_force);

    /**
     * @brief 计算已插入深度
     * 
     * @param[in] cur_pose_base_tcp 当前base系下tcp坐标
     * @return 已插入深度
     */
    double calculateInsertDepth(const std::vector<double>& cur_pose_base_tcp);

    /**
     * @brief 输入当前原始的力控值，输出机械臂调整量
     * 
     * @param[in] cur_tcp_force: 当前通过sdk获取到的6维力控值
     * @param[out] adjustment: 机械臂调整量[tx, ty, tx, rx, ry, rz]
     */
    void plan(const std::vector<double>& cur_tcp_force, std::vector<double>& adjustment);

    /**
     * @brief 输入多个当前原始的力控值，输出机械臂调整量
     *  
     * @param[in] cur_tcp_forces: 当前通过sdk获取到的多个原始6维力控值
     * @param[out] adjustment: 机械臂调整量[tx, ty, tx, rx, ry, rz]
     */
    void plan(std::vector<std::vector<double>> cur_tcp_forces, std::vector<double>& adjustment);

    /**
     * @brief 二阶段插枪到fz达到130N后，检测插入深度<15mm时执行此函数输出调整量。为防止机械臂在充电口深处报死，此方法只允许调用一次。
     * @param[in] inserted_depth: plan后插入的深度(mm)
     * @param[in] cur_tcp_forces: 当前通过sdk获取到的多个原始6维力控值 (actual_torque)
     * @param[out] adjustment: 机械臂调整量[tx, ty, tx, rx, ry, rz]
     *
     */ 
    void deepPlan(
        const double& inserted_depth, 
        const std::vector<std::vector<double>>& cur_tcp_forces,
        std::vector<double>& adjustment);
    
    /**
     * deepPlan后调用，输入tcp及force，输出调整量
     * @param[in] cur_tcp_force: 当前通过sdk获取到的6维力控值 (actual_torque)
     * @param[in] pose_base_tcp 当前base系下tcp坐标
     * @param[out] adjustment: 机械臂调整量[tx, ty, tx, rx, ry, rz]
     */
    void decouplingSearch(
        const std::vector<std::vector<double>>& cur_tcp_forces, 
        const std::vector<std::vector<double>>& poses_base_tcp,
        std::vector<double>& adjustment);

    /**
     * @brief 输入多个当前原始的力控值，输出拔枪时机械臂的调整量
     *  
     * @param[in] cur_tcp_forces: 当前通过sdk获取到的多个原始6维力控值
     * @param[in] pose_base_tcp 当前base系下tcp坐标
     * @param[out] adjustment: 机械臂调整量[tx, ty, tx, rx, ry, rz]
     */
    void planExtraction(std::vector<std::vector<double>> cur_tcp_forces, std::vector<double> pose_base_tcp, std::vector<double>& adjustment);

    /**
     * @brief 输入当前力控坐标，输出每个joint的相对于joint0的6dim位姿
     *  
     * @param[in] joint_pos: 当前通过sdk获取到的多个原始6维力控值
     * @param[out] cartesian_jpos: 各个关节点的笛卡尔坐标(m,rad) [[x,y,z,rx,ry,rz], [], [], [], [], []]
     */
    void forwardKinematics(const std::vector<double>& joint_pos, std::vector<std::vector<double>>& cartesian_jpos);

    /**
    * @brief: 输入两个基于base系下的tcp坐标，计算基于tcp1.z坐标系下pose_tcp2的值。
    * @param[in] pose_tcp1: tcp1在Base系下的坐标，平移x,y,z单位m，旋转欧拉角rx,ry,rz单位弧度
    * @param[in] pose_tcp2: tcp2在Base系下的坐标，平移x,y,z单位m，旋转欧拉角rx,ry,rz单位弧度
    * @return 深度值：<0表示拔出深度，>0表示插入深度。单位m
    */ 
    double depthCalculator(const std::vector<double>& pose_tcp1, const std::vector<double>& pose_tcp2);
    
private:
    // 这里的dh_table默认值是缙云大厦2号机sdk获取的
    std::vector<double> dh_alpha_ = {0.000549081, 1.57078, 0.000307702, 0.00073042, 1.57318, -1.57704};
    std::vector<double> dh_a_ = {0, 0, 0.428363, 0.369016, 0, 0};
    std::vector<double> dh_d_ = {0.121014, 0, 0, -0.113808, 0.114212, 0.105547};
    std::vector<double> dh_joint_homeoff_ = {-0.0034191, 0.00467207, -0.0179814, -0.0033835, -0.0107694, 0};

    // 是否在做tcp标定
    bool is_tcp_calib_ = 0;

    // 车型id
    // 1:eletre; 2:emeya;
    int car_type_ = 1;

    // 插枪尝试次数
    int plan_cnt_ = 0;

    // 拔枪尝试次数
    int ex_plan_cnt_ = 0;

    // 上次调整完成的时间
    int last_success_plan_time_ = 0;

    // 上次拔枪调整完成的时间
    int last_success_extra_plan_time_ = 0;

    // 插枪前洞口处当前tcp坐标
    std::vector<double> ini_pose_base_tcp_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // 插到底时tcp坐标
    std::vector<double> deepest_pose_base_tcp_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // 插到底时tcp坐标系下的力控值
    std::vector<double> deepest_force_tcp_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    // 是否是第一次调用plan函数，用于记录调整前力控的数值
    bool first_plan_ = true;
    // 调整前力控的数值
    std::vector<double> first_force_tcp_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 期望候选值
    cv::Mat desired_fx_nominal_;
    cv::Mat desired_fy_nominal_;

    // 单次最大调整角度
    double max_adjust_r_ = 0.035;
    // 总共最大调整角度
    double max_sum_adjust_r_ = 0.11;
    // 统计rx调整角度和
    double sum_adjust_rx_ = 0.0;
    // 统计ry调整角度和
    double sum_adjust_ry_ = 0.0;

    // 在线搜索倍率深度阈值
    double two_search_ratio_depth_ = 4.0;
    double three_search_ratio_depth_ = 1.0;

    // 拔枪时 调整到刚连接上快换时的力值 加上下面的偏差
    double ex_fx_bias_ = -30.0;
    double ex_fy_bias_ = 0.0;

    // 拔枪时最大调整次数和
    double ex_max_adjust_count_ = 3;
    // 拔枪时单次最大调整角度
    double ex_max_adjust_r_ = 0.01;
    // 拔枪时总共最大调整角度
    double ex_sum_adjust_r_ = 0.02;

    // Adjustments Params
    // 原始6维力值偏差
    std::vector<double> force_bias_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // 去除偏差的真实6维力值
    std::vector<double> actual_tcp_force_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 已调整次数
    int adjust_count_ = 0;
    // 最大调整次数和
    int max_adjust_count_ = 0;
    // 力控清零需要的数据量
    int num_force_container_ = 40;
    // 存储力控清零数据的容器
    std::vector<std::vector<double>> ori_force_container_;
    
    ControllerParams controller_params_;
    ControllerParams ex_controller_params_;

private:
    /**
     * @brief 限制调整角度不大于limit，不小于0.001rad
     * 
     * @param inp 原始调整角度
     * @param limit 最大值
     * @return double 限制后的调整角度值
     */
    double limitAdjustment(const double& inp, const double& limit);
    
    /**
     * @brief 对输入的多个当前原始力控值进行滤波
     *  
     * @param[in] cur_tcp_forces: 当前通过sdk获取到的多个原始6维力控值
     * @param[out] filtered_force: 滤波后的原始力控值
     */
    void forceContainerFilter(std::vector<std::vector<double>> cur_tcp_forces, std::vector<double>& filtered_force);

    /**
    * @brief: 根据DH参数，计算相对位姿：T_i_{i-1}
    * @param[in] α_{i-1} 单位弧度
    * @param[in] a_{i-1} 单位米
    * @param[in] d_i 单位米
    * @param[in] θ_i 单位弧度
    * @param[out] T T_i_{i-1}
    **/
    void getRelativeTFromDH(const double& alpha, const double& a, const double& d, const double& theta, Eigen::Matrix4d& T);

    /**
    * @@brief: 多项式拟合
    * @param[in] x: x
    * @param[in] y: y
    * @param[in] order: 选择的多项式阶数
    * @return[out]: 拟合的y
    **/
    Eigen::VectorXd curveFitting(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const int& order);
};

#endif