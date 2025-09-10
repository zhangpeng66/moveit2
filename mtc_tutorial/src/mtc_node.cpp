#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  /*
  设置任务属性
  group 属性：定义要控制的机械臂本体（关节组），对应MoveIt配置中的规划组名称
  eef 属性：定义末端执行器（如夹爪、吸盘等工具）对应的控制组
  ik_frame 属性：指定逆运动学计算时使用的末端坐标系，通常对应工具坐标系
  */
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop
  //******************************************1、生成阶段（CurrentState）******************************************
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));


  //PipelinePlanner使用 MoveIt 的规划管道，通常默认为OMPL。
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  //JointInterpolation是一个简单的规划器，用于在起始关节状态和目标关节状态之间进行插值。它通常用于简单运动，因为它计算速度快，但不支持复杂运动。
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  //CartesianPath用于在笛卡尔空间中沿直线移动末端执行器。
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  //******************************************2、传播阶段（propagating）******************************************
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));    

  //******************************************3、连接阶段（Connect）******************************************
  {
    // 创建路径连接阶段（从起点运动到目标位姿）
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",  // 阶段标识（用于调试和可视化）
        mtc::stages::Connect::GroupPlannerVector{ 
            { arm_group_name, sampling_planner }  // 规划组与规划器绑定
        });  // 参数说明：{机械臂组名称, 采样规划器实例}
    // 配置阶段超时（单位：秒）
    stage_move_to_pick->setTimeout(5.0);  // 若5秒内未找到可行路径则阶段失败
    // 继承父级属性（通常包括规划组、末端执行器等基础配置）
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    // 将连接阶段加入任务流水线
    task.add(std::move(stage_move_to_pick));  // 通常用于衔接抓取前的位置过渡
  }


  //******************************************4、用串行容器表示多个阶段抓取目标过程（pick object）******************************************
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  {
    /*
    下一段代码创建一个SerialContainer。这是一个可以添加到我们的任务中并可容纳多个子阶段的容器。在本例中，我们创建一个串行容器，它将包含与拾取操作相关的阶段。
    我们使用exposeTo()在新的串行容器中声明父任务中的任务属性，并使用configureInitFrom()初始化它们。这允许所包含的阶段访问这些属性。
    */
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });

    //******************************************4.1、接近物体（传播）阶段加入串行容器（approach object）******************************************
    {
      /*
      然后我们创建一个阶段来接近物体。这个阶段是一个MoveRelative阶段，它允许我们指定从当前位置的相对移动。MoveRelative是一个传播器阶段：
      它从相邻阶段接收解决方案并将其传播到下一个或上一个阶段。使用cartesian_planner找到一个涉及直线移动末端执行器的解决方案。我们设置属性，
      并设置移动的最小和最大距离。现在我们创建一条Vector3Stamped消息来指示我们想要移动的方向——在本例中，从手框架开始的 Z 方向。
      最后，我们将这个阶段添加到我们的串行容器中
      */
     //在抓取任务中，该阶段控制机械臂末端以直线运动方式，从当前位置向目标物体方向移动10-15厘米，用于执行安全的预抓取接近动作。
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // 设置可视化标记命名空间，在RViz中显示运动轨迹标记时使用的唯一命名空间标识符
      stage->properties().set("marker_ns", "approach_object");
      // 指定运动控制的基准连杆，设置机械臂末端执行器的坐标系（通常与工具坐标系一致）
      stage->properties().set("link", hand_frame);
      // 从父级任务继承配置参数，自动继承父级任务中设置的"group"属性（机械臂规划组名称）
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      // 设置相对运动的距离范围，最小移动距离：0.1米（保证有效运动检测），最大移动距离：0.15米（限制最大接近幅度）
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    //******************************************4.2、创建逆运动学计算（grasp pose IK，是一个封装器），ComputeIK属于生成器阶段。继承前序阶段的抓取位姿生成结果（generate grasp pose），GenerateGraspPose属于监视生成阶段，将封装器加入容器******************************************
    {
      /****************************************************
               Generate Grasp Pose  
      1.核心功能：自动生成围绕物体的候选抓取位姿集合
      2.生成策略：基于角度增量参数(M_PI/12=15°)在球坐标系中采样
      3.与场景联动：通过monitoredStage实时跟踪物体位置变化
     ***************************************************/
      // Sample grasp pose
      // 创建抓取姿态生成阶段
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      // 从父级继承属性配置（如机械臂组、末端执行器等参数）
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      // 设置可视化标记命名空间（在RViz中显示抓取姿态的标识）
      stage->properties().set("marker_ns", "grasp_pose");
      // 指定预抓取姿态名称（对应机械手未闭合的预设姿态）
      stage->setPreGraspPose("open");
      // 设置目标物体名称（需与场景中的物体标识匹配），//在函数MTCTaskNode::setupPlanningScene()中定义
      stage->setObject("object");
      // 设置目标物体名称（需与场景中的物体标识匹配）
      stage->setAngleDelta(M_PI / 12);
      // 绑定当前状态监控（实时获取物体位置信息）
      stage->setMonitoredStage(current_state_ptr);  

      /* 定义物体坐标系到末端执行器的抓取变换矩阵 */
      Eigen::Isometry3d grasp_frame_transform;
      // 构建复合旋转（绕X/Y/Z轴各旋转90度，用于调整抓取方向）
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      // 设置旋转矩阵部分（调整末端执行器姿态）
      grasp_frame_transform.linear() = q.matrix();
      // 设置Z轴平移量（调整抓取点与物体表面的垂直距离）
      grasp_frame_transform.translation().z() = 0.1;

      
      /* 
      Compute IK核心功能说明：
      1、​多解空间探索：通过setMaxIKSolutions(8)对每个候选抓取位姿计算最多8个逆运动学解，增加找到无碰撞路径的概率。例如当机械臂处于奇异位形时，多解策略能提高容错性。
      2、​解空间过滤机制：setMinSolutionDistance(1.0)确保相邻解的关节空间距离≥1弧度，避免生成重复的关节状态。例如6轴机械臂的两个相似解可能导致后续路径规划冗余。
      3、​坐标系对齐原理：setIKFrame()通过grasp_frame_transform矩阵，将物体坐标系与末端执行器坐标系对齐。例如当物体位姿更新时，该变换矩阵能动态调整抓取姿态。
      4、​属性继承体系：configureInitFrom()实现参数继承，确保规划组、末端执行器等关键参数在流水线中一致。例如当使用多机械臂系统时，该机制能正确区分不同机器人的配置。
      
      参数说明：
      - "grasp pose IK"：阶段标识符（用于调试和可视化）
      - std::move(stage)：接收前序阶段生成的候选抓取位姿（如GenerateGraspPose的输出）
      */
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // 设置最大逆运动学解数量（平衡计算效率与成功率），典型值范围：5-10，过多会增加计算时间，过少可能无法找到可行解
      wrapper->setMaxIKSolutions(8);
      // 设置解之间的最小关节距离阈值（过滤相似解），单位：弧度（根据机械臂尺寸调整）阈值越高，解的差异性越大，避免生成过于接近的关节状态*/
      wrapper->setMinSolutionDistance(1.0);
  
      /*
      作用：指定末端执行器相对于物体的预期抓取姿态，参数解析：
      - grasp_frame_transform：物体坐标系到末端执行器的变换矩阵
      - hand_frame：末端执行器链接名称（如”panda_hand“）
      */
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      /* 
      继承父级属性（末端执行器和规划组参数），关键继承参数：
      - "eef"：末端执行器名称（与URDF定义一致）
      - "group"：规划组名称（如"arm_group"）
      */
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      /* 
      配置接口属性（接收前序阶段生成的位姿目标），数据流说明：
      从GenerateGraspPose阶段获取"target_pose"属性，作为逆运动学的输入位姿集合
      */
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      // 将封装器阶段插入串行容器
      grasp->insert(std::move(wrapper));
    }

    //******************************************4.3 临时允许手部与物体碰撞（allow collision (hand,object)），ModifyPlanningScene属于传播阶段，此阶段加入串行容器******************************************
    {
        // 创建规划场景修改阶段（临时允许手部与物体碰撞）
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        // 配置碰撞规则：允许物体与手部碰撞模型的接触
        stage->allowCollisions("object",  // 目标物体名称（需与场景定义一致）
            task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),  // 获取机器人模型，手部关节组（如"gripper"），获取手部含碰撞体的链接名称集合
            true);  // 启用碰撞允许（关键配置，避免抓取时误判碰撞）
        // 将阶段插入串行容器
        grasp->insert(std::move(stage));  
    }

    //******************************************4.4 手爪闭合（close hand），MoveTo属于传播阶段，此阶段加入串行容器******************************************
    {
        // 创建关节空间运动阶段（手爪闭合动作）
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);  // 使用插值规划器
        //参数配置：
        stage->setGroup(hand_group_name);  // 指定控制的手部关节组
        stage->setGoal("close");  // 设置预定义的手部闭合姿态（需在SRDF中定义）
        // 将阶段插入串行容器
        grasp->insert(std::move(stage));  
    }

    //******************************************4.5 临时允许手部与物体碰撞（allow collision (hand,object)），ModifyPlanningScene属于传播阶段，此阶段加入串行容器******************************************
    {
        // 创建物体附着阶段（建立手部与物体的连接关系）
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        // 执行物体附着操作：
        stage->attachObject("object",    // 被操作物体标识
                            hand_frame); // 手部末端链接名称（如"gripper_link"）
        attach_object_stage = stage.get();  // 记录阶段指针（用于后续分离操作）
        // 将阶段插入串行容器
        grasp->insert(std::move(stage));    
    }
    
    //******************************************4.6 执行物体提升动作（lift object），MoveRelative属于传播阶段，此阶段加入串行容器******************************************
    {
        // 创建相对移动阶段（执行物体提升动作） 
        auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
   
        // 配置基础属性
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });  // 继承机械臂规划组
        stage->setMinMaxDistance(0.1, 0.3);  // 设置移动距离范围（最小0.1m，最大0.3m）
        stage->setIKFrame(hand_frame);       // 指定逆运动学参考坐标系为手部末端
        stage->properties().set("marker_ns", "lift_object");  // 设置可视化标记的命名空间

        /* 定义提升方向向量（世界坐标系Z轴正方向）*/
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";  // 参考坐标系设为世界坐标系
        vec.vector.z = 1.0;             // Z轴正方向（垂直向上）
        stage->setDirection(vec);       // 应用运动方向参数

        // 将阶段插入串行容器
        grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}