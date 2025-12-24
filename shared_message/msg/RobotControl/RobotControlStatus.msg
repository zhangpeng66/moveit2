# 机械臂状态枚举 arm_ctrl_status
uint8 ARM_CTRL_STATUS_UNINIT = 0                     # 未初始化 
uint8 ARM_CTRL_STATUS_INIT_OK = 1                    #  初始化完成 
uint8 ARM_CTRL_STATUS_INIT_FAILED = 2                # 初始化失败 
uint8 ARM_CTRL_STATUS_INITTING = 3                   # 初始化中 
uint8 ARM_CTRL_STATUS_BACK_TO_ZERO_RUNNING = 4       #  动作回零ing  
uint8 ARM_CTRL_STATUS_OUT_WITH_GUN_RUNNING = 5       #  伸出到可以做精定位识别的位置ing  
uint8 ARM_CTRL_STATUS_OUT_WITHOUT_GUN_RUNNING = 6    #  伸出到可以做快换定位识别的位置ing  
uint8 ARM_CTRL_STATUS_BACK_WITH_GUN_RUNNING = 7      #   带充电枪，收紧快换，然后收回到可以做精定位识别的位置*ing  
uint8 ARM_CTRL_STATUS_BACK_WITHOUT_GUN_RUNNING = 8   #   不带充电枪，松开快换，然后移动到可以做精定位识别的位置ing  
uint8 ARM_CTRL_STATUS_INSERT_GUN_RUNNING = 9         #   插枪中  
uint8 ARM_CTRL_STATUS_EXTRACT_GUN_RUNNING = 10       #   拔枪中  
uint8 ARM_CTRL_STATUS_BACK_TO_ZERO_FINISH = 11       #   动作回零完成  
uint8 ARM_CTRL_STATUS_OUT_WITH_GUN_FINISH = 12       #   伸出到可以做精定位识别的位置完成  
uint8 ARM_CTRL_STATUS_OUT_WITHOUT_GUN_FINISH = 13    #   伸出到可以做快换定位识别的位置完成  
uint8 ARM_CTRL_STATUS_BACK_WITH_GUN_FINISH = 14      #   带充电枪，收紧快换，然后收回到可以做精定位识别的位置完成  
uint8 ARM_CTRL_STATUS_BACK_WITHOUT_GUN_FINISH = 15   #   不带充电枪，松开快换，然后移动到可以做精定位识别的位置完成  
uint8 ARM_CTRL_STATUS_INSERT_GUN_FINISH = 16         #   插枪完成  
uint8 ARM_CTRL_STATUS_EXTRACT_GUN_FINISH = 17        #   拔枪完成  
uint8 ARM_CTRL_STATUS_ERROR = 18                     #   执行异常  
uint8 ARM_CTRL_STATUS_FORCE_STOP = 19                #   强制停止(急停或发送停止指令等)  
#   v1.1 新增  
uint8 ARM_CTRL_STATUS_BACK_TO_ZERO_WITH_CHARGER_FINISH = 30        #    ARM_CTRL_ACT_BACK_TO_ZERO_WITH_CHARGER = 14 的完成状态  
#   v2.0  
uint8 ARM_CTRL_STATUS_FIRST_SECOND_P_LOC_FINISH = 31               #    ARM_CTRL_ACT_FIRST_SECOND_P_LOC = 20 的完成状态  
uint8 ARM_CTRL_STATUS_THIRD_P_LOC_FINISH = 32                      #    ARM_CTRL_ACT_THIRD_P_LOC = 21 的完成状态  
uint8 ARM_CTRL_STATUS_INSERT_CHARGER_FINISH = 33                   #    ARM_CTRL_ACT_INSERT_CHARGER = 22 的完成状态   
uint8 ARM_CTRL_STATUS_EXTRACT_QC_FINISH = 34                       #    ARM_CTRL_ACT_EXTRACT_QC = 23 的完成状态   
uint8 ARM_CTRL_STATUS_TO_QC_LOC_FINISH = 35                        #    ARM_CTRL_ACT_TO_QC_LOC = 24 的完成状态   
uint8 ARM_CTRL_STATUS_QC_LOC_FINISH = 36                           #    ARM_CTRL_ACT_QC_LOC = 25 的完成状态   
uint8 ARM_CTRL_STATUS_INSERT_QC_FINISH = 37                        #    ARM_CTRL_ACT_INSERT_QC = 26 的完成状态   
uint8 ARM_CTRL_STATUS_EXTRACT_CHARGER_FINISH = 38                  #    ARM_CTRL_ACT_EXTRACT_CHARGER = 27 的完成状态   
uint8 ARM_CTRL_STATUS_BACK_TO_OUT_POSE_WITH_CHARGER_FINISH = 39    #    ARM_CTRL_ACT_BACK_TO_OUT_POSE_WITH_CHARGER = 28 的完成状态   
uint8 ARM_CTRL_STATUS_BACK_CHARGER_FINISH = 40                     #    ARM_CTRL_ACT_BACK_CHARGER = 34 拔出充电枪的完成状态  
uint8 ARM_CTRL_STATUS_WEIGHT_JUDGE_FINISH = 41                     #    ARM_CTRL_ACT_JUDGE_WEIGHT = 35 判断重量的完成状态  
uint8 ARM_CTRL_STATUS_STOP_FINISH = 42                             #    ARM_CTRL_ACT_JUDGE_WEIGHT = 35 判断重量的完成状态  

# 机械臂状态 arm_ctrl_status_t
uint8 arm_status 

# 机械臂的当前位姿和收起位姿在一定误差范围内
# 初始化方式 无需初始化
uint8 ARM_FOLD_POSE = 1               #   机械臂收起位姿  

# 机械臂不处于收起位姿，机械臂末端到基座中心距离在0~250mm范围内
# 初始化方式 直接发送收起指令 ARM_CTRL_ACT_BACK_TO_ZERO_WITH_CHARGER = 14
uint8 ARM_IN_CABIN_POSE = 2           #   机械臂在舱内，但不是收起位姿  

# 机械臂末端到基座中心距离在250~550mm范围内，但和出舱位姿区别较大
# 初始化方式 先移动到出舱位姿 ARM_CTRL_ACT_BACK_TO_OUT_POSE_WITH_CHARGER = 28
# 再收起 ARM_CTRL_ACT_BACK_TO_ZERO_WITH_CHARGER = 14
uint8 ARM_OUTSIDE_CABIN_POSE = 3      #   机械臂在舱外，但不在出舱位姿  

# 机械臂当前位姿和出舱位姿在一定误差范围内
# 初始化方式 直接发送收起指令 ARM_CTRL_ACT_BACK_TO_ZERO_WITH_CHARGER = 14
uint8 ARM_OUT_POSE = 4                #   机械臂在出舱位姿  

# 机械臂末端到基座中心距离大于550mm
# 初始化方式 先拔枪 ARM_CTRL_ACT_BACK_CHARGER = 34 （指令待新增）
# 拔枪前需要进行判断：
# 1. 如果滑台伸出距离超过350mm，不进行干涉判断，直接执行拔枪动作
# 2. 如果滑台伸出距离不超过350mm，需要判断机械臂有无干涉
#    a. 如果机械臂有干涉，需要人工接入判断机械臂是否带着充电枪插在充电口
#       1. 如果插在充电口，需要人工移除，之后直接移动到出舱位姿
#       2. 如果没有插在充电口，直接移动到出舱位姿
#    b. 如果机械臂没有干涉，可以执行拔枪动作
# 再移动到出舱位姿 ARM_CTRL_ACT_BACK_TO_OUT_POSE_WITH_CHARGER = 28
# 再收起 ARM_CTRL_ACT_BACK_TO_ZERO_WITH_CHARGER = 14
uint8 ARM_INSERT_POSE = 5             #   机械臂处于插枪位姿  

# 机械臂位姿
uint8 arm_pose # 见上方枚举

# 机械臂舱门干涉判断
# TRUE 机械臂和机舱有干涉
# FALSE 机械臂和机舱无干涉
bool arm_interfere_ext

# 机械臂自身干涉判断
# TRUE 机械臂自身产生干涉
# FALSE 机械臂自身无干涉
bool arm_interfere_self