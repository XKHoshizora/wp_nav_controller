# 导航命令类型
uint8 STOP = 0           # 停止当前导航
uint8 PAUSE = 1         # 暂停导航
uint8 RESUME = 2        # 恢复导航
uint8 GOTO_WAYPOINT = 3 # 导航到指定航点
uint8 GOTO_POSE = 4     # 导航到指定位置
uint8 START_SEQUENCE = 5 # 开始执行航点序列
uint8 STOP_SEQUENCE = 6  # 停止序列执行

# 命令类型
uint8 command

# 目标航点索引 (用于GOTO_WAYPOINT)
int32 waypoint_index

# 目标位置 (用于GOTO_POSE)
geometry_msgs/PoseStamped target_pose

# 序列执行方向 (用于START_SEQUENCE)
bool reverse

# 可选的命令参数
string params