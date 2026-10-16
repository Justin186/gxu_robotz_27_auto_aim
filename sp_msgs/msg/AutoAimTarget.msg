# AutoAimTarget.msg — 自瞄瞄准目标
#
# 话题: auto_aim_target        方向: 视觉 → 导航
#
# 说明: 取代原先把 4 个 double 拼成逗号字符串用 std_msgs/String 发出去的做法。
#       那种写法有两个问题：std::to_string(double) 只有 6 位小数会丢位置精度；
#       字段顺序没有任何类型约束，发送端和接收端改了也不会报错。

std_msgs/Header header

float64 x         # 目标位置 x (m，世界坐标系)
float64 y         # 目标位置 y (m)
float64 z         # 目标位置 z (m)
float64 yaw       # 目标朝向 (rad)

bool tracking     # 是否处于有效跟踪；false 表示本帧无目标
