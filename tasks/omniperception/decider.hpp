#ifndef OMNIPERCEPTION__DECIDER_HPP
#define OMNIPERCEPTION__DECIDER_HPP

#include <list>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "detection.hpp"
#include "io/command.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"

namespace omniperception
{
// 全向感知的决策层：拿到"后置相机看到了什么"，算出云台该往哪转。
//
// 27 赛季只有一路后置 USB 相机（主相机是海康、归自瞄用），所以这里不再持有
// 相机、也不再自己跑 YOLO —— 检测由 Perceptron 在后台线程做，结果通过队列送过来。
// 这三个方法都是纯函数（不读写任何可变成员），因此可以跨线程调用：
//   decide       跑在控制线程
//   get_target_info 跑在主线程
class Decider
{
public:
  Decider(const std::string & config_path);

  // 从后置相机的检测结果里挑一个，给出相对当前云台角度的偏转量
  io::Command decide(const std::vector<DetectionResult> & detection_queue) const;

  // 单块装甲相对相机光轴的角度（度）
  Eigen::Vector2d delta_angle(
    const std::list<auto_aim::Armor> & armors, const std::string & camera) const;

  // 发给导航的瞄准目标 (x, y, z, 装甲编号+1)
  Eigen::Vector4d get_target_info(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets) const;

private:
  // 后置 USB 相机实际使用的视场角（度），来自 yaml 的 new_fov_h / new_fov_v
  double new_fov_h_ = 0;
  double new_fov_v_ = 0;
};

}  // namespace omniperception

#endif  // OMNIPERCEPTION__DECIDER_HPP
