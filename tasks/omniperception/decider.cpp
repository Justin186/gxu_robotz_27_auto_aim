#include "decider.hpp"

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  new_fov_h_ = tools::read<double>(yaml, "new_fov_h");
  new_fov_v_ = tools::read<double>(yaml, "new_fov_v");
}

io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue) const
{
  if (detection_queue.empty()) return io::Command{false, false, 0, 0};

  const auto & dr = detection_queue.front();
  if (dr.armors.empty()) return io::Command{false, false, 0, 0};

  tools::logger()->info(
    "omniperceptron find {},delta yaw is {:.4f}", auto_aim::ARMOR_NAMES[dr.armors.front().name],
    dr.delta_yaw * 57.3);

  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
}

Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera) const
{
  Eigen::Vector2d delta_angle;

  // 27 赛季只有一路后置相机，所以实际走的是最后那个分支。
  // left / right 是 26 赛季四路 USB 全向感知留下的，先保留（将来加侧向相机要用），
  // 两条分支里的 ±62 是侧向相机的安装偏角。
  if (camera == "left") {
    delta_angle[0] = 62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  if (camera == "right") {
    delta_angle[0] = -62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  // 后置相机：装在云台正后方（偏角 170°），54.2 / 44.5 是它的水平 / 垂直视场角。
  // TODO: 这两个数应该从 yaml 读，现在写死是因为当初没和下位机对过实际视场。
  delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
  delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
  return delta_angle;
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets) const
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  const auto & target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name != target.name) continue;
    return Eigen::Vector4d{
      armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
      static_cast<double>(armor.name) + 1};  // 避免歧义 +1（详见通信协议）
  }

  return Eigen::Vector4d::Zero();
}

}  // namespace omniperception
