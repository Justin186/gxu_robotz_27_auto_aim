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

  // 侧向相机只有左 / 右两路（±62 是安装偏角），其余名字视为异常。
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

  tools::logger()->warn("Unknown side camera name '{}', returning zero delta angle.", camera);
  return Eigen::Vector2d::Zero();
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
