#include "camera.hpp"

#include <stdexcept>

// 相机后端由 CMake 开关决定是否编进来（io/CMakeLists.txt）：
//   IO_HIKROBOT   -> IO_ENABLE_HIKROBOT
//   IO_MINDVISION -> IO_ENABLE_MINDVISION
// 两个都开时行为与以往完全一致。
#if defined(IO_ENABLE_HIKROBOT)
#include "hikrobot/hikrobot.hpp"
#endif
#if defined(IO_ENABLE_MINDVISION)
#include "mindvision/mindvision.hpp"
#endif
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  auto need_rotation = tools::read<bool>(yaml, "need_rotation", false);

  if (camera_name == "mindvision") {
#if defined(IO_ENABLE_MINDVISION)
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
#else
    throw std::runtime_error("This build has no mindvision support (IO_MINDVISION=OFF)!");
#endif
  }

  else if (camera_name == "hikrobot") {
#if defined(IO_ENABLE_HIKROBOT)
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, need_rotation);
#else
    throw std::runtime_error("This build has no hikrobot support (IO_HIKROBOT=OFF)!");
#endif
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
