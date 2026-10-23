#include "perceptron.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(io::USBCamera * usbcam, const std::string & config_path)
: detection_queue_(10), decider_(config_path), stop_flag_(false)
{
  if (usbcam == nullptr) {
    throw std::invalid_argument("[Perceptron] USB 相机指针为空");
  }

  yolo_ = std::make_shared<auto_aim::YOLO>(config_path, false);

  // 等相机与模型都就绪（沿用原实现的等待，避免第一帧拿到空图）
  std::this_thread::sleep_for(std::chrono::seconds(2));

  thread_ = std::thread([this, usbcam] { infer_loop(usbcam); });

  tools::logger()->info("Perceptron initialized. ({} camera)", 1);
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;
  }
  condition_.notify_all();

  if (thread_.joinable()) thread_.join();

  tools::logger()->info("Perceptron destructed.");
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult temp;

  // 非阻塞地取走队列里现有的全部结果
  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);
    result.push_back(std::move(temp));
  }

  return result;
}

void Perceptron::infer_loop(io::USBCamera * cam)
{
  try {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;
      }

      cv::Mat usb_img;
      std::chrono::steady_clock::time_point ts;
      cam->read(usb_img, ts);

      if (usb_img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      auto armors = yolo_->detect(usb_img);
      if (armors.empty()) continue;

      auto delta_angle = decider_.delta_angle(armors, cam->device_name);

      DetectionResult dr;
      dr.armors = std::move(armors);
      dr.timestamp = ts;
      dr.delta_yaw = delta_angle[0] / 57.3;
      dr.delta_pitch = delta_angle[1] / 57.3;
      detection_queue_.push(dr);
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Exception in Perceptron::infer_loop: {}", e.what());
  }
}

}  // namespace omniperception
