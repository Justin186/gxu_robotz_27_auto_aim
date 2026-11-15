#include "perceptron.hpp"

#include <chrono>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  io::USBCamera * usbcam1, io::USBCamera * usbcam2, const std::string & config_path)
: detection_queue_(10), decider_(config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  const int side_infer_threads =
    yaml["side_infer_threads"] ? yaml["side_infer_threads"].as<int>() : 0;

  yolo_parallel1_ =
    std::make_shared<auto_aim::YOLO>(config_path, false, side_infer_threads);
  yolo_parallel2_ =
    std::make_shared<auto_aim::YOLO>(config_path, false, side_infer_threads);

  std::this_thread::sleep_for(std::chrono::seconds(2));
  threads_.emplace_back(
    [this, usbcam1] { parallel_infer(usbcam1, yolo_parallel1_); });
  threads_.emplace_back(
    [this, usbcam2] { parallel_infer(usbcam2, yolo_parallel2_); });

  tools::logger()->info("Perceptron initialized.");
}

Perceptron::~Perceptron()
{
  stop_flag_ = true;
  for (auto & thread : threads_) {
    if (thread.joinable()) thread.join();
  }
  tools::logger()->info("Perceptron destructed.");
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult item;
  while (!detection_queue_.empty()) {
    detection_queue_.pop(item);
    result.push_back(std::move(item));
  }
  return result;
}

void Perceptron::set_pause(bool pause)
{
  bool expected = !pause;
  if (pause_.compare_exchange_strong(expected, pause)) {
    tools::logger()->info("Perceptron side inference {}.", pause ? "paused" : "resumed");
  }
}

std::unordered_map<std::string, uint64_t> Perceptron::get_frame_counts()
{
  std::lock_guard<std::mutex> lock(frame_count_mutex_);
  return frame_counts_;
}

std::unordered_map<std::string, cv::Mat> Perceptron::get_latest_frames()
{
  std::lock_guard<std::mutex> lock(frame_count_mutex_);
  return latest_frames_;
}

void Perceptron::parallel_infer(
  io::USBCamera * cam, const std::shared_ptr<auto_aim::YOLO> & yolo)
{
  if (cam == nullptr) {
    tools::logger()->warn("Camera pointer is null, inference thread exits.");
    return;
  }

  try {
    while (!stop_flag_) {
      if (pause_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }

      cv::Mat image;
      std::chrono::steady_clock::time_point timestamp;
      if (!cam->try_read(image, timestamp)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if (image.empty()) continue;

      {
        std::lock_guard<std::mutex> lock(frame_count_mutex_);
        latest_frames_[cam->device_name] = image;
      }

      auto armors = yolo->detect(image);

      {
        std::lock_guard<std::mutex> lock(frame_count_mutex_);
        ++frame_counts_[cam->device_name];
      }

      if (armors.empty()) continue;

      auto delta_angle = decider_.delta_angle(armors, cam->device_name);
      DetectionResult result;
      result.armors = std::move(armors);
      result.timestamp = timestamp;
      result.delta_yaw = delta_angle[0] / 57.3;
      result.delta_pitch = delta_angle[1] / 57.3;
      result.camera_name = cam->device_name;
      result.img = image;
      detection_queue_.push(result);
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Exception in parallel_infer: {}", e.what());
  }
}
}  // namespace omniperception
