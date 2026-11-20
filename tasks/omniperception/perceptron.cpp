#include "perceptron.hpp"

#include <fmt/core.h>

#include <chrono>
#include <cstdio>
#include <yaml-cpp/yaml.h>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  io::USBCamera * usbcam1, io::USBCamera * usbcam2, const std::string & config_path, bool debug)
: detection_queue_(10), decider_(config_path), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);
  const int side_infer_threads =
    yaml["side_infer_threads"] ? yaml["side_infer_threads"].as<int>() : 0;
  // 侧向推理设备：默认 CPU（把 GPU 留给主相机），可用 side_device 切换为 GPU；
  // 切到 GPU 时 side_infer_threads 不再生效
  const std::string side_device =
    yaml["side_device"] ? yaml["side_device"].as<std::string>() : "CPU";

  yolo_parallel1_ =
    std::make_shared<auto_aim::YOLO>(config_path, false, side_device, side_infer_threads);
  yolo_parallel2_ =
    std::make_shared<auto_aim::YOLO>(config_path, false, side_device, side_infer_threads);

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

std::unordered_map<std::string, cv::Mat> Perceptron::get_latest_views()
{
  std::lock_guard<std::mutex> lock(frame_count_mutex_);
  return latest_views_;
}

std::unordered_map<std::string, std::list<auto_aim::Armor>> Perceptron::get_latest_armors()
{
  std::lock_guard<std::mutex> lock(frame_count_mutex_);
  return latest_armors_;
}

void Perceptron::parallel_infer(
  io::USBCamera * cam, const std::shared_ptr<auto_aim::YOLO> & yolo)
{
  if (cam == nullptr) {
    tools::logger()->warn("Camera pointer is null, inference thread exits.");
    return;
  }

  try {
    uint64_t side_frame_cnt = 0;                            // 推理帧数
    uint64_t capture_cnt_last = cam->capture_count();       // 采集帧数基准
    auto side_fps_start = std::chrono::steady_clock::now();

    while (!stop_flag_) {
      // 帧率打印（每秒一次，独立于 pause：主相机推理暂停侧向时也能看到采集/推理帧率）
      {
        auto fps_now = std::chrono::steady_clock::now();
        double fps_elapsed =
          std::chrono::duration<double>(fps_now - side_fps_start).count();
        if (fps_elapsed >= 1.0) {
          uint64_t capture_now = cam->capture_count();
          tools::logger()->info(
            "[SIDE] {} capture={:.1f} infer={:.1f}", cam->device_name,
            (capture_now - capture_cnt_last) / fps_elapsed, side_frame_cnt / fps_elapsed);
          capture_cnt_last = capture_now;
          side_frame_cnt = 0;
          side_fps_start = fps_now;
        }
      }

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

      if (debug_) {
        cv::Mat view = image.clone();
        for (const auto & armor : armors) {
          cv::rectangle(view, armor.box, {0, 255, 0}, 1);
          tools::draw_points(view, armor.points, {0, 255, 0});
          tools::draw_text(
            view,
            fmt::format(
              "{:.2f} {} {}", armor.confidence, auto_aim::ARMOR_NAMES[armor.name],
              auto_aim::ARMOR_TYPES[armor.type]),
            armor.box.tl(), {0, 255, 0}, 0.6, 1);
        }
        std::lock_guard<std::mutex> lock(frame_count_mutex_);
        latest_views_[cam->device_name] = view;
      }

      {
        std::lock_guard<std::mutex> lock(frame_count_mutex_);
        ++frame_counts_[cam->device_name];
        latest_armors_[cam->device_name] = armors;
      }
      ++side_frame_cnt;  // 推理帧数（完成一次推理）

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
