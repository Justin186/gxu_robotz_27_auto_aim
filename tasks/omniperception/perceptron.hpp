#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{
class Perceptron
{
public:
  Perceptron(
    io::USBCamera * usbcam1, io::USBCamera * usbcam2, const std::string & config_path,
    bool debug = false);
  ~Perceptron();

  std::vector<DetectionResult> get_detection_queue();
  void set_pause(bool pause);
  std::unordered_map<std::string, uint64_t> get_frame_counts();
  std::unordered_map<std::string, cv::Mat> get_latest_frames();
  std::unordered_map<std::string, cv::Mat> get_latest_views();
  std::unordered_map<std::string, std::list<auto_aim::Armor>> get_latest_armors();

private:
  void parallel_infer(io::USBCamera * cam, const std::shared_ptr<auto_aim::YOLO> & yolo);

  std::vector<std::thread> threads_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel1_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel2_;
  Decider decider_;
  bool debug_;
  std::atomic<bool> stop_flag_{false};
  std::atomic<bool> pause_{false};
  std::mutex frame_count_mutex_;
  std::unordered_map<std::string, uint64_t> frame_counts_;
  std::unordered_map<std::string, cv::Mat> latest_frames_;
  std::unordered_map<std::string, cv::Mat> latest_views_;
  std::unordered_map<std::string, std::list<auto_aim::Armor>> latest_armors_;
};
}  // namespace omniperception

#endif  // OMNIPERCEPTION__PERCEPTRON_HPP
