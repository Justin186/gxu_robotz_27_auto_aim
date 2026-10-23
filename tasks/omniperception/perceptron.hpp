#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <thread>

#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{
// 全向感知的并联推理：单独一条线程读后置 USB 相机、持续跑 YOLO，
// 把"看到了什么、相对当前云台角度差多少"推进队列，主线程随时取走。
//
// 26 赛季是同济那套 4 路 USB 相机 + 4 个 YOLO 实例并行推理；
// 27 赛季只用一路后置 USB 相机（主相机是海康，归自瞄用），所以这里收到一路。
class Perceptron
{
public:
  Perceptron(io::USBCamera * usbcam, const std::string & config_path);

  ~Perceptron();

  std::vector<DetectionResult> get_detection_queue();

private:
  void infer_loop(io::USBCamera * cam);

  std::thread thread_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;

  std::shared_ptr<auto_aim::YOLO> yolo_;

  Decider decider_;
  bool stop_flag_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace omniperception

#endif
