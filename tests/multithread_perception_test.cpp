// 主相机（海康）+ 左/右 USB 相机的多线程推理联调测试：
//   主线程      主海康 → yolo.detect → 统计 FPS / 推理耗时
//   Perceptron  左/右 USB → 各自 yolo → detection_queue（与哨兵主程序同一条代码路径）
//   统计        三路 FPS、侧向丢帧率、detection_queue 到达量、总推理 FPS
// 可选 --display 拼屏显示三路画面，--rerun 推送原图 + 检测框到 viewer（需 BUILD_DEBUG=ON 编译）

#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <list>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

#ifdef USE_RERUN
#include <rerun.hpp>
#endif

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml    | 位置参数，yaml配置文件路径 }"
  "{left           | video0                 | 左 USB 相机设备名 }"
  "{right          | video2                 | 右 USB 相机设备名 }"
  "{d display      |                        | 拼屏显示三路画面 }"
  "{rerun          |                        | 推送到 rerun viewer（需 BUILD_DEBUG=ON 编译）}"
  "{ip             | 127.0.0.1              | rerun viewer 的 IP 地址 }";

void draw_armors(cv::Mat & img, const std::list<auto_aim::Armor> & armors)
{
  for (const auto & armor : armors) {
    cv::rectangle(img, armor.box, {0, 255, 0}, 1);
    tools::draw_points(img, armor.points, {0, 255, 0});
    tools::draw_text(
      img, fmt::format("{:.2f}", armor.confidence), armor.box.tl(), {0, 255, 0}, 0.6, 1);
  }
}

// 统一缩放到固定高度后横向拼接
cv::Mat hstack_views(const std::vector<cv::Mat> & views, int height)
{
  std::vector<cv::Mat> resized;
  for (const auto & v : views) {
    if (v.empty()) continue;
    cv::Mat r;
    double scale = static_cast<double>(height) / v.rows;
    cv::resize(v, r, {}, scale, scale);
    resized.push_back(r);
  }
  if (resized.empty()) return {};
  cv::Mat out;
  cv::hconcat(resized, out);
  return out;
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>(0);
  auto left_name = cli.get<std::string>("left");
  auto right_name = cli.get<std::string>("right");
  auto display = cli.has("display");
  auto use_rerun = cli.has("rerun");
  auto rerun_ip = cli.get<std::string>("ip");

  // SSH/无头环境下 imshow 会因 GTK 初始化失败直接 abort，自动关掉
  const bool has_display_env =
    std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
  if (display && !has_display_env) {
    tools::logger()->warn("no DISPLAY/WAYLAND_DISPLAY, --display disabled");
    display = false;
  }

#ifdef USE_RERUN
  std::optional<rerun::RecordingStream> rec;
  if (use_rerun) {
    rec.emplace("gxu_multithread_perception_test");
    rec->connect_grpc("rerun+http://" + rerun_ip + ":9876/proxy").exit_on_failure();
    tools::logger()->info("rerun connected: {}:9876", rerun_ip);
  }
#else
  if (use_rerun) {
    tools::logger()->error("未启用 BUILD_DEBUG 编译，没有 rerun 支持（cmake -B build -DBUILD_DEBUG=ON）");
    return 1;
  }
#endif

  io::Camera camera(config_path);
  io::USBCamera left_camera(left_name, config_path);
  io::USBCamera right_camera(right_name, config_path);

  auto_aim::YOLO yolo(config_path, false);
  omniperception::Perceptron perceptron(&left_camera, &right_camera, config_path);

  // ---- 统计状态 ----
  uint64_t main_frames = 0;
  double main_infer_ms_sum = 0;
  double last_main_fps = 0, last_main_ms = 0, last_total_fps = 0;
  size_t last_pending = 0;
  std::unordered_map<std::string, uint64_t> last_counts, last_caps;
  std::map<std::string, double> side_infer_fps, side_cap_fps, side_drop_pct;
  std::unordered_map<std::string, std::list<auto_aim::Armor>> last_side_armors;

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_stats = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, t);

    auto t0 = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img);
    auto t1 = std::chrono::steady_clock::now();

    main_frames++;
    main_infer_ms_sum += std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 取空侧向检测队列，记录每路最新的装甲板（用于显示 / rerun）
    auto detections = perceptron.get_detection_queue();
    last_pending = detections.size();
    for (const auto & d : detections) last_side_armors[d.camera_name] = d.armors;

    // ---- 每秒统计一次 ----
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_stats).count();
    if (dt >= 1.0) {
      last_main_fps = main_frames / dt;
      last_main_ms = main_frames ? main_infer_ms_sum / main_frames : 0;

      std::unordered_map<std::string, uint64_t> caps{
        {left_camera.device_name, left_camera.capture_count()},
        {right_camera.device_name, right_camera.capture_count()}};

      auto counts = perceptron.get_frame_counts();
      double total = last_main_fps;
      for (const auto & [name, cnt] : counts) {
        uint64_t infer_delta = cnt - last_counts[name];
        uint64_t cap_delta = caps[name] - last_caps[name];
        side_infer_fps[name] = infer_delta / dt;
        side_cap_fps[name] = cap_delta / dt;
        side_drop_pct[name] =
          cap_delta > 0 ? (1.0 - static_cast<double>(infer_delta) / cap_delta) * 100.0 : 0.0;
        total += side_infer_fps[name];
      }
      last_counts = counts;
      last_caps = caps;
      last_total_fps = total;

      std::string side_stats;
      for (const auto & [name, fps] : side_infer_fps) {
        side_stats += fmt::format(
          " | {} cap {:.1f} infer {:.1f} (drop {:.0f}%)", name, side_cap_fps[name], fps,
          side_drop_pct[name]);
      }
      tools::logger()->info(
        "main {:.1f} fps ({:.1f} ms){} | queue {} | total infer {:.1f} fps", last_main_fps,
        last_main_ms, side_stats, last_pending, last_total_fps);

      main_frames = 0;
      main_infer_ms_sum = 0;
      last_stats = now;
    }

    // ---- 组画面：主视角 + 侧向最新帧 + 检测框 ----
    if (display || use_rerun) {
      std::vector<std::pair<std::string, cv::Mat>> named_views;

      cv::Mat main_view = img.clone();
      draw_armors(main_view, armors);
      tools::draw_text(
        main_view, fmt::format("main {:.1f}fps {:.1f}ms", last_main_fps, last_main_ms), {10, 30},
        {255, 255, 255}, 0.8, 1);
      named_views.emplace_back("main", main_view);

      auto frames = perceptron.get_latest_frames();
      std::vector<std::string> names;
      for (const auto & [name, f] : frames) names.push_back(name);
      std::sort(names.begin(), names.end());
      for (const auto & name : names) {
        cv::Mat view = frames[name].clone();
        if (view.empty()) continue;
        auto it = last_side_armors.find(name);
        if (it != last_side_armors.end()) draw_armors(view, it->second);
        tools::draw_text(
          view,
          fmt::format(
            "{} infer {:.1f}fps drop {:.0f}%", name, side_infer_fps[name], side_drop_pct[name]),
          {10, 30}, {255, 255, 255}, 0.8, 1);
        named_views.emplace_back(name, view);
      }

      if (display) {
        std::vector<cv::Mat> views;
        for (auto & [name, v] : named_views) views.push_back(v);
        cv::Mat tiled = hstack_views(views, 360);
        if (!tiled.empty()) {
          cv::imshow("multithread_perception", tiled);
          if (cv::waitKey(1) == 'q') break;
        }
      }

#ifdef USE_RERUN
      if (use_rerun) {
        for (auto & [name, v] : named_views) {
          cv::Mat half;
          cv::resize(v, half, {}, 0.5, 0.5);
          std::vector<uchar> encoded;
          cv::imencode(".jpg", half, encoded);
          rec->log("camera/" + name, rerun::EncodedImage::from_bytes(encoded));
        }
        rec->log("fps/main", rerun::Scalars(last_main_fps));
        for (const auto & [name, fps] : side_infer_fps)
          rec->log("fps/side/" + name, rerun::Scalars(fps));
        rec->log("fps/total", rerun::Scalars(last_total_fps));
        rec->log("queue/pending", rerun::Scalars(static_cast<double>(last_pending)));
      }
#endif
    }
  }

  return 0;
}
