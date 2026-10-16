#include "gimbal.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received. (protocol: {})", GimbalProtocol::name());
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(const GimbalCommand & cmd)
{
  uint8_t frame[GimbalProtocol::max_frame_size];
  GimbalProtocol::tx_encode(cmd, frame);
  write_frame(frame, GimbalProtocol::tx_frame_size);
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  send(GimbalCommand{control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc});
}

const RxFrameSpec * Gimbal::find_rx_frame(uint8_t head0)
{
  for (size_t i = 0; i < GimbalProtocol::rx_frame_count; ++i) {
    if (GimbalProtocol::rx_frames[i].head0 == head0) return &GimbalProtocol::rx_frames[i];
  }
  return nullptr;
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::write_frame(const uint8_t * frame, size_t size)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    serial_.write(frame, size);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info(
    "[Gimbal] read_thread started. (protocol: {}, {} 种上行帧)", GimbalProtocol::name(),
    GimbalProtocol::rx_frame_count);

  int error_count = 0;

  static_assert(GimbalProtocol::max_frame_size <= 128, "一帧不该超过 128 字节");
  uint8_t frame[GimbalProtocol::max_frame_size];

  while (!quit_) {
    if (error_count > 10000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // ① 先读一个字节，按它在帧表里找是哪一种帧
    uint8_t head0;
    if (!read(&head0, 1)) {
      error_count++;
      continue;
    }

    const auto * spec = find_rx_frame(head0);
    if (spec == nullptr) continue;  // 不在帧表里，丢弃这个字节重新对齐

    frame[0] = head0;

    // ② 校验第二个帧头字节。先校验再往下读，避免帧头不匹配时把后面一帧的开头吃掉
    uint8_t head1;
    if (!read(&head1, 1)) {
      error_count++;
      continue;
    }
    if (head1 != spec->head1) continue;
    frame[1] = head1;

    auto t = std::chrono::steady_clock::now();

    // ③ 读完剩下的部分
    if (!read(frame + 2, spec->size - 2)) {
      error_count++;
      continue;
    }

    if (!GimbalProtocol::rx_check(*spec, frame)) {
      tools::logger()->debug("[Gimbal] CRC16 check failed (kind {}).", static_cast<int>(spec->kind));
      continue;
    }

    error_count = 0;

    RxOut out;
    GimbalProtocol::rx_decode(*spec, frame, out);

    if (spec->kind == RxKind::Vision) {
      const auto & fb = out.vision;
      queue_.push({fb.q, t});

      std::lock_guard<std::mutex> lock(mutex_);
      state_.yaw = fb.yaw;
      state_.yaw_vel = fb.yaw_vel;
      state_.pitch = fb.pitch;
      state_.pitch_vel = fb.pitch_vel;
      state_.bullet_speed = fb.bullet_speed;
      state_.bullet_count = fb.bullet_count;
      state_.yaw_offset = fb.yaw_offset;
      state_.pitch_offset = fb.pitch_offset;
      mode_ = fb.mode;
      if (!fb.mode_valid) {
        tools::logger()->warn("[Gimbal] Invalid mode: {}", fb.raw_mode);
      }
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      nav_state_ = out.nav;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
