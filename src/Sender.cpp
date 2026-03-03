#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  data.x = static_cast<double>(rand() % 100);
  data.y = static_cast<double>(rand() % 100);
  data.z = static_cast<double>(rand() % 100);

  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };
  if (!Utils::Message::serialize(frame, data)) {
    RCLCPP_ERROR(logger, "Failed to serialize message for '%s:%d'", frame.address.c_str(), frame.port);
    return;
  }

  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);

  if (!send(frame)) {
    RCLCPP_ERROR(logger, "Failed to send frame to '%s:%d'", frame.address.c_str(), frame.port);
  } else {
    RCLCPP_INFO(logger, "\n\tstamp: %ld\n\tx: %f\n\ty: %f\n\tz: %f", data.timestamp, data.x, data.y, data.z);
  }
}
