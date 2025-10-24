#include 
#include 
#include 
#include 
#include 

class RealtimeController : public rclcpp::Node {
public:
  RealtimeController()
  : Node("realtime_controller")
  {
    // realtime_pub pre-allocates message memory to avoid runtime allocs.
    rt_pub_.reset(new realtime_tools::RealtimePublisher(this->get_node_base_interface(), "torque_cmd", 10));
    // Configure parameters, reserve memory, etc.
  }

  void run() {
    // Set thread to SCHED_FIFO real-time priority.
    struct sched_param param { .sched_priority = 80 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, ¶m);
    // Bind to a dedicated CPU (core 1) to avoid interference.
    cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(1, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    const long period_ns = 1000000; // 1 ms loop
    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);

    while (rclcpp::ok()) {
      // Sleep until next period with clock_nanosleep for minimal jitter.
      next.tv_nsec += period_ns;
      while (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; ++next.tv_sec; }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

      // Read sensor data from shared lock-free buffer (not shown).
      // Compute low-level control (deterministic, no allocations).
      float torque = compute_torque(); // synchronous, bounded time.

      // Publish using realtime_publisher to avoid blocking allocations.
      if (rt_pub_->trylock()) { rt_pub_->msg_.data = torque; rt_pub_->unlockAndPublish(); }

      // Optional: update watchdog timer.
    }
  }

private:
  float compute_torque() { /* deterministic PID/impedance compute */ return 0.0f; }
  std::unique_ptr> rt_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared();
  node->run(); // runs on main thread with SCHED_FIFO.
  rclcpp::shutdown();
  return 0;
}