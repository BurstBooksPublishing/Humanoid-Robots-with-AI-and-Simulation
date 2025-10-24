volatile bool control_flag = false;

// Timer ISR triggers at 1 kHz (strict deadline)
extern "C" void TIM1_IRQHandler() {
  control_flag = true;                 // set flag for main loop
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

int main() {
  init_peripherals();                  // encoders, ADCs, CAN, timer
  init_watchdog();                     // hardware watchdog
  while (1) {
    if (control_flag) {
      control_flag = false;
      int32_t pos = read_encoder();    // fast, low-jitter read
      int32_t vel = differentiate(pos);
      float torque_cmd = pid_update(pos, vel); // tuned gains
      send_can_torque_command(torque_cmd);     // deterministic bus
      feed_watchdog();                 // safety: ensures health
    }
    // low-priority housekeeping runs when slack available
    handle_can_messages_noncritical();
    log_status_if_time();
  }
}