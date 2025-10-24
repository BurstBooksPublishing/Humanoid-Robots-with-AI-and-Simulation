while (rt_loop_running) {                         // 1 kHz loop
  timestamp = high_res_clock();                   // measure latency
  readSensors(state);                             // IMU, encoders, F/T
  state = estimator.update(state, sensor_data);   // high-rate EKF

  // compute desired accelerations from mid-level planner (100 Hz)
  qdd_des = planner.get_qdd_des(state.timestamp);

  // inverse dynamics compute desired joint torques (feedforward)
  tau_ff = inverseDynamics(M, C, g, q, qdot, qdd_des); 

  // low-gain feedback PD on joint positions (stability)
  tau_fb = Kp*(q_des - q) + Kd*(qd_des - qdot);

  tau_cmd = tau_ff + tau_fb;                      // combine feedforward + feedback

  // enforce actuator limits and safety clamps
  for (int i=0;i torque_limit[i]) tau_cmd[i] = sign(tau_cmd[i])*torque_limit[i];
  }

  // contact safety: reduce torques if force exceeds threshold
  if (ft_sensor.overload()) {                      // emergency precaution
    tau_cmd.setZero();                             // immediate safe reduction
    estop.raise();                                 // trigger higher-level handler
  }

  actuator_interface.sendTorques(tau_cmd);        // send commands to actuators

  monitor_latency_and_drop_if_needed(timestamp);  // watch jitter and deadlines
  sleep_until_next_period();
}