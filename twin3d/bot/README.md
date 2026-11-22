# bot: one robot (phase 3)

Physics (`bot_step`), encoders, dead reckoning (`bot_odometry`), camera fix and fusion (`cam_measure`, `pose_fusion`), go-to-point (`bot_controller`, `bot_goto`), `demo_one_bot`. Test: `tests/test_bot.m`. One line per function: `../HANDOVER.md` section 3.

Camera latency (`P.cam.latency` 50 ms) is modelled and compensated: `cam_measure(s, P)` returns the pose of 50 ms ago
and its stamp, `pose_fusion(s, z, P, tz)` moves it forward by the odometry since the stamp (`s.hist`, `hist_at`). The
controller has a 3 cm capture radius (no rotate-in-place near the goal) and rotate hysteresis 25 / 15 deg. Wheel travel
per step is trapezoidal, so distances do not depend on dt. Noise comes from the stream `s.rs` (`bot_init`).
