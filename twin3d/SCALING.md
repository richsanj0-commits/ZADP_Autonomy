# Scaling: 1:200, set by the robot

The robot (0.100 x 0.084 m) is one haul truck (20.0 x 16.8 m), so the scale is fixed at
1:200. A bigger site later needs a bigger arena or a smaller robot. Source: `plan/plan_arena.json` (other map shapes at
the same scale: `maps/`).

| Quantity | Arena (hardware / twin) | Site (what it represents) |
|---|---|---|
| Arena / site | 1 m x 1 m | 200 m x 200 m, about 10 acres |
| Truck footprint | 0.100 x 0.084 m | 20.0 x 16.8 m |
| Pile radius | 0.0353 m | 7.06 m |
| Spine (two lanes) | 0.240 m (0.120 m each) | 48.0 m (24.0 m each) |
| Turning radius | 0.120 m | 24.0 m |
| Work zones / spots | 2 zones, 125 spots (72 zone, 14 branch, 39 lane) | same |
| Zone size (bounding box) | 0.274 x 0.815 m each | 54.9 x 163 m |
| Boundary clearance to arena edge | 0.05 m | 10 m |

## Time
Robot cruise 0.20 m/s equals 40 m/s at site scale. A real hauler does about 8.3 m/s
(30 km/h), so the hardware runs about 4.8x faster than site time. Logs record both arena time and site time (site time = arena time x 4.8,
`P.scale.siteTimeFactor` = cruise x 200 / `P.scale.realHaulSpeed`).

## Piles
Volume and radius are computed at site scale (payload, density, repose angle from the
planner) and drawn at 1:200. On real hardware the robot only drives to the spot and back;
the pile exists only in the twin.

## Accuracy at site scale
| Item | Arena | Site |
|---|---|---|
| One encoder count (wheel circumference / 2772.2) | 0.0385 mm | 0.0077 m |
| Camera position noise | 3 mm | 0.6 m |
| Camera latency | 50 ms (10 mm at cruise if not compensated; compensated with odometry) | 0.24 s site time |
| Arrive tolerance (on the estimate, `P.ctrl.arriveTol`) | 5 mm | 1 m |
| True arrival error (measured, 40 random go-tos, camera on) | under 10 mm (max about 6.5 mm) | under 2 m |
| Odometry only (camera off), position / heading error | 1.7 % / 1.7 deg over 0.2 m, 6 % / 6 deg over 1 m | same |

Wheel circumference = pi x 0.034 m = 106.8 mm.
