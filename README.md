# RoboDragons SSL Simulator

Java 25 / Gradle simulator for RoboCup SSL (field, robot, ball dimensions follow the official rules).
Math: Apache Commons Math 3. Protocol: the attached protobuf files (`src/main/proto`). The referee/GC layer is intentionally not implemented.

```
./gradlew run                         # 3D viewer (OpenGL 3.3) + network, Div B
./gradlew run --args="--division A --robots 11"
./gradlew run --args="--headless"
./gradlew test
```

## Network (UDP)
| purpose | address |
|---|---|
| vision (SSL_WrapperPacket: detection per camera 60 Hz, geometry 1 Hz) | multicast 224.5.23.2:10020 |
| simulator control (SimulatorCommand -> SimulatorResponse) | 10300 |
| blue / yellow robot control (RobotControl -> RobotControlResponse) | 10301 / 10302 |
| legacy grSim packets | 20011 |
| synchronous mode (SimulationSyncRequest -> Response; robot_control is applied to blue, `--sync-port`) | 10400 |

## Design (package `ssl.sim`)
- `physics`: `RigidBody2D`, `ConvexPolygon`, `Collisions` (SAT, circle-polygon)
- `ball.Ball`: 3D position + spin; ground Coulomb friction on contact-point slip gives slide -> roll, backspin, chip bounces. `BallContacts`: spring-damper + friction contact incl. spin.
- `robot.Robot` = rigid body + components (`RobotComponent`): `OmniDrive` (4x `OmniWheel`: DC motor, gearbox, PI speed control, traction along drive direction, rollers modelled as lateral sliding), `Kicker` (straight/chip, capacitor model), `Dribbler` (rubber roller with own motor; 3D contact normal, friction on full slip velocity incl. ball spin, reaction on robot and roller load torque). `RobotShape`: D-shaped hull plus dribbler notch so the ball sits ~20 % inside the hull.
- `vision.Camera`/`VisionSystem`: pinhole cameras with calibration, noise, latency. `core.World`/`Simulator`: world stepping (0.5 ms), real-time thread.
- `net`: protobuf conversion and UDP servers. `viewer`: LWJGL OpenGL 3.3 viewer (low-poly, one shader).

Viewer keys: mouse orbit/pan/zoom, 1-4 camera presets, V cameras, Space pause, [ ] speed, Tab/T select robot,
M manual drive (W/S/A/D/Q/E, Shift fast, R dribbler, F kick, C chip), B ball to centre.
`--screenshot file.png` saves a frame and exits.

Known simplifications: dribbler retention depends on acceleration (a held ball is lost beyond roughly 1-2 m/s^2 backwards),
`by_force` teleports unsupported, marker patterns approximate.

## Configuration (Gson / JSON)
`--config sim.json` loads settings; CLI options override them. `--write-default-config sim.json` writes every default.
Any subset of keys is valid, e.g.
```json
{ "division": "A", "cameras": { "count": 3, "height": 5.5 }, "robot": { "mass": 2.2 }, "network": { "visionPort": 10006 } }
```
Sections: `division`, `robotsPerTeam`, `cameras` (count, height, focalLengthPx, image size, noise, latency, overlap),
`robot` (all `RobotSpecs` incl. dribbler/kicker geometry), `ball` (`Ball.Params`), `network` (ports/addresses).
Cameras default to **2 for Division A, 1 for Division B, 6 m high**, tiled along the field length with auto-fitted focal length.

## Device colours in the viewer
Geometry of roller and kicker plate is read from the `Dribbler` / `Kicker` objects (so config changes show up).
Roller: grey = off, green = spinning, orange = spinning and touching the ball (white stripe shows rotation).
Kicker plate: teal (brightness = capacitor charge), orange = kick pending, red + pushed out = just fired.
Wheels: purple, brighter when faster. `--demo` runs a scripted dribble + kick of blue 0 (see docs/viewer-dribbler.png).

## Ball handling model (dribbler / chipper / kicker)
Layout (all derived in `RobotSpecs`, robot frame x forward, z up) for the held ball:
- the ball centre sits at `holdX() = centerToDribbler + R(1 - 2*ballCoverage)`: **20 % of its diameter is inside the convex hull when seen from above** (`topViewCoverage()`);
- the **roller is above the kick devices**: it touches the ball behind/above the centre (`rollerContactAngleDeg`, default 30 deg);
- the **chipper plate** (ramp) touches it behind/below the centre (`chipperContactAngleDeg`, 40 deg), the straight kicker plate sits at centre height between them;
- `validateLayout()` checks the 20 % rule, that no device sticks out of the hull and that the roller is above chipper and kicker.

Physics: roller, chipper plate and ground are separate penalty contacts with Coulomb friction on the full slip (spin included).
- **3-point hold** (roller + chipper + ground) and **2-point hold** (`chipperEnabled: false`, roller + ground) both come out of the simulation, and `DribblerStatics` solves the same static force balance analytically
  (3 point: Nr, Nc, Ng with sliding contacts; 2 point: roller sticks, unknowns Nr, Fr, Ng, check Fr <= mu*Nr). `--dribble-statics` prints both; `DribbleTest` checks that simulated normal forces and hold position match.
- Ball-wall friction: `ball.muWall` (default 0.05) so that a spinning dribbler can pull a ball off a wall.

## Roller damper (optional, `robot.rollerDamperEnabled`, default off)
The roller can retract rearwards on a spring (preload `rollerPreload`, `rollerSpringK`), damper (`rollerDampingC`) and stroke (`rollerStroke`).
To keep the 20 % rule even when fully retracted the nominal hold position moves forward by the stroke; the chipper plate and kicker plate stay at the compressed position.
Comparison (spin in place, curves, pass reception, reversing): `./gradlew test --tests "*DamperStudy*" -Dstudy=true -i`.

## Velocity tracking study
`./gradlew test --tests "*TrackingStudy*" -Dstudy=true` writes CSVs to `build/study`, `python tools/plot_tracking.py` plots them to `docs/tracking_*.png` (steps with default limits, raw steps without limiter, frequency response).

## Vision latency
`cameras.latencySeconds` is a real delay: a frame captured at t is published at t + latency (default 0). Note: the RoboDragonsCafe server's velocity control degrades quickly with
latency in this simulator (see docs/server_investigation/ and `tools/` notes): 0 ms is stable, 20 ms already causes saturated command chatter.
`network.replyToRobotControl` (default true) switches the RobotControlResponse feedback off. `--telemetry DIR` logs received commands and robot states as CSV.

## Network interface
Without `network.networkInterface` / `--interface` the simulator now picks the local LAN interface (up, multicast, non-loopback, non-virtual, site-local IPv4) for the multicast vision stream and prints it at start-up.
FORCE_START study (server driven through its game AI, both teams): `docs/server_investigation/force_start_blue1.png`.

## ER-Force compatible response (default)
The default robot drive model `robot.driveModel = "ER_FORCE"` reproduces the closed-loop response of the framework's `simulator-cli` (ER-Force `SimRobot`):
body acceleration `(v_cmd - v) / tau` with tau = 5 ms / ln 2 (half the error per 5 ms), limited through the wheel coupling of the 2014 generation
(35 / 30 rev/s^2 -> 7.6 m/s^2 forward, 6.5 m/s^2 sideways, 70 rad/s^2 rotation), a command timeout of 0.1 s after which the robot coasts with damping (0.7 / 0.8).
`"MOTOR"` keeps the wheel-level model (DC motor + PI + traction).
`realism` (default "Realistic", also `--realism`) applies the ER-Force realism presets: vision delay 35 ms, vision every 15 ms, position/orientation noise, detection dropouts,
robot command loss, command delay and robot rotation error (Realistic: 1.3 mm / 10 mrad / 3 % loss / 0.5). JSON fields override the preset.
Clients must re-send commands (< 0.1 s), as with ER-Force. Comparison with the old model: docs/server_investigation/force_start_er_model.png

[ER-Force Open Source](https://github.com/robotics-erlangen/framework)

## Kicking (metal plate with friction)
A kick is a short stroke of the kicker plate (`kickerStroke` 20 mm, uniform acceleration to the plate speed) that pushes the ball through a penalty contact with
Coulomb friction between the metal plate and the ball (`kickerFriction` 0.4) on the full contact-point slip, spin included. The backspin of a spinning dribbler is largely wiped out
while the plate pushes (impulse limit: mu * m * v * R / I, i.e. about 230 rad/s of a 380 rad/s backspin at a 5 m/s kick; the remainder decays on the ground).
Chip kicks move a ramp along the commanded elevation. Ball speeds match the command (2 / 4 / 6 m/s -> 1.9 / 3.9 / 6.0 m/s). See `KickTest`.

## config.json (network settings)

`config.json` in the working directory is loaded automatically when `--config` is not given; it is created on first start if it does not exist (and is git-ignored, so local changes stay local). The generated file only
contains the `network` section (the vision multicast address/port, control/blue/yellow/grSim/sync ports,
`networkInterface` as IP or interface name — `null` selects the local LAN interface automatically — `syncTeam`,
`replyToRobotControl`). Any other `SimConfig` field can be added to the same file; entries that are left out keep
their defaults (and the realism preset values). `robotCommandLoss` / `commandDelaySeconds` are not in the file on
purpose: they come from the `realism` preset unless you add them. CLI options (`--interface`, `--vision-port`, ...)
override the file.
