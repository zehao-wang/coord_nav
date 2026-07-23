# Changelog

Findings and notable changes. README is for day-to-day usage; this file records
*why* things are the way they are (hard-won during bring-up).

## 0.1.0 — 2026-07-23 — initial bring-up

Brought the Jetson Nano mecanum car under **pure-ROS** control from the
workstation, with a PySide6 console and a `carclient` Python API.

### Built
- **Network**: workstation WiFi hotspot `coord_nav` (TP-Link RTL8811AU, DKMS
  `8821au`); car fixed at `10.42.0.187`; `roscar` env; `smoke/` self-tests.
- **Perception** (`obstacle_perception/`, C++): `/scan` → DBSCAN → enclosing
  circles → redundancy drop → `/obstacles` (`[frame_id, x,y,r, ...]`, metres,
  base frame). ~3.4 ms/frame (≈20× the old Python `perception_server.py`), 3 Hz.
- **Control** (`car_ros/drive_action_node.py`): discrete mecanum actions
  `/drive_action [id, magnitude, duration_s]` → timed `/wheel_cmd` pulses →
  `/drive_result`. Per-wheel `/wheel_cmd` retained for bring-up.
- **carclient/** (pip package): `CarClient` — obstacles / drive / result / estop
  / MCU-link health; bounded 100-frame in-memory history + on-disk dump.
- **gui/** (PySide6): obstacle top-down view, steering wheel, colour log.
- Car runs **headless** (`multi-user.target`, gdm disabled, packagekit masked);
  `coord_nav` made **system-wide** so a headless boot still connects.

### Findings (the important gotchas)
- **Rear motor ports M2/M4 are swapped in hardware** (rear-left ↔ rear-right)
  and the rear plugs are polarity-reversed. Fixed in car_base's `/wheel_cmd`
  handler: `rl→M4, rr→M2`, both negated. Verified wheel-by-wheel.
- **Cross-distro message md5**: `sensor_msgs/BatteryState` differs between the
  car's Melodic and the workstation's Noetic, so the client CANNOT subscribe to
  `/battery` (connection dropped). Worked around by republishing voltage as
  `std_msgs/Float32` on `/battery_v`. Prefer std_msgs types for new car→client
  topics. (`LaserScan`/`Odometry`/`Float32MultiArray`/`String` are compatible.)
- **The MCU firmware has NO motor timeout.** Both `set_motor` (raw PWM) and
  `set_car_motion` (velocity) latch the last command forever — verified by
  sending one command and watching it run for 3 s. So a single lost "stop" =
  permanent runaway. The RC remote is safe only because it streams commands
  continuously (it never goes silent); the Nano can (it hangs).
- **Runaway root cause**: commanding max magnitude drives peak current past what
  the source can supply → brownout, and/or the Nano CPU saturates → the stop
  loop stalls → the MCU latches → the car keeps moving. **WiFi is onboard (not
  USB)**, so the "network drop" during a runaway is CPU saturation, not a
  power/USB glitch. When the Nano fully hangs, only a **physical power-cut**
  stops the car — no host-side software can.
- **The frequent lidar "disconnected" was self-inflicted.** An `rplidar-watchdog`
  service (added, then **removed**) misfired: its own node wasn't receiving
  `/scan`, so it judged the lidar wedged and `pkill`'d a perfectly healthy lidar
  every ~27 s — each kill a "disconnect". `dmesg` showed NO USB disconnects; the
  lidar hardware/connector is fine. With it gone, `/scan` is rock-solid.
- **Never USB unbind/rebind on the car** — it ripples the USB bus and disturbs
  things (and risks the network). Recover a genuinely wedged lidar with
  `sudo systemctl restart car-ros` (+ wait ~15 s) or `sudo reboot`.
- `rostopic hz`/`echo` often hang ignoring `timeout`; use a bounded python
  `wait_for_message`. `pkill -f <x>` matches the shell running it if the command
  text contains `<x>` — use `pkill -x <name>` or kill by PID.

### Safety layers (host-side; a **physical motor E-STOP is still recommended**)
- **Magnitude hard-capped at 50** (car-side): full PWM on 4 wheels browns out the
  12V 2A supply.
- **Sustained brake**: drive_action holds `[0,0,0,0]` for ~1.5 s after every move
  (~30 messages), so a few dropped stops can't latch.
- Conservative defaults (magnitude 40, duration 0.8 s); hard duration cap 3 s.
- **GUI**: MCU-link banner (green/red), steering disabled on link loss, spacebar
  = E-STOP, obstacle disconnect logged as ERR.
- **Overload audit**: rates capped ~3 Hz (car_base 20→10 Hz, `/battery_v` 20→3 Hz,
  `/obstacles` 3 Hz, GUI refresh 3 Hz); queues bounded (client history
  `deque(maxlen=100)`, GUI display 200 lines); GUI log file rotates at 5 MB.
- **E-STOP** = SSH `sh ~/estop.sh` (ROS-independent; brings car-ros down;
  recover with `sudo systemctl start car-ros`).
