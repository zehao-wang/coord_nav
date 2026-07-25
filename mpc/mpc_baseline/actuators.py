"""Actuation backends that turn a plan into car motion (ROS side, live only).

Two backends, one per action space:

  VelocityPulseActuator ("velocity")  -- variant 1. Each plan cycle it mecanum-
    mixes the body velocity (vx, vy, wz) to wheel PWM and sends ONE
    /drive_wheels PULSE [FL,RL,FR,RR,duration]. The CAR keep-alives it locally at
    20 Hz for `duration` then brakes. The workstation does NOT stream: on lossy
    WiFi a high-rate stream wedges the MCU serial, so the keep-alive lives on the car.

  DriveActionActuator ("discrete") -- variant 2. Sends one discrete /drive_action
    pulse via carclient.drive() and paces by its duration.

Only import this module when driving the real car; it uses carclient (rospy).
"""

import time

from .kinematics import velocity_to_wheel_pwm


class VelocityPulseActuator(object):
    def __init__(self, client, robot_cfg, pulse_duration=0.65):
        self.client = client
        self.robot = robot_cfg
        # the car keep-alives each pulse this long; make it > the plan period so a
        # dropped/late next command holds the last velocity briefly, then brakes.
        self.pulse_duration = pulse_duration

    def start(self):
        pass                                   # the car does the keep-alive

    def set_velocity(self, vx, vy, wz):
        """Send one velocity pulse. Body velocity -> wheel PWM -> /drive_wheels."""
        pwm = velocity_to_wheel_pwm(vx, vy, wz, self.robot)
        self.client.drive_wheels(pwm[0], pwm[1], pwm[2], pwm[3], self.pulse_duration)

    def stop(self):
        """Brake: a zero/zero-duration pulse stops the car (car-side sustained brake)."""
        for _ in range(3):
            self.client.drive_wheels(0.0, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.02)

    def close(self):
        self.stop()


class DriveActionActuator(object):
    """Execute discrete hops via /drive_action, NON-BLOCKING.

    One hop is fired per tick and the next tick's hop supersedes it -- the car-side
    handler ends a running move ("superseded") and starts the new one immediately,
    so back-to-back hops are continuous with no brake in between. This used to
    sleep for the hop duration, which paced the loop off the hop instead of off the
    perception tick (0.5 + 0.12 s = ~1.6 Hz against 3 Hz perception, dropping about
    half the frames).

    The hop DURATION must be >= one tick, otherwise the hop expires before the next
    tick supersedes it and the car brakes in the gap (visible stutter). The runner
    checks this."""

    def __init__(self, client):
        self.client = client

    def step(self, action, magnitude, duration):
        """Fire one discrete hop and return immediately. Returns the latest
        /drive_result dict, best-effort, for logging."""
        self.client.drive(action, magnitude, duration)
        return self.client.last_result()

    def stop(self):
        self.client.stop()

    def close(self):
        self.stop()
