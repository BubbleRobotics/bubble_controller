#!/usr/bin/env python3
import time
from enum import Enum
from typing import Dict, List, Optional
import os 
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from control_msgs.msg import SingleDOFStateStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import bluerobotics_navigator as navigator
from bluerobotics_navigator import Raspberry, NavigatorVersion

from utils import clamp_float, us_to_value, axes_to_pwm_raw, vertical_mix_pwm_us, clamp, normalize_xy, trigger_to_01

navigator.set_raspberry_pi_version(Raspberry.Pi5)
navigator.set_navigator_version(NavigatorVersion.Version2)

def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}

class ControlMode(str, Enum):
    GAMEPAD = "gamepad"
    AUV_CONTROLLER = "auv_controller"
    GAMEPAD_TWIST = "gamepad_twist"


class Ll_controller(Node):
    def __init__(self) -> None:
        super().__init__("ll_controller")

        self.declare_parameter("config_path")
        self.CONFIG_PATH = str(self.get_parameter("config_path").value)

        self.get_logger().info(f"config file {self.CONFIG_PATH}")
        config = _load_yaml(self.CONFIG_PATH)

        self.GAMEPAD_SPAN = float(config.get("gamepad_span", 100.0))

        self.PWM_FREQ_HZ = float(config.get("pwm_freq_hz", 50.0))

        self.NEUTRAL_US = float(config.get("neutral_us", 1500.0))
        self.MIN_US = float(config.get("min_pwm_us", 1100.0))
        self.MAX_US = float(config.get("max_pwm_us", 1900.0))

        self.NEUTRAL_RAW = us_to_value(self.NEUTRAL_US, self.PWM_FREQ_HZ)

        self.UPDATE_RATE_HZ = float(config.get("update_rate_hz", 50.0))
        self.TOPIC_TIMEOUT_S = 0.3

        self.TWIST_VEL_XY = float(config.get("twist_vel_xy", 0.4))      
        self.TWIST_VEL_Z = float(config.get("twist_vel_z", 0.3))        
        self.TWIST_YAW_RATE = float(config.get("twist_yaw_rate", 0.6))  
        self.TWIST_PITCH_RATE = float(config.get("twist_pitch_rate", 0.6))

        self.ENABLE_PITCH_CONTROL = bool(config.get("enable_pitch_control", False))

        self.channels = [0, 1, 2, 3, 4, 5, 6, 7]

        self.topics: List[str] = [
            "/thruster_1_controller/status",
            "/thruster_2_controller/status",
            "/thruster_3_controller/status",
            "/thruster_4_controller/status",
            "/thruster_5_controller/status",
            "/thruster_6_controller/status",
            "/thruster_7_controller/status",
            "/thruster_8_controller/status",
        ]

        self.declare_parameter("joy_button_a", 0)
        self.declare_parameter("joy_button_b", 1)
        self.declare_parameter("joy_button_x", 2)
        self._btn_a: int = int(self.get_parameter("joy_button_a").value)
        self._btn_b: int = int(self.get_parameter("joy_button_b").value)
        self._btn_x: int = int(self.get_parameter("joy_button_x").value)

        self._mode: ControlMode = ControlMode.GAMEPAD

        self._prev_joy_buttons: Optional[List[int]] = None

        self.pub_pwm_us = self.create_publisher(
            Float64MultiArray,
            "/thrusters/pwm_us",
            10,
        )

        self.pub_twist_ref = self.create_publisher(
            Twist,
            "/adaptive_integral_terminal_sliding_mode_controller/reference",
            10,
        )

        now = self.get_clock().now()
        # store last received output (float), and time (used in AUV_CONTROLLER mode)
        self.last_out: Dict[int, float] = {i: 0.0 for i in range(8)}
        self.last_time: Dict[int, Time] = {i: now for i in range(8)}

        navigator.init()
        navigator.set_pwm_freq_hz(self.PWM_FREQ_HZ)
        navigator.set_pwm_enable(True)

        # Neutral for ESC arming
        self._send_neutral_all()
        time.sleep(2.0)

        # AUV controller subscribers
        self.sub1 = self.create_subscription(SingleDOFStateStamped, self.topics[0], lambda m: self._cb_thruster(0, m), 10)
        self.sub2 = self.create_subscription(SingleDOFStateStamped, self.topics[1], lambda m: self._cb_thruster(1, m), 10)
        self.sub3 = self.create_subscription(SingleDOFStateStamped, self.topics[2], lambda m: self._cb_thruster(2, m), 10)
        self.sub4 = self.create_subscription(SingleDOFStateStamped, self.topics[3], lambda m: self._cb_thruster(3, m), 10)
        self.sub5 = self.create_subscription(SingleDOFStateStamped, self.topics[4], lambda m: self._cb_thruster(4, m), 10)
        self.sub6 = self.create_subscription(SingleDOFStateStamped, self.topics[5], lambda m: self._cb_thruster(5, m), 10)
        self.sub7 = self.create_subscription(SingleDOFStateStamped, self.topics[6], lambda m: self._cb_thruster(6, m), 10)
        self.sub8 = self.create_subscription(SingleDOFStateStamped, self.topics[7], lambda m: self._cb_thruster(7, m), 10)

        # Gamepad subscriber
        self.subjoy = self.create_subscription(Joy, "/joy", self._cb_joy, 10)

        # Timer to push PWM at fixed rate
        self.timer = self.create_timer(1.0 / self.UPDATE_RATE_HZ, self._on_timer)

        self.get_logger().info(
            "Started. "
            f"mode={self._mode.value} PWM_FREQ_HZ={self.PWM_FREQ_HZ}, update={self.UPDATE_RATE_HZ}Hz, "
            f"joy A={self._btn_a}, B={self._btn_b}, X={self._btn_x}, Span={self.GAMEPAD_SPAN}, "
            f"TwistXY={self.TWIST_VEL_XY} m/s TwistZ={self.TWIST_VEL_Z} m/s "
            f"YawRate={self.TWIST_YAW_RATE} rad/s PitchRate={self.TWIST_PITCH_RATE} rad/s"
        )

    def _set_mode(self, mode: ControlMode) -> None:
        if mode == self._mode:
            return

        self._mode = mode

        if self._mode in (ControlMode.GAMEPAD, ControlMode.GAMEPAD_TWIST):
            self._send_neutral_all()

        self.get_logger().info(f"Switched mode -> {self._mode.value}")

    def _cb_thruster(self, idx: int, msg: SingleDOFStateStamped) -> None:
        if idx in (0, 1, 4, 7):  # invert ccw thrusters
            value = float(msg.dof_state.output)
            asd = 1500 - value
            self.last_out[idx] = 1500 + asd
        else:
            self.last_out[idx] = float(msg.dof_state.output)

        self.last_time[idx] = self.get_clock().now()

    def _cb_joy(self, msg: Joy) -> None:
        # mode switching on A/B
        buttons = list(msg.buttons) if msg.buttons is not None else []
        axes = list(msg.axes) if msg.axes is not None else []
        axes_4 = axes[4] if self.ENABLE_PITCH_CONTROL else 0.0

        if self._prev_joy_buttons is None:
            self._prev_joy_buttons = buttons
            return

        def rising_edge(btn_idx: int) -> bool:
            if btn_idx < 0:
                return False
            if btn_idx >= len(buttons) or btn_idx >= len(self._prev_joy_buttons):
                return False
            return (self._prev_joy_buttons[btn_idx] == 0) and (buttons[btn_idx] == 1)

        if rising_edge(self._btn_a):
            self._set_mode(ControlMode.GAMEPAD)

        if rising_edge(self._btn_b):
            self._set_mode(ControlMode.AUV_CONTROLLER)

        if rising_edge(self._btn_x):
            self._set_mode(ControlMode.GAMEPAD_TWIST)

        self._prev_joy_buttons = buttons

        if self._mode == ControlMode.GAMEPAD:
            t1 = axes_to_pwm_raw(axes[0], axes[1], axes[3], span = self.GAMEPAD_SPAN) # XY yaw

            t2 = vertical_mix_pwm_us(axes[5], axes[2], axes_4, span = self.GAMEPAD_SPAN) # vertical thrusters

            t = t1 + t2

            navigator.set_pwm_channels_values(self.channels, t)

            # Publish debug output
            msg = Float64MultiArray()
            msg.data = t
            self.pub_pwm_us.publish(msg)

        elif self._mode == ControlMode.GAMEPAD_TWIST:
            if len(axes) < 6:
                return
            x_cmd = clamp(-axes[0], -1.0, 1.0)  # right (+)
            y_cmd = clamp( axes[1], -1.0, 1.0)  # forward (+)
            yaw_cmd = clamp(-axes[3], -1.0, 1.0)  # yaw right (+)

            x_cmd, y_cmd = normalize_xy(x_cmd, y_cmd)

            t_up = trigger_to_01(axes[5])   
            t_down = trigger_to_01(axes[2])
            u_z_up = clamp(t_up - t_down, -1.0, 1.0)

            # Pitch command
            u_pitch = clamp(axes_4, -1.0, 1.0)

            tw = Twist()
            tw.linear.x = float(y_cmd * self.TWIST_VEL_XY)     # forward
            tw.linear.y = float(x_cmd * self.TWIST_VEL_XY)     # right
            tw.linear.z = float((-u_z_up) * self.TWIST_VEL_Z)  # down positive

            tw.angular.x = 0.0
            tw.angular.y = float(u_pitch * self.TWIST_PITCH_RATE)
            tw.angular.z = float(yaw_cmd * self.TWIST_YAW_RATE)

            self.pub_twist_ref.publish(tw)

    def _send_neutral_all(self) -> None:
        navigator.set_pwm_channels_values(self.channels, [self.NEUTRAL_RAW] * 8)

    def _to_pwm_raw(self, out: float) -> int:
        pulse_us = clamp_float(out, self.MIN_US, self.MAX_US)
        return us_to_value(pulse_us, self.PWM_FREQ_HZ)

    def _on_timer(self) -> None:
        # IMPORTANT: do not apply auv_controller commands while in GAMEPAD mode
        if self._mode == ControlMode.GAMEPAD:
            return

        now = self.get_clock().now()
        values_raw: List[int] = []

        for i in range(8):
            age_s = (now - self.last_time[i]).nanoseconds * 1e-9
            if age_s > self.TOPIC_TIMEOUT_S:
                raw = self.NEUTRAL_RAW
                us = self.NEUTRAL_US
            else:
                us = clamp_float(self.last_out[i], self.MIN_US, self.MAX_US)
                raw = us_to_value(us, self.PWM_FREQ_HZ)

            values_raw.append(raw)

        # Command to thrusters
        navigator.set_pwm_channels_values(self.channels, values_raw)

        # Publish debug output
        msg = Float64MultiArray()
        msg.data = values_raw
        self.pub_pwm_us.publish(msg)

    def deinit(self) -> None:
        try:
            self._send_neutral_all()
            navigator.set_pwm_enable(False)
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = Ll_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected. Shutting down...")
        node.deinit()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
