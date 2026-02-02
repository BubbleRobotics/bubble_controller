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
from std_msgs.msg import Float64MultiArray, Float32, String, Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data

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
    SAFETY = "safety"


class Ll_controller(Node):
    def __init__(self) -> None:
        super().__init__("ll_controller")


        # Parameters: battery monitor

        self.declare_parameter("battery_rate_hz", 10.0)
        self.declare_parameter("voltage_divider", 11.0)

        # Voltage thresholds (4S Li-ion)
        self.declare_parameter("warn_v", 14.0)
        self.declare_parameter("critical_v", 13.6)

        battery_rate_hz = float(self.get_parameter("battery_rate_hz").value)
        self._v_div = float(self.get_parameter("voltage_divider").value)
        self._warn_v = float(self.get_parameter("warn_v").value)
        self._critical_v = float(self.get_parameter("critical_v").value)

        # ADC channel used to read battery voltage
        self._adc_voltage = navigator.AdcChannel.Ch3


        # Parameters: config file

        self.declare_parameter("config_path")
        self.CONFIG_PATH = str(self.get_parameter("config_path").value)

        self.get_logger().info(f"config file {self.CONFIG_PATH}")
        config = _load_yaml(self.CONFIG_PATH)


        # Config: joystick + PWM setup

        self.GAMEPAD_SPAN = float(config.get("gamepad_span", 100.0))

        self.PWM_FREQ_HZ = float(config.get("pwm_freq_hz", 50.0))

        self.NEUTRAL_US = float(config.get("neutral_us", 1500.0))
        self.MIN_US = float(config.get("min_pwm_us", 1100.0))
        self.MAX_US = float(config.get("max_pwm_us", 1900.0))

        # Precomputed neutral raw value for the PWM driver
        self.NEUTRAL_RAW = us_to_value(self.NEUTRAL_US, self.PWM_FREQ_HZ)

        self.UPDATE_RATE_HZ = float(config.get("update_rate_hz", 50.0))
        self.TOPIC_TIMEOUT_S = 1


        # Config: twist references

        self.TWIST_VEL_XY = float(config.get("twist_vel_xy", 0.4))
        self.TWIST_VEL_Z = float(config.get("twist_vel_z", 0.4))
        self.TWIST_YAW_RATE = float(config.get("twist_yaw_rate", 0.6))
        self.TWIST_PITCH_RATE = float(config.get("twist_pitch_rate", 0.6))

        self.enable_pitch = bool(config.get("enable_pitch_control", False))


        # Config: distance safety

        self.distance_topic = str(config.get("distance_topic", "/stereo/min_distance"))
        self.min_distance = float(config.get("min_distance_before_safety", 0.5))


        # Config: EMA smoothing

        self.EMA_ENABLE = bool(config.get("ema_enable", True))
        self.EMA_ALPHA = float(config.get("ema_alpha", 0.2))
        self._ema_us: Dict[int, float] = {i: self.NEUTRAL_US for i in range(8)}


        # Parameters: mode publishing

        self.declare_parameter("mode_topic", "/controller_state")
        self.declare_parameter("mode_pub_period_s", 0.5)

        self._mode_topic = str(self.get_parameter("mode_topic").value)
        self._mode_pub_period_s = float(self.get_parameter("mode_pub_period_s").value)


        # State / channels / topics

        self.isArmed = False
        self.lightMode = 0
        
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


        # Parameters: joystick mapping

        self.declare_parameter("use_bluetooth", False)
        self.use_bluetooth = bool(self.get_parameter("use_bluetooth").value)
        if self.use_bluetooth:
            self.declare_parameter("joy_button_a", config.get("bluetooth_joy_button_a", 0))
            self.declare_parameter("joy_button_b", config.get("bluetooth_joy_button_b", 1))
            self.declare_parameter("joy_button_x", config.get("bluetooth_joy_button_x", 3))
            self.declare_parameter("joy_button_y", config.get("bluetooth_joy_button_y", 4))
            self.declare_parameter("joy_button_power", config.get("bluetooth_joy_button_power", 12))
            self.declare_parameter("joy_button_setting", config.get("bluetooth_joy_button_setting", 11))
            self.declare_parameter("joy_button_option", config.get("bluetooth_joy_button_option", 10))

            self.declare_parameter("joy_left_stick_up_down", config.get("bluetooth_joy_left_stick_up_down", 1))
            self.declare_parameter("joy_left_stick_left_right", config.get("bluetooth_joy_left_stick_left_right", 0))
            self.declare_parameter("joy_right_stick_up_down", config.get("bluetooth_joy_right_stick_up_down", 3))
            self.declare_parameter("joy_right_stick_left_right", config.get("bluetooth_joy_right_stick_left_right", 2))

            self.declare_parameter("joy_left_trigger", config.get("bluetooth_joy_left_trigger", 5))
            self.declare_parameter("joy_right_trigger", config.get("bluetooth_joy_right_trigger", 4))
        else:
            self.declare_parameter("joy_button_a", config.get("usb_joy_button_a", 0))
            self.declare_parameter("joy_button_b", config.get("usb_joy_button_b", 1))
            self.declare_parameter("joy_button_x", config.get("usb_joy_button_x", 2))
            self.declare_parameter("joy_button_y", config.get("usb_joy_button_y", 3))
            self.declare_parameter("joy_button_power", config.get("usb_joy_button_power", 8))
            self.declare_parameter("joy_button_setting", config.get("usb_joy_button_setting", 7))
            self.declare_parameter("joy_button_option", config.get("usb_joy_button_option", 6))

            self.declare_parameter("joy_left_stick_up_down", config.get("usb_joy_left_stick_up_down", 1))
            self.declare_parameter("joy_left_stick_left_right", config.get("usb_joy_left_stick_left_right", 0))
            self.declare_parameter("joy_right_stick_up_down", config.get("usb_joy_right_stick_up_down", 4))
            self.declare_parameter("joy_right_stick_left_right", config.get("usb_joy_right_stick_left_right", 3))

            self.declare_parameter("joy_left_trigger", config.get("usb_joy_left_trigger", 2))
            self.declare_parameter("joy_right_trigger", config.get("usb_joy_right_trigger", 5))
    
        self.declare_parameter("joy_dpad_up_down", 1)

        self._btn_a = int(self.get_parameter("joy_button_a").value)
        self._btn_b = int(self.get_parameter("joy_button_b").value)
        self._btn_x = int(self.get_parameter("joy_button_x").value)
        self._btn_y = int(self.get_parameter("joy_button_y").value)
        self._btn_power = int(self.get_parameter("joy_button_power").value)
        self._btn_setting = int(self.get_parameter("joy_button_setting").value)
        self._btn_option = int(self.get_parameter("joy_button_option").value)

        self._dpad_up_down = int(self.get_parameter("joy_dpad_up_down").value)
        
        self._left_stick_ud = int(self.get_parameter("joy_left_stick_up_down").value)
        self._left_stick_lr = int(self.get_parameter("joy_left_stick_left_right").value)
        self._right_stick_ud = int(self.get_parameter("joy_right_stick_up_down").value)
        self._right_stick_lr = int(self.get_parameter("joy_right_stick_left_right").value)
        self._left_trigger = int(self.get_parameter("joy_left_trigger").value)
        self._right_trigger = int(self.get_parameter("joy_right_trigger").value)

        # Initial control mode
        self._mode: ControlMode = ControlMode.GAMEPAD

        # Cached previous joy state (used for edge detection)
        self._prev_joy_buttons: Optional[List[int]] = None
        self._prev_joy_dpad: Optional[List[int]] = None


        # Publishers

        self.pub_pwm_us = self.create_publisher(Float64MultiArray, "/thrusters/pwm_us", 10)

        self.pub_twist_ref = self.create_publisher(
            Twist,
            "/adaptive_integral_terminal_sliding_mode_controller/reference",
            10,
        )

        # Store last received outputs and timestamps (used in AUV_CONTROLLER mode)
        now = self.get_clock().now()
        self.last_out: Dict[int, float] = {i: 0.0 for i in range(8)}
        self.last_time: Dict[int, Time] = {i: now for i in range(8)}


        # Navigator hardware init

        navigator.init()
        navigator.set_pwm_freq_hz(self.PWM_FREQ_HZ)
        # navigator.set_pwm_enable(True)


        # Battery publishers + timer

        self._pub_voltage = self.create_publisher(Float64, "/battery/voltage", qos_profile_sensor_data)
        self._pub_status = self.create_publisher(String, "/battery/status", qos_profile_sensor_data)

        self.battery_timer = 1.0 / max(battery_rate_hz, 0.1)
        self.create_timer(self.battery_timer, self._on_battery_timer)

        self.get_logger().info(
            f"Publishing /battery/voltage and /battery/status at {battery_rate_hz:.1f} Hz"
        )


        # Subscribers: AUV controller

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

        # Distance subscriber (best-effort, low-latency)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subdistance = self.create_subscription(Float32, self.distance_topic, self._cb_distance, qos)


        # Timers: control loop + mode publish

        # Timer to push PWM at fixed rate
        self.timer = self.create_timer(1.0 / self.UPDATE_RATE_HZ, self._on_timer)

        # Periodic publish timer
        self.pub_mode = self.create_publisher(String, self._mode_topic, 10)
        self._mode_timer = self.create_timer(self._mode_pub_period_s, self._publish_mode)
        self._publish_mode()

        # Start disarmed by default
        self._set_arm(False)


        # Startup log

        self.get_logger().info(
            "Started. "
            f"mode={self._mode.value}, Span={self.GAMEPAD_SPAN}, "
            f"TwistXY={self.TWIST_VEL_XY} m/s TwistZ={self.TWIST_VEL_Z} m/s "
            f"YawRate={self.TWIST_YAW_RATE} rad/s PitchRate={self.TWIST_PITCH_RATE} rad/s"
        )


    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self._mode.value
        self.pub_mode.publish(msg)

    def _set_arm(self, flag) -> None:
        # se non e' armato e la flag e' true, allora arma
        # se e' armato e la flag e' false, allora disarma
        if flag and not self.isArmed:
            navigator.set_pwm_enable(True)
            self.isArmed = True
            self._send_neutral_all()
            time.sleep(2.0)
            self.get_logger().info("Armed thrusters")
        elif not flag and self.isArmed:
            self._send_neutral_all()
            navigator.set_pwm_enable(False)
            self.isArmed = False
            self.get_logger().info("Disarmed thrusters")
            time.sleep(2.0)

    def _set_lights(self, us) -> None:
        navigator.set_pwm_channel_value(12, us_to_value(us, self.PWM_FREQ_HZ))
        self.isLightsOn = True
        self.get_logger().info(f"Lights set to {us} us")

    def _set_mode(self, mode: ControlMode) -> None:
        if mode == self._mode:
            return

        self._mode = mode
        self._reset_ema()

        if self._mode in (ControlMode.GAMEPAD, ControlMode.GAMEPAD_TWIST):
            self._send_neutral_all()

        self._publish_mode()
        self.get_logger().info(f"Switched mode -> {self._mode.value}")
    
    def _set_enable_ema(self, flag: bool) -> None:
        self.EMA_ENABLE = flag
        self.get_logger().info(f"EMA filtering: {self.EMA_ENABLE}")

    def _set_enable_pitch(self, flag: bool) -> None:
        self.enable_pitch = flag
        self.get_logger().info(f"Pitch control: {self.enable_pitch}")
    
    def _set_linear_vel(self, val:int) -> None:
        delta = 0.1
        if val > 0:
            self.TWIST_VEL_XY += delta
            self.TWIST_VEL_Z += delta
        elif val < 0:
            self.TWIST_VEL_XY = max(0.1, self.TWIST_VEL_XY - delta)
            self.TWIST_VEL_Z = max(0.1, self.TWIST_VEL_Z - delta)
        self.get_logger().info(f"Linear velocities updated: TWIST_VEL_XY={self.TWIST_VEL_XY} m/s, TWIST_VEL_Z={self.TWIST_VEL_Z} m/s")

    def _set_angular_vel(self, val:int) -> None:
        delta = 0.1
        if val > 0:
            self.TWIST_YAW_RATE += delta
            self.TWIST_PITCH_RATE += delta
        elif val < 0:
            self.TWIST_YAW_RATE = max(0.1, self.TWIST_YAW_RATE - delta)
            self.TWIST_PITCH_RATE = max(0.1, self.TWIST_PITCH_RATE - delta)
        self.get_logger().info(f"Angular velocities updated: TWIST_YAW_RATE={self.TWIST_YAW_RATE} rad/s, TWIST_PITCH_RATE={self.TWIST_PITCH_RATE} rad/s")

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
        joy_right_stick_up_down = axes[self._right_stick_ud] if self.enable_pitch else 0.0
        dpad = (axes[6], axes[7])

        if self._prev_joy_buttons is None:
            self._prev_joy_buttons = buttons
            self._prev_joy_dpad = dpad
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
            self.lightMode = (self.lightMode + 1) % 3
            if self.lightMode == 0:
                self._set_lights(1500.0)
            elif self.lightMode == 1:
                self._set_lights(1700.0)
            else:
                self._set_lights(1900.0)

        if rising_edge(self._btn_y):
            self._set_mode(ControlMode.GAMEPAD_TWIST)
            
        if rising_edge(self._btn_power):
            self._set_arm(not self.isArmed)

        if rising_edge(self._btn_setting):
            self._set_enable_pitch(not self.enable_pitch)

        if rising_edge(self._btn_option):
            self._set_enable_ema(not self.EMA_ENABLE)
        
        prev_dx, prev_dy = self._prev_joy_dpad
        dx, dy = dpad  # dx: left/right, dy: up/down

        def dpad_rising(prev: float, cur: float, direction: int) -> bool:
            """True only on transition from 0 -> +/-1 for the given direction."""
            return (prev == 0.0) and (cur == float(direction))
        if dpad_rising(prev_dy, dy, +1):   # up pressed
            self._set_linear_vel(+1)
        elif dpad_rising(prev_dy, dy, -1): # down pressed
            self._set_linear_vel(-1)

        if dpad_rising(prev_dx, dx, +1):   # left pressed
            self._set_angular_vel(+1)
        elif dpad_rising(prev_dx, dx, -1): # right pressed
            self._set_angular_vel(-1)

        self._prev_joy_buttons = buttons
        self._prev_joy_dpad = dpad

        if self._mode == ControlMode.GAMEPAD:
            t1 = axes_to_pwm_raw(axes[self._left_stick_lr], axes[self._left_stick_ud], axes[self._right_stick_lr], span = self.GAMEPAD_SPAN) # XY yaw

            t2 = vertical_mix_pwm_us(axes[self._right_trigger], axes[self._left_trigger], joy_right_stick_up_down, span = self.GAMEPAD_SPAN) # vertical thrusters

            t = t1 + t2

            navigator.set_pwm_channels_values(self.channels, t)

            # Publish debug output
            msg = Float64MultiArray()
            msg.data = t
            self.pub_pwm_us.publish(msg)

        elif self._mode == ControlMode.GAMEPAD_TWIST:
            if len(axes) < 6:
                return
            x_cmd = clamp(-axes[self._left_stick_lr], -1.0, 1.0)  # right (+)
            y_cmd = clamp( axes[self._left_stick_ud], -1.0, 1.0)  # forward (+)
            yaw_cmd = clamp(-axes[self._right_stick_lr], -1.0, 1.0)  # yaw right (+)

            x_cmd, y_cmd = normalize_xy(x_cmd, y_cmd)

            t_up = trigger_to_01(axes[self._right_trigger])   
            t_down = trigger_to_01(axes[self._left_trigger])
            u_z_up = clamp(t_up - t_down, -1.0, 1.0)

            # Pitch command
            u_pitch = clamp(joy_right_stick_up_down, -1.0, 1.0)

            tw = Twist()
            tw.linear.x = float(y_cmd * self.TWIST_VEL_XY)     # forward
            tw.linear.y = float(x_cmd * self.TWIST_VEL_XY)     # right
            tw.linear.z = float((-u_z_up) * self.TWIST_VEL_Z)  # down positive

            tw.angular.x = 0.0
            tw.angular.y = float(u_pitch * self.TWIST_PITCH_RATE)
            tw.angular.z = float(yaw_cmd * self.TWIST_YAW_RATE)

            self.pub_twist_ref.publish(tw)

    def _cb_distance(self, msg: Float32) -> None:
        distance = msg.data
        if distance < self.min_distance:
            if self._mode != ControlMode.SAFETY:
                self._set_mode(ControlMode.SAFETY)
                self.get_logger().warn(f"distance {distance:.2f} m below minimum {self.min_distance:.2f} m! Switching to SAFETY mode.")
                self._send_backward()

    def _reset_ema(self) -> None:
        for i in range(8):
            self._ema_us[i] = self.NEUTRAL_US

    def _send_backward(
        self,
        backward_us: float = 1300.0,
        duration_s: float = 0.5,
    ):
        horizontal = [0, 1, 2, 3]
        vertical = [4, 5, 6, 7]

        # Convert us -> raw
        backward_raw = us_to_value(backward_us, self.PWM_FREQ_HZ)
        neutral_raw = us_to_value(self.NEUTRAL_US, self.PWM_FREQ_HZ)

        channels = horizontal + vertical
        values = (
            [backward_raw] * len(horizontal) +
            [neutral_raw] * len(vertical)
        )

        navigator.set_pwm_channels_values(channels, values)

        time.sleep(duration_s)

        self._send_neutral_all()
        self._set_arm(False)

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

            # Target command (in us)
            if age_s > self.TOPIC_TIMEOUT_S:
                target_us = self.NEUTRAL_US
            else:
                target_us = clamp_float(self.last_out[i], self.MIN_US, self.MAX_US)

            # EMA filtering (in us)
            #get timestamp before and after to check difference

            if self.EMA_ENABLE:
                prev = self._ema_us[i]
                filt = (1.0 - self.EMA_ALPHA) * prev + self.EMA_ALPHA * target_us
                self._ema_us[i] = filt
                send_us = filt
            else:
                send_us = target_us
            # Convert to raw PWM value
            send_us = clamp_float(send_us, self.MIN_US, self.MAX_US)
            raw = us_to_value(send_us, self.PWM_FREQ_HZ)
            values_raw.append(raw)

        # Command to thrusters
        navigator.set_pwm_channels_values(self.channels, values_raw)

        # Publish debug output
        msg = Float64MultiArray()
        msg.data = values_raw
        self.pub_pwm_us.publish(msg)

    def _on_battery_timer(self) -> None:
        try:
            # Read ADC voltage (0..3.3V)
            v_adc = float(navigator.read_adc(self._adc_voltage))

            # Convert to battery voltage
            v_bat = v_adc * self._v_div

            # Publish voltage
            v_msg = Float64()
            v_msg.data = v_bat
            self._pub_voltage.publish(v_msg)

            # Determine status
            if v_bat <= self._critical_v:
                status = "CRITICAL"
            elif v_bat <= self._warn_v:
                status = "WARNING"
            else:
                status = "OK"

            s_msg = String()
            s_msg.data = status
            self._pub_status.publish(s_msg)

        except Exception as e:
            self.get_logger().error(f"Battery read failed: {e}")

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
