#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes
import threading
import time

# --- Standard-Konfiguration (Default Parameterwerte) ---
DEFAULT_DEVICE_PATH = '/dev/input/event0'
DEFAULT_DEADZONE = 0.1
DEFAULT_TOPIC = 'joy'
DEFAULT_FRAME_ID = ''
DEFAULT_RECONNECT_SECONDS = 3.0

# --- Mappings ---
AXIS_MAP = {
    # Linker Stick
    ecodes.ABS_X: 0,
    ecodes.ABS_Y: 1,

    # Left Trigger
    ecodes.ABS_RX: 4,

    # Rechter Stick
    ecodes.ABS_Z: 2,
    ecodes.ABS_RZ: 3,

    # Right Trigger
    ecodes.ABS_RY: 5,

    # D-Pad
    ecodes.ABS_HAT0X: 6,
    ecodes.ABS_HAT0Y: 7,
}

BUTTON_MAP = {
    ecodes.BTN_EAST: 0,
    ecodes.BTN_SOUTH: 1,
    ecodes.BTN_WEST: 2,
    ecodes.BTN_NORTH: 3,
    ecodes.BTN_TL: 4,
    ecodes.BTN_TR: 5,
    ecodes.BTN_TL2: 6,
    ecodes.BTN_TR2: 7,
    ecodes.BTN_SELECT: 8,
    ecodes.BTN_START: 9,
    ecodes.BTN_MODE: 10,
    ecodes.BTN_THUMBL: 11,
    ecodes.BTN_THUMBR: 12,
}

class EvdevJoyNode(Node):
    def __init__(self):
        super().__init__('evdev_joy_node')

        self.declare_parameter('joy_topic', DEFAULT_TOPIC)
        self.declare_parameter('device_path', DEFAULT_DEVICE_PATH)
        self.declare_parameter('deadzone', DEFAULT_DEADZONE)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.declare_parameter('reconnect_seconds', DEFAULT_RECONNECT_SECONDS)

        self.joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.reconnect_seconds = float(self.get_parameter('reconnect_seconds').value)

        self.publisher_ = self.create_publisher(Joy, self.joy_topic, 10)
        self.get_logger().info(f"Joystick Publisher auf Topic '{self.joy_topic}' gestartet (Device: {self.device_path})")

        self.add_on_set_parameters_callback(self.on_parameters_set)

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 13

        self._stop_event = threading.Event()

        self.thread = threading.Thread(target=self.controller_read_loop, name='evdev_reader')
        self.thread.daemon = True
        self.thread.start()

    # ---------------- Parameter Update Callback ----------------
    def on_parameters_set(self, params):
        updates = []
        for p in params:
            if p.name == 'deadzone':
                val = float(p.value)
                if not (0.0 <= val < 1.0):
                    from rclpy.parameter import SetParametersResult
                    self.get_logger().error('Deadzone muss zwischen 0.0 (inkl.) und 1.0 (exkl.) liegen.')
                    return SetParametersResult(successful=False)
                self.deadzone = val
                updates.append(f"deadzone={val:.3f}")
            elif p.name == 'frame_id':
                self.frame_id = str(p.value)
                updates.append(f"frame_id='{self.frame_id}'")
            elif p.name == 'joy_topic':
                self.get_logger().warn('Änderung von joy_topic zur Laufzeit nicht unterstützt. Bitte Node neu starten.')
            elif p.name == 'device_path':
                self.get_logger().warn('Änderung von device_path zur Laufzeit nicht unterstützt. Bitte Node neu starten.')
            elif p.name == 'reconnect_seconds':
                try:
                    self.reconnect_seconds = max(0.5, float(p.value))
                    updates.append(f"reconnect_seconds={self.reconnect_seconds}")
                except ValueError:
                    from rclpy.parameter import SetParametersResult
                    self.get_logger().error('reconnect_seconds muss eine Zahl sein.')
                    return SetParametersResult(successful=False)

        if updates:
            self.get_logger().info('Parameter aktualisiert: ' + ', '.join(updates))
        from rclpy.parameter import SetParametersResult
        return SetParametersResult(successful=True)

    def controller_read_loop(self):
        """Liest kontinuierlich Events; versucht bei Fehlern erneut zu verbinden."""
        last_error_logged = 0.0
        while not self._stop_event.is_set():
            try:
                device = InputDevice(self.device_path)
                self.get_logger().info(f"Controller '{device.name}' verbunden (Pfad: {self.device_path}).")
                for event in device.read_loop():
                    if self._stop_event.is_set():
                        break
                    self.process_event(event)
                    self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                    self.joy_msg.header.frame_id = self.frame_id
                    self.publisher_.publish(self.joy_msg)
            except (FileNotFoundError, PermissionError, OSError) as e:
                now = time.time()
                if now - last_error_logged > 2.0:
                    self.get_logger().warn(f"Kann Device '{self.device_path}' nicht öffnen: {e}. Erneuter Versuch in {self.reconnect_seconds}s")
                    last_error_logged = now
                if self._stop_event.wait(self.reconnect_seconds):
                    break
            except Exception as e:
                self.get_logger().error(f"Unerwarteter Fehler im Lesethread: {e}")
                if self._stop_event.wait(self.reconnect_seconds):
                    break

    def destroy_node(self):
        self._stop_event.set()
        try:
            if self.thread.is_alive():
                self.thread.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()

    def process_event(self, event):
        """Verarbeitet ein einzelnes evdev-Event und aktualisiert self.joy_msg."""
        # --- BUTTONS ---

        if event.type == ecodes.EV_KEY and event.code in BUTTON_MAP:
            button_index = BUTTON_MAP[event.code]
            self.joy_msg.buttons[button_index] = int(event.value > 0)
            if event.value > 0:
                self.get_logger().debug(f"Event: type={event.type}, code={event.code}, value={event.value}")

        # --- ACHSEN ---
        elif event.type == ecodes.EV_ABS and event.code in AXIS_MAP:
            axis_index = AXIS_MAP[event.code]
            value = event.value

            if event.code in [ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y]:
                norm_val = value
            else:
                norm_val = value / 255.0

            if event.code not in [ecodes.ABS_RX, ecodes.ABS_RY, ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y]:
                norm_val = norm_val * 2.0 - 1.0

            if event.code in [ecodes.ABS_Y, ecodes.ABS_RZ, ecodes.ABS_HAT0Y]:
                norm_val *= -1.0

            if abs(norm_val) < self.deadzone:
                norm_val = 0.0

            self.joy_msg.axes[axis_index] = norm_val

            if norm_val > 0:
                self.get_logger().debug(f"Event: type={event.type}, code={event.code}, value={event.value}")

def main(args=None):
    rclpy.init(args=args)
    node = EvdevJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()