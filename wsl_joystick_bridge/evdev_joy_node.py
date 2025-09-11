#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes
import threading

# --- Konfiguration ---
DEVICE_PATH = '/dev/input/event0'
DEADZONE_TOL = 0.1

# --- Mappings ---
AXIS_MAP = {
    # Linker Stick
    ecodes.ABS_X: 0,
    ecodes.ABS_Y: 1,

    # Rechter Stick
    ecodes.ABS_Z: 2,
    ecodes.ABS_RZ: 3,

    # Trigger
    ecodes.ABS_RX: 4,
    ecodes.ABS_RY: 5,

    # D-Pad
    ecodes.ABS_HAT0X: 6,
    ecodes.ABS_HAT0Y: 7,
}

BUTTON_MAP = {
    ecodes.BTN_SOUTH: 0,
    ecodes.BTN_EAST: 1,
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
        
        # ROS Publisher initialisieren
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        
        # Initialisiere den Zustand der Joy-Nachricht
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 13

        # Starte den Controller-Lese-Thread
        self.thread = threading.Thread(target=self.controller_read_loop)
        self.thread.daemon = True
        self.thread.start()

    def controller_read_loop(self):
        """Diese Funktion läuft in einem separaten Thread und wartet auf Controller-Events."""
        try:
            device = InputDevice(DEVICE_PATH)
            self.get_logger().info(f"Controller '{device.name}' erfolgreich verbunden.")
        except (FileNotFoundError, PermissionError) as e:
            self.get_logger().error(f"Fehler beim Verbinden mit dem Controller: {e}")
            return

        for event in device.read_loop():
            # Aktualisiere die Joy-Nachricht basierend auf dem Event
            self.process_event(event)

            # Veröffentliche die aktualisierte Nachricht
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.joy_msg)

    def process_event(self, event):
        """Verarbeitet ein einzelnes evdev-Event und aktualisiert self.joy_msg."""
        # --- BUTTONS (unverändert) ---
        if event.type == ecodes.EV_KEY and event.code in BUTTON_MAP:
            button_index = BUTTON_MAP[event.code]
            self.joy_msg.buttons[button_index] = int(event.value > 0)

        # --- ACHSEN (komplett überarbeitet) ---
        elif event.type == ecodes.EV_ABS and event.code in AXIS_MAP:
            axis_index = AXIS_MAP[event.code]
            value = event.value

            norm_val = value / 255.0
            if event.code not in [ecodes.ABS_RX, ecodes.ABS_RY]:
                norm_val = norm_val * 2.0 - 1.0

            if event.code in [ecodes.ABS_Y, ecodes.ABS_RZ]:
                norm_val *= -1.0

            if abs(norm_val) < DEADZONE_TOL:
                norm_val = 0.0

            self.joy_msg.axes[axis_index] = norm_val

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