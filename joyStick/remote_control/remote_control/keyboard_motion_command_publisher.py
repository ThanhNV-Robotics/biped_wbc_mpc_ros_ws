"""Keyboard motion commands for the WBC controller.

A small Tk window: the terminal sends Ctrl+Enter as a plain Enter, so the key events are read from
an X window instead (keep it focused).
    Ctrl+Enter -> MotionCommand.INIT_WALKING (lower the base to the initial walking posture)
    Ctrl+Space -> MotionCommand.HOMING       (ramp all the joints to 0)
    Ctrl+Up    -> MotionCommand.WALKING      (hold: step; release: stop and rest)
Ctrl+Enter / Ctrl+Space publish one message per key press on /humanoid/motion_command (reliable:
a command is an event). While Ctrl+Up is held, WALKING is published at 20 Hz; the controller stops
the robot when the messages stop.
"""

import tkinter as tk

import rclpy
from rclpy.node import Node
from humanoid_mpc_msgs.msg import MotionCommand


class KeyboardMotionCommandPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_motion_command_publisher")
        self.publisher_ = self.create_publisher(MotionCommand, "/humanoid/motion_command", 10)

    def publish(self, command, name=None):
        msg = MotionCommand()
        msg.command = command
        self.publisher_.publish(msg)
        if name:
            self.get_logger().info(f"sent {name} ({command})")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardMotionCommandPublisher()

    # window: key bindings + the last command sent
    root = tk.Tk()
    root.title("Biped motion command")
    tk.Label(root, text="Ctrl+Enter: init walking (bend knees)\nCtrl+Space: homing (joints to 0)\n"
                        "Ctrl+Up (hold): walk, release: stop",
             justify="left", padx=20, pady=10).pack()
    status = tk.Label(root, text="no command sent", fg="gray", pady=10)
    status.pack()

    def send(command, name):
        node.publish(command, name)
        status.config(text=f"sent: {name}", fg="black")

    # Ctrl+Enter on both Enter keys (main and keypad)
    for key in ("<Control-Return>", "<Control-KP_Enter>"):
        root.bind(key, lambda _e: send(MotionCommand.INIT_WALKING, "init walking"))
    root.bind("<Control-space>", lambda _e: send(MotionCommand.HOMING, "homing"))

    # Ctrl+Up held: X key autorepeat sends release / press pairs while a key is held, so a release
    # only counts if no new press follows within 100 ms
    walk = {"held": False, "release_job": None}

    def walk_press(_e):
        if walk["release_job"] is not None:
            root.after_cancel(walk["release_job"])
            walk["release_job"] = None
        if not walk["held"]:
            walk["held"] = True
            node.get_logger().info("walking (Ctrl+Up held)")
            status.config(text="walking", fg="black")

    def walk_stop():
        walk["held"] = False
        walk["release_job"] = None
        node.get_logger().info("walk key released: stop")
        status.config(text="stopped", fg="black")

    def walk_release(_e):
        if walk["held"] and walk["release_job"] is None:
            walk["release_job"] = root.after(100, walk_stop)

    root.bind("<Control-Up>", walk_press)
    for key in ("<KeyRelease-Up>", "<KeyRelease-Control_L>", "<KeyRelease-Control_R>"):
        root.bind(key, walk_release)

    # WALKING at 20 Hz while the key is held
    def walk_tick():
        if walk["held"]:
            node.publish(MotionCommand.WALKING)
        root.after(50, walk_tick)

    root.after(50, walk_tick)

    # ROS callbacks between Tk events (the node has no subscriptions, this keeps it responsive)
    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(50, spin_once)

    root.after(50, spin_once)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
