"""Keyboard motion commands for the WBC controller.

A small Tk window: the terminal sends Ctrl+Enter as a plain Enter, so the key events are read from
an X window instead (keep it focused).
    Ctrl+Enter -> MotionCommand.INIT_WALKING (lower the base to the initial walking posture)
    Ctrl+Space -> MotionCommand.HOMING       (ramp all the joints to 0)
    Ctrl+Up    -> MotionCommand.WALKING      (hold: step; release: stop and rest)
    Ctrl+A / Ctrl+D -> turn left / right in place (hold; also while Ctrl+Up is held: no forward step)
Ctrl+Enter / Ctrl+Space publish one message per key press on /humanoid/motion_command (reliable:
a command is an event). While Ctrl+Up, Ctrl+A or Ctrl+D is held, WALKING is published at 20 Hz; the
controller stops the robot when the messages stop. The turn (angular_velocity_z = +1 left, -1
right, 0 none) goes at 20 Hz on /humanoid/walking_velocity_command (WalkingVelocityCommand, best
effort as the other velocity publishers; the controller scales it by its max_yaw_rate).
"""

import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from humanoid_mpc_msgs.msg import MotionCommand, WalkingVelocityCommand


class KeyboardMotionCommandPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_motion_command_publisher")
        self.publisher_ = self.create_publisher(MotionCommand, "/humanoid/motion_command", 10)
        self.turn_publisher_ = self.create_publisher(
            WalkingVelocityCommand, "/humanoid/walking_velocity_command",
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))

    def publish_turn(self, turn):
        # turn only: the forward speed is the controller's walk_vx
        msg = WalkingVelocityCommand()
        msg.angular_velocity_z = float(turn)
        self.turn_publisher_.publish(msg)

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
                        "Ctrl+Up (hold): walk, release: stop\n"
                        "Ctrl+A / Ctrl+D (hold): turn left / right in place",
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

    # Ctrl+A / Ctrl+D held: turn (+1 left, -1 right); the same autorepeat filter as walking
    turn = {"value": 0, "release_job": None}

    def turn_press(value):
        if turn["release_job"] is not None:
            root.after_cancel(turn["release_job"])
            turn["release_job"] = None
        if turn["value"] != value:
            turn["value"] = value
            name = "left" if value > 0 else "right"
            node.get_logger().info(f"turning {name}")
            status.config(text=f"walking, turning {name}", fg="black")

    def turn_stop():
        turn["value"] = 0
        turn["release_job"] = None
        status.config(text="walking" if walk["held"] else "stopped", fg="black")

    def turn_release(_e):
        if turn["value"] != 0 and turn["release_job"] is None:
            turn["release_job"] = root.after(100, turn_stop)

    # both cases: Caps Lock gives the upper-case keysyms
    for key in ("<Control-a>", "<Control-A>"):
        root.bind(key, lambda _e: turn_press(1))
    for key in ("<Control-d>", "<Control-D>"):
        root.bind(key, lambda _e: turn_press(-1))
    for key in ("<KeyRelease-a>", "<KeyRelease-A>", "<KeyRelease-d>", "<KeyRelease-D>",
                "<KeyRelease-Control_L>", "<KeyRelease-Control_R>"):
        root.bind(key, turn_release, add="+")

    # WALKING at 20 Hz while the walk or a turn key is held, with the turn command
    def walk_tick():
        if walk["held"] or turn["value"] != 0:
            node.publish(MotionCommand.WALKING)
            node.publish_turn(turn["value"])
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
