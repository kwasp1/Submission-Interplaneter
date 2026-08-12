#!/usr/bin/env python3
"""
Telemetry dashboard + manual control GUI node.

Combines two of the assignment's required components:
  - Telemetry Dashboard: real-time display of position, orientation,
    velocities, and system status (subscribes to /model/x3/odometry).
  - Dual-Mode Switching: a Voice/Manual toggle button, plus manual
    directional control buttons that only act while in Manual mode.

Threading note: Tkinter owns the main thread (GUI toolkits expect
this). Instead of running rclpy.spin() in a background thread, we use
Tkinter's own .after() scheduler to periodically call
rclpy.spin_once() - this interleaves ROS2 message processing into
Tkinter's existing event loop rather than fighting over the main
thread.
"""
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def quaternion_to_euler(x, y, z, w):
    """Returns (roll, pitch, yaw) in radians from a quaternion."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for numerical safety
    pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# Manual button -> (linear.x, linear.y, linear.z, angular.z)
MANUAL_COMMANDS = {
    'Forward':  (0.4, 0.0, 0.0, 0.0),
    'Backward': (-0.4, 0.0, 0.0, 0.0),
    'Left':     (0.0, 0.4, 0.0, 0.0),
    'Right':    (0.0, -0.4, 0.0, 0.0),
    'Up':       (0.0, 0.0, 0.4, 0.0),
    'Down':     (0.0, 0.0, -0.4, 0.0),
    'Stop':     (0.0, 0.0, 0.0, 0.0),
}


class TelemetryGuiNode(Node):
    """ROS2 node half of this app - pub/sub only, no GUI code here."""

    def __init__(self):
        super().__init__('telemetry_gui_node')

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/X3/gazebo/command/twist', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/model/x3/odometry', self.odom_callback, 10)

        # Latest telemetry snapshot, read by the GUI on each refresh.
        self.latest_odom = None

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def publish_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def publish_twist(self, values):
        lx, ly, lz, az = values
        twist = Twist()
        twist.linear.x = lx
        twist.linear.y = ly
        twist.linear.z = lz
        twist.angular.z = az
        self.cmd_vel_pub.publish(twist)


class TelemetryApp:
    """Tkinter GUI half - owns the main thread and the mainloop."""

    def __init__(self, root: tk.Tk, ros_node: TelemetryGuiNode):
        self.root = root
        self.ros_node = ros_node
        self.mode = 'manual'  # default mode, matches voice_control_node's default

        root.title('Quadrotor Telemetry & Control')
        root.geometry('480x520')

        # --- Mode toggle ---
        mode_frame = ttk.LabelFrame(root, text='Control Mode')
        mode_frame.pack(fill='x', padx=10, pady=8)

        self.mode_label = ttk.Label(
            mode_frame, text='Current Mode: MANUAL',
            font=('Arial', 12, 'bold'))
        self.mode_label.pack(pady=4)

        self.toggle_btn = ttk.Button(
            mode_frame, text='Switch to Voice Mode',
            command=self.toggle_mode)
        self.toggle_btn.pack(pady=4)

        # --- Telemetry display ---
        telem_frame = ttk.LabelFrame(root, text='Telemetry')
        telem_frame.pack(fill='x', padx=10, pady=8)

        self.position_label = ttk.Label(telem_frame, text='Position (X, Y, Z): --')
        self.position_label.pack(anchor='w', padx=6, pady=2)

        self.orientation_label = ttk.Label(telem_frame, text='Orientation (R, P, Y deg): --')
        self.orientation_label.pack(anchor='w', padx=6, pady=2)

        self.lin_vel_label = ttk.Label(telem_frame, text='Linear Velocity (X, Y, Z): --')
        self.lin_vel_label.pack(anchor='w', padx=6, pady=2)

        self.ang_vel_label = ttk.Label(telem_frame, text='Angular Velocity (X, Y, Z): --')
        self.ang_vel_label.pack(anchor='w', padx=6, pady=2)

        self.status_label = ttk.Label(
            telem_frame, text='System Status: Waiting for odometry...',
            foreground='orange')
        self.status_label.pack(anchor='w', padx=6, pady=4)

        # --- Manual controls ---
        control_frame = ttk.LabelFrame(root, text='Manual Control')
        control_frame.pack(fill='both', expand=True, padx=10, pady=8)

        grid = ttk.Frame(control_frame)
        grid.pack(pady=10)

        ttk.Button(grid, text='Up', width=10,
                   command=lambda: self.send_manual('Up')).grid(row=0, column=1, pady=4)
        ttk.Button(grid, text='Left', width=10,
                   command=lambda: self.send_manual('Left')).grid(row=1, column=0, padx=4)
        ttk.Button(grid, text='Forward', width=10,
                   command=lambda: self.send_manual('Forward')).grid(row=1, column=1, pady=4)
        ttk.Button(grid, text='Right', width=10,
                   command=lambda: self.send_manual('Right')).grid(row=1, column=2, padx=4)
        ttk.Button(grid, text='Backward', width=10,
                   command=lambda: self.send_manual('Backward')).grid(row=2, column=1, pady=4)
        ttk.Button(grid, text='Down', width=10,
                   command=lambda: self.send_manual('Down')).grid(row=3, column=1, pady=4)

        stop_btn = ttk.Button(
            control_frame, text='STOP', width=15,
            command=lambda: self.send_manual('Stop'))
        stop_btn.pack(pady=10)

        # Kick off the periodic ROS2 spin + telemetry refresh loop.
        self.spin_ros()
        self.refresh_telemetry()

    def toggle_mode(self):
        if self.mode == 'manual':
            self.mode = 'voice'
            self.mode_label.config(text='Current Mode: VOICE')
            self.toggle_btn.config(text='Switch to Manual Mode')
        else:
            self.mode = 'manual'
            self.mode_label.config(text='Current Mode: MANUAL')
            self.toggle_btn.config(text='Switch to Voice Mode')
        self.ros_node.publish_mode(self.mode)
        self.ros_node.get_logger().info(f'GUI switched control mode to: {self.mode}')

    def send_manual(self, command_name: str):
        if self.mode != 'manual':
            self.status_label.config(
                text='System Status: Ignored manual input (in Voice Mode)',
                foreground='orange')
            return
        values = MANUAL_COMMANDS[command_name]
        self.ros_node.publish_twist(values)
        self.status_label.config(
            text=f'System Status: Manual command sent - {command_name}',
            foreground='green')

    def spin_ros(self):
        """Called repeatedly via .after() - processes any pending ROS2
        callbacks (like odom_callback) without blocking the GUI."""
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        self.root.after(50, self.spin_ros)  # ~20Hz

    def refresh_telemetry(self):
        """Called repeatedly via .after() - updates the displayed
        telemetry labels from the node's latest received odometry."""
        odom = self.ros_node.latest_odom
        if odom is not None:
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            lv = odom.twist.twist.linear
            av = odom.twist.twist.angular

            roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

            self.position_label.config(
                text=f'Position (X, Y, Z): {p.x:.2f}, {p.y:.2f}, {p.z:.2f} m')
            self.orientation_label.config(
                text=(f'Orientation (R, P, Y deg): '
                      f'{math.degrees(roll):.1f}, '
                      f'{math.degrees(pitch):.1f}, '
                      f'{math.degrees(yaw):.1f}'))
            self.lin_vel_label.config(
                text=f'Linear Velocity (X, Y, Z): {lv.x:.2f}, {lv.y:.2f}, {lv.z:.2f} m/s')
            self.ang_vel_label.config(
                text=f'Angular Velocity (X, Y, Z): {av.x:.2f}, {av.y:.2f}, {av.z:.2f} rad/s')

            if self.status_label.cget('text').startswith('System Status: Waiting'):
                self.status_label.config(
                    text='System Status: Receiving telemetry', foreground='green')

        self.root.after(100, self.refresh_telemetry)  # ~10Hz display refresh


def main(args=None):
    rclpy.init(args=args)
    ros_node = TelemetryGuiNode()

    root = tk.Tk()
    app = TelemetryApp(root, ros_node)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
