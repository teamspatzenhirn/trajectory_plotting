import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from spatz_interfaces.msg import ILQRTrajectory

import matplotlib.pyplot as plt
import numpy as np


def steerAnglesToTurnRate(vels, delta_fronts, delta_rears, track_length):
    tanF = np.tan(np.array(delta_fronts))
    tanR = np.tan(np.array(delta_rears))
    steerSum = tanF + tanR
    return np.array(vels) * (tanF - tanR) / np.sqrt(steerSum * steerSum + 4) / track_length


class PlottingNode(Node):

    def __init__(self):
        super().__init__('trajectory_plotting')
        self.subscription = self.create_subscription(
            ILQRTrajectory,
            '/trajectory_ilqr',
            self.listener_callback,
            10)
        self.figure, ((self.ax_yaw, self.ax_vel), (self.ax_steer, self.ax_acc), (self.acc_2d, _)) = plt.subplots(
            nrows=3, ncols=2)
        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        states = msg.states
        vels = [s.vel for s in states]
        yaws = [s.psi * 180 / math.pi for s in states]

        controls = msg.controls
        accs = [c.acc for c in controls]
        delta_fronts_rad = [c.delta_front for c in controls]
        delta_fronts = [c * 180 / math.pi for c in delta_fronts_rad]
        delta_rears_rad = [c.delta_rear for c in controls]
        delta_rears = [c * 180 / math.pi for c in delta_rears_rad]

        dt = Duration.from_msg(msg.delta_t).nanoseconds * 10 ** -9
        x = np.linspace(0, dt * len(states), num=len(states), endpoint=False)

        lateralAccs = vels * steerAnglesToTurnRate(vels, delta_fronts_rad, delta_rears_rad, 0.2565)

        self.acc_2d.clear()
        self.acc_2d.set(adjustable='box', aspect='equal')
        self.acc_2d.set_xlim([-5, 5])
        self.acc_2d.set_ylim([-5, 5])
        self.acc_2d.scatter(lateralAccs, accs)
        circle = plt.Circle((0, 0), 2, fill=False)
        self.acc_2d.add_patch(circle)
        self.acc_2d.set_title("Acc [m/s²]")
        self.acc_2d.set_ylabel("longitudinal")
        self.acc_2d.set_xlabel("lateral")

        self.ax_yaw.clear()
        self.ax_yaw.plot(x, yaws)
        self.ax_yaw.set_title("Yaw")
        self.ax_yaw.set_ylabel("deg")

        self.ax_vel.clear()
        self.ax_vel.plot(x, vels)
        self.ax_vel.set_ylim([-2, 5])
        self.ax_vel.set_title("Vel")
        self.ax_vel.set_ylabel("m/s")

        self.ax_acc.clear()
        self.ax_acc.plot(x, accs)
        self.ax_acc.set_ylim([-5, 5])
        self.ax_acc.set_title("Acc")
        self.ax_acc.set_ylabel("m/s²")

        self.ax_steer.clear()
        self.ax_steer.plot(x, delta_fronts, label="Front")
        self.ax_steer.plot(x, delta_rears, label="Rear")
        self.ax_steer.set_ylim([-30, 30])
        self.ax_steer.set_title("Steer")
        self.ax_steer.set_ylabel("deg")
        self.ax_steer.legend()

        self.get_logger().info('Updated plots')

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)

    plotting_node = PlottingNode()

    rclpy.spin(plotting_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotting_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
