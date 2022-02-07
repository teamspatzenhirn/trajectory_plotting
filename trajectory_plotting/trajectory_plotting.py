import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from spatz_interfaces.msg import ILQRTrajectory

import matplotlib.pyplot as plt
import numpy as np


class PlottingNode(Node):

    def __init__(self):
        super().__init__('trajectory_plotting')
        self.subscription = self.create_subscription(
            ILQRTrajectory,
            '/trajectory_ilqr',
            self.listener_callback,
            10)
        self.figure, ((self.ax_yaw, self.ax_vel), (self.ax_steer, self.ax_acc)) = plt.subplots(nrows=2, ncols=2)
        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        states = msg.states
        vels = [s.vel for s in states]
        yaws = [s.psi * 180 / math.pi for s in states]

        controls = msg.controls
        accs = [c.acc for c in controls]
        delta_fronts = [c.delta_front * 180 / math.pi for c in controls]
        delta_rears = [c.delta_rear * 180 / math.pi for c in controls]

        dt = Duration.from_msg(msg.delta_t).nanoseconds * 10 ** -9
        x = np.linspace(0, dt * len(states), num=len(states), endpoint=False)

        self.ax_yaw.clear()
        self.ax_yaw.plot(x, yaws)
        self.ax_yaw.set_title("Yaw")
        self.ax_yaw.set_ylabel("deg")

        self.ax_vel.clear()
        self.ax_vel.plot(x, vels)
        self.ax_vel.set_title("Vel")
        self.ax_vel.set_ylabel("m/s")

        self.ax_acc.clear()
        self.ax_acc.plot(x, accs)
        self.ax_acc.set_title("Acc")
        self.ax_acc.set_ylabel("m/s²")

        self.ax_steer.clear()
        self.ax_steer.plot(x, delta_fronts, label="Front")
        self.ax_steer.plot(x, delta_rears, label="Rear")
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
