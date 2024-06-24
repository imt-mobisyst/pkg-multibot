#!/usr/bin/python3
import rclpy


from include.warehouse_controller import WarehouseRobotController
from include.kobuki_controller import KobukiController

from include.package import Package

class KobukiWarehouseController(KobukiController, WarehouseRobotController):

    def __init__(self):
        super().__init__('kobuki_warehouse_controller')

        # Set package positions
        Package.setSimulation(False)




def main(args=None):
    rclpy.init(args=args)
    node = KobukiWarehouseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()