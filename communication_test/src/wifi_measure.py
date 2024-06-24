#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import subprocess

from communication_test_interfaces.srv import WifiMeasure

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WifiMeasureNode(Node):

    def __init__(self):
        super().__init__('wifi_measure')


        self.declare_parameter('interface', "wlp2s0")

        # Create service server
        self.srv = self.create_service(WifiMeasure, 'wifi_measure', self.serviceHandler)

        # Create publisher
        self.measurePublisher = self.create_publisher(Marker, 'wifi_measure', 10)

        # Store values
        self.nbMeasures = 0




    def serviceHandler(self, request:WifiMeasure.Request, response:WifiMeasure.Response):
        # Take measure
        measure = self.takeMeasure()

        if measure:
            # Split measure
            quality, dBm = measure

            # Create response
            response.signal_quality = float(quality)
            response.signal_dbm     = float(dBm)

            # Publish measure
            if request.publish:
                self.publishMeasure(request.position, quality)

        
        return response


    def takeMeasure(self):
        # Get wifi measure for the current network
        proc = subprocess.Popen(["iwconfig", self.interface()],stdout=subprocess.PIPE, universal_newlines=True)
        out, err = proc.communicate()


        # Get the line that interests us
        lines = out.split("\n")
        line = None
        for l in lines:
            if "Link Quality=" in l:
                line = l.strip()
                break


        # The line is in the form : "Link Quality=59/70  Signal level=-51 dBm"
        if line is not None:
            self.nbMeasures += 1

            # Get signal quality
            quality = line.split('=')[1].split(" ")[0].split('/')
            quality = float(quality[0]) / float(quality[1]) * 100

            # Get signal strength in dBm
            dBm = int(line.split('=')[2].split(" ")[0])


            self.get_logger().debug(f"Quality : {quality:.3f}\t Strength : {dBm} dBm")


            return quality, dBm


    def publishMeasure(self, position:Point, quality:float):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        # Set shape
        marker.type = Marker.CYLINDER
        marker.id = self.nbMeasures

        # Set the scale of the marker
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.01

        # Set the color
        marker.color.r = 1.0 - quality
        marker.color.g = quality
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = position

        # Publish marker
        self.measurePublisher.publish(marker)


    def interface(self):
        return self.get_parameter('interface').get_parameter_value().string_value





def main(args=None):
    rclpy.init(args=args)
    node = WifiMeasureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()