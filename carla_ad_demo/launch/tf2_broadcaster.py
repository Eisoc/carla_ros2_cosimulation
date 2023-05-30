#!/usr/bin/env python


import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformStamped
import tf2_ros
import geometry_msgs.msg

class TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')

        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)

    def broadcast_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ego_vehicle'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    tfb = TF2Broadcaster()

    rclpy.spin(tfb)

    tfb.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()