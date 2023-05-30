#!/usr/bin/env python


import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformStamped
import geometry_msgs.msg

class TF2Broadcaster(Node):
    def __init__(self):
        super().__init__('tf2_broadcaster')

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)

        # 初始化变量用于保存 ego_vehicle 的位置信息
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_z = 0.0

    def update_vehicle_position(self):
        # 根据实际情况更新 ego_vehicle 的位置信息
        # 示例中假设 ego_vehicle 在 x 方向上以固定速度移动
        self.vehicle_x += 0.1  # 假设每次定时器触发时 ego_vehicle 在 x 方向上移动 0.1

    def broadcast_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ego_vehicle'
        t.child_frame_id = 'base_link'

        # 更新 ego_vehicle 的位置信息
        self.update_vehicle_position()

        t.transform.translation.x = self.vehicle_x
        t.transform.translation.y = self.vehicle_y
        t.transform.translation.z = self.vehicle_z

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