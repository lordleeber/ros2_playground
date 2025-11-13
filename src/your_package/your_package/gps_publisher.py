#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from your_interfaces.msg import GpsData
from std_msgs.msg import Header


class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(GpsData, 'gps_data', 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        self.counter = 0
        self.get_logger().info('GPS Publisher started')

    def publish_gps_data(self):
        msg = GpsData()
        
        # 設定 header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'
        
        # 模擬 GPS 狀態 (每 5 次切換一次)
        if self.counter % 5 == 0:
            msg.status = GpsData.STATUS_NO_FIX
        else:
            msg.status = GpsData.STATUS_FIX
        
        # 模擬台北位置，並稍微變化
        msg.latitude = 25.0330 + (self.counter * 0.0001)
        msg.longitude = 121.5654 + (self.counter * 0.0001)
        msg.altitude = 10.0 + (self.counter * 0.1)
        
        # 精度與衛星資訊
        if msg.status == GpsData.STATUS_FIX:
            msg.hdop = 1.2
            msg.used_sats = 8
            msg.visible_sats = 12
        else:
            msg.hdop = 99.9
            msg.used_sats = 0
            msg.visible_sats = 5
        
        self.publisher_.publish(msg)
        
        status_str = "Fix" if msg.status == GpsData.STATUS_FIX else "No Fix"
        self.get_logger().info(
            f'Publishing GPS [{status_str}]: '
            f'Lat={msg.latitude:.4f}, Lon={msg.longitude:.4f}, '
            f'Alt={msg.altitude:.1f}m, Sats={msg.used_sats}/{msg.visible_sats}'
        )
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    