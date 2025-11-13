#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from your_interfaces.msg import GpsData


class GpsSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            GpsData,
            'gps_data',
            self.gps_callback,
            10
        )
        self.get_logger().info('GPS Subscriber started')

    def gps_callback(self, msg):
        # 判斷狀態
        if msg.status == GpsData.STATUS_FIX:
            status_str = "✓ Fix"
            status_color = '\033[92m'  # Green
        else:
            status_str = "✗ No Fix"
            status_color = '\033[91m'  # Red
        
        reset_color = '\033[0m'
        
        # 顯示詳細資訊
        self.get_logger().info(
            f'{status_color}{status_str}{reset_color} | '
            f'Position: ({msg.latitude:.6f}, {msg.longitude:.6f}) | '
            f'Altitude: {msg.altitude:.2f}m | '
            f'HDOP: {msg.hdop:.1f} | '
            f'Satellites: {msg.used_sats}/{msg.visible_sats}'
        )
        
        # 如果沒有定位，顯示警告
        if msg.status == GpsData.STATUS_NO_FIX:
            self.get_logger().warn('GPS signal lost! Waiting for fix...')


def main(args=None):
    rclpy.init(args=args)
    node = GpsSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    