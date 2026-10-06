import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class DepthMonitor(Node):
    def __init__(self):
        super().__init__('depth_monitor')
        self.declare_parameter('insertion.depth_m', 0.025)
        self.max_depth = float(self.get_parameter('insertion.depth_m').value)
        self.phase = 'idle'
        self.cmd = 0.0
        self.pub = self.create_publisher(Float32, '/needle/depth', 10)
        self.create_subscription(String, '/needle/phase', self.on_phase, 10)
        self.create_subscription(Float32, '/needle/depth_command', self.on_cmd, 10)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info('depth_monitor ready (max_depth=%.3f)' % self.max_depth)

    def on_phase(self, msg: String):
        self.phase = msg.data

    def on_cmd(self, msg: Float32):
        self.cmd = float(msg.data)

    def on_timer(self):
        depth = 0.0
        if self.phase in ('insert', 'hold'):
            depth = max(0.0, min(self.cmd, self.max_depth))
        elif self.phase == 'retract':
            depth = max(0.0, self.cmd)
        out = Float32()
        out.data = float(depth)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
