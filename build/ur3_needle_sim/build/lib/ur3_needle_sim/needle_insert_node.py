"""ROS state machine: /needle/hole_id + /needle/start → insert into that hole (IK)."""
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool, Float32, Int32, String

from ur3_needle_sim.hole_poses_loader import default_path, load_hole_poses

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class Phase(Enum):
    IDLE = auto()
    MOVE_HOME = auto()
    MOVE_APPROACH = auto()
    INSERT = auto()
    HOLD = auto()
    RETRACT = auto()
    DONE = auto()
    ABORTED = auto()


class NeedleInsertNode(Node):
    def __init__(self):
        super().__init__('needle_insert_node')

        self.declare_parameter('insertion.depth_m', 0.025)
        self.declare_parameter('insertion.hold_sec', 1.5)
        self.declare_parameter('poses.home', [0.0, -1.570, 1.570, -1.570, -1.570, 0.0])
        self.declare_parameter('trajectory.home_sec', 3.0)
        self.declare_parameter('trajectory.approach_sec', 4.0)
        self.declare_parameter('trajectory.insert_sec', 3.0)
        self.declare_parameter('trajectory.retract_sec', 3.0)
        self.declare_parameter('hole_poses_file', '')
        self.declare_parameter(
            'action_name',
            '/ur3/ur_joint_trajectory_controller/follow_joint_trajectory',
        )

        self.depth_m = float(self.get_parameter('insertion.depth_m').value)
        self.hold_sec = float(self.get_parameter('insertion.hold_sec').value)
        self.t_home = float(self.get_parameter('trajectory.home_sec').value)
        self.t_approach = float(self.get_parameter('trajectory.approach_sec').value)
        self.t_insert = float(self.get_parameter('trajectory.insert_sec').value)
        self.t_retract = float(self.get_parameter('trajectory.retract_sec').value)
        self.action_name = self.get_parameter('action_name').value

        poses_file = self.get_parameter('hole_poses_file').value
        if not poses_file:
            poses_file = default_path()
        baked = load_hole_poses(poses_file)
        self.home = baked.get('home') or [float(x) for x in self.get_parameter('poses.home').value]
        self.holes = {}
        self.approach_qs = {}
        self.insert_qs = {}
        self.z_paths = {}
        n_z = 0
        for hid, row in baked['holes'].items():
            self.holes[hid] = tuple(row['xyz'])
            self.approach_qs[hid] = list(row['approach'])
            self.insert_qs[hid] = list(row['insert'])
            z_path = row.get('z_path') or []
            if len(z_path) >= 2:
                self.z_paths[hid] = [list(q) for q in z_path]
                n_z += 1
            else:
                # fallback: 2-point path (not Cartesian-straight)
                self.z_paths[hid] = [
                    list(row['approach']),
                    list(row['insert']),
                ]
        self.get_logger().info(
            'Loaded baked hole poses from %s (%d holes, %d with Z-path)'
            % (poses_file, len(self.holes), n_z)
        )

        self.hole_id = 4 if 4 in self.holes else (next(iter(self.holes)) if self.holes else 0)
        self.phase = Phase.IDLE
        self.returning_home = False
        self.goal_handle = None
        self.hold_until = None
        self.insert_started_at = None

        self.phase_pub = self.create_publisher(String, '/needle/phase', 10)
        self.depth_cmd_pub = self.create_publisher(Float32, '/needle/depth_command', 10)
        self.create_subscription(Bool, '/needle/start', self.on_start, 10)
        self.create_subscription(Bool, '/needle/abort', self.on_abort, 10)
        self.create_subscription(Int32, '/needle/hole_id', self.on_hole_id, 10)
        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.create_timer(0.1, self.on_timer)

        self.set_phase(Phase.IDLE)
        self.get_logger().info(
            'needle_insert_node ready. Set /needle/hole_id then pub /needle/start'
        )

    def set_phase(self, phase: Phase):
        self.phase = phase
        msg = String()
        msg.data = phase.name.lower()
        self.phase_pub.publish(msg)
        self.get_logger().info('phase -> %s (hole=%s)' % (msg.data, self.hole_id))

    def on_hole_id(self, msg: Int32):
        hid = int(msg.data)
        if hid in self.holes:
            self.hole_id = hid
            h = self.holes[self.hole_id]
            self.get_logger().info('hole_id set to %d world=(%.3f, %.3f, %.3f)' % (
                self.hole_id, h[0], h[1], h[2]))
        else:
            self.get_logger().warn('invalid/missing hole_id %d' % msg.data)

    def on_start(self, msg: Bool):
        if not msg.data:
            return
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.ABORTED):
            self.get_logger().warn('busy in phase %s' % self.phase.name)
            return
        if self.hole_id not in self.approach_qs or self.hole_id not in self.insert_qs:
            self.get_logger().error('no baked pose for hole_id=%d' % self.hole_id)
            return
        self.returning_home = False
        self.set_phase(Phase.MOVE_HOME)
        self.send_joints(self.home, self.t_home)

    def on_abort(self, msg: Bool):
        if not msg.data:
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.hold_until = None
        self.insert_started_at = None
        self.set_phase(Phase.ABORTED)

    def send_joints(self, positions, duration_sec: float):
        self.send_joint_path([positions], duration_sec)

    def send_joint_path(self, path, duration_sec: float):
        """Multi-point trajectory (straight-Z insert uses many IK waypoints)."""
        if not path:
            self.get_logger().error('empty joint path')
            self.set_phase(Phase.ABORTED)
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('trajectory action server not available')
            self.set_phase(Phase.ABORTED)
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        n = len(path)
        duration_sec = max(float(duration_sec), 0.2 * n)
        points = []
        for i, positions in enumerate(path):
            t = duration_sec if n == 1 else duration_sec * (i + 1) / float(n)
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in positions]
            whole = int(t)
            nanos = int((t - whole) * 1e9)
            point.time_from_start = Duration(sec=whole, nanosec=nanos)
            points.append(point)
        goal.trajectory.points = points
        self.get_logger().info('trajectory %d pts over %.2fs' % (n, duration_sec))
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.set_phase(Phase.ABORTED)
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        self.goal_handle = None
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            if self.phase != Phase.ABORTED:
                self.set_phase(Phase.ABORTED)
            return
        self.advance()

    def advance(self):
        if self.phase == Phase.MOVE_HOME:
            if self.returning_home:
                self.returning_home = False
                self.depth_cmd_pub.publish(Float32(data=0.0))
                self.set_phase(Phase.DONE)
                return
            self.set_phase(Phase.MOVE_APPROACH)
            self.send_joints(self.approach_qs[self.hole_id], self.t_approach)
            return

        if self.phase == Phase.MOVE_APPROACH:
            self.set_phase(Phase.INSERT)
            self.insert_started_at = self.get_clock().now()
            self.depth_cmd_pub.publish(Float32(data=float(self.depth_m)))
            # Straight-Z path (fixed XY IK waypoints), not single joint leap
            z_path = self.z_paths.get(self.hole_id) or [self.insert_qs[self.hole_id]]
            self.send_joint_path(z_path, self.t_insert)
            return

        if self.phase == Phase.INSERT:
            self.set_phase(Phase.HOLD)
            self.hold_until = self.get_clock().now() + RclDuration(seconds=self.hold_sec)
            return

        if self.phase == Phase.RETRACT:
            self.returning_home = True
            self.set_phase(Phase.MOVE_HOME)
            self.send_joints(self.home, self.t_home)
            return

    def on_timer(self):
        if self.phase == Phase.HOLD and self.hold_until is not None:
            if self.get_clock().now() >= self.hold_until:
                self.hold_until = None
                self.set_phase(Phase.RETRACT)
                self.depth_cmd_pub.publish(Float32(data=0.0))
                z_path = self.z_paths.get(self.hole_id) or [self.approach_qs[self.hole_id]]
                # Retract = reverse straight-Z path
                self.send_joint_path(list(reversed(z_path)), self.t_retract)

        if self.phase == Phase.INSERT and self.insert_started_at is not None:
            elapsed = (self.get_clock().now() - self.insert_started_at).nanoseconds / 1e9
            ratio = max(0.0, min(1.0, elapsed / max(self.t_insert, 1e-3)))
            self.depth_cmd_pub.publish(Float32(data=float(ratio * self.depth_m)))

        # keep phase heartbeat
        msg = String()
        msg.data = self.phase.name.lower()
        self.phase_pub.publish(msg)


def main():
    rclpy.init()
    node = NeedleInsertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
