"""Mirror Crazyswarm2 simulation TF poses into static Gazebo models."""

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import Buffer, TransformException, TransformListener


class GazeboPoseBridge(Node):
    def __init__(self):
        super().__init__('gazebo_pose_bridge')
        self.declare_parameter('robot_names', ['cf231'])
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('world_name', 'empty')

        self.robot_names = list(
            self.get_parameter('robot_names').get_parameter_value().string_array_value)
        self.reference_frame = self.get_parameter('reference_frame').value
        world_name = self.get_parameter('world_name').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(
            SetEntityPose, f'/world/{world_name}/set_pose')
        self.pending = {name: None for name in self.robot_names}
        self.create_timer(1.0 / 30.0, self.update_poses)

    def update_poses(self):
        if not self.client.service_is_ready():
            return
        for name in self.robot_names:
            future = self.pending[name]
            if future is not None and not future.done():
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame, name, rclpy.time.Time())
            except TransformException:
                continue

            request = SetEntityPose.Request()
            request.entity = Entity(name=name, type=Entity.MODEL)
            request.pose = Pose()
            request.pose.position.x = transform.transform.translation.x
            request.pose.position.y = transform.transform.translation.y
            request.pose.position.z = transform.transform.translation.z
            request.pose.orientation = transform.transform.rotation
            self.pending[name] = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
