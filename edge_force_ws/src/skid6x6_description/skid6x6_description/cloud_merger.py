#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import message_filters
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs_py import point_cloud2 as pc2  # ✅ correct import


class CloudMerger(Node):
    def __init__(self):
        super().__init__('cloud_merger_node')

        self.declare_parameter('cloud1_topic', '/scan/front')
        self.declare_parameter('cloud2_topic', '/scan/rear')
        self.declare_parameter('output_topic', '/merged_points')
        self.declare_parameter('target_frame', 'base_link')

        cloud1_topic = self.get_parameter('cloud1_topic').get_parameter_value().string_value
        cloud2_topic = self.get_parameter('cloud2_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        qos_profile = rclpy.qos.qos_profile_sensor_data

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ✅ Store the last received messages
        self.last_cloud1 = None
        self.last_cloud2 = None

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        # --- inside __init__ ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # ✅ match LiDAR QoS
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers
        sub1 = message_filters.Subscriber(self, PointCloud2, cloud1_topic, qos_profile=qos_profile)
        sub2 = message_filters.Subscriber(self, PointCloud2, cloud2_topic, qos_profile=qos_profile)

        # ✅ Increase queue size and slop to allow more timing flexibility
        sync = message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=30, slop=0.3)
        sync.registerCallback(self.merge_callback)

        # Publisher
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile)
        self.get_logger().info("✅ Cloud Merger Node Started")

    def merge_callback(self, cloud1, cloud2):
        try:
            transform1 = self.tf_buffer.lookup_transform(
                self.target_frame, cloud1.header.frame_id, rclpy.time.Time()
            )
            transform2 = self.tf_buffer.lookup_transform(
                self.target_frame, cloud2.header.frame_id, rclpy.time.Time()
            )

            cloud1_tf = do_transform_cloud(cloud1, transform1)
            cloud2_tf = do_transform_cloud(cloud2, transform2)

            points1 = list(pc2.read_points(cloud1_tf, field_names=("x", "y", "z"), skip_nans=True))
            points2 = list(pc2.read_points(cloud2_tf, field_names=("x", "y", "z"), skip_nans=True))
            merged_points = points1 + points2

            # ✅ Create fields from sensor_msgs.msg.PointField
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            merged_msg = pc2.create_cloud(cloud1.header, fields, merged_points)
            merged_msg.header.stamp = self.get_clock().now().to_msg()
            merged_msg.header.frame_id = self.target_frame

            self.publisher.publish(merged_msg)

        except Exception as e:
            self.get_logger().warn(f"⚠️ Merge failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
