#!/usr/bin/env python3

# Copyright (C) - All Rights Reserved
#
# Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class ListenerNode(Node):
    def __init__(self):
        super().__init__('tf2_listener_node')

        # Get the topic name parameter value
        self.target_frame = self.declare_parameter('target_frame', 'target_frame').value
        self.source_frame = self.declare_parameter('source_frame', 'source_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_publisher = self.create_publisher(TFMessage, 'tf2_transforms', 10)

        self.timer = self.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        msg = TFMessage()
        transforms = []

        try:
            # Get transforms from tf buffer
            transform = self.tf_buffer.lookup_transform('target_frame', 'source_frame', self.get_clock().now())
            transforms.append(transform)
        except Exception as ex:
            self.get_logger().error('Failed to lookup transform: {}'.format(str(ex)))
            return

        # Add transforms to the message
        for transform in transforms:
            msg.transforms.append(transform)

        # Publish the transforms
        self.tf_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

