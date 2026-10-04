#!/usr/bin/env python3
"""Publish measured Gazebo contacts and mirror collision state into MoveIt."""

import time

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.time import Time
from robot_interfaces.msg import GraspContact
from ros_gz_interfaces.msg import Contacts
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _fresh_source_stamp(source_ns, clock_ns, max_age_ns=500_000_000):
    return 0 < source_ns <= clock_ns and clock_ns - source_ns <= max_age_ns


def _frame_score(child, name):
    if child == name:
        return 3
    if child.endswith("::" + name):
        return 2
    if child.startswith(name + "::"):
        return 1
    return 0


def _pose_from_transform(transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation = transform.rotation
    return pose


def _inverse_quaternion(q):
    norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if norm < 1.0e-12:
        raise ValueError("invalid measured quaternion")
    return (-q.x / norm, -q.y / norm, -q.z / norm, q.w / norm)


def _rotate(q, point):
    # q * (point, 0) * inverse(q), expanded without temporary message types.
    x, y, z, w = q.x, q.y, q.z, q.w
    px, py, pz = point
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (px + w * tx + y * tz - z * ty,
            py + w * ty + z * tx - x * tz,
            pz + w * tz + x * ty - y * tx)


def _relative_pose(parent, child):
    inv = _inverse_quaternion(parent.orientation)
    delta = (child.position.x - parent.position.x,
             child.position.y - parent.position.y,
             child.position.z - parent.position.z)
    local = _rotate(Quaternion(x=inv[0], y=inv[1], z=inv[2], w=inv[3]), delta)
    result = Pose()
    result.position.x, result.position.y, result.position.z = local
    # inverse(parent rotation) * child rotation
    ax, ay, az, aw = inv
    bx, by, bz, bw = child.orientation.x, child.orientation.y, child.orientation.z, child.orientation.w
    result.orientation.x = aw * bx + ax * bw + ay * bz - az * by
    result.orientation.y = aw * by - ax * bz + ay * bw + az * bx
    result.orientation.z = aw * bz + ax * by - ay * bx + az * bw
    result.orientation.w = aw * bw - ax * bx - ay * by - az * bz
    return result


def _box(size, pose):
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)
    return primitive, pose


class GazeboSceneSync(Node):
    def __init__(self):
        super().__init__("gazebo_scene_sync")
        self.declare_parameter("pose_topics", ["/panda_gz/workpiece/pose", "/panda_gz/tray/pose"])
        self.declare_parameter("contact_topic", "/world/panda_pick_place/model/workpiece/link/link/sensor/workpiece_contact/contact")
        self.declare_parameter("grasp_topic", "/panda/grasp_contact")
        self.declare_parameter("grasp_stamped_topic", "/panda/grasp_contact_stamped")
        self.declare_parameter("publish_grasp_feedback", True)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("gazebo_world_name", "panda_pick_place")
        self.declare_parameter("object_id", "workpiece")
        self.declare_parameter("hand_frame", "panda_hand")
        self.pose_topics = self.get_parameter("pose_topics").value
        self.object_id = self.get_parameter("object_id").value
        self.hand_frame = self.get_parameter("hand_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.gazebo_world_name = self.get_parameter("gazebo_world_name").value
        self.poses = {}
        self.epoch = 0
        self.pose_source_ns = {}
        self.reset_reference = {}
        self.awaiting_reset_epoch = set()
        self.last_contact_stamp_ns = 0
        self.contact_run = None
        self.contact_count = 0
        self.contact_confirmed = False
        self.last_grasp_publish = None
        self.attached = False
        self.scene_dirty = True
        self.scene_in_flight = False
        self.last_scene_request = 0.0

        self.contact_pub = (self.create_publisher(Bool, self.get_parameter("grasp_topic").value, 10)
                            if self.get_parameter("publish_grasp_feedback").value else None)
        self.stamped_contact_pub = (
            self.create_publisher(GraspContact, self.get_parameter("grasp_stamped_topic").value, 10)
            if self.get_parameter("publish_grasp_feedback").value else None)
        self.create_subscription(Contacts, self.get_parameter("contact_topic").value, self._on_contacts, 20)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self._flush_scene)
        for topic in self.pose_topics:
            self.create_subscription(TFMessage, topic, self._on_poses, 10)

    def _on_poses(self, message):
        candidates = {}
        clock_ns = self.get_clock().now().nanoseconds
        for transform in message.transforms:
            if transform.header.frame_id not in (self.world_frame, self.gazebo_world_name):
                continue
            source_ns = _stamp_ns(transform.header.stamp)
            if not _fresh_source_stamp(source_ns, clock_ns):
                continue
            for name in (self.object_id, "tray"):
                score = _frame_score(transform.child_frame_id, name)
                if score and (name not in candidates or score > candidates[name][0]):
                    candidates[name] = (score, transform, source_ns)
        if not candidates:
            return

        rewound = any(name in self.pose_source_ns and stamp + 100_000_000 < self.pose_source_ns[name]
                      for name, (_, _, stamp) in candidates.items())
        if rewound:
            self.epoch += 1
            self.reset_reference = dict(self.pose_source_ns)
            self.awaiting_reset_epoch = set(self.reset_reference)
            self.pose_source_ns.clear()
            self.poses.clear()
            self.attached = False
            self.contact_confirmed = False
            self.contact_count = 0
            self.contact_run = None
            if self.contact_pub is not None:
                self.contact_pub.publish(Bool(data=False))
            self.scene_dirty = True
            self.get_logger().warning(f"Gazebo time moved backwards; reset truth cache to epoch {self.epoch}")
        for name, (_, transform, stamp) in candidates.items():
            if stamp <= 0:
                continue
            if name in self.awaiting_reset_epoch:
                if stamp + 100_000_000 >= self.reset_reference[name]:
                    continue
                self.awaiting_reset_epoch.remove(name)
            if stamp <= self.pose_source_ns.get(name, 0):
                continue
            self.pose_source_ns[name] = stamp
            if transform.header.frame_id == self.world_frame:
                self.poses[name] = (_pose_from_transform(transform.transform), stamp, time.monotonic())
                self.scene_dirty = True

    def _on_contacts(self, message):
        stamp = _stamp_ns(message.header.stamp)
        clock_ns = self.get_clock().now().nanoseconds
        if not _fresh_source_stamp(stamp, clock_ns) or stamp == self.last_contact_stamp_ns:
            return
        if self.last_contact_stamp_ns and stamp < self.last_contact_stamp_ns:
            self.contact_run = None
            self.contact_count = 0
            self.contact_confirmed = False
            self.last_grasp_publish = None
            self.last_contact_stamp_ns = 0
            if self.contact_pub is not None:
                self.contact_pub.publish(Bool(data=False))
            self.get_logger().warning("Gazebo time moved backwards; discarded cached contact evidence")
        self.last_contact_stamp_ns = stamp

        left = right = False
        for contact in message.contacts:
            names = (contact.collision1.name.lower(), contact.collision2.name.lower())
            if not any(self.object_id in name for name in names):
                continue
            if any("leftfinger" in name for name in names):
                left = True
            if any("rightfinger" in name for name in names):
                right = True
        measured = left and right
        if measured == self.contact_run:
            self.contact_count += 1
        else:
            self.contact_run = measured
            self.contact_count = 1
        if self.contact_count >= 2 and self.contact_confirmed != measured:
            self.contact_confirmed = measured
            self.scene_dirty = True
        # Republish only on advancing Gazebo sensor time. Steady receipt age in
        # the runtime makes this evidence expire while simulation is paused.
        if self.contact_count >= 2 and self.contact_pub is not None:
            self.contact_pub.publish(Bool(data=measured))
            stamped = GraspContact()
            stamped.source_stamp = message.header.stamp
            stamped.detected = measured
            self.stamped_contact_pub.publish(stamped)
            self.last_grasp_publish = measured

    def _world_collision(self, object_id, shapes):
        message = CollisionObject()
        message.header.frame_id = self.world_frame
        message.id = object_id
        for dimensions, pose in shapes:
            primitive, primitive_pose = _box(dimensions, pose)
            message.primitives.append(primitive)
            message.primitive_poses.append(primitive_pose)
        message.operation = CollisionObject.ADD
        return message

    def _fixed_pose(self, x, y, z):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        return pose

    def _planning_scene(self):
        if self.object_id not in self.poses or "tray" not in self.poses:
            return None
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        table_pose = self._fixed_pose(0.58, 0.0, 0.325)
        scene.world.collision_objects.append(self._world_collision("sim_table", [((0.90, 0.80, 0.05), table_pose)]))

        tray_center, _, _ = self.poses["tray"]
        tray_shapes = []
        tray_shapes.append(((0.10, 0.10, 0.001), self._fixed_pose(
            tray_center.position.x, tray_center.position.y, tray_center.position.z - 0.0195)))
        wall_z = tray_center.position.z - 0.0095
        tray_shapes.extend([
            ((0.10, 0.002, 0.02), self._fixed_pose(tray_center.position.x, tray_center.position.y + 0.049, wall_z)),
            ((0.10, 0.002, 0.02), self._fixed_pose(tray_center.position.x, tray_center.position.y - 0.049, wall_z)),
            ((0.002, 0.10, 0.02), self._fixed_pose(tray_center.position.x + 0.049, tray_center.position.y, wall_z)),
            ((0.002, 0.10, 0.02), self._fixed_pose(tray_center.position.x - 0.049, tray_center.position.y, wall_z)),
        ])
        scene.world.collision_objects.append(self._world_collision("sim_tray", tray_shapes))

        object_pose, _, _ = self.poses[self.object_id]
        box = ((0.04, 0.04, 0.04), object_pose)
        if self.contact_confirmed:
            try:
                hand_transform = self.tf_buffer.lookup_transform(self.world_frame, self.hand_frame, Time())
            except Exception:
                self.scene_dirty = True
                return None
            hand_pose = _pose_from_transform(hand_transform.transform)
            attached = AttachedCollisionObject()
            attached.link_name = self.hand_frame
            attached.object.header.frame_id = self.hand_frame
            attached.object.id = self.object_id
            primitive, pose = _box(box[0], _relative_pose(hand_pose, object_pose))
            attached.object.primitives = [primitive]
            attached.object.primitive_poses = [pose]
            attached.object.operation = CollisionObject.ADD
            attached.touch_links = [self.hand_frame, "panda_leftfinger", "panda_rightfinger"]
            detach = CollisionObject()
            detach.id = self.object_id
            detach.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(detach)
            scene.robot_state.attached_collision_objects.append(attached)
        else:
            if self.attached:
                attached = AttachedCollisionObject()
                attached.object.id = self.object_id
                attached.object.operation = CollisionObject.REMOVE
                scene.robot_state.attached_collision_objects.append(attached)
            scene.world.collision_objects.append(self._world_collision(self.object_id, [box]))
        return scene

    def _flush_scene(self):
        if self.scene_in_flight or not self.scene_dirty or not self.scene_client.service_is_ready():
            return
        if time.monotonic() - self.last_scene_request < 0.1:
            return
        scene = self._planning_scene()
        if scene is None:
            return
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.scene_in_flight = True
        self.scene_dirty = False
        self.last_scene_request = time.monotonic()
        future = self.scene_client.call_async(request)

        def complete(done):
            self.scene_in_flight = False
            try:
                result = done.result()
                if not result.success:
                    self.scene_dirty = True
                    self.get_logger().error("MoveIt rejected the Gazebo collision scene update")
                    return
                self.attached = self.contact_confirmed
            except Exception as error:  # service shutdown or transport failure
                self.scene_dirty = True
                self.get_logger().error(f"MoveIt scene update failed: {error}")

        future.add_done_callback(complete)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSceneSync()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
