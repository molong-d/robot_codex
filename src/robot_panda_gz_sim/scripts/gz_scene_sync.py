#!/usr/bin/env python3
"""Publish measured Gazebo contacts and mirror collision state into MoveIt."""

import time
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from robot_interfaces.msg import GraspContact
from ros_gz_interfaces.msg import Contacts
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scene_state import SceneReconciler
from source_time import SourceTimeGuard


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
        self.declare_parameter("evidence_max_age_ms", 500)
        self.pose_topics = self.get_parameter("pose_topics").value
        self.object_id = self.get_parameter("object_id").value
        self.hand_frame = self.get_parameter("hand_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.gazebo_world_name = self.get_parameter("gazebo_world_name").value
        self.poses = {}
        self.epoch = 0
        evidence_max_age_ms = self.get_parameter("evidence_max_age_ms").value
        if evidence_max_age_ms <= 0:
            raise ValueError("evidence_max_age_ms must be positive")
        self.time_guard = SourceTimeGuard(max_age_ns=evidence_max_age_ms * 1_000_000)
        self.contact_run = None
        self.contact_count = 0
        self.contact_confirmed = None
        self.contact_sequence = 0
        self.scene = SceneReconciler(response_timeout_s=5.0)
        self.last_scene_request = 0.0
        self.feedback_enabled = self.get_parameter("publish_grasp_feedback").value

        self.contact_pub = (self.create_publisher(Bool, self.get_parameter("grasp_topic").value, 10)
                            if self.get_parameter("publish_grasp_feedback").value else None)
        self.stamped_contact_pub = (
            self.create_publisher(GraspContact, self.get_parameter("grasp_stamped_topic").value, 10)
            if self.get_parameter("publish_grasp_feedback").value else None)
        self.create_subscription(Contacts, self.get_parameter("contact_topic").value, self._on_contacts, 20)
        self.scene_ready_pub = self.create_publisher(Bool, "/panda_gz/scene_ready", 1)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self._flush_scene,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self._on_parameters)
        for topic in self.pose_topics:
            self.create_subscription(TFMessage, topic, self._on_poses, 10)

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "publish_grasp_feedback":
                self.feedback_enabled = bool(parameter.value)
        return SetParametersResult(successful=True)

    def _begin_epoch(self, epoch):
        if epoch <= self.epoch:
            return
        self.epoch = epoch
        self.poses.clear()
        self.contact_run = None
        self.contact_count = 0
        self.contact_confirmed = None
        self.scene.reset()
        if self.contact_pub is not None and self.feedback_enabled:
            self.contact_pub.publish(Bool(data=False))
        self.get_logger().warning(f"Confirmed ROS simulation clock rewind; invalidated truth to epoch {self.epoch}")

    def _on_poses(self, message):
        clock_ns = self.get_clock().now().nanoseconds
        candidates = {}
        for transform in message.transforms:
            if transform.header.frame_id not in (self.world_frame, self.gazebo_world_name):
                continue
            source_ns = _stamp_ns(transform.header.stamp)
            for name in (self.object_id, "tray"):
                score = _frame_score(transform.child_frame_id, name)
                if score and (name not in candidates or score > candidates[name][0]):
                    candidates[name] = (score, transform, source_ns)
        for name, (_, transform, source_ns) in candidates.items():
            accepted, epoch, reason = self.time_guard.observe(f"pose:{name}", source_ns, clock_ns)
            self._begin_epoch(epoch)
            if not accepted:
                if reason not in ("duplicate_or_out_of_order", "stale_source_time"):
                    self.get_logger().debug(f"discarding {name} pose sample: {reason}")
                continue
            if transform.header.frame_id != self.world_frame:
                continue
            pose = _pose_from_transform(transform.transform)
            previous = self.poses.get(name)
            moved = previous is None or any(abs(a - b) > 0.001 for a, b in zip(
                (pose.position.x, pose.position.y, pose.position.z),
                (previous[0].position.x, previous[0].position.y, previous[0].position.z)))
            self.poses[name] = (pose, source_ns, time.monotonic(), epoch, self.world_frame)
            if moved and (name == "tray" or self.contact_confirmed is not True):
                self.scene.mark_dirty()

    def _on_contacts(self, message):
        stamp = _stamp_ns(message.header.stamp)
        clock_ns = self.get_clock().now().nanoseconds
        accepted, epoch, reason = self.time_guard.observe("contact", stamp, clock_ns)
        self._begin_epoch(epoch)
        if not accepted:
            if reason not in ("duplicate_or_out_of_order", "stale_source_time"):
                self.get_logger().debug(f"discarding contact sample: {reason}")
            return

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
            self.scene.set_desired(measured)
        # Republish only on advancing Gazebo sensor time. Steady receipt age in
        # the runtime makes this evidence expire while simulation is paused.
        if self.contact_count >= 2 and self.contact_pub is not None and self.feedback_enabled:
            self.contact_pub.publish(Bool(data=measured))
            stamped = GraspContact()
            stamped.source_stamp = message.header.stamp
            stamped.frame_id = self.world_frame
            stamped.object_id = self.object_id
            stamped.epoch = self.epoch
            self.contact_sequence += 1
            stamped.sequence = self.contact_sequence
            stamped.detected = measured
            self.stamped_contact_pub.publish(stamped)

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

    def _planning_scene(self, request):
        if request.cleanup:
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            attached = AttachedCollisionObject()
            attached.object.id = self.object_id
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)
            removed = CollisionObject()
            removed.id = self.object_id
            removed.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(removed)
            return scene
        now = time.monotonic()
        if any(name not in self.poses or now - self.poses[name][2] > 0.5 or
               self.poses[name][3] != self.epoch or self.poses[name][4] != self.world_frame
               for name in (self.object_id, "tray")):
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
        if request.attached:
            try:
                hand_transform = self.tf_buffer.lookup_transform(self.world_frame, self.hand_frame, Time())
            except Exception:
                self.scene.mark_dirty()
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
            # REMOVE is idempotent and repairs state after a reset or a late
            # attach response even when our prior local confirmation was lost.
            attached = AttachedCollisionObject()
            attached.object.id = self.object_id
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)
            scene.world.collision_objects.append(self._world_collision(self.object_id, [box]))
        return scene

    def _flush_scene(self):
        now = time.monotonic()
        if self.scene.check_timeout(now):
            self.get_logger().error("MoveIt scene response exceeded 5.0 s; scene remains unsynchronized")
        fresh_inputs = self.scene.cleanup_pending or all(
            name in self.poses and now - self.poses[name][2] <= 0.5 and
            self.poses[name][3] == self.epoch and self.poses[name][4] == self.world_frame
            for name in (self.object_id, "tray"))
        request = None
        if self.scene_client.service_is_ready() and now - self.last_scene_request >= 0.1:
            request = self.scene.submit(now, fresh_inputs)
        if request is None:
            self.scene_ready_pub.publish(Bool(data=bool(self.scene.ready and fresh_inputs)))
            return
        scene = self._planning_scene(request)
        if scene is None:
            self.scene.complete(request, False)
            self.scene_ready_pub.publish(Bool(data=False))
            return
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.last_scene_request = time.monotonic()
        submitted = self.scene.in_flight
        try:
            future = self.scene_client.call_async(request)
        except Exception as error:
            self.scene.complete(submitted, False)
            self.get_logger().error(f"MoveIt scene request could not be sent: {error}")
            self.scene_ready_pub.publish(Bool(data=False))
            return

        def complete(done):
            try:
                result = done.result()
                if not self.scene.complete(submitted, result.success):
                    self.get_logger().warning("Ignoring duplicate or stale MoveIt scene callback")
                if not result.success:
                    self.get_logger().error("MoveIt rejected the Gazebo collision scene update")
            except Exception as error:  # service shutdown or transport failure
                self.scene.complete(submitted, False)
                self.get_logger().error(f"MoveIt scene update failed: {error}")
            self.scene_ready_pub.publish(Bool(data=bool(self.scene.ready and not self.scene.cleanup_pending)))

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
