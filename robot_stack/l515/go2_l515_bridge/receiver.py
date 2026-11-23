"""Run on Ubuntu alongside your existing Go2 ROS2 SDK nodes (not on the Pi)."""
import argparse
import array
import math
import signal
import socket
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from .protocol import recv_frame
from .geometry import xyz_from_depth, color_pose_in_depth

DEPTH_FRAME = 'l515_depth_optical_frame'
COLOR_FRAME = 'l515_color_optical_frame'
LINK_FRAME = 'l515_link'
ACK = b'\x06'

def byte_array(a):
    # rclpy copies array('B') directly; bytes would be converted element by element (~70 ms per image).
    return array.array('B', np.ascontiguousarray(a).tobytes())

def static_transform(parent, child, xyz, quat):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = [float(v) for v in xyz]
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = [float(v) for v in quat]
    return t

def rpy_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

class Receiver(Node):
    def __init__(self, args, web=None):
        super().__init__('go2_l515_receiver')
        self.args = args
        self.web = web
        # Reliable so multi-fragment frames (depth, cloud) survive; best-effort subscribers still match.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.rgb_pub = self.create_publisher(Image, '/go2/l515/color/image_raw', qos)
        self.jpeg_pub = self.create_publisher(CompressedImage, '/go2/l515/color/image_raw/compressed', qos)
        self.depth_pub = self.create_publisher(Image, '/go2/l515/depth/image_raw', qos)
        self.meters_pub = self.create_publisher(Image, '/go2/l515/depth/image_meters', qos)
        self.dc_pub = self.create_publisher(CameraInfo, '/go2/l515/depth/camera_info', qos)
        self.cc_pub = self.create_publisher(CameraInfo, '/go2/l515/color/camera_info', qos)
        self.points_pub = self.create_publisher(PointCloud2, '/go2/l515/points', qos)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.tf_sent = False
        self.count = self.bytes = 0
        self.busy = 0.0
        self.last_stats = time.monotonic()

    def _stamp(self, frame):
        if self.args.stamp_source == 'pi':
            # Requires synchronized Pi and Go2 host clocks, e.g. chrony/NTP.
            from builtin_interfaces.msg import Time
            return Time(sec=frame.timestamp_ns // 1_000_000_000,
                        nanosec=frame.timestamp_ns % 1_000_000_000)
        return self.get_clock().now().to_msg()

    def _camera_info(self, stamp, frame_id, w, h, intr, model, coeff):
        fx, fy, cx, cy = intr
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, frame_id
        m.width, m.height = int(w), int(h)
        # L515 typically reports none for depth, Brown-Conrady for color.
        # For other SDK models, ROS plumb_bob may not be a compatible distortion model.
        m.distortion_model = 'plumb_bob'
        m.d = [float(c) for c in coeff] if model in (1, 4) else [0.0]*5
        m.k = [fx,0.0,cx, 0.0,fy,cy, 0.0,0.0,1.0]
        m.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
        m.p = [fx,0.0,cx,0.0, 0.0,fy,cy,0.0, 0.0,0.0,1.0,0.0]
        return m

    def _image(self, stamp, frame_id, array, encoding):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = array.strides[0]
        msg.data = byte_array(array)
        return msg

    def _pointcloud(self, stamp, points):
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp, DEPTH_FRAME
        msg.height, msg.width = 1, len(points)
        msg.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i,n in enumerate(('x','y','z'))]
        msg.is_bigendian = False
        msg.point_step, msg.row_step = 12, 12*len(points)
        msg.is_dense = True
        msg.data = byte_array(points)
        return msg

    def _publish_static_tf(self, frame, stamp):
        if self.tf_sent:
            return
        # ROS body convention x-forward, y-left, z-up -> optical x-right,y-down,z-forward.
        # Physical L515 mounting is user-specific; set --parent-frame and measured --mount-*.
        transforms = [static_transform(LINK_FRAME, DEPTH_FRAME, (0,0,0),
                                       rpy_quat(-math.pi/2, 0, -math.pi/2))]
        xyz, quat = color_pose_in_depth(frame.extrinsics)
        transforms.append(static_transform(DEPTH_FRAME, COLOR_FRAME, xyz, quat))
        if self.args.parent_frame:
            transforms.append(static_transform(self.args.parent_frame, LINK_FRAME,
                                               self.args.mount_xyz,
                                               rpy_quat(*[math.radians(v) for v in self.args.mount_rpy_deg])))
        for t in transforms:
            t.header.stamp = stamp
        self.tf_pub.sendTransform(transforms)
        self.tf_sent = True
        self.get_logger().info('Published static camera TF; parent=' + (self.args.parent_frame or '(not attached)'))

    def publish(self, f):
        stamp = self._stamp(f)
        self._publish_static_tf(f, stamp)
        color = cv2.imdecode(np.frombuffer(f.color_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if color is None:
            raise ValueError('JPEG decode failed')
        dh, dw = f.depth.shape
        ch, cw = color.shape[:2]
        self.rgb_pub.publish(self._image(stamp, COLOR_FRAME, color, 'bgr8'))
        jpeg = CompressedImage()
        jpeg.header.stamp, jpeg.header.frame_id = stamp, COLOR_FRAME
        jpeg.format = 'jpeg'
        jpeg.data = array.array('B', f.color_jpeg)
        self.jpeg_pub.publish(jpeg)
        self.cc_pub.publish(self._camera_info(stamp, COLOR_FRAME, cw, ch, f.color_intr, f.color_model, f.color_coeffs))
        # ROS 16UC1 convention: millimeters; 32FC1 preserves native sub-mm precision in meters.
        depth_m = f.depth.astype(np.float32) * f.depth_scale
        depth_mm = np.clip(np.rint(depth_m * 1000), 0, 65535).astype('<u2')
        self.depth_pub.publish(self._image(stamp, DEPTH_FRAME, depth_mm, '16UC1'))
        if self.args.publish_meters:
            self.meters_pub.publish(self._image(stamp, DEPTH_FRAME, depth_m, '32FC1'))
        self.dc_pub.publish(self._camera_info(stamp, DEPTH_FRAME, dw, dh, f.depth_intr, f.depth_model, f.depth_coeffs))
        points = xyz_from_depth(f.depth, f.depth_intr, f.depth_scale,
                                stride=self.args.stride, max_range=self.args.max_range)
        self.points_pub.publish(self._pointcloud(stamp, points))
        if self.web:
            self.web.update(f, color, len(points))
        self.count += 1
        self.bytes += f.nbytes
        now = time.monotonic()
        if now - self.last_stats >= 10:
            dt = now - self.last_stats
            self.get_logger().info(
                f'{self.count/dt:.1f} fps; {self.bytes/self.count/1024:.0f} KB/frame, '
                f'{self.bytes*8/dt/1e6:.1f} Mbit/s; {self.busy/self.count*1000:.0f} ms/frame here; '
                f'{len(points)} points/frame')
            self.count = self.bytes = 0
            self.busy = 0.0
            self.last_stats = now

def _raise_interrupt(*_):
    raise KeyboardInterrupt

def main():
    parser = argparse.ArgumentParser(description='Receive L515 stream and publish ROS2 Go2 topics')
    parser.add_argument('--host', default='127.0.0.1', help='Pi IP (direct) or localhost (SSH tunnel)')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--stride', type=int, default=2, help='Point cloud stride 1=dense, 2=balanced, 4=fast')
    parser.add_argument('--max-range', type=float, default=9.0)
    parser.add_argument('--stamp-source', choices=('receiver','pi'), default='receiver')
    parser.add_argument('--publish-meters', action='store_true', help='Publish 32FC1 depth meters')
    parser.add_argument('--parent-frame', default='', help='Your Go2 base frame, e.g. base_link or go2_base_link')
    parser.add_argument('--mount-xyz', type=float, nargs=3, default=[0.,0.,0.], metavar=('X','Y','Z'))
    parser.add_argument('--mount-rpy-deg', type=float, nargs=3, default=[0.,0.,0.], metavar=('ROLL','PITCH','YAW'))
    parser.add_argument('--web-port', type=int, default=0,
                        help='Serve a color/depth/point-cloud page and L515 recorder on this port (0 = off)')
    parser.add_argument('--web-bind', default='0.0.0.0')
    args = parser.parse_args()
    if args.stride < 1 or args.max_range <= 0:
        parser.error('Invalid stride/max-range')
    rclpy.init()
    signal.signal(signal.SIGTERM, _raise_interrupt)
    web = None
    if args.web_port:
        from .web import WebApp
        web = WebApp(args.web_port, args.web_bind, args.max_range)
    node = Receiver(args, web)
    if web:
        node.get_logger().info(f'L515 web page on http://{args.web_bind}:{args.web_port}')
    try:
        while rclpy.ok():
            try:
                node.get_logger().info(f'Connecting to L515 at {args.host}:{args.port}')
                with socket.create_connection((args.host, args.port), timeout=5) as sock:
                    sock.settimeout(10)
                    node.get_logger().info('Connected to Pi L515 sender')
                    while rclpy.ok():
                        frame = recv_frame(sock)
                        t0 = time.monotonic()
                        node.publish(frame)
                        node.busy += time.monotonic() - t0
                        # One ack per frame: the Pi keeps only a couple of frames in flight and
                        # drops the rest, so a slow link lowers the frame rate instead of adding delay.
                        sock.sendall(ACK)
                        rclpy.spin_once(node, timeout_sec=0)
            except (OSError, ConnectionError, ValueError) as e:
                node.get_logger().warning(f'Camera stream disconnected: {e}; retrying in 2s')
                time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        if web:
            web.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
