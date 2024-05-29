import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
import numpy as np
from collections import deque

class LaserFuser(Node):
  """
  this node takes in the depth image from realsense and laser scan data from 2d lidar
  and fuses them into a single laser scan message by adding flattened points from the depth image
  """
  def __init__(self, buffer_size=5):
    super().__init__('laser_fuser')

    self.K = None
    self.depth_image_msgs = deque(maxlen=buffer_size)

    self.depth_image_sub = self.create_subscription(
      Image,
      '/camera/aligned_depth_to_color/image_raw',
      self.depth_image_callback,
      10
    )

    self.K_sub = self.create_subscription(
      CameraInfo,
      '/camera/aligned_depth_to_color/camera_info',
      self.K_callback,
      10
    )
    
    self.laser_scan_sub = self.create_subscription(
      LaserScan,
      '/scan',
      self.laser_scan_callback,
      10
    )

    self.fused_scan_pub = self.create_publisher(
      LaserScan,
      '/fused_scan',
      10
    )

  def laser_scan_callback(self, msg):
    """ simply save the message """
    if self.K is None or len(self.depth_image_msgs) == 0:
      return
    depth_image = None
    closest_time = np.inf
    for depth_image_msg in self.depth_image_msgs:
      depth_time = depth_image_msg.header.stamp.sec + depth_image_msg.header.stamp.nanosec * 1e-9
      laser_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
      if abs(depth_time - laser_time) < closest_time:
        closest_time = abs(depth_time - laser_time)
        depth_image = np.frombuffer(depth_image_msg.data, dtype=np.uint16).reshape(
          depth_image_msg.height, depth_image_msg.width
        ).astype(np.float32) / 1000.0  # realsense default unit is mm
      else:
        break
    self.process_depth_image(msg, depth_image, self.K)
    # the laser scan message has been modified in place
    self.fused_scan_pub.publish(msg)

  def rad_to_pixel(self, rad, f, c):
    """
    Get the pixel where a ray shooting from the camera center with a certain degree will intersect with
    
    Args:
      rad: rad of the ray, 0 is the center, positive is right, negative is left
      f: focal length of the camera in pixel
      c: center of the camera in pixel
    """
    return np.round(f * np.tan(rad) + c).astype(np.int32)

  def process_depth_image(self, laser_scan, depth_image, K, fov_x=1.22, obs_y=(0.0, 0.32), valid_range=(0.3, 6.0), laser_rotation=3.14):
    """
    Process the depth image and fuse with laser_scan by modifying the it in place
    
    Args:
      laser_scan: the laser scan message from the liadr
      depth_image: the depth image from the camera
      K: the camera intrinsics
      fov_x: the field of view of the camera x axis in rad
      # obs_min_y: the minimum y value to consider as obstacle (in camera frame. up is negative, down is positive)
      # obs_max_y: the maximum y value to consider as obstacle
      obs_y: the range of y values to consider as obstacle (in camera frame. up is negative, down is positive)
      valid_range: the valid range of the depth image
      laser_rotation: the rotation of the laser w.r.t. the base frame (used to align the laser scan with camera scan)
    """
    fx = K[0, 0]
    cx = K[0, 2]

    rads = np.arange(-fov_x/2, fov_x/2, laser_scan.angle_increment)
    xs = self.rad_to_pixel(rads, fx, cx)
    # first, set all pixels that are invalid to have really large depth
    depth_image[depth_image < valid_range[0]] = 100
    depth_image[depth_image > valid_range[1]] = 100
    scan_data = np.ones_like(rads) * 100
    for i, x in enumerate(xs):
      # find the points for this x and all y
      arr = np.array([x*np.ones(depth_image.shape[0]), np.arange(depth_image.shape[0]), np.ones(depth_image.shape[0])])
      pts = np.linalg.inv(K) @ arr * depth_image[:, x]
      # cut off the points that are too low or too high
      pts = pts[:, (pts[1] > obs_y[0]) & (pts[1] < obs_y[1])]
      # add the points to the point cloud
      if pts.shape[1] != 0:
        # need to flip this because we want the data to go counter clockwise
        scan_data[len(xs)-1-i] = np.min(np.linalg.norm(pts, axis=0))
    
    angle_offset = -fov_x/2 - laser_scan.angle_min + laser_rotation
    offset = int(angle_offset / laser_scan.angle_increment)
    arr_len = len(laser_scan.ranges)
    for i, d in enumerate(scan_data):
      laser_scan.ranges[(i + offset) % arr_len] = min(d, laser_scan.ranges[(i + offset) % arr_len])

  def depth_image_callback(self, msg):
    self.depth_image_msgs.append(msg)

  def K_callback(self, msg):
    # Process camera info data here
    K = np.array(msg.k).reshape(3, 3).astype(np.float32)
    self.K = K

def main(args=None):
  rclpy.init(args=args)
  node = LaserFuser()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()