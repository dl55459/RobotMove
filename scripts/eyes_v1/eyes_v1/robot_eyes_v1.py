import rclpy
from rclpy.node import Node
from rclpy.time import Time
import rclpy.duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2 # type: ignore
import numpy as np

from std_msgs.msg import Header
from yolo_msgs.msg import DetectionArray, Detection
from eyes_msgs.msg import DetectedEntity, DetectedEntityArray
from geometry_msgs.msg import Point

checkerboard_dimensions = (9, 7)

def generate_world_points_R(checkerboard_dim=checkerboard_dimensions, square_size=0.0195):
    objp = np.zeros((checkerboard_dim[0]*checkerboard_dim[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard_dim[0],0:checkerboard_dim[1]].T.reshape(-1,2)
    objp *= square_size

    ## transform points to {R} frame
    T_R_checkerboard = np.eye(4)
    T_R_checkerboard[0,0] = 1
    T_R_checkerboard[1,1] = -1
    T_R_checkerboard[2,2] = -1
    T_R_checkerboard[0,3] = 0.0423
    T_R_checkerboard[1,3] = 0.1016
    T_R_checkerboard[2,3] = 0

    objp_h = np.hstack((objp, np.ones((objp.shape[0], 1))))
    objp_t = T_R_checkerboard.dot(objp_h.T)
    objp = objp_t[:3, :]
    objp = objp.T

    return objp

def image2world(imgpoint, camera_intrinsics, T_camera_world, z=0):
    R = T_camera_world[:3,:3]
    t = T_camera_world[:3, 3]

    R_inv = np.linalg.inv(R)
    intrinsic_inv = np.linalg.inv(camera_intrinsics)

    P = camera_intrinsics.dot(R)

    inverse_P = np.linalg.inv(P)
    unrotate_t = R_inv.dot(t)

    imgpoint_h = np.array([imgpoint[0], imgpoint[1], 1]).reshape((3,1))
   
    s = (z + unrotate_t[2]) / ((inverse_P.dot(imgpoint_h))[2])
    world_point = R_inv.dot(intrinsic_inv.dot(imgpoint_h.dot(s)) - t )

    return world_point

class RobotEyes(Node):
    def __init__(self):
        super().__init__('cam_to_world') # Topics are currently hardcoded

        self.br = CvBridge()
        
        # Subscribing
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.image_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.detections_sub = self.create_subscription(DetectionArray, '/yolo/detections', self.detections_callback, 10)
        # Publishing
        self.entities_pub = self.create_publisher(DetectedEntityArray, '/robot_eyes/entities', 10)

        self.latest_T_cam_world = None
        self.k_matrix = None
        self.distortion_params = None
        self.got_camera_info = False

        self.last_calibration_time = self.get_clock().now() - rclpy.duration.Duration(seconds=10)
        self.calibration_cooldown = rclpy.duration.Duration(seconds=3)

#--------------------------------------------------------------------------------------------------

    def camera_info_callback(self, data):
        self.get_logger().info("Got camera info")
        self.k_matrix = np.array(data.k, dtype=np.float64).reshape((3, 3))
        self.distortion_params = np.array(data.d, dtype=np.float64)
        self.got_camera_info = True
        self.destroy_subscription(self.image_info_sub)

#--------------------------------------------------------------------------------------------------

    def image_callback(self, data):
        now = self.get_clock().now()

        if not self.got_camera_info:
            self.get_logger().info('Waiting for camera info')
#        current_frame = self.br.imgmsg_to_cv2(data)
#        rgb_image = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR) ## ROS is RGB, OpenCV is BGR
        rgb_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')


        ## Detect checkerboard
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, checkerboard_dimensions, None)
        if ret:
            if now - self.last_calibration_time < self.calibration_cooldown:
#                self.get_logger().info("Checkerboard detected, but skipping calibration (cooldown active).")
                return
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(rgb_image, checkerboard_dimensions, corners2, ret)

            ## Solve PnP
            if self.k_matrix is None or self.distortion_params is None:
                self.get_logger().warn("Camera parameters not set.")
                return
            world_points = generate_world_points_R()
            ret_pnp, rvecs, tvecs = cv2.solvePnP(world_points, corners2, self.k_matrix, self.distortion_params, flags=cv2.SOLVEPNP_ITERATIVE)

            if ret_pnp:
                T_cam_world = np.eye(4)
                T_cam_world[:3, :3] = cv2.Rodrigues(rvecs)[0]
                T_cam_world[:3, 3] = tvecs.T
                self.latest_T_cam_world = T_cam_world
                self.last_calibration_time = self.get_clock().now()
                self.get_logger().info('New checkerboard calibration applied.')
#                self.get_logger().info('Got matrix\n'+np.array2string(T_cam_world, precision=5, separator=',', suppress_small=True))


    def detections_callback(self, msg):
        if ((not self.got_camera_info) or (self.latest_T_cam_world is None)):
            self.get_logger().warn("Camera info or transformation matrix not ready.")
            return
        
        # Data extraction and packing
        entity_array_msg = DetectedEntityArray()

        for detection in msg.detections:

            # Bounding box center
            x = detection.bbox.center.position.x
            y = detection.bbox.center.position.y
            cam_center_point = np.array([x,y])

            R_center_point = image2world(cam_center_point, self.k_matrix, self.latest_T_cam_world, z=0).flatten()

            # Bounding box corners (upper left, lower right)
            cam_dx = (detection.bbox.size.x)/2
            cam_dy = (detection.bbox.size.y)/2
            cam_p1 = np.array([x - cam_dx, y - cam_dy])
            cam_p2 = np.array([x + cam_dx, y + cam_dy])

            R_c_ul = image2world(cam_p1, self.k_matrix, self.latest_T_cam_world, z=0).flatten()
            R_c_dr = image2world(cam_p2, self.k_matrix, self.latest_T_cam_world, z=0).flatten()

            # Object's name
            name = detection.class_name

            # Packup
            entity = DetectedEntity()
            entity.name = name
            entity.center    = Point(x=R_center_point[0], y=R_center_point[1], z=R_center_point[2])
            entity.corner_ul = Point(x=R_c_ul[0],         y=R_c_ul[1],         z=R_c_ul[2])
            entity.corner_dr = Point(x=R_c_dr[0],         y=R_c_dr[1],         z=R_c_dr[2])

            entity_array_msg.entities.append(entity)

            # Logs
            self.get_logger().info(f"'{entity.name}' at ({entity.center.x:.3f}m, {entity.center.y:.3f}m, {entity.center.z:.3f}m)")
#            self.get_logger().info(f"Corners: {entity.corner_ul}m x {entity.corner_dr}m")

        if entity_array_msg.entities:
            self.entities_pub.publish(entity_array_msg)

#--------------------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    robot_eyes = RobotEyes()
    rclpy.spin(robot_eyes)
    robot_eyes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()