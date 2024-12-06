import math
import numpy as np
import rclpy
import rclpy.time
import rclpy.timer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped, Pose
from irob_utils.irob_utils.rigid_transform_3D import rigid_transform_3D
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import yaml
import matplotlib.pyplot as plt
import time

class HandEyeRegistrator(Node):
    def __init__(self):
        super().__init__('hand_eye_registrator')

        self.arm = self.get_parameter('~arm')
        self.camera_registration_filename = self.get_parameter('~camera_registration_file')
        self.mode = self.get_parameter('~mode')    # simple, auto, save
        self.poses_filename = self.get_parameter('~poses_filename')

        self.poses_to_save: list[PoseStamped] = []
        self.robot_positions = np.zeros((0,3))
        self.cylmarker_positions = np.zeros((0,3))
        self.cylmarker_tf: PoseStamped = None
        self.clutch_N = 0

        self.cylmarker_tf_sub = self.create_subscription(
            PoseStamped,
            'cylmarker_tf',
            self.cb_cylmarker_tf,
            10)
        self.measured_cp_sub = self.create_subscription(
            PoseStamped,
            f"/{self.arm}/measured_cp",
            self.cb_measured_cp,
            10)
        self.jaw_measured_js_sub = self.create_subscription(
            JointState,
            f"/{self.arm}/jaw/measured_js",
            self.cb_jaw_measured_js,
            10)
        self.manip_clutch_sub = self.create_subscription(
            Joy,
            f"/{self.arm}/manip_clutch",
            self.cb_manip_clutch,
            10)
        
        self.servo_cp_pub = self.create_publisher(
            PoseStamped,
            f"/{self.arm}/servo_cp",
            10)
        self.servo_jaw_pub = self.create_publisher(
            JointState,
            f"/{self.arm}/jaw/servo_jp",
            10)
    

    def cb_cylmarker_tf(self, msg: PoseStamped):
        """Callback function for cylmarker pose."""
        self.cylmarker_tf = msg

    def cb_measured_cp(self, msg: PoseStamped):
        """Callback function for measured_cp."""
        self.measured_cp = msg
    
    def cb_jaw_measured_js(self, msg: JointState):
        """Callback function jaw/measured_js"""
        self.measured_jaw = msg

    def cb_manip_clutch(self, msg: Joy):
        """Callback function when clutch button is pressed.
        Collect one sample from the positions.
        """
        if self.cylmarker_tf is not None and self.clutch_N > 0 and msg.buttons[0] == 0:
            self.gather_actual_position()

        self.clutch_N = self.clutch_N + 1

    
    def gather_actual_position(self):
        """Gather a single position from the camera and the robot."""
        time.sleep(0.5)
        if ((self.get_clock().now() - self.cylmarker_tf.header.stamp) < 4e8):  # 0.4s in nanosec
            robot_pos = np.array([self.measured_cp.pose.position.x,
                                self.measured_cp.pose.position.y,
                                self.measured_cp.pose.position.z]).T
            cylmarker_pos = np.array([self.cylmarker_tf.pose.position.x,
                                    self.cylmarker_tf.pose.position.y,
                                    self.cylmarker_tf.pose.position.z]).T
            
            if self.mode == "save":
                self.poses_to_save.append(self.measured_cp)

            self.robot_positions = np.vstack((self.robot_positions, robot_pos))
            self.cylmarker_positions = np.vstack((self.cylmarker_positions, cylmarker_pos))

            print("Positions collected: " + str(self.robot_positions.shape[0]))
        else:
            print("Couldn't locate cylmarker in this setup.")
    

    def reset_arm(self):
        t = Pose()

        # TODO: Find appropriate pose for calib
        t.position.x = 0.0
        t.position.y = 0.0
        t.position.z = -0.12
        t.orientation.x = 0.393899553586202
        t.orientation.y = 0.9179819355728568
        t.orientation.z = -0.046392890942680814
        t.orientation.w = -0.00000000855
    
        self.move_tcp_to(t, 0.05, 0.1)
    

    def move_tcp_to(self, target: Pose, v: float, dt: float):
        """Move the TCP to the desired pose on linear trajectory.

        Keyword arguments:
        target -- desired pose
        v -- TCP linear velocity
        dt -- sampling time
        """
        # Calculate the linear trajectory
        pos_current_np = np.array([self.measured_cp.pose.position.x,
                                self.measured_cp.pose.position.y,
                                self.measured_cp.pose.position.z])
        pos_target_np = np.array([target.position.x,
                                    target.position.y,
                                    target.position.z])
        d = np.linalg.norm(pos_target_np - pos_current_np)
        T = d / v
        N = int(math.floor(T / dt))
        tx = np.linspace(pos_current_np[0], pos_target_np[0], N)
        ty = np.linspace(pos_current_np[1], pos_target_np[1], N)
        tz = np.linspace(pos_current_np[2], pos_target_np[2], N)

        #SLERP
        rotations = Rotation.from_quat([[self.measured_cp.pose.orientation.x,
                                           self.measured_cp.pose.orientation.y,
                                           self.measured_cp.pose.orientation.z,
                                           self.measured_cp.pose.orientation.w],
                                           [target.orientation.x,
                                           target.orientation.y,
                                           target.orientation.z,
                                           target.orientation.w]])


        times = np.linspace(0, T, N)
        do_slerp = False    # If the rotations are the same
        try:
            slerp = Slerp([0,T], rotations)
            interp_rots = slerp(times)
            do_slerp = True
        except ValueError as e:
            do_slerp = False


        # Set the rate of the loop
        # TODO: Use threading
        rate = self.create_rate(1.0 / dt)

        # Send the robot to the points of the calculated trajectory
        # with the desired rate
        for i in range(N):
            if not rclpy.ok():
                break # CTRL-C is pressed

            p = self.measured_cp
            p.pose.position.x = tx[i]
            p.pose.position.y = ty[i]
            p.pose.position.z = tz[i]

            if do_slerp:
                quat_helper = interp_rots[i].as_quat()
            else:
                quat_helper = rotations[1].as_quat()

            p.pose.orientation.x = quat_helper[0]
            p.pose.orientation.y = quat_helper[1]
            p.pose.orientation.z = quat_helper[2]
            p.pose.orientation.w = quat_helper[3]

            #rospy.loginfo(p)
            self.servo_cp_pub.publish(p)
            rclpy.spin_once(self)


    def save_robot_poses(self):
        """Save robot poses to config file for auto registration."""
        data = dict(p = [])
        
        for r in self.poses_to_save:
            data["p"].append([r.pose.position.x, r.pose.position.y,
                             r.pose.position.z, r.pose.orientation.x,
                             r.pose.orientation.y, r.pose.orientation.z,
                             r.pose.orientation.w])

        with open(self.poses_filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
            outfile.close()


    def load_robot_poses(self):
        """Load robot poses from file for auto registration."""
        with open(self.poses_filename, "r") as file:
            documents = yaml.full_load(file)
            self.poses_for_reg = []
            for item, doc in documents.items():
                #print(item, ":", doc)
                for p in doc:
                    t = Pose()
                    t.position.x = p[0]
                    t.position.y = p[1]
                    t.position.z = p[2]
                    t.orientation.x = p[3]
                    t.orientation.y = p[4]
                    t.orientation.z = p[5]
                    t.orientation.w = p[6]
                    self.poses_for_reg.append(t)


    def do_auto_registration(self, v: float, dt: float):
        """Register arm with a predefined set of positions autonomously.

        Keyword arguments:
        v -- TCP linear velocity
        dt -- sampling time
        """
        input("Starting auto registration. The robot will do large movements. " +
                                                    "Press Enter when ready...")
        for t in self.poses_for_reg:
            self.move_tcp_to(t, v, dt)
            self.gather_actual_position()

    
    def collect_and_register(self):
        """Wait for data colection. When key pressed,
        do the registration.
        """
        input("Collect data for registration. Press Enter when done...")

        R, t = rigid_transform_3D(self.cylmarker_positions.T, self.robot_positions.T)
        points_transformed = np.zeros(self.robot_positions.shape)

        for i in range(self.robot_positions.shape[0]):
            p = np.dot(R, self.robot_positions[i,:].T) + t.T
            points_transformed[i,:] = p
        
        # Draw plot
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.scatter(self.cylmarker_positions[0,:], self.cylmarker_positions[1,:], self.cylmarker_positions[2,:], marker='o')
        self.ax.scatter(points_transformed[0,:], points_transformed[1,:], points_transformed[2,:], marker='^')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # Save robot poses for auto registration
        if self.mode == "save":
            self.save_robot_poses()
        return R, t


    def save_registration(self, R: np.ndarray, t: np.ndarray):
        """Save registration to config file.

        Keyword arguments:
        R -- rotation matrix
        t -- translation vector
        """
        data = dict(
            t = [float(t[0,0]), float(t[1,0]), float(t[2,0])],
            R = [float(R[0,0]), float(R[0,1]), float(R[0,2]),
                float(R[1,0]), float(R[1,1]), float(R[1,2]),
                float(R[2,0]), float(R[2,1]), float(R[2,2])]
        )

        with open(self.camera_registration_filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
            outfile.close()
            input("Registration saved to file " + self.camera_registration_filename + ".")


if __name__ == '__main__':
    rclpy.init()
    reg = HandEyeRegistrator()
    dt = 0.01

    if reg.mode == "auto":
        reg.load_robot_poses()
        reg.do_auto_registration(0.05, dt)
    elif reg.mode == "save" or reg.mode == "simple":
        R, t = reg.collect_and_register()
        reg.save_registration(R, t)
    else:
        print("Please define a correct mode (simple, save, auto). Exiting...")

    # try:
    #     #rclpy.spin(reg)
    # except (ExternalShutdownException, KeyboardInterrupt):
    #     pass
    # finally:
    #     rclpy.try_shutdown()
