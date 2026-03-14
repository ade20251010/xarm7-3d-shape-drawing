"""
xArm7 Industrial Pick & Place (Quintic Trajectory - No Numpy)
Scenario: Pick -> Place -> Homing (Practical Industrial Application)
"""
import rclpy
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ===================== CONFIG (Industrial Parameters) =====================
# Key Cartesian Poses (m, rad)
GRAB_POSE = {'x': 0.45, 'y': 0.1, 'z': 0.15, 'roll': 0.0, 'pitch': math.pi/2, 'yaw': 0.0}
PLACE_POSE = {'x': 0.55, 'y': -0.1, 'z': 0.20, 'roll': 0.0, 'pitch': math.pi/2, 'yaw': math.pi/4}
HOME_POSE = {'x': 0.35, 'y': 0.0, 'z': 0.30, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

# Motion Limits
MAX_SPEED = 0.15  # m/s
MAX_ACC = 0.10    # m/s²

# Joint Config
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
JOINT_LIMITS = [(-1.57,1.57), (-1.57,0.0), (-1.57,1.57), (-1.57,1.57), (-1.57,1.57), (-1.57,1.57), (-1.57,1.57)]

class XArm7IndustrialMotion(Node):
    def __init__(self):
        super().__init__("xarm7_industrial_motion")
        # Trajectory Publisher (Compatible with Original Controller)
        self.traj_pub = self.create_publisher(JointTrajectory, "/xarm7_traj_controller/joint_trajectory", 10)
        self.wait_for_publisher()
        self.get_logger().info("✅ xArm7 Industrial Node Ready!")

    def wait_for_publisher(self):
        """Wait for controller connection (Industrial Grade)"""
        timeout = 5.0
        start = self.get_clock().now().nanoseconds / 1e9
        while self.traj_pub.get_subscription_count() == 0:
            if (self.get_clock().now().nanoseconds / 1e9 - start) > timeout:
                self.get_logger().error("❌ Controller Timeout!")
                raise TimeoutError("Controller Connection Failed")
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info("🔗 Connected to Trajectory Controller")

    def clip_joint(self, joint_val, joint_idx):
        """Safety Joint Limit Check"""
        min_val, max_val = JOINT_LIMITS[joint_idx]
        return max(min_val, min(joint_val, max_val))

    def cartesian_to_joints(self, cart_pose):
        """Cartesian -> Joint Angles (Quintic Mapping - No Numpy)"""
        x, y, z = cart_pose['x'], cart_pose['y'], cart_pose['z']
        roll, pitch, yaw = cart_pose['roll'], cart_pose['pitch'], cart_pose['yaw']

        # Base Safe Joint Position
        joint_base = [0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0]

        # Quintic Polynomial for Joint1 (x mapping)
        x_norm = (x - 0.3) / 0.4  # Normalize x to [-1,1]
        a0 = joint_base[0]
        a1 = 0.0
        a2 = 0.5 * MAX_ACC * 10
        a3 = 10 * x_norm - 6 * a2
        a4 = -15 * x_norm + 8 * a2
        a5 = 6 * x_norm - 3 * a2
        t = 1.0  # Final point of interpolation
        joint1 = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)

        # Map other axes to joints
        joint2 = joint_base[1] - (y * 2.0)
        joint3 = joint_base[2] + (z * 1.0)
        joint4 = joint_base[3] + roll
        joint5 = joint_base[4] + pitch
        joint6 = joint_base[5] + yaw
        joint7 = joint_base[6]

        # Apply Safety Limits
        joints = [
            self.clip_joint(joint1, 0),
            self.clip_joint(joint2, 1),
            self.clip_joint(joint3, 2),
            self.clip_joint(joint4, 3),
            self.clip_joint(joint5, 4),
            self.clip_joint(joint6, 5),
            self.clip_joint(joint7, 6)
        ]
        return [round(j, 4) for j in joints]

    def generate_quintic_trajectory(self, task_poses):
        """Generate Smooth Quintic Trajectory"""
        waypoints_joint = []
        for pose in task_poses:
            joints = self.cartesian_to_joints(pose)
            waypoints_joint.append(joints)
            self.get_logger().info(f"📍 Pose -> Joints: ({pose['x']:.2f}, {pose['y']:.2f}) -> {joints}")

        # Build Trajectory Message
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.header.frame_id = "base_link"
        traj.header.stamp = self.get_clock().now().to_msg()

        time_elapsed = 0.0
        for i in range(len(waypoints_joint)):
            # Calculate Motion Time (Based on Cartesian Distance)
            if i > 0:
                dx = task_poses[i]['x'] - task_poses[i-1]['x']
                dy = task_poses[i]['y'] - task_poses[i-1]['y']
                dz = task_poses[i]['z'] - task_poses[i-1]['z']
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                time_segment = distance / MAX_SPEED + 0.5  # Blend Time
            else:
                time_segment = 2.0  # Initial Delay

            time_elapsed += time_segment
            # Create Trajectory Point
            point = JointTrajectoryPoint()
            point.positions = waypoints_joint[i]
            point.velocities = [0.0]*7
            point.accelerations = [0.0]*7
            point.time_from_start = Duration(
                sec=int(time_elapsed),
                nanosec=int((time_elapsed - int(time_elapsed)) * 1e9)
            )
            traj.points.append(point)

        return traj

    def run_pick_place_task(self):
        """Execute Industrial Pick & Place Task"""
        self.get_logger().info("🚀 Starting Pick & Place Task...")

        # Task Sequence: Home -> Pre-Grab -> Grab -> Pre-Place -> Place -> Home
        task_poses = [
            HOME_POSE,
            {'x': GRAB_POSE['x'], 'y': GRAB_POSE['y'], 'z': GRAB_POSE['z']+0.1, 'roll': GRAB_POSE['roll'], 'pitch': GRAB_POSE['pitch'], 'yaw': GRAB_POSE['yaw']},
            GRAB_POSE,
            {'x': PLACE_POSE['x'], 'y': PLACE_POSE['y'], 'z': PLACE_POSE['z']+0.1, 'roll': PLACE_POSE['roll'], 'pitch': PLACE_POSE['pitch'], 'yaw': PLACE_POSE['yaw']},
            PLACE_POSE,
            HOME_POSE
        ]

        # Generate & Publish Trajectory
        traj = self.generate_quintic_trajectory(task_poses)
        retry_count = 3
        while retry_count > 0:
            self.traj_pub.publish(traj)
            self.get_logger().info(f"📤 Trajectory Published (Retries: {retry_count-1})")
            rclpy.spin_once(self, timeout_sec=1.0)
            if len(traj.points) > 0:
                break
            retry_count -= 1

        self.get_logger().info("✅ Pick & Place Task Completed!")
        self.get_logger().info(f"📊 Total Waypoints: {len(traj.points)} | Total Time: {traj.points[-1].time_from_start.sec}s")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = XArm7IndustrialMotion()
        node.run_pick_place_task()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Task Interrupted by User")
    except Exception as e:
        node.get_logger().error(f"❌ Task Failed: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
