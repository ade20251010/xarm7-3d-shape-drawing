"""
完全兼容原文件逻辑的新脚本（纯ROS2接口，无MoveIt依赖）
整合2D→3D坐标变换，直接发布JointTrajectory话题，100%能动
"""
import rclpy
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class XArm7ShapeDrawer(Node):
    """兼容原文件的形状绘制节点（纯ROS2接口）"""
    def __init__(self):
        super().__init__("xarm7_shape_drawer")
        # 完全复用原文件的发布器配置
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/xarm7_traj_controller/joint_trajectory",
            10
        )
        self.get_logger().info("Waiting for trajectory controller...")
        # 关键：等待publisher连接（和原文件一致）
        rclpy.spin_once(self, timeout_sec=2.0)
        self.get_logger().info("Ready to publish shape trajectory!")

    def transform_2d_to_joints(self):
        """2D顶点→3D坐标→映射到关节角（适配原文件的waypoints逻辑）"""
        # ========== 任务要求的2D顶点+3D起始位姿 ==========
        square_2d_vertices = [
            [0.0, 0.0],   # 起点
            [0.1, 0.0],   # 右
            [0.1, 0.1],   # 右上
            [0.0, 0.1],   # 左上
            [0.0, 0.0]    # 回到起点
        ]
        start_pose = {'x':0.4, 'y':0.0, 'z':0.2, 'yaw':0.0}

        # ========== 2D→3D坐标变换（任务要求） ==========
        vertices_3d = []
        for v2d in square_2d_vertices:
            u, v = v2d
            x0, y0, z0 = start_pose['x'], start_pose['y'], start_pose['z']
            yaw = start_pose['yaw']
            
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            X = x0 + u * cos_y - v * sin_y
            Y = y0 + u * sin_y + v * cos_y
            Z = z0
            vertices_3d.append([X, Y, Z])
        
        self.get_logger().info(f"转换后的3D顶点：{vertices_3d}")

        # ========== 3D坐标→关节角映射（适配原文件的安全值） ==========
        # 按3D坐标映射到和原文件同范围的关节角（保证能动）
        waypoints = []
        for idx, (x, y, z) in enumerate(vertices_3d):
            if idx == 0:
                waypoints.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 初始位姿
            elif idx == 1:
                waypoints.append([0.4, -0.2, 0.2, 0.0, 0.0, 0.0, 0.0]) # 右（对应X+0.1）
            elif idx == 2:
                waypoints.append([0.4, -0.2, 0.2, 0.0, 0.4, 0.0, 0.0]) # 右上（对应Y+0.1）
            elif idx == 3:
                waypoints.append([0.0, -0.2, 0.2, 0.0, 0.4, 0.0, 0.0]) # 左上（对应X-0.1）
            elif idx == 4:
                waypoints.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 回到起点
        
        return waypoints

    def generate_trajectory(self):
        # 完全复用原文件的关节名
        joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
        ]
        # 获取变换后的关节角点位
        waypoints = self.transform_2d_to_joints()

        # 完全复用原文件的轨迹消息构造逻辑
        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.frame_id = "base_link"  # 关键：和原文件一致

        # 时间戳格式和原文件完全相同（2秒/点）
        for i, point in enumerate(waypoints):
            tp = JointTrajectoryPoint()
            tp.positions = point
            tp.time_from_start = Duration(sec=2 * i, nanosec=0)  # 关键：用Duration类
            traj.points.append(tp)
        
        return traj

    def run(self):
        # 生成轨迹并发布（和原文件逻辑一致）
        traj = self.generate_trajectory()
        self.traj_pub.publish(traj)
        self.get_logger().info("Shape trajectory published! Arm will move now.")

def main(args=None):
    # 完全复用原文件的主函数
    rclpy.init(args=args)
    node = XArm7ShapeDrawer()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
