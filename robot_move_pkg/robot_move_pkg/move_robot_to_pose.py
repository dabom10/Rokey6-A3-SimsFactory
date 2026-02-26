#!/usr/bin/env python3
"""
Isaac Sim robot.usd를 ROS2 /cmd_vel을 통해 지정 좌표로 이동시키는 노드

목표 좌표: (-4.586694073776432, 6.555723926308726, 0)
방식: Nav2 스타일의 /cmd_vel 퍼블리싱 + Odometry 구독
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import time

class RobotMoveNode(Node):
    def __init__(self):
        super().__init__('robot_move_node')
        
        # 목표 좌표 설정
        self.target_x = 0
        self.target_y = -7.889842200937677
        self.target_theta = 0.0  # 목표 방향 (라디안)
        
        # 현재 로봇 위치 및 방향
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # 제어 파라미터
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.1  # m
        self.angle_tolerance = 0.1  # rad
        
        # 퍼블리셔 및 서브스크라이버
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 타이머 (제어 루프)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(
            f'로봇을 목표 좌표 ({self.target_x}, {self.target_y})로 이동합니다.'
        )
        
    def odom_callback(self, msg: Odometry):
        """
        Odometry 메시지로부터 현재 로봇 위치 업데이트
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 사원수에서 오일러 각도로 변환
        quat = msg.pose.pose.orientation
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        euler = euler_from_quaternion(quaternion)
        self.current_theta = euler[2]  # yaw 각도
        
    def normalize_angle(self, angle: float) -> float:
        """
        각도를 [-pi, pi] 범위로 정규화
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """
        두 점 사이의 거리 계산
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def control_loop(self):
        """
        주 제어 루프 - 목표 위치로 로봇을 이동
        """
        # 현재 위치와 목표 위치 사이의 거리
        distance = self.calculate_distance(
            self.current_x, self.current_y,
            self.target_x, self.target_y
        )
        
        # 목표 방향 계산 (atan2 사용)
        goal_direction = math.atan2(
            self.target_y - self.current_y,
            self.target_x - self.current_x
        )
        
        # 현재 각도와 목표 각도의 차이
        angle_error = self.normalize_angle(goal_direction - self.current_theta)
        
        # Twist 메시지 생성
        twist = Twist()
        
        # 위치에 도달했는지 확인
        if distance < self.position_tolerance:
            self.get_logger().info('목표 위치에 도달했습니다!')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            # 타이머 정지
            self.control_timer.cancel()
            return
        
        # 방향 정렬 로직
        if abs(angle_error) > self.angle_tolerance:
            # 먼저 방향을 맞춤
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            
            self.get_logger().debug(
                f'방향 조정 - 각도 오류: {math.degrees(angle_error):.2f}도'
            )
        else:
            # 방향이 맞으면 전진
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
            self.get_logger().debug(
                f'전진 중 - 거리: {distance:.2f}m, '
                f'위치: ({self.current_x:.2f}, {self.current_y:.2f})'
            )
        
        # cmd_vel 퍼블리시
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMoveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
