#!/usr/bin/env python3
"""
Pure Pursuit 알고리즘을 사용한 로봇 이동 노드
지정된 고정 목표지점으로 이동하도록 수정됨
목표지점: (0, -7.889842200937677, 0)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class PurePursuitMoveNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_move_node')
        
        # ==================== 수정 가능한 파라미터 ====================
        # 고정 목표 좌표 설정
        self.target_x = 0.0
        self.target_y = -7.889842200937677
        self.target_received = True  # 목표가 고정되어 있으므로 항상 True
        
        # 속도 설정
        self.linear_speed = 3.0          # ← 전진 속도 (m/s) 고정
        self.max_angular_speed = 1.5     # ← 회전 속도 (rad/s)
        self.position_tolerance = 0.2    # ← 도달 거리 (m)
        
        # Pure Pursuit 파라미터
        self.look_ahead_distance = 0.4   # ← Look-ahead 거리
        self.wheel_base = 0.4            # 로봇 휠베이스
        
        # 제어 게인
        self.steering_gain = 0.5         # 조향 각도 게인
        # ================================================================
        
        # 현재 로봇 상태
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        
        # 상태 추적
        self.min_distance = float('inf')
        
        # 퍼블리셔 및 서브스크라이버
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 제어 루프 타이머
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🚀 고정 좌표 Pure Pursuit 노드 시작')
        self.get_logger().info(f'📍 고정 목표 지점: ({self.target_x}, {self.target_y})')
        self.get_logger().info(f'⚙️  속도: linear={self.linear_speed} m/s, angular={self.max_angular_speed} rad/s')
        
    def odom_callback(self, msg: Odometry):
        """Odometry 메시지로부터 위치 업데이트"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        euler = euler_from_quaternion(quaternion)
        self.current_theta = euler[2]
        
        self.odom_received = True

    def normalize_angle(self, angle: float) -> float:
        """각도를 [-pi, pi] 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """두 점 사이의 유클리드 거리"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def calculate_steering_angle(self) -> float:
        """
        Pure Pursuit 알고리즘을 사용한 조향각 계산
        """
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        if distance_to_target < 0.01:
            return 0.0
        
        target_angle = math.atan2(dy, dx)
        angle_to_target = self.normalize_angle(target_angle - self.current_theta)
        
        if distance_to_target < self.look_ahead_distance:
            steering_angle = angle_to_target
        else:
            alpha = math.atan2(dy, dx) - self.current_theta
            alpha = self.normalize_angle(alpha)
            steering_angle = math.atan2(2.0 * self.wheel_base * math.sin(alpha), distance_to_target)
        
        steering_angle = max(min(steering_angle, self.max_angular_speed), -self.max_angular_speed)
        return steering_angle
    
    def control_loop(self):
        """주 제어 루프"""
        if not self.odom_received:
            return
        
        distance_to_goal = self.calculate_distance(
            self.current_x, self.current_y,
            self.target_x, self.target_y
        )
        
        if distance_to_goal < self.min_distance:
            self.min_distance = distance_to_goal
        
        goal_direction = math.atan2(
            self.target_y - self.current_y,
            self.target_x - self.current_x
        )
        
        angle_error = self.normalize_angle(goal_direction - self.current_theta)
        
        self.get_logger().info(
            f'📍 로봇: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'목표: ({self.target_x:.2f}, {self.target_y:.2f}) | '
            f'거리: {distance_to_goal:.3f}m | '
            f'각도: {math.degrees(angle_error):.1f}°'
        )
        
        twist = Twist()
        
        # 목표 도달 판정
        if distance_to_goal < self.position_tolerance:
            self.get_logger().info(
                f'✅ 목표 도달! 최종 위치: ({self.current_x:.3f}, {self.current_y:.3f})'
            )
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.timer.cancel()
            return
        
        # 전진 속도 계산 (각도 오차가 크면 속도 감소)
        angle_factor = math.cos(angle_error)
        linear_speed = self.linear_speed * max(angle_factor, 0.3)
        linear_speed = max(linear_speed, self.linear_speed * 0.2)
        
        # 조향각 계산
        steering_angle = self.calculate_steering_angle()
        
        twist.linear.x = linear_speed
        twist.angular.z = steering_angle
        
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitMoveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 노드 종료')
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()