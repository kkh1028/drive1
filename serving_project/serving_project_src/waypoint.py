import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import threading
import sys
import select
import termios
import tty
from std_srvs.srv import Empty  
from nav_msgs.msg import Odometry 


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # 1~9번 테이블 좌표 매핑 (x, y, z, w)
        self.table_positions = {
            1: (3.1855, 1.3044, 0.1388, 0.9903),
            2: (3.1423, 0.2024, -0.1397, 0.9901),
            3: (3.1387, -0.8759, -0.0874, 0.9961),
            4: (1.9815, 1.1707, 0.2005, 0.9904),
            5: (2.0812, 0.1124, 0.1093, 0.9898),
            6: (2.0263, -0.9178, -0.1669, 0.9906),
            7: (0.8760, 1.3392, 0.1532, 0.9881),
            8: (0.9056, 0.1988, -0.9831, 0.1828),
            9: (0.9312, -0.9836, 0.1348, 0.4767),
        }

        # 출발점 좌표
        self.start_position = (0.0, 0.0, 0.0, 1.0)

        # 이동 상태 플래그
        self.is_moving = False

        # 새로운 명령을 저장하는 큐
        self.pending_tasks = []

        # `selected_tables` 토픽 구독
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            Int32MultiArray, '/selected_tables', self.table_selection_callback, qos_profile
        )

        self.get_logger().info("Subscribed to /selected_tables")

        # 🚀 서비스 추가
        self.srv_get_position = self.create_service(Empty, 'get_robot_position', self.get_robot_position_callback)
        self.srv_get_status = self.create_service(Empty, 'get_robot_status', self.get_robot_status_callback)
        self.srv_cancel_movement = self.create_service(Empty, 'cancel_robot_movement', self.cancel_robot_movement_callback)

        # 위치 구독 (Odometry 데이터 사용)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info("WaypointFollower services initialized!")


    def table_selection_callback(self, msg):
        """
        테이블 선택 토픽 수신 콜백 함수 (1~9개의 랜덤 테이블이 들어올 수 있음)
        """
        table_numbers = list(msg.data)
        self.get_logger().info(f'Received Table Numbers: {table_numbers}')

        if self.is_moving:
            # 🚀 현재 이동 중이면 대기열에 저장 후, 기존 경로 종료 후 실행
            self.pending_tasks.append(table_numbers)
            self.get_logger().info(f'Queued Task: {table_numbers}')
        else:
            # 🚀 바로 실행 가능하면 실행
            self.execute_route(table_numbers)

    def execute_route(self, table_numbers):
        """
        받은 테이블 번호에 따라 웨이포인트 경로를 설정하고 실행
        """
        waypoints_list = []

        # 테이블 번호에 해당하는 좌표를 경로에 추가
        for table in table_numbers:
            if table in self.table_positions:
                x, y, z, w = self.table_positions[table]
                waypoint = self.create_pose(x, y, z, w)
                waypoints_list.append(waypoint)

        # 출발점으로 복귀
        x, y, z, w = self.start_position
        waypoints_list.append(self.create_pose(x, y, z, w))

        # 🚀 웨이포인트가 없으면 이동하지 않음
        if not waypoints_list:
            self.get_logger().warn("No valid waypoints received. Skipping movement.")
            return

        # 이동 시작
        self.is_moving = True
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints_list

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def create_pose(self, x, y, z, w):
        """
        좌표를 PoseStamped 객체로 변환
        """
        waypoint = PoseStamped()
        waypoint.header.stamp.sec = 0
        waypoint.header.stamp.nanosec = 0
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = 0.0
        waypoint.pose.orientation.z = z
        waypoint.pose.orientation.w = w
        return waypoint

    def goal_response_callback(self, future):
        """
        목표가 수락되었는지 확인하는 콜백 함수
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.is_moving = False
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        이동 중 피드백을 출력하는 콜백 함수
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def get_result_callback(self, future):
        """
        목표 완료 후 실행되는 콜백 함수
        """
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

        # 🚀 이동 종료 후 다음 작업 실행 여부 확인
        self.is_moving = False
        if self.pending_tasks:
            next_task = self.pending_tasks.pop(0)
            self.get_logger().info(f'Executing Next Task: {next_task}')
            self.execute_route(next_task)

    def odom_callback(self, msg):
        """ 로봇의 현재 위치를 업데이트하는 콜백 함수 """
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def get_robot_position_callback(self, request, response):
        """ 현재 로봇의 위치를 반환하는 서비스 """
        if self.current_position:
            self.get_logger().info(f"Current Position: {self.current_position}")
        else:
            self.get_logger().info("Position data not available.")
        return response

    def get_robot_status_callback(self, request, response):
        """ 현재 로봇의 상태를 반환하는 서비스 """
        status_msg = "이동 중" if self.is_moving else "대기 중"
        self.get_logger().info(f"Robot Status: {status_msg}")
        return response

    def cancel_robot_movement_callback(self, request, response):
        """ 현재 로봇의 경로를 취소하는 서비스 """
        if self.goal_handle:
            self.get_logger().info("Cancelling current movement...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        return response

    def cancel_done_callback(self, future):
        """ 목표 취소 후 콜백 """
        cancel_response = future.result()
        if len(cancel_response.goals_cancelled) > 0:
            self.get_logger().info("Movement successfully cancelled.")
        else:
            self.get_logger().info("Failed to cancel movement.")
def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    node.get_logger().info('Exiting program...')
                    node.destroy_node()
                    rclpy.shutdown()
                    sys.exit(0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
