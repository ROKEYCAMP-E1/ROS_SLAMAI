import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
from collections import deque
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_position = (0.0, 0.0)  # 초기 위치

    def odom_callback(self, msg):
        # Odometry 메시지에서 x, y 위치를 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = (x, y)
        self.get_logger().info(f'Current position: {self.current_position}')

# 1. MapSubscriber 클래스
class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg.data
        self.width = msg.info.width  # 맵의 가로 셀 개수
        self.height = msg.info.height  # 맵의 세로 셀 개수
        self.resolution = msg.info.resolution  # 맵 해상도
        self.origin = msg.info.origin  # 맵의 원점 좌표
        self.get_logger().info('Map data received')

# 2. 장애물 감지 클래스
class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        ranges = msg.ranges
        if min(ranges) < 0.5:  # 0.5m 이하에 장애물 감지
            self.get_logger().warn('Obstacle detected!')

# 3. Navigator 클래스
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0  # 단순한 회전 없음
        self.client.wait_for_server()
        self.client.send_goal_async(goal)
        self.get_logger().info(f'Sending goal to ({x}, {y})')

# 4. 미탐지 구역 탐지 함수 (첫 번째 미탐지 영역 반환)
def find_unknown_area(map_data, width, height):
    map_array = np.array(map_data).reshape((height, width))
    unknown_points = np.argwhere(map_array == -1)  # 값이 -1인 위치 찾기
    if len(unknown_points) == 0:
        return None
    return unknown_points[0]  # 첫 번째 미탐지 구역 반환

# 5. 로봇과 가장 가까운 미탐지 영역 탐지 함수
def find_nearest_unknown_area(map_data, width, height, robot_position, resolution, origin):
    map_array = np.array(map_data).reshape((height, width))
    unknown_points = np.argwhere(map_array == -1)
    if len(unknown_points) == 0:
        return None
    # 실제 좌표로 변환 후, 로봇 위치와 거리 계산
    distances = []
    for point in unknown_points:
        x, y = convert_to_coordinates(point, resolution, origin)
        dist = np.sqrt((x - robot_position[0])**2 + (y - robot_position[1])**2)
        distances.append((dist, point))
    # 가장 가까운 지점 반환
    distances.sort(key=lambda d: d[0])
    return distances[0][1]

# 6. 맵 완성 여부 판단 함수
def is_map_complete(map_data, width, height):
    map_array = np.array(map_data).reshape((height, width))

    # BFS를 통해 접근 가능한 미탐지 영역 탐색
    start_points = np.argwhere(map_array == 0)  # 빈 공간 찾기
    if len(start_points) == 0:
        return True  # 빈 공간이 없으면 맵 완성으로 간주

    visited = np.zeros_like(map_array, dtype=bool)
    queue = deque()

    for start in start_points:
        queue.append(tuple(start))
        visited[tuple(start)] = True

    while queue:
        row, col = queue.popleft()

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 상하좌우 탐색
            r, c = row + dr, col + dc
            if 0 <= r < height and 0 <= c < width and not visited[r, c]:
                if map_array[r, c] == -1:  # 미탐지 영역 발견
                    return False
                if map_array[r, c] == 0:  # 빈 공간 탐색 계속
                    queue.append((r, c))
                    visited[r, c] = True

    return True  # 접근 가능한 미탐지 영역 없음

# 7. 좌표 변환 함수
def convert_to_coordinates(target, resolution, origin):
    x = target[1] * resolution + origin.position.x
    y = target[0] * resolution + origin.position.y
    return x, y

# 8. 메인 함수
def main():
    rclpy.init()
    map_subscriber = MapSubscriber()
    navigator = Navigator()
    obstacle_avoider = ObstacleAvoider()
    odom_subscriber = OdomSubscriber()

    robot_position = (0.0, 0.0)  # 초기 로봇 위치 (실제 로봇의 위치를 받아올 수 있으면 수정)

    while rclpy.ok():
        rclpy.spin_once(map_subscriber)
        rclpy.spin_once(odom_subscriber)  # Odometry 데이터 업데이트
        if map_subscriber.map_data:
            # 맵 완성 여부 확인
            if is_map_complete(
                map_subscriber.map_data,
                map_subscriber.width,
                map_subscriber.height
            ):
                print("Map is complete!")
                break

            # 가장 가까운 미탐지 영역 탐지
            target = find_nearest_unknown_area(
                map_subscriber.map_data,
                map_subscriber.width,
                map_subscriber.height,
                robot_position,
                map_subscriber.resolution,
                map_subscriber.origin
            )
            if target is not None:
                x, y = convert_to_coordinates(target, map_subscriber.resolution, map_subscriber.origin)
                navigator.send_goal(x, y)

        rclpy.spin_once(obstacle_avoider)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
