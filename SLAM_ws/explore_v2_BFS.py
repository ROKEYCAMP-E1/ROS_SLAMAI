import math

import numpy as np

from nav_msgs.msg import OccupancyGrid, Odometry

from geometry_msgs.msg import PoseStamped

import rclpy

from rclpy.node import Node



class MapExplorer(Node):

    def __init__(self):

        super().__init__('map_explorer')

        

        # 맵 데이터 구독

        self.map_subscriber = self.create_subscription(

            OccupancyGrid,

            'bot1/map',

            self.map_callback,

            10

        )

        

        # 오도메트리 구독

        self.odom_subscriber = self.create_subscription(

            Odometry,

            'bot1/odom',

            self.odom_callback,

            10

        )



        # 목표 지점 발행

        self.nav_goal_publisher = self.create_publisher(PoseStamped, 'bot1/goal_pose', 10)



        # 로봇 위치 초기화

        self.robot_x = 0.0

        self.robot_y = 0.0



        # 맵 데이터 초기화

        self.current_map = None

        self.map_width = 0

        self.map_height = 0

        self.map_info = None  # OccupancyGrid.info 저장



    def map_callback(self, map_data):
        """맵 데이터 업데이트"""
        new_map = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        if self.current_map is not None and np.array_equal(self.current_map, new_map):
            self.get_logger().info("Map has not changed.")
        else:
            self.get_logger().info("Map updated.")
            self.current_map = new_map
            self.map_width = map_data.info.width
            self.map_height = map_data.info.height
            self.map_info = map_data.info  # 맵 정보 저장

            # 탐색 프로세스 시작
            self.explore_map()


            # 디버깅: 맵 데이터 요약 출력

            unique_values, counts = np.unique(self.current_map, return_counts=True)

            self.get_logger().info(f"Map data: {dict(zip(unique_values, counts))}")



            # 탐색 프로세스 시작

            self.explore_map()



    def odom_callback(self, msg):

        """로봇의 현재 위치를 업데이트"""

        self.robot_x = msg.pose.pose.position.x

        self.robot_y = msg.pose.pose.position.y



    def cell_to_map_coordinates(self, x_cell, y_cell):

        """셀 좌표를 맵 좌표로 변환"""

        resolution = self.map_info.resolution

        origin_x = self.map_info.origin.position.x

        origin_y = self.map_info.origin.position.y



        x_map = x_cell * resolution + origin_x

        y_map = y_cell * resolution + origin_y

        return x_map, y_map

    
    def find_exploration_boundary(self):

        """탐지 가능한 경계(-1과 0이 맞닿은 부분)를 찾음"""

        exploration_boundary = []

        for y in range(self.map_height):

            for x in range(self.map_width):

                if self.current_map[y, x] == -1:  # 미탐지 영역

                    # 인접한 탐지된 영역이 있는지 확인

                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:

                        nx, ny = x + dx, y + dy

                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height:  # 경계 조건

                            if self.current_map[ny, nx] == 0:
                                # Inflation radius 내의 셀은 제외
                                if not self.is_within_inflation_radius(x, y):
                                    exploration_boundary.append((x, y))

                                exploration_boundary.append((x, y))

                                break

        # 디버깅: 탐지된 경계 수 출력

        self.get_logger().info(f"Found {len(exploration_boundary)} exploration boundaries.")

        return exploration_boundary

       

    







    def find_closest_boundary(self, exploration_boundary):

        """탐지된 경계 중 로봇과 가장 가까운 지점을 찾음"""

        closest_point = None

        min_distance = float('inf')



        for x_cell, y_cell in exploration_boundary:

            # 셀 좌표를 맵 좌표로 변환

            x_map, y_map = self.cell_to_map_coordinates(x_cell, y_cell)

            # 유클리드 거리 계산
            print(self.robot_x, self.robot_y)

            distance = math.sqrt((self.robot_x - x_map)**2 + (self.robot_y - y_map)**2)

            if distance < min_distance:

                min_distance = distance

                closest_point = (x_cell, y_cell)

        

        return closest_point

        



    def navigate_to_goal(self, goal_x_cell, goal_y_cell):

        """셀 좌표를 맵 좌표로 변환 후 Nav2를 이용해 목표 지점으로 이동"""

        x_map, y_map = self.cell_to_map_coordinates(goal_x_cell, goal_y_cell)

        goal = PoseStamped()

        goal.header.frame_id = 'map'

        goal.pose.position.x = x_map

        goal.pose.position.y = y_map

        goal.pose.orientation.w = 1.0  # 기본 방향 설정

        self.nav_goal_publisher.publish(goal)

        self.get_logger().info(f"Navigating to map coordinates ({x_map}, {y_map})")



    def explore_map(self):

        """탐색 프로세스"""

        if self.current_map is None:

            self.get_logger().info("맵 데이터가 아직 없습니다.")

            return



        # 탐지 가능한 경계 찾기

        exploration_boundary = self.find_exploration_boundary()

        if exploration_boundary:

            # 가장 가까운 탐지 가능한 셀 좌표 선택

            closest_boundary = self.find_closest_boundary(exploration_boundary)

            if closest_boundary:

                goal_x_cell, goal_y_cell = closest_boundary

                goal_x_map, goal_y_map = self.cell_to_map_coordinates(goal_x_cell, goal_y_cell)



                # 목표 도달 확인

                if self.is_goal_reached(goal_x_map, goal_y_map):

                    self.get_logger().info("Goal reached. Searching for new boundary.")

                else:

                    self.navigate_to_goal(goal_x_cell, goal_y_cell)  # 맵 좌표로 변환 후 이동

            



        else:

            self.get_logger().info("No exploration boundaries found. Map state:")

            unique_values, counts = np.unique(self.current_map, return_counts=True)

            self.get_logger().info(f"Map data: {dict(zip(unique_values, counts))}")

            self.destroy_node()



    def is_goal_reached(self, goal_x_map, goal_y_map, threshold=0.1):

        """목표 좌표에 도달했는지 확인"""

        distance = math.sqrt((self.robot_x - goal_x_map)**2 + (self.robot_y - goal_y_map)**2)

        return distance <= threshold
    
    def is_within_inflation_radius(self, x_cell, y_cell):
        """Boundary 지점이 inflation 영역 내에 있는지 확인"""
        resolution = self.map_info.resolution
        x_map, y_map = self.cell_to_map_coordinates(x_cell, y_cell)

        # Local Costmap의 inflation_radius 반영
        inflation_radius = 0.25  # Local Costmap의 inflation_radius 값
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x_cell + dx, y_cell + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if self.current_map[ny, nx] != -1:  # 주위가 미탐지 영역이 아니면
                    distance = math.sqrt((x_map - self.robot_x)**2 + (y_map - self.robot_y)**2)
                    if distance < inflation_radius:  # Local Costmap 기준으로 필터링
                        return False
        return True



def main(args=None):

    rclpy.init(args=args)

    map_explorer = MapExplorer()

    rclpy.spin(map_explorer)

    rclpy.shutdown()



if __name__ == '__main__':

    main()
