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

        # 목표 타이머 설정
        self.goal_timeout_duration = 10.0  # 10초 후 타임아웃
        self.goal_timer = None

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
            return  # 맵이 바뀌지 않았으면 종료
        
        # 맵 데이터 업데이트
        self.get_logger().info("Map updated.")
        self.current_map = new_map
        self.map_width = map_data.info.width
        self.map_height = map_data.info.height
        self.map_info = map_data.info  # 맵 정보 저장

        # 맵이 업데이트되었을 때 경계값만 계산
        self.exploration_boundary = self.find_exploration_boundary()
        self.get_logger().info(f"Boundary updated with {len(self.exploration_boundary)} points.")

        # 현재 목표가 진행 중이면 추가 처리를 하지 않음
        if hasattr(self, 'current_goal_x') and hasattr(self, 'current_goal_y'):
            if not self.is_goal_reached(self.current_goal_x, self.current_goal_y):
                self.get_logger().info("Current goal in progress. Waiting for goal to complete.")
                return

        # 목표가 없으면 새로운 탐색 시작
        if not hasattr(self, 'current_goal_x') or self.current_goal_x is None:
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

                            if self.current_map[ny, nx]  ==0:

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
            if not self.is_within_inflation_radius(x_cell, y_cell):
                continue

            x_map, y_map = self.cell_to_map_coordinates(x_cell, y_cell)

            # 유클리드 거리 계산
            print(self.robot_x, self.robot_y)

            distance = math.sqrt((self.robot_x - x_map)**2 + (self.robot_y - y_map)**2)

            if distance < min_distance:

                min_distance = distance

                closest_point = (x_cell, y_cell)

        

        return closest_point
    
    '''
    def find_farthest_boundary(self, exploration_boundary):
        """탐지된 경계 중 로봇과 가장 먼 지점을 찾음"""
        farthest_point = None
        max_distance = -1

        for x_cell, y_cell in exploration_boundary:
            # 셀 좌표를 맵 좌표로 변환
            if not self.is_within_inflation_radius(x_cell, y_cell):
                continue

            x_map, y_map = self.cell_to_map_coordinates(x_cell, y_cell)
            
            # 유클리드 거리 계산
            distance = math.sqrt((self.robot_x - x_map)**2 + (self.robot_y - y_map)**2)
            if distance > max_distance:
                max_distance = distance
                farthest_point = (x_cell, y_cell)

        if farthest_point:
            self.get_logger().info(f"Farthest boundary: {farthest_point} at distance {max_distance:.2f}")
        else:
            self.get_logger().info("No valid farthest boundary found.")

        return farthest_point
        '''



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
        
        # 현재 목표 저장
        self.current_goal_x = goal_x_cell
        self.current_goal_y = goal_y_cell

        # 목표 타이머 설정
        if self.goal_timer:
            self.goal_timer.cancel()  # 기존 타이머 취소
        self.goal_timer = self.create_timer(self.goal_timeout_duration, self.retry_with_new_goal)

        
    def force_new_goal(self):
        """목표 강제 취소 및 새 목표 설정"""
        self.get_logger().info("Goal timeout reached. Forcing new exploration.")
        if hasattr(self, 'current_goal_x') and self.current_goal_x is not None:
            self.current_goal_x = None
            self.current_goal_y = None
        self.explore_map()  # 새로운 목표 설정

    def retry_with_new_goal(self):
        """10초 후 새로운 목표 설정"""
        self.get_logger().info("10 seconds elapsed. Retrying with a new goal.")
        self.current_goal_x = None
        self.current_goal_y = None
        self.explore_map()
        
    def explore_map(self):
        """목표를 설정하거나 탐색을 시작"""
        if self.current_map is None or not self.exploration_boundary:
            self.get_logger().info("No map data or boundary available for exploration.")
            return

        # 목표가 설정되어 있으면 중복 설정 방지
        if hasattr(self, 'current_goal_x') and self.current_goal_x is not None:
            self.get_logger().info("Current goal already set. Waiting for completion.")
            return

        # 가장 가까운 경계점 찾기
        closest_boundary = self.find_closest_boundary(self.exploration_boundary)
        if closest_boundary:
            goal_x_cell, goal_y_cell = closest_boundary
            goal_x_map, goal_y_map = self.cell_to_map_coordinates(goal_x_cell, goal_y_cell)

            # 목표 지점 저장
            self.current_goal_x = goal_x_map
            self.current_goal_y = goal_y_map

            # 목표로 이동
            self.navigate_to_goal(goal_x_cell, goal_y_cell)
        else:
            self.get_logger().info("No valid closest boundary found.")



    def is_goal_reached(self, goal_x_map, goal_y_map, threshold=0.1):
        """목표 도달 여부를 확인하고, 실패 조건을 처리"""
        distance = math.sqrt((self.robot_x - goal_x_map)**2 + (self.robot_y - goal_y_map)**2)
        self.get_logger().info(f"Checking goal: Target=({goal_x_map}, {goal_y_map}), Current=({self.robot_x}, {self.robot_y}), Distance={distance:.2f}")

        if distance <= threshold:
            self.get_logger().info(f"Goal ({goal_x_map}, {goal_y_map}) reached.")
            self.current_goal_x = None
            self.current_goal_y = None
            self.explore_map()  # 새로운 목표 설정
            return True
        elif self.path_planning_failed():
            self.get_logger().info("Path planning failed. Retrying with a new goal.")
            self.retry_with_new_goal()
            return False

        return False
    


    def path_planning_failed(self):
        """경로 계획 실패 조건 확인 (예: 상태, 주변 장애물)"""
        # 실제 실패 조건 감지 코드 필요 (예: costmap, 로봇 상태, GoalStatus)
        return False  # 상태 메시지에서 실패 조건을 확인해야 함

    
    def is_within_inflation_radius(self, x_cell, y_cell):
        """Boundary 지점이 inflation 영역 내에 있는지 확인"""
        resolution = self.map_info.resolution
        inflation_radius_cells = int(0.4 / resolution)  # inflation_radius를 셀 단위로 변환

        for dx in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for dy in range(-inflation_radius_cells, inflation_radius_cells + 1):
                nx, ny = x_cell + dx, y_cell + dy

                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    if self.current_map[ny, nx] == 100:  # 장애물
                        self.get_logger().debug(f"Cell ({x_cell}, {y_cell}) is too close to obstacle at ({nx}, {ny}).")
                        return False
        return True
    

    def retry_with_new_goal(self):
        if self.exploration_boundary:
            # 이전 실패 목표를 제외하고 새로운 경계점 찾기
            new_boundary = [point for point in self.exploration_boundary 
                            if point != (self.current_goal_x, self.current_goal_y)]
            
            if not new_boundary:
                self.get_logger().info("No valid boundaries left for exploration.")
                return

            # 새로운 목표 찾기 (예: 가장 먼 경계점)
            new_goal = self.find_closest_boundary(new_boundary)
            if new_goal:
                self.get_logger().info(f"Retrying with a new goal: {new_goal}")
                goal_x_cell, goal_y_cell = new_goal
                self.current_goal_x, self.current_goal_y = self.cell_to_map_coordinates(goal_x_cell, goal_y_cell)
                self.navigate_to_goal(goal_x_cell, goal_y_cell)
        else:
            self.get_logger().info("Exploration boundaries are empty. Waiting for map update.")




def main(args=None):

    rclpy.init(args=args)

    map_explorer = MapExplorer()

    rclpy.spin(map_explorer)

    rclpy.shutdown()



if __name__ == '__main__':

    main()
