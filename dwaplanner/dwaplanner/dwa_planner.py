''' 
This Dwa Planner is devloped by Prudhvi Raj Chalapak
Name: Prudhvi Raj Chalapaka
Contact: prudhvirajchalapaka@gmail.com
 '''

import rclpy
import math
import numpy as np
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

goal_x = None
goal_y = None
odom_data = None
scan_data = None
goal_reached = False
MAX_SPEED = 0.22
MAX_TURN = 1.0
STEP_TIME = 0.1
PREDICT_TIME = 2.0
NUM_SAMPLES = 500
GOAL_COST_GAIN = 1.0
SPEED_COST_GAIN = 0.1
OBSTACLE_COST_GAIN = 1.5
ROBOT_RADIUS = 0.25

def ask_for_goal():
    global goal_x, goal_y, goal_reached
    try:
        goal_x = float(input("Enter goal X: "))
        goal_y = float(input("Enter goal Y: "))
        goal_reached = False
        print(f"New goal set to ({goal_x}, {goal_y})")
    except ValueError:
        print("Invalid input. Please enter a number.")
        ask_for_goal()

def odom_callback(msg):
    """Updates global odometry data."""
    global odom_data
    odom_data = msg

def scan_callback(msg):
    """Updates global laser scan data."""
    global scan_data
    scan_data = msg

def predict_motion(start_x, start_y, start_yaw, speed, turn_rate):
    """Simulates a trajectory from a starting pose given a speed and turn rate."""
    path = []
    current_yaw = start_yaw
    current_x = start_x
    current_y = start_y
    num_steps = int(PREDICT_TIME / STEP_TIME)
    for i in range(num_steps):
        x = speed * math.cos(current_yaw) * STEP_TIME
        y = speed * math.sin(current_yaw) * STEP_TIME
        current_x += x
        current_y += y
        current_yaw += turn_rate * STEP_TIME
        path.append((current_x, current_y))
    return path

def calculate_obstacle_cost(path, robot_radius):
    """Calculates the cost of a trajectory based on its proximity to obstacles."""
    if scan_data is None:
        return 0.0
    min_dist_to_obstacle = float('inf')
    obstacle_points = []
    for i, scan_range in enumerate(scan_data.ranges):
        if np.isfinite(scan_range):
            angle = scan_data.angle_min + i * scan_data.angle_increment
            obs_x = scan_range * math.cos(angle)
            obs_y = scan_range * math.sin(angle)
            obstacle_points.append((obs_x, obs_y))
    if not obstacle_points:
        return 0.0
    for path_x, path_y in path:
        for obs_x, obs_y in obstacle_points:
            dist = math.hypot(path_x - obs_x, path_y - obs_y)
            if dist < min_dist_to_obstacle:
                min_dist_to_obstacle = dist
    if min_dist_to_obstacle <= robot_radius:
        return float('inf')
    return 1.0 / min_dist_to_obstacle

def choose_best_path(node, possible_paths):
    """Chooses the best speed and turn rate from a list of possible paths."""
    global goal_reached
    if odom_data is None or goal_x is None:
        return 0.0, 0.0
    current_x = odom_data.pose.pose.position.x
    current_y = odom_data.pose.pose.position.y
    orient = odom_data.pose.pose.orientation
    _, _, current_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    distance_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)
    if distance_to_goal < 0.2:
        if not goal_reached:
            node.get_logger().info(f"Goal reached at ({goal_x}, {goal_y})!")
            goal_reached = True
        return 0.0, 0.0
    best_score = -float('inf')
    best_speed, best_turn = 0.0, 0.0
    for speed, turn, path in possible_paths:
        if not path: continue
        end_path_x_local, end_path_y_local = path[-1]
        end_path_x_world = current_x + end_path_x_local * math.cos(current_yaw) - end_path_y_local * math.sin(current_yaw)
        end_path_y_world = current_y + end_path_x_local * math.sin(current_yaw) + end_path_y_local * math.cos(current_yaw)
        goal_dist_cost = math.hypot(goal_x - end_path_x_world, goal_y - end_path_y_world)
        speed_cost = MAX_SPEED - speed
        obstacle_cost = calculate_obstacle_cost(path, ROBOT_RADIUS)
        if obstacle_cost == float('inf'):
            continue
        total_score = - (GOAL_COST_GAIN * goal_dist_cost +
                         SPEED_COST_GAIN * speed_cost +
                         OBSTACLE_COST_GAIN * obstacle_cost)
        if total_score > best_score:
            best_score = total_score
            best_speed, best_turn = speed, turn
    if best_score == -float('inf'):
        node.get_logger().warn("No valid path found! Stopping.")
        return 0.0, 0.0
    return best_speed, best_turn

def generate_paths():
    """Generates a list of random sample paths."""
    possible_paths = []
    for _ in range(NUM_SAMPLES):
        speed = random.uniform(0.0, MAX_SPEED)
        turn = random.uniform(-MAX_TURN, MAX_TURN)
        path = predict_motion(0.0, 0.0, 0.0, speed, turn)
        possible_paths.append((speed, turn, path))
    return possible_paths

def movement_loop(node, cmd_publisher, path_publisher):
    """Main control loop with corrected visualization."""
    if odom_data is None or scan_data is None or goal_reached or goal_x is None:
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        cmd_publisher.publish(move_cmd)
        return

    possible_paths = generate_paths()
    speed, turn = choose_best_path(node, possible_paths)
    move_cmd = Twist()
    move_cmd.linear.x = speed
    move_cmd.angular.z = turn
    cmd_publisher.publish(move_cmd)

    marker = Marker()
    marker.header.frame_id = "base_link" 
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "dwa_paths"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.8

    marker.points = []
    for _, _, path in possible_paths:
        for i in range(len(path) - 1):
            p1 = Point()
            p1.x = path[i][0]
            p1.y = path[i][1]
            p1.z = 0.0
            marker.points.append(p1)

            p2 = Point()
            p2.x = path[i+1][0]
            p2.y = path[i+1][1]
            p2.z = 0.0
            marker.points.append(p2)
            
    path_publisher.publish(marker)

def main():
    rclpy.init()
    node = Node('dwa_planner_functional')
    ask_for_goal()

    node.create_subscription(Odometry, '/odom', odom_callback, 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    cmd_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    path_publisher = node.create_publisher(Marker, '/visual_paths', 10)
    timer_period = STEP_TIME
    node.create_timer(timer_period, lambda: movement_loop(node, cmd_publisher, path_publisher))

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
