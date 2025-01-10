
import math
import numpy as np
from rsoccer_gym.Entities import Robot
from utils.Point import Point
from utils.Geometry import Geometry


PROP_VELOCITY_MIN_FACTOR: float = 0.1
MAX_VELOCITY: float = 2
ANGLE_EPSILON: float = 0.1
ANGLE_KP: float = 5
MIN_DIST_TO_PROP_VELOCITY: float = 720

ADJUST_ANGLE_MIN_DIST: float = 50
M_TO_MM: float = 1000.0


class Navigation:

  @staticmethod
  def degrees_to_radians(degrees):
    return degrees * (math.pi / 180.0)
  
  @staticmethod
  def radians_to_degrees(radians):
    return radians * (180.0 / math.pi)
  
  @staticmethod
  def global_to_local_velocity(vx, vy, theta):
    vx_local = vx * math.cos(theta) + vy * math.sin(theta)
    vy_local = -vx * math.sin(theta) + vy * math.cos(theta)
    return Point(vx_local, vy_local)

  @staticmethod
  def map_value(value, lLower, lHigher, rLower, rHigher):
    if (lHigher - lLower) == 0:
      return
    
    return ((value - lLower) * (rHigher - rLower) / (lHigher - lLower) + rLower)
  
  @staticmethod
  def check_collision(robot: Robot, obstacles: dict[int, Robot], collision_distance: float) -> bool:
      """
      Verifica se o robô está prestes a colidir com outro robô.

      Args:
          robot (Robot): O robô atual.
          obstacles (dict[int, Robot]): Outros robôs no campo.
          collision_distance (float): Distância mínima para considerar uma colisão.

      Returns:
          bool: True se houver risco de colisão; False caso contrário.
      """
      robot_position = Point(robot.x, robot.y)
      closest_obstacle = None
      min_distance = collision_distance

      for _, obstacle in obstacles.items():
          obstacle_position = Point(obstacle.x, obstacle.y)
          distance = robot_position.dist_to(obstacle_position)
          if distance <= min_distance:
            closest_obstacle = obstacle 
      return closest_obstacle
  @staticmethod
  def obstacle_direction(robot: Robot, obstacle: Robot, target: Point) -> str:
      """
      Determina se o obstáculo está à esquerda ou à direita do robô em relação ao objetivo.

      Args:
          robot (Robot): O robô atual.
          obstacle (Robot): O obstáculo a ser analisado.
          target (Point): O objetivo do robô.

      Returns:
          str: "left" se o obstáculo estiver à esquerda, "right" se estiver à direita.
      """
      robot_position = Point(robot.x, robot.y)
      obstacle_position = Point(obstacle.x, obstacle.y)

      # Vetores do robô para o objetivo e para o obstáculo
      vector_to_target = Point(target.x - robot_position.x, target.y - robot_position.y)
      vector_to_obstacle = Point(obstacle_position.x - robot_position.x, obstacle_position.y - robot_position.y)

      # Produto vetorial para determinar a direção
      cross_product = vector_to_target.x * vector_to_obstacle.y - vector_to_target.y * vector_to_obstacle.x

      if cross_product > 0:
          return False
      else:
          return True  

  @staticmethod
  def goToPoint(robot: Robot, target: Point):
    target = Point(target.x * M_TO_MM, target.y * M_TO_MM)
    robot_position = Point(robot.x * M_TO_MM, robot.y * M_TO_MM)
    robot_angle = Navigation.degrees_to_radians(Geometry.normalize_angle(robot.theta, 0, 180))

    max_velocity = MAX_VELOCITY
    distance_to_target = robot_position.dist_to(target)
    kp = ANGLE_KP

    # Use proportional speed to decelerate when getting close to desired target
    proportional_velocity_factor = PROP_VELOCITY_MIN_FACTOR
    min_proportional_distance = MIN_DIST_TO_PROP_VELOCITY

    if distance_to_target <= min_proportional_distance:
      max_velocity = max_velocity * Navigation.map_value(distance_to_target, 0.0, min_proportional_distance, proportional_velocity_factor, 1.0)

    target_angle = (target - robot_position).angle()
    d_theta = Geometry.smallest_angle_diff(target_angle, robot_angle)

    if distance_to_target > ADJUST_ANGLE_MIN_DIST:
      v_angle = Geometry.abs_smallest_angle_diff(math.pi - ANGLE_EPSILON, d_theta)

      v_proportional = v_angle * (max_velocity / (math.pi - ANGLE_EPSILON))
      global_final_velocity = Geometry.from_polar(v_proportional, target_angle)
      target_velocity = Navigation.global_to_local_velocity(global_final_velocity.x, global_final_velocity.y, robot_angle)

      return target_velocity, -kp * d_theta
    else:
      return Point(0.0, 0.0), -kp * d_theta