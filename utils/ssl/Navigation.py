
import math
import numpy as np
from rsoccer_gym.Entities import Robot
from utils.Point import Point
from utils.Geometry import Geometry


PROP_VELOCITY_MIN_FACTOR: float = 0.15
MAX_VELOCITY: float = 1.0
ANGLE_EPSILON: float = 0.1
ANGLE_KP: float = 3
MIN_DIST_TO_PROP_VELOCITY: float = 800

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
  def goToPoint(robot: Robot, target: Point, obstacles: dict[int, Robot] = None, influence_radius: float = 1.0):
    """
    Navegação para o ponto alvo com desvio de obstáculos e controle de velocidade.

    Args:
        robot (Robot): O robô atual.
        target (Point): O objetivo.
        obstacles (dict[int, Robot], opcional): Obstáculos no ambiente.
        influence_radius (float): Raio de influência para desvio de obstáculos.

    Returns:
        Tuple[Point, float]: Velocidade translacional e velocidade angular.
    """
    # Converter posições do robô e do alvo para milímetros
    target_mm = Point(target.x * M_TO_MM, target.y * M_TO_MM)
    robot_position_mm = Point(robot.x * M_TO_MM, robot.y * M_TO_MM)
    robot_angle_rad = Navigation.degrees_to_radians(Geometry.normalize_angle(robot.theta, 0, 180))

    # Velocidade máxima inicial
    max_velocity = MAX_VELOCITY

    # Distância até o objetivo
    distance_to_target = robot_position_mm.dist_to(target_mm)

    # Controle proporcional para velocidade perto do alvo
    proportional_velocity_factor = PROP_VELOCITY_MIN_FACTOR
    min_proportional_distance = MIN_DIST_TO_PROP_VELOCITY
    if distance_to_target <= min_proportional_distance:
        max_velocity *= Navigation.map_value(
            distance_to_target, 0.0, min_proportional_distance, proportional_velocity_factor, 1.0
        )
    
    

    # Vetor direção para o alvo
    direction = target_mm - robot_position_mm

    # Se houver obstáculos, ajuste a direção com campos potenciais
    if obstacles:
        #direction = Navigation.potential_field_navegation(robot, target, obstacles)
        repulsion_force = Point(0, 0)
        for _, obstacle in obstacles.items():
            obstacle_position = Point(obstacle.x * M_TO_MM, obstacle.y * M_TO_MM)
            distance_to_obstacle = robot_position_mm.dist_to(obstacle_position)
            
            # Aplicar força repulsiva apenas se dentro do raio de influência
            if distance_to_obstacle < influence_radius * M_TO_MM and distance_to_obstacle > 0:
                repulsion = robot_position_mm - obstacle_position
                repulsion = repulsion / (distance_to_obstacle ** 2)  # Decresce com o quadrado da distância
                repulsion_force += repulsion

        # Combinar forças de atração e repulsão
        direction += repulsion_force
        if direction.length() > 0:
            direction = direction / direction.length()  # Normalizar direção

    # Calcular ângulo para o alvo ajustado
    target_angle = direction.angle()
    d_theta = Geometry.smallest_angle_diff(target_angle, robot_angle_rad)

    # Controle de ângulo e velocidade
    if distance_to_target > ADJUST_ANGLE_MIN_DIST:
        v_angle = Geometry.abs_smallest_angle_diff(math.pi - ANGLE_EPSILON, d_theta)
        v_proportional = v_angle * (max_velocity / (math.pi - ANGLE_EPSILON))
        global_final_velocity = Geometry.from_polar(v_proportional, target_angle)
        target_velocity = Navigation.global_to_local_velocity(global_final_velocity.x, global_final_velocity.y, robot_angle_rad)

        return target_velocity, -ANGLE_KP * d_theta
    else:
        return Point(0.0, 0.0), -ANGLE_KP * d_theta