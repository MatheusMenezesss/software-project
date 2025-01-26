
import math
import numpy as np
from rsoccer_gym.Entities import Robot
from utils.Point import Point
from utils.Geometry import Geometry
from queue import PriorityQueue


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
      
      """
      
         aplicando o algoritmo A*, vai ser respondavel pela gestão do caminho globalmente.
         o compo potencial será responsavel pelo 
      """
  
  @staticmethod
  def a_star_search(start: Point, goal: Point, obstacles: list[Point], grid_size: tuple[int, int], resolution: float):
      """
        Implementação do algoritmo A* para encontrar o caminho mais curto.

        Args:
            start (Point): Ponto inicial.
            goal (Point): Ponto objetivo.
            obstacles (list[Point]): Lista de obstáculos.
            grid_size (tuple[int, int]): Tamanho da área de navegação (em células).
            resolution (float): Tamanho de cada célula em metros.

        Returns:
            list[Point]: Lista de pontos no caminho mais curto.
        """
        # Função de heurística (distância euclidiana)
      def heuristic(a: Point, b: Point):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

        # Inicialização de estruturas de dados
      open_set = PriorityQueue()
      open_set.put((0, start))
      came_from = {}
      g_score = {start: 0}
      f_score = {start: heuristic(start, goal)}

        # Direções permitidas (8 movimentos possíveis)
      directions = [
          Point(1, 0), Point(-1, 0), Point(0, 1), Point(0, -1),
          Point(1, 1), Point(-1, -1), Point(1, -1), Point(-1, 1)
      ]

      while not open_set.empty():
          _, current = open_set.get()

            # Verifica se o objetivo foi alcançado
          if current == goal:
              path = []
              while current in came_from:
                  path.append(current)
                  current = came_from[current]
              path.reverse()
              return path

          for direction in directions:
              neighbor = Point(
                  current.x + direction.x * resolution,
                  current.y + direction.y * resolution
              )

                # Verificar se está fora dos limites ou colidindo com obstáculos
              if (
                  neighbor.x < 0 or neighbor.x >= grid_size[0] or
                  neighbor.y < 0 or neighbor.y >= grid_size[1] or
                  any(neighbor.dist_to(obstacle) < resolution for obstacle in obstacles)
              ):
                  continue

              tentative_g_score = g_score[current] + heuristic(current, neighbor)

              if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                  came_from[neighbor] = current
                  g_score[neighbor] = tentative_g_score
                  f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                  open_set.put((f_score[neighbor], neighbor))

      return []  # Retorna vazio se não houver caminh
  
  @staticmethod
  def goToPoint(robot: Robot, target: Point, obstacles: dict[int, Robot] = None, influence_radius: float = 1.0):
    """
    Navegação para um ponto, ajustando a rota dinamicamente para evitar obstáculos.

    Args:
        robot (Robot): O robô atual.
        target (Point): O ponto objetivo para onde o robô deve ir.
        obstacles (dict[int, Robot], opcional): Obstáculos a serem evitados.
        influence_radius (float): Raio de influência dos obstáculos (em metros).
    """
    if obstacles is None:
        obstacles = {}

    # Configurações do campo
    repulsive_force_factor = 1.5  # Intensidade da força repulsiva dos obstáculos
    attractive_force_factor = 1.0  # Intensidade da força atrativa ao objetivo
    field_resolution = 0.01  # Resolução do campo em metros

    # Converte posição do robô e do objetivo para milímetros
    robot_pos = Point(robot.x * M_TO_MM, robot.y * M_TO_MM)
    target_pos = Point(target.x * M_TO_MM, target.y * M_TO_MM)

    # Calcula a força atrativa (direção para o objetivo)
    attractive_force = Point(
        target_pos.x - robot_pos.x,
        target_pos.y - robot_pos.y
    )

    # Normaliza a força atrativa
    attractive_force = attractive_force.normalize() * attractive_force_factor

    # Calcula forças repulsivas para evitar colisões
    repulsive_force = Point(0, 0)
    for _, obstacle in obstacles.items():
        obstacle_pos = Point(obstacle.x * M_TO_MM, obstacle.y * M_TO_MM)
        distance = robot_pos.dist_to(obstacle_pos)

        if distance < influence_radius * M_TO_MM:  # Dentro do raio de influência
            direction_away = Point(
                robot_pos.x - obstacle_pos.x,
                robot_pos.y - obstacle_pos.y
            ).normalize()

            # Intensidade da força repulsiva diminui com a distância
            intensity = repulsive_force_factor / (distance + 1e-6)
            repulsive_force += direction_away * intensity

    # Combina as forças atrativa e repulsiva
    resultant_force = attractive_force + repulsive_force

    # Converte a força resultante para velocidades
    velocity_x = resultant_force.x / M_TO_MM
    velocity_y = resultant_force.y / M_TO_MM

    # Limita as velocidades máximas
    velocity_magnitude = math.sqrt(velocity_x**2 + velocity_y**2)
    if velocity_magnitude > MAX_VELOCITY:
        velocity_x = (velocity_x / velocity_magnitude) * MAX_VELOCITY
        velocity_y = (velocity_y / velocity_magnitude) * MAX_VELOCITY

    # Converte velocidades globais para o referencial local do robô
    local_velocity = Navigation.global_to_local_velocity(velocity_x, velocity_y, robot.theta)

    # Gira o robô se necessário
    angle_to_target = math.atan2(target_pos.y - robot_pos.y, target_pos.x - robot_pos.x)
    angle_error = angle_to_target - robot.theta

    # Ajusta a velocidade angular com base no erro angular
    angular_velocity = ANGLE_KP * angle_error
    if abs(angle_error) < ANGLE_EPSILON:
        angular_velocity = 0  # Para de girar se já estiver alinhado

    # Retorna as velocidades controladas
    return local_velocity.x, local_velocity.y, angular_velocity