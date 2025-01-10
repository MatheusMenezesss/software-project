from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import math

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.is_avoiding = False
        self.avoid_target = None
        self.recent_obstacle = None  # Para evitar se concentrar no mesmo obstáculo

    def decision(self):
        if len(self.targets) == 0:
            return

        collision_distance = 0.45  # Distância mínima de colisão (em metros)
        avoid_distance = 0.4  # Distância para desviar ao redor do obstáculo

        # Detectar o obstáculo mais próximo
        closest_obstacle = Navigation.check_collision(self.robot, self.opponents, collision_distance)

        # Lógica de desvio
        if self.is_avoiding and self.avoid_target:
            # Verificar se o ponto de desvio foi alcançado
            if self.pos.dist_to(self.avoid_target) < 0.1:  # Ponto alcançado
                self.is_avoiding = False
                self.avoid_target = None
                self.recent_obstacle = None
                return

            # Continuar movimentando para o ponto de desvio
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.avoid_target)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
            return

        # Se há um obstáculo à frente e o agente não está desviando
        if closest_obstacle and closest_obstacle != self.recent_obstacle:
            self.is_avoiding = True
            self.recent_obstacle = closest_obstacle
            obstacle_pos = Point(closest_obstacle.x, closest_obstacle.y)
            robot_pos = Point(self.robot.x, self.robot.y)

            # Determinar direção do desvio (esquerda ou direita)
            direction = Navigation.obstacle_direction(self.robot, closest_obstacle, self.targets[0])
            if not direction:  # Esquerda
                avoid_direction_1 = (obstacle_pos - robot_pos).angle() - math.radians(45)
                avoid_direction_2 = (obstacle_pos - robot_pos).angle() - math.radians(90)
            else:  # Direita
                avoid_direction_1 = (obstacle_pos - robot_pos).angle() + math.radians(45)
                avoid_direction_2 = (obstacle_pos - robot_pos).angle() + math.radians(90)

            # Definir dois pontos para contorno do obstáculo
            avoid_point_1 = Point(
                robot_pos.x + math.cos(avoid_direction_1) * avoid_distance,
                robot_pos.y + math.sin(avoid_direction_1) * avoid_distance
            )
            avoid_point_2 = Point(
                robot_pos.x + math.cos(avoid_direction_2) * avoid_distance,
                robot_pos.y + math.sin(avoid_direction_2) * avoid_distance
            )

            # Escolher o ponto mais próximo ao objetivo
            target_pos = Point(self.targets[0].x, self.targets[0].y)
            if target_pos.dist_to(avoid_point_1) < target_pos.dist_to(avoid_point_2):
                self.avoid_target = avoid_point_1
            else:
                self.avoid_target = avoid_point_2

            return

        # Movimentação normal para o objetivo
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)
        return

    def post_decision(self):
        pass
