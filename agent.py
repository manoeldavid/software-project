import math
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    @staticmethod
    def normalize_vector(vector):
        magnitude = math.sqrt(vector.x**2 + vector.y**2)
        if magnitude == 0:
            return Point(0, 0)
        return Point(vector.x / magnitude, vector.y / magnitude)

    def distance_to(self, point):
        dx = self.robot.x - point.x
        dy = self.robot.y - point.y
        return math.sqrt(dx**2 + dy**2)

    def get_obstacles(self):
        obstacles = {**self.opponents, **self.teammates}
        obstacles.pop(self.id, None)  # Remove o próprio robô dos obstáculos
        return obstacles
    
    def avoid_obstacle(self, obstacle, target):
        direction_vector = Point(target.x - obstacle.x, target.y - obstacle.y)
        perpendicular_vector = Point(-direction_vector.y, direction_vector.x)
        perpendicular_vector = ExampleAgent.normalize_vector(perpendicular_vector) * 0.5
        return Point(obstacle.x + perpendicular_vector.x, obstacle.y + perpendicular_vector.y)

    def point_to_line_distance(self, line_start, line_end, point):
        x0, y0 = point.x, point.y
        x1, y1 = line_start.x, line_start.y
        x2, y2 = line_end.x, line_end.y
        numerator = abs((y2 - y1)*x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        return numerator / denominator if denominator != 0 else float("inf")

    def decision(self):
        if len(self.targets) == 0:
            return
        
        # Ponto de destino
        target_point = self.targets[0]
        distance_to_target = self.distance_to(target_point)  # Chamada do método corrigida
        velocidade = 0.5 if distance_to_target > 1.0 else 0.3

        # Obtendo obstáculos ao redor
        obstacles = self.get_obstacles()
        for obstacle in obstacles.values():
            if self.distance_to(Point(obstacle.x, obstacle.y)) < 0.35:  #0,5 muito alto, se ela chega tm perto do obstaculo ele para
                # Calcular um ponto de desvio
                avoidance_point = self.avoid_obstacle(obstacle, target_point)
                target_point = avoidance_point #faz o desvio pra depois voltar ao destino principal  
                break

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, target_point)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)  
    
    def post_decision(self):
        pass
