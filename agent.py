from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
import math
from utils.Point import Point

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def point_to_line_distance(self, line_start, line_end, point):
        #calcula a distância de um ponto a uma linha
        x0, y0 = point.x, point.y
        x1, y1 = line_start.x, line_start.y
        x2, y2 = line_end.x, line_end.y
        #fóirmula da distancia ponto-linha
        numerator = abs((y2 - y1)*x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        denominator  = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
        return numerator / denominator if denominator != 0 else float("inf")
    
    def has_obstacle_in_path(self, target_point):
        self.obstacles = {**self.opponents, **self.teammates}
        #vê se obstacles no caminho
        for obstacle_id, obstacle in self.obstacles.items():
            #observa o obstacle como um ponto
            obstacle_position = Point(obstacle.x, obstacle.y)
            #calcula a distância do obstáculo para a linha do trajeto
            distance = self.point_to_line_distance(
                Point(self.robot.x, self.robot.y),
                target_point,
                obstacle_position
            )
        #verifica se obstaculo ta proximo o suficiente do trajeto
        if distance < 0.18: #raio de segurança pra evitarmos colisões
            return True
    
        return False

    def decision(self):
        if len(self.targets) == 0:
            return
        
        #pt de destino
        target_point = self.targets[0]
        #checa por obstáculos no caminho
        if self.has_obstacle_in_path(target_point):
            print("Obstáculo detectado no caminho!")
            #se for encontrado, pararemos o robô
            self.set_vel(Point(0, 0))
            self.set_angle_vel(0)
        else: 
            #movimenta-se para o destino se não houver obstáculos
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
    
    def post_decision(self):
        pass