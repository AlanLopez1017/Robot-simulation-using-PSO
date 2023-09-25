import pygame
import math
import numpy as np
import spritesheet

def distance(x,y):
    return np.linalg.norm(np.array(x)-np.array(y))

class Ground:
    def __init__(self, dimentions, robot_img, map_img):
        pygame.init()

        # terreno
       # self.robot = pygame.image.load(robot_img)
        self.map_img = pygame.image.load(map_img)
        # dimensiones
        self.height, self.width = dimentions

        # configuración de la ventana
        pygame.display.set_caption("Map")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0,0))

       # rect = self.robot.get_rect()
      #  self.map.blit(self.robot, rect)

   # def draw_robot(self, x, y, theta):
   #     rotated = pygame.transform.rotozoom(self.robot, math.degrees(theta), 1) # math.degrees(np.pi)
   #     rect = rotated.get_rect(center = (x,y))
   #     self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, (0,255,0), point, 3, 0)

class Obstacle:
    def __init__(self, image):
        self.image = pygame.image.load(image)
        self.rect = self.image.get_rect()
        self.mask = pygame.mask.from_surface(self.image)
        
    def draw(self, gnd):
        gnd.blit(self.image, self.rect)
      

class Robot(pygame.sprite.Sprite):
    def __init__(self, image, startPos, width):
        super().__init__()
        self.w = width
        self.x = startPos[0]
        self.y = startPos[1]
        self.theta = 0
        self.direction = np.array([-0.00001,1])
        self.v_x = 77.43 # lateral (cm/s)
        self.v_y = 68 # 68 frontal (cm/s)
        self.min_obs_dist = 70 # pixeles 70

        self.image = pygame.transform.rotate(pygame.image.load(image).convert_alpha(), 0)
        self.rect = self.image.get_rect()
        self.mask = pygame.mask.from_surface(self.image)

    def avoid_obstacles(self, point_cloud, alpha):
        closest_obs = None
        dist = np.inf

        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = dist

            if closest_obs < self.min_obs_dist:
                self.obstacles = np.sum(np.array(point_cloud), axis = 0)/len(point_cloud)
                self.direction = -(self.obstacles - np.array([self.x, self.y])) #np.dot(np.array([[np.cos(alpha), -np.sin(alpha)],[np.sin(alpha), np.cos(alpha)]]),-(self.obstacles - np.array([self.x, self.y])))
                #self.theta = np.arctan(abs(self.direction[1]/self.direction[0]))*180/np.pi
    
    def kinematics(self, alpha, dt):
        
        #self.v_x = -(self.v_x*math.cos(alpha)-self.v_y*math.sin(alpha))
        #self.v_y = (self.v_x*math.sin(alpha)+self.v_y*math.cos(alpha))

        self.x += (self.direction[0]/abs(self.direction[0]))*(self.v_x)*dt#(self.direction[0]/abs(self.direction[0]))*(self.v)*math.cos(math.radians(self.theta))*dt
        self.y += (self.direction[1]/abs(self.direction[1]))*(self.v_y)*dt#(self.direction[1]/abs(self.direction[1]))*(self.v)*math.sin(math.radians(self.theta))*dt

    def draw(self, x, y, theta, gnd):
        #rotated = pygame.transform.rotozoom(self.image, math.degrees(theta), 1) # math.degrees(np.pi)
        rotated = pygame.transform.rotate(self.image, math.degrees(theta))
        self.mask = pygame.mask.from_surface(rotated)
        self.rect = rotated.get_rect(center = (x,y))
        gnd.blit(rotated, self.rect)
        

class Wall(pygame.sprite.Sprite):
    def __init__(self, image, x, y):
        super().__init__()
        self.image = pygame.image.load(image).convert_alpha()
        #self.image.set_colorkey("black")
        self.rect = self.image.get_rect()
        self.rect.x, self.rect.y = x, y
        self.mask = pygame.mask.from_surface(self.image)

    def draw(self, gnd):
        gnd.blit(self.image, self.rect)


class Sensor:
    def __init__(self, sensor_range, py_map, sprites):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = py_map
        self.sprites = sprites
        self.pixel_mask = pygame.mask.from_surface(pygame.Surface((1,1)))
        
    def sense_obstacles(self, x, y, pos, sensor_angle):
        obstacles = []
        x1, y1 = x+pos[0]*np.cos(math.radians(sensor_angle)), y+pos[1]*np.sin(math.radians(sensor_angle))
        start_angle = math.radians(sensor_angle) - self.sensor_range[1]
        end_angle = math.radians(sensor_angle) + self.sensor_range[1]
        points = 20 #20
        for angle in np.linspace(start_angle, end_angle, 10, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range(0,points):
                u = i / points
                x = int(x2*u + x1*(1-u))
                y = int(y2*u + y1*(1-u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    #color = self.map.get_at((x,y))
                    self.map.set_at((x,y), (255,0, 0))

                    for sprite in self.sprites:
                        if sprite.mask.overlap(self.pixel_mask, (x-sprite.rect.x, y-sprite.rect.y)): #sprite.rect.collidepoint(x,y): sprite.mask.overlap(self.pixel_mask, (x-sprite.rect.x, y-sprite.rect.y))
                            obstacles.append([x,y])
                            break

                    #if (((color[0], color[1], color[2]) == (0,0,0))):
                    #    obstacles.append([x,y])
                    #    break
        return obstacles

