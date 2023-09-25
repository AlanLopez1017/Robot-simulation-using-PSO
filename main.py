import math
import pygame
from robot import Ground
import numpy as np
from robot import *

MAP_DIMENSIONS = (600,1200) # (10.5,20.1) metros
ROBOT = "solid_80.png"#"meteor.png"
BG = "white.png"
WALLS = ["upper_wall.png", "side_wall.png"]
# Define el mapa a recorrer
gnd = Ground(MAP_DIMENSIONS, ROBOT, BG) #robott.png, mapa1_red.png

# Instancias de las paredes y robots

walls = [Wall(WALLS[0], 0, 0), Wall(WALLS[0], 0, 550), Wall(WALLS[1], 0, 50), Wall(WALLS[1], 1150, 50)]

robot_position = [(200,200), (200,400), (600,300), (1000,200), (1000,400)] # (300,400), (400,400), (800,350), (400,400), (500,400)
number_of_robots = 5
robots = []
for n in range(number_of_robots):
    robots.append(Robot(ROBOT, robot_position[n], 0.01*3779.52))

#gnd.map.blit(gnd.map_img, (0,0))
#for w in walls:
    #wall_group.add(w)
#    w.draw(gnd.map)

obstacles = walls+robots
obstacles_group = pygame.sprite.Group()
for obstacle in obstacles:
    obstacles_group.add(obstacle)

# Define las características del sensor
sensor_range = 70, math.radians(10)  # 70,10
sensor1 = Sensor(sensor_range, gnd.map, obstacles_group)
sensor2 = Sensor(sensor_range, gnd.map, obstacles_group)
sensor3 = Sensor(sensor_range, gnd.map, obstacles_group)


'''
SENSADO

pixels = paredes (fijos) + robots (variantes)
obtener pixeles de puntos rojos
si esos pixeles están dentro de la lista pixeles -> añadir a point_cloud

'''

dt = 0
last_time = pygame.time.get_ticks()
running = True

theta = math.radians(0)
g = 0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = (pygame.time.get_ticks()-last_time)/1000
    last_time = pygame.time.get_ticks()

    gnd.map.blit(gnd.map_img, (0,0))
    for w in walls:
        w.draw(gnd.map)

    for n in range(number_of_robots):
        robots[n].kinematics(theta+math.radians(g), dt)
        robots[n].draw(robots[n].x, robots[n].y, theta+math.radians(g), gnd.map)
        obstacles_group.remove(robots[n])
        if pygame.sprite.spritecollide(robots[n], obstacles_group, False, pygame.sprite.collide_mask): # and robots[n] != obstacle
            pygame.draw.rect(gnd.map, (255,0,0), robots[n].rect, 2)
        else:
            #pygame.draw.rect(gnd.map, (0,255,0), robots[n].rect, 2)
            pass
        obstacles_group.add(robots[n])
        point_cloud1 = sensor1.sense_obstacles(robots[n].x, robots[n].y, (35,-35), 0+math.degrees(theta)+g) #35,-10, 0°-> movidos 50,-18 ...(center[0]+38)*math.cos(theta)-(center[1])*math.sin(theta), -((center[0]+38)*math.sin(theta)+(center[1])*math.cos(theta))
        point_cloud2 = sensor2.sense_obstacles(robots[n].x, robots[n].y, (24,-24), 120+math.degrees(theta)+g) #10,-25, 120°
        point_cloud3 = sensor3.sense_obstacles(robots[n].x, robots[n].y, (24,-24), 240+math.degrees(theta)+g) #10,-25, 240° -> movidos 32,-12
        point_cloud = point_cloud1 + point_cloud2 + point_cloud3
        robots[n].avoid_obstacles(point_cloud, theta+math.radians(g))
        gnd.draw_sensor_data(point_cloud)
        g += 0.5

    pygame.display.update()
    