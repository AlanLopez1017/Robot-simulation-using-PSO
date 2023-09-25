import numpy as np
import math
import matplotlib.pyplot as plt
import random

def costFunction(X):
    z = np.sum(np.sqrt(X))
    return z

bounds = [(0,600), (0,600)] # límite inferior y superior de las variables

# parameters
epochs = 3000
particle_size = 50


# PSO

class Particle():
    def __init__(self, bounds):
        self.particle_position = np.random.uniform(bounds[0][0], bounds[0][1], 2) # posición de la partícula
        self.particle_velocity = np.random.uniform(-1,1,2) # velocidad de la partícula
        self.particle_pBest = np.copy(self.particle_position)
        self.particle_pBest_cost = -1

    def evaluate(self, costFunc):
        self.particle_cost = costFunc(self.particle_position)

        if self.particle_cost < self.particle_pBest_cost or self.particle_pBest_cost == -1:
            self.particle_pBest = np.copy(self.particle_position)
            self.particle_pBest_cost = np.copy(self.particle_cost)

    def update_velocity(self, gBest):
        w = 0.75 # constante de inercia
        c1 = 2 # constante cognitiva
        c2 = 2 # constante social
        r1 = np.random.rand(2)
        r2 = np.random.rand(2)

        self.particle_velocity = w*self.particle_velocity + c1*r1*(self.particle_pBest-self.particle_position) + c2*r2*(gBest-self.particle_position)

    def update_position(self, bounds):
        self.particle_position = self.particle_position + self.particle_velocity

        # ajustar los límites superior e inferior
        if self.particle_position[0] < bounds[0][0]:
            self.particle_position[0] = bounds[0][0]
        if self.particle_position[1] > bounds[0][1]:
            self.particle_position[1] = bounds[0][1]

class PSO():
    def __init__(self, costFunc, bounds, particle_size, epochs):
        swarm = [Particle(bounds) for i in range(particle_size)]

        

    