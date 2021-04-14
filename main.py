# -*- coding: utf-8 -*-

import numpy as np
import math

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

NB_TOWNS = 6

starting_town: int
ending_town: int

best_solution = [None] * NB_TOWNS
best_eval = -1.0


coord = np.array([
    [565.0,  575.0],
    [25.0,  185.0],
    [345.0,  750.0],
    [945.0,  685.0],
    [845.0,  655.0],
    [880.0,  660.0]
])

dist = np.zeros((NB_TOWNS,NB_TOWNS))

#Builds final solution
def build_solution():
    pass


#Calculates distance matrix
def calculate_dist(coord):
    for i in range(NB_TOWNS):
        x1 = coord[i][0]
        y1 = coord[i][1]
        for j in range(NB_TOWNS):
            x2 = coord[j][0]
            y2 = coord[j][1]
            if i == j:
                dist[i][j] = -1
            else:
                dist[i][j] = math.sqrt(pow((x2-x1),2) + pow((y2-y1),2))



#Evaluation function (total distance)
def evaluation_solution(sol):

    eval = 0.0
    
    for i in range(NB_TOWNS-1):

        eval += dist[sol[i]][sol[i+1]]
    
    eval += dist[sol[NB_TOWNS-1]][sol[0]]

    return eval



#Builds a solution using the nearest neighbour heuristic
def build_nearest_neighbour():
    i: int
    sol = [None] * NB_TOWNS

    eval = 0.0

    sol[0] = 0
    
    #Temporary heuristic
    for i  in range(1,NB_TOWNS):
        sol[i] = i

    eval = evaluation_solution(sol)
    print("Nearest neighbour ", (sol, eval))

    for i in range(NB_TOWNS):
        best_solution[i] = sol[i]

    best_eval = eval


    return eval


def little_algorithm(dist, i, evalParentNode):
    pass




### Main ###

# Prints cities coordinates
print("\nCoordinates:\n")
for i in range(NB_TOWNS):
    print("Node ", i, " : " , coord[i])

#Calculates and prints distance matrix
calculate_dist(coord)
print("\nDistance matrix:\n")
print(dist)

#Building nearest neighbour
nearest_neighbour = build_nearest_neighbour()