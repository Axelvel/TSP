# -*- coding: utf-8 -*-

import numpy as np
import math
import copy
import time
import os

start_time = time.time()
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

NB_TOWNS = 10

starting_town = [None] * NB_TOWNS
ending_town = [None] * NB_TOWNS

best_solution = [None] * NB_TOWNS
best_eval = -1.0

count = 0

coord = np.empty((NB_TOWNS, 2))

"""
coord = np.array([
    [565.0,  575.0],
    [25.0,  185.0],
    [345.0,  750.0],
    [945.0,  685.0],
    [845.0,  655.0],
    [880.0,  660.0], 
    [25.0,  230.0],
    [525.0,  1000.0],
    [580.0,  1175.0],
    [650.0,  1130.0],

])"""

dist = np.zeros((NB_TOWNS,NB_TOWNS))




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



#Builds a solution using the next neighbour heuristic
def build_next_neighbour():
    global best_eval
    sol = [None] * NB_TOWNS

    eval = 0.0

    sol[0] = 0
    
    for i  in range(1,NB_TOWNS):
        sol[i] = i

    eval = evaluation_solution(sol)
    print("Next neighbour ", (sol, eval))

    for i in range(NB_TOWNS):
        best_solution[i] = sol[i]

    best_eval = eval


    return eval


#Builds final solution
def build_solution():
    global best_eval
    solution = [None] * NB_TOWNS
    currentIndex = 0
    currentNode = 0

    while currentIndex < NB_TOWNS:
        solution[currentIndex] = currentNode

        #Test if cycle is hamiltonien
        for i in range(currentIndex):
            if solution[i] == currentNode:
                #print("Cycle non-hamiltonien")
                return

        #Recherche de la ville suivante
        found = False
        i = 0

        while ((not found) and i < NB_TOWNS):

            if starting_town[i] == currentNode:
                found = True
                currentNode = ending_town[i]
            i += 1
        currentIndex += 1

    eval = evaluation_solution(solution)

    if best_eval < 0 or eval < best_eval:
        best_eval = eval
        for i in range(NB_TOWNS):
            best_solution[i] = solution[i]
        print("New best solution : ")
        print(solution)
        print(best_eval)

    return
    


def branch_and_bound(dist, iteration, evalParentNode):
   
    #Number of total iterations
    global count
    count += 1
    #print(count)


    if (iteration == NB_TOWNS):
        build_solution()
        return


    #Creation of a copy of the distance matrix
    m = copy.deepcopy(dist)


    evalChildNode = evalParentNode

    #Substracting min value of rows 
    minValueRow = np.amin(m, 1)

    for i in range(NB_TOWNS): 
        if not 0 in m[i,:] and minValueRow[i] != math.inf:
            m[i] -= minValueRow[i]

            evalChildNode += minValueRow[i] #Updating the current lower bound


    #Substracting min value of columns 
    minValueColumn = np.amin(m, 0,)

    for i in range(NB_TOWNS):
        if not 0 in m[:,i] and minValueColumn[i] != math.inf:
            m[:,i] -= minValueColumn[i]
            evalChildNode += minValueColumn[i] #Updating the current lower bound

    
    #Cut : stop the exploration of this node
    if (best_eval >= 0 and evalChildNode >= best_eval):
        return 

    #Calculating penalties (for zeros)
    minValueRow = np.amin(m, 1)
    minValueColumn = np.amin(m, 0,)

    listZeros = []

    #Count number of zeros on each row and column
    nbZerosR = NB_TOWNS - np.count_nonzero(m, 0)
    nbZerosC = NB_TOWNS - np.count_nonzero(m, 1)

    maxZero = (-1,0,0)

    for i in range(NB_TOWNS):
        for j in range(NB_TOWNS):
            if m[i,j] == 0:

                minR = 0 if nbZerosR[i] > 1 else min([value for value in m[i] if value != 0])
                minC = 0 if nbZerosC[j] > 1 else min([value for value in m[:,j] if value!=0])
  
                if minR == math.inf:
                    minR = 0

                if minC == math.inf:
                    minC = 0
                
                v = minR + minC

                listZeros.append((v, i, j))
                

                if (maxZero[0] < v):
                    maxZero = (v,i,j)


    if listZeros == []:
        return
    

    #Updates paths
    starting_town[iteration] = maxZero[1]
    ending_town[iteration] = maxZero[2]
    

    #Creating a copy of current distance matrix for left exploration (choice)
    m2 = copy.deepcopy(m)
   
    #Modifying new distance matrix
    m2[maxZero[2], maxZero[1]] = math.inf  #Set inf value
    m2[maxZero[1],:] = math.inf
    m2[:,maxZero[2]] = math.inf

    #Explore left branch of tree (choice)
    branch_and_bound(m2, iteration + 1, evalChildNode)




    #Creating a copy of current distance matrix for right exploration (non-choice)
    m3 = copy.deepcopy(m)

    #Modifying new distance matrix
    m3[maxZero[2], maxZero[1]] = math.inf  #Set inf value
    m3[maxZero[1], maxZero[2]] = math.inf  #Set inf value

    #Explore right branch of tree (non-choice)
    branch_and_bound(m3, iteration , evalChildNode)
    






### Main ###

# Open the file
f = open("berlin52.tsp", "r")

#Go to line 7
f.seek(128)

#Read each line
for i in range(NB_TOWNS):
    line = f.readline()
    l = line.split()
    coord[i][0] = float(l[1])
    coord[i][1] = float(l[2])


#Prints cities coordinates
print("\nCoordinates:\n")
for i in range(NB_TOWNS):
    print("Node ", i, " : " , coord[i])

#Calculates and prints distance matrix
calculate_dist(coord)
print("\nDistance matrix:\n")
print(dist)

#Building next neighbour
next_neighbour = build_next_neighbour()

#Branch-and-bound Algorithm
iteration = 0
lowerbound = 0.0

np.fill_diagonal(dist, math.inf)

branch_and_bound(dist, iteration, lowerbound)

print("Number of iterations :", count)
print("Best solution:", best_solution)
print("Best evaluation :", best_eval)
print("Next neighbour heuristic :", next_neighbour)
print("Runtime : %s seconds " % (time.time() - start_time))
