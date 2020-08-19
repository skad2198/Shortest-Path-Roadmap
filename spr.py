import sys
import numpy as np
from collections import defaultdict
from typing import List
from heapq import heappush, heappop

'''
Report reflexive vertices
'''
def findReflexiveVertices(polygons):
    vertices=[] 

    # Your code goes here
    # You should return a list of (x,y) values as lists, i.e.
    # vertices = [[x1,y1],[x2,y2],...] 

    for x in range(0, len(polygons)): #x co-cordinate on the graph 
        for y in range(0, len(polygons[x])): #y co-ordinate on the graph 

            n = polygons[x][y] #co-ordinates of the polygon on the x-y plane..
            #print(n)
            #print(len(polygons))

            if y == 0:  
                m = polygons[x][len(polygons[x]) - 1]  
                #print(m)
                p = polygons[x][y + 1] 
                #print(p)

            elif y == len(polygons[x]) - 1: 
                m = polygons[x][y - 1]  
                p = polygons[x][0]  

            else:
                m = polygons[x][y - 1] 
                p = polygons[x][y + 1] 

            sign = (n[0] - m[0]) * (p[1] - n[1]) - (n[1] - m[1]) * (p[0] - n[0]) 
            if sign <= 0: 
                vertices.append(n)

    return vertices 


'''
Compute the roadmap graph
'''
def distance_formula(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    val = np.sqrt(np.sum(np.power([dx, dy], 2))) #distance formula..
    #print(val)

    return val

def calcSide(ploys, a, b, c):
    ret = 0
    for points in ploys:
        v = a*points[0] + b*points[1] + c
        if v > 0:
            ret = ret + 1
        elif v < 0:
            ret = ret - 1
        else:
            pass

    if(abs(ret) < len(ploys) - 2):
        return False
    else:
        return True

def checkPosiblity(polygons, map, p1, p2):
    x1 = map[p1][0]
    y1 = map[p1][1]
    x2 = map[p2][0]
    y2 = map[p2][1]

    a = y2 - y1
    b = x1 - x2
    c = -(a * x1 + b * y1)

    for ploys in polygons:
        if map[p1] in ploys:
            break

    if map[p2] in ploys:
        return calcSide(ploys, a, b, c)
    else:
        for nextploys in polygons:
            if map[p2] in nextploys:
                break
        return calcSide(ploys, a, b, c) and calcSide(nextploys, a, b, c)


def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = defaultdict(list)
    k = 1

    for x in range(0, len(reflexVertices)):
        vertexMap[k] = reflexVertices[x]
        k = k + 1    

    print (vertexMap[2])

    for p1 in vertexMap:
        adlist = []
        for p2 in vertexMap:

            if p1 == p2:
                continue
            if checkPosiblity(polygons, vertexMap, p1, p2) == False:
                continue

            adpos = []
            adpos.append(p2)
            val = distance_formula(vertexMap[p1], vertexMap[p2])
            adpos.append(float(val))
            adlist.append(adpos)

        adjacencyListMap[p1] = adlist

    # Your code goes here
    # You should check for each pair of vertices whether the
    # edge between them should belong to the shortest path
    # roadmap. 
    #
    # Your vertexMap should look like
    # {1: [5.2,6.7], 2: [9.2,2.3], ... }
    #
    # and your adjacencyListMap should look like
    # {1: [[2, 5.95], [3, 4.72]], 2: [[1, 5.95], [5,3.52]], ... }
    #
    # The vertex labels used here should start from 1    
    return vertexMap, adjacencyListMap

# will not be using this class ..
class PriorityQueue:
    nodes = []

    def __init__(self):
        pass

    def push(self, node):
        if len(self.nodes) > 0:
            for i in range(0, len(self.nodes)):
                if self.nodes[i][1] > node[1]:
                    self.nodes.insert(i, node)
                    return
        self.nodes.append(node)

    def length(self):
        return len(self.nodes)

    def pop(self):
        return self.nodes.pop(0)

    def include(self, node):
        for i in range(0, len(self.nodes)):
            if self.nodes[i][0] == node[0]:
                if self.nodes[i][1] > node[1]:
                    self.nodes.pop(i)
                    self.push(node)
                    return True

        return False


def checkIfInExplored(nodes, node):
    for item in nodes:
        if item[0] == node[0]:
            return True

    return False

def Result(nodes, node, start):
    for item in nodes:
        if item[0] == node[2]:
            if item[0] == start:
                return start

            return [Result(nodes, item, start), item[0]]


'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):

    queue = [(0, start, ())]
    seen = set()
    mins = {start: 0}

    while queue:
        (cost, v1, path) = heappop(queue)
        if v1 not in seen:
            seen.add(v1)
            path += (v1,)

            if v1 == goal:
                return (list(path), cost)

            for v2, c in adjListMap.get(v1, ()):
                if v2 in seen:
                    continue

                prev = mins.get(v2, None)
                nxt = cost + c

                if not prev or nxt < prev:
                    mins[v2] = nxt
                    heappush(queue, (nxt, v2, path))
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.

    return [], float("inf")

'''
Agument roadmap to include start and goal
'''
def perp(a: np.array) -> np.array:
    b = np.empty_like(a)

    b[0] = -a[1]
    b[1] = a[0]

    return b

def intersect(
        x1: np.array,
        x2: np.array,
        y1: np.array,
        y2: np.array) -> np.array:
    dx = x2 - x1
    dy = y2 - y1

    dp = x1 - y1
    dxp = perp(dx)

    num = np.dot(dxp, dp)
    denom = np.dot(dxp, dy)

    if denom == 0:
        return np.array([])

    return (num / denom.astype(float)) * dy + y1


def is_within_bounds(
        x1: List[float],
        x2: List[float],
        y1: List[float],
        y2: List[float],
        int_pt: List[float]):
    ix, iy = int_pt

    if not min(x1[0], x2[0]) <= ix <= max(x1[0], x2[0]):
        return False
    elif not min(x1[1], x2[1]) <= iy <= max(x1[1], x2[1]):
        return False
    elif not min(y1[0], y2[0]) <= ix <= max(y1[0], y2[0]):
        return False
    elif not min(y1[1], y2[1]) <= iy <= max(y1[1], y2[1]):
        return False
    else:
        return True

def visible_outside(
        polygons: List[List[List[float]]], target: List[float], possible_neighbor: List[float]):
    for polygon in polygons:
        n = len(polygon)
        for i in range(n):
            p_i = polygon[i]
            p_j = polygon[(i + 1) % n]

            if p_i == target or p_i == possible_neighbor:
                continue

            if p_j == target or p_j == possible_neighbor:
                continue

            int_pt = list(
                intersect(
                    np.array(p_i),
                    np.array(p_j),
                    np.array(target),
                    np.array(possible_neighbor)))
            if int_pt and is_within_bounds(
                    target, possible_neighbor, p_i, p_j, int_pt):
                return False

    return True

def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = defaultdict(list)
    startLabel = 0
    goalLabel = -1

    updatedALMap.update(adjListMap)

    for (possible_neighbor_index, possible_neighbor) in vertexMap.items():
        if visible_outside(polygons, [x1, y1], possible_neighbor):
            distance = np.linalg.norm(np.array([x1, y1]) - np.array(possible_neighbor))

            updatedALMap[possible_neighbor_index].append([startLabel, distance])
            updatedALMap[startLabel].append([possible_neighbor_index, distance])

        if visible_outside(polygons, [x2, y2], possible_neighbor):
            distance = np.linalg.norm(np.array([x2, y2]) - np.array(possible_neighbor))

            updatedALMap[possible_neighbor_index].append([goalLabel, distance])
            updatedALMap[goalLabel].append([possible_neighbor_index, distance])

    # Your code goes here. Note that for convenience, we
    # let start and goal have vertex labels 0 and -1,
    # respectively. Make sure you use these as your labels
    # for the start and goal vertices in the shortest path
    # roadmap. Note that what you do here is similar to
    # when you construct the roadmap.

    return startLabel, goalLabel, updatedALMap


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append([float(i) for i in xys[p].split(',')])
        polygons.append(polygon)

    # Print out the data
    print("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print(str(polygons[p]))
    print("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print("Reflexive vertices:")
    print(str(reflexVertices))
    print("")

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print("Vertex map:")
    print(str(vertexMap))
    print("")
    print("Base roadmap:")
    print(dict(adjListMap))
    print("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print("Updated roadmap:")
    print(dict(updatedALMap))
    print("")

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print("Final path:")
    print(str(path))
    print("Final path length:" + str(length))
