import math
from gurobipy import *



# Gurobi initialize ===========================================================
n = 48
edges = {}
nodeIDs = range(n)

# Read from local file ========================================================
f = open("TSP.txt")

lines = f.readlines()
i = 0
for line in lines:
    dist = line.split()
    for j in range(n):
        edges[i, j] = float(dist[j])
    i += 1

# Initialize ==================================================================
TSP = Model('TSP')

# Define DVs ==================================================================
x = {}
for i in range(n):
    for j in range(n):
        if (i != j):
            x[i, j] = TSP.addVar(
                vtype = GRB.BINARY, 
                obj = edges[nodeIDs[i], nodeIDs[j]], 
                name = 'x_%s_%s' % (i, j))
                
# TSP objective function ======================================================
TSP.modelSense = GRB.MINIMIZE
TSP.Params.lazyConstraints = 1
TSP.update()

# Degree constraints --------------------------------------------------
for i in range(n):
    TSP.addConstr(quicksum(x[i, j] for j in range(n) if i != j) == 1, name = 'leave_%s' % i)
    TSP.addConstr(quicksum(x[j, i] for j in range(n) if i != j) == 1, name = 'enter_%s' % i)

# Sub-tour elimination ------------------------------------------------
TSP._x = x
def arcs2AdjList(arcs):
    neighbors = {}
    for i in range(len(arcs)):
        if (arcs[i][0] not in neighbors):
            neighbors[arcs[i][0]] = [arcs[i][1]]
        else:
            neighbors[arcs[i][0]].append(arcs[i][1])
        if (arcs[i][1] not in neighbors):
            neighbors[arcs[i][1]] = [arcs[i][0]]
        else:
            neighbors[arcs[i][1]].append(arcs[i][0])

    return neighbors

def findComponentsUndirected(arcs):
    # Create adj list, each vertex start with an empty list
    adjList = arcs2AdjList(arcs)

    # Initialize
    found = {}
    for node in adjList:
        found[node] = 0
    components = []

    # Main algorithm, mark neighbors
    for i in adjList:
        comp = []
        q = []
        if (found[i] == 0):
            found[i] = 1
            comp.append(i)
            q.append(i)
            while (q):
                v = q.pop(0)
                for u in adjList[v]:
                    if (found[u] == 0):
                        found[u] = 1
                        comp.append(u)
                        q.append(u)
            components.append(comp)
    return components

def subtourelim(model, where):
    if (where == GRB.Callback.MIPSOL):
        x_sol = model.cbGetSolution(model._x)
        arcs = tuplelist((i, j) for i, j in model._x.keys() if x_sol[i, j] > 0.9)
        components = findComponentsUndirected(arcs)
        for component in components:
            if (len(component) < n):
                model.cbLazy(quicksum(x[i,j] for i in component for j in component if i != j) <= len(component) - 1)

# TSP with callback ---------------------------------------------------
TSP.optimize(subtourelim)

# Reconstruct solution ------------------------------------------------
ofv = None
seq = []
arcs = []
if (TSP.status == GRB.status.OPTIMAL):
    ofv = TSP.getObjective().getValue()
    for i, j in x:
        if (x[i, j].x > 0.5):
            arcs.append([i, j])
    currentNode = 0
    currentTime = 0
    seq.append(nodeIDs[currentNode])
    while (len(arcs) > 0):
        for i in range(len(arcs)):
            if (arcs[i][0] == currentNode):
                currentNode = arcs[i][1]
                seq.append(nodeIDs[currentNode])
                arcs.pop(i)
                break
    gap = 0
    lb = ofv
    ub = ofv
    runtime = TSP.Runtime
elif (TSP.status == GRB.status.TIME_LIMIT):
    ofv = None
    seq = []
    gap = TSP.MIPGap
    lb = TSP.ObjBoundC
    ub = TSP.ObjVal
    runtime = TSP.Runtime

res = {
    'ofv': ofv,
    'seq': seq,
    'gap': gap,
    'lowerBound': lb,
    'upperBound': ub,
    'runtime': runtime
}
print(res)
