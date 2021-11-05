import gurobipy as grb
import random

# Configurations ==========================================================
timeLimit = 3 # 3 # [sec]
gapLimit = None # 0.03 # 1 as 100%

# Input data ==============================================================
numMachines = 3
numJobs = 20000000
jobDurations = []
for j in range(numJobs):
	jobDurations.append(random.uniform(10, 20))

# Initialize ==============================================================
PMS = grb.Model('PMS')
    
# Decision variables ======================================================
x = {}
for i in range(numMachines):
    for j in range(len(jobDurations)):
        x[i, j] = PMS.addVar(vtype = grb.GRB.BINARY, name = 'x_%s_%s' % (i, j))
t = PMS.addVar(vtype = grb.GRB.CONTINUOUS, obj = 1, name = 't')

# Constraints =============================================================
# Time span
for i in range(numMachines):
    PMS.addConstr(grb.quicksum(jobDurations[j] * x[i, j] for j in range(len(jobDurations))) <= t)
# Unique machine
for j in range(len(jobDurations)):
    PMS.addConstr(grb.quicksum(x[i, j] for i in range(numMachines)) == 1)

# PMS objective function ==================================================
PMS.modelSense = grb.GRB.MINIMIZE
if (timeLimit != None):
	PMS.setParam(grb.GRB.Param.TimeLimit, timeLimit)
	PMS.setParam("OutputFlag", 1)
if (gapLimit != None):
	PMS.setParam(grb.GRB.Param.MIPGap, gapLimit)
PMS.update()

# Optimize ================================================================
PMS.optimize()

# Reconstruct solution ====================================================
ofv = None
if (PMS.status == grb.GRB.status.OPTIMAL or PMS.status == grb.GRB.status.TimeLimit):
    ofv = PMS.getObjective().getValue()
    print(ofv)


