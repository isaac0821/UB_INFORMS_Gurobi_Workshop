import gurobipy as grb

PMS = grb.Model("PMS")

M = 3
job = [3, 13, 5, 12, 8, 10, 16, 4]

x = {}
for i in range(M):
	for j in range(len(job)):
		x[i, j] = PMS.addVar(vtype = grb.GRB.BINARY)

z = PMS.addVar(vtype = grb.GRB.CONTINUOUS)

PMS.setObjective(z)
PMS.modelSense = grb.GRB.MINIMIZE

for i in range(M):
	PMS.addConstr(grb.quicksum(job[j] * x[i, j] for j in range(len(job))) <= z)

for j in range(len(job)):
	PMS.addConstr(grb.quicksum(x[i, j] for i in range(M)) == 1)

PMS.optimize()