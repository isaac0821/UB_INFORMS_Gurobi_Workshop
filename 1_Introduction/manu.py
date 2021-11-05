import gurobipy as grb

m = grb.Model("Manufactures")

x1 = m.addVar(vtype=grb.GRB.CONTINUOUS)
x2 = m.addVar(vtype=grb.GRB.CONTINUOUS)

m.setObjective(3 * x1 + 2 * x2)
m.modelSense = grb.GRB.MAXIMIZE

m.addConstr(2 * x1 + x2 <= 100)
m.addConstr(x1 + x2 <= 80)
m.addConstr(x1 <= 40)

m.update()
m.optimize()