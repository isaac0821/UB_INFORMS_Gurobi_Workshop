import gurobipy as grb

# Initialize the model ========================================================
m = grb.Model("Manufacture")

# Define decision variables ===================================================
x1 = m.addVar(vtype = grb.GRB.CONTINUOUS)
x2 = m.addVar(vtype = grb.GRB.CONTINUOUS)

# Define objective function ===================================================
m.setObjective(3 * x1 + 2 * x2)
m.modelSense = grb.GRB.MAXIMIZE

# Define constraints ==========================================================
m.addConstr(2 * x1 + x2 <= 100)
m.addConstr(x1 + x2 <= 80)
m.addConstr(x1 <= 40)
m.update()

# Optimize ====================================================================
m.setParam('OutputFlag', 0)
m.optimize()

# Interpret ===================================================================
if (m.status == grb.GRB.status.OPTIMAL):
	ofv = m.getObjective().getValue()
	print("The maximum weekly profit: ", ofv)
	print("Num of soldier: ", x1.x)
	print("Num of trains: ", x2.x)