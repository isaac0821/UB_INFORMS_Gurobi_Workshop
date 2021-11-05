import gurobipy as grb
import random

def generateInstance(
    n:          "Number of CUSTOMERS",
    m:          "Number of FACILITIES",
    instancePath: "Export path for instances" = "instance.txt"
    ) -> "Generate a facility location problem instance":

    # Cost for assigning customer i to facility j =============================
    c = {}
    for i in range(n):
        for j in range(m):
            c[i, j] = random.randint(50, 200)
    # Cost for opening facility j =============================================
    d = {}
    for j in range(m):
        d[j] = random.randint(1000, 3000)

    # Save to a local file ====================================================
    f = open(instancePath, "w")
    # Meta info
    f.write(str(m) + "\t" + str(n) + "\n")
    # Save d[j]
    for j in range(m):
        f. write(str(d[j]) + "\n")
    # Save c[i, j]
    for j in range(m):
        s = ""
        for i in range(n):
            s += str(c[i, j]) + "\t"
        f.write(s + "\n")
    f.close()

    return {
        'n': n,
        'm': m,
        'c': c,
        'd': d
    }

def readInstance(
    instancePath: "Import path for instance"
    ) -> "Read an instance file from local":

    # Initialize ==============================================================
    c = {}
    d = {}

    # Read file ===============================================================
    f = open(instancePath, "r")
    meta = f.readline().split("\t")
    m = int(meta[0])
    n = int(meta[1])

    d = {}
    for i in range(m):
        d[i] = int(f.readline())

    c = {}
    for j in range(m):
        ci = f.readline().split("\t")
        for i in range(n):
            c[i, j] = int(ci[i])

    f.close()

    return {
        'n': n,
        'm': m,
        'c': c,
        'd': d
    }

def directSolveProblem(
    instance: "Facility location problem, with n, m, c, d"
    ) -> "Use gurobi to find the optimal ofv of the instance without Benders Decomposition":
    # Read instance ===========================================================
    n = instance['n']
    m = instance['m']
    c = instance['c']
    d = instance['d']

    # Directly use Gurobi =====================================================
    FL = grb.Model("Facility Location")
    x = {}
    for i in range(n):
        for j in range(m):
            x[i, j] = FL.addVar(vtype=grb.GRB.CONTINUOUS, obj=c[i, j])
    y = {}
    for j in range(m):
        y[j] = FL.addVar(vtype=grb.GRB.BINARY, obj=d[j])

    for i in range(n):
        FL.addConstr(grb.quicksum(x[i, j] for j in range(m)) >= 1)
    for i in range(n):
        for j in range(m):
            FL.addConstr(x[i, j] <= y[j])

    FL.update()
    FL.modelSense = grb.GRB.MINIMIZE
    FL.optimize()

    ofv = None
    masterY = []
    if (FL.status == grb.GRB.status.OPTIMAL):
        ofv = FL.getObjective().getValue()
        for j in y:
            if (y[j].x >= 0.9):
                masterY.append(j)
        alloc = {}
        for j in range(m):
            alloc[j] = []
        for i in range(n):
            for j in range(m):
                if (x[i, j].x >= 0.9):
                    alloc[j].append(i)

    return {
        'ofv': ofv,
        'y': masterY,
        'alloc': alloc
    }

def masterProblem(
    instance: "Facility location problem, with n, m, c, d"
    ) -> "Use Benders Decomposition to solve Facility Location Problem":
    # Read instance ===========================================================
    n = instance['n']
    m = instance['m']
    c = instance['c']
    d = instance['d']

    # Initialize model ========================================================
    master = grb.Model("Master-IP")
    x = {}
    for i in range(n):
        for j in range(m):
            x[i, j] = master.addVar(vtype=grb.GRB.CONTINUOUS, obj=c[i, j])
    y = {}
    for j in range(m):
        y[j] = master.addVar(vtype=grb.GRB.BINARY, obj=d[j])

    # Integer part, no constraint =============================================
    pass

    # Call back to add cuts ===================================================
    master._y = y
    def addCuts(model, where):
        if (where == grb.GRB.Callback.MIPSOL):
            y_sol = model.cbGetSolution(model._y)
            subY = {}
            for i in range(m):
                if (y_sol[i] >= 0.9):
                    subY[i] = 1
                else:
                    subY[i] = 0
            findCut = dualSubproblem(instance, subY)
            if (findCut['type'] == "infeasible"):
                model.terminate()
            elif (findCut['type'] == "optimality"):
                print("Add optimality cut")
                subLam = findCut['lam']
                subPi = findCut['pi']
                model.cbLazy(grb.quicksum(c[i, j] * x[i, j] for i in range(n) for j in range(m)) 
                    >= grb.quicksum(subLam[i] for i in range(n)) - grb.quicksum(subPi[i, j] * y[j] for i in range(n) for j in range(m)))
            elif (findCut['type'] == "feasibility"):
                print("Add feasibility cut")
                subLam = findCut['lam']
                subPi = findCut['pi']
                model.cbLazy(grb.quicksum(subLam[i] for i in range(n)) - grb.quicksum(subPi[i, j] * y[j] for i in range(n) for j in range(m)) <= 0)
        return
        
    master.Params.lazyConstraints = 1
    master.setParam('OutputFlag', 0)
    master.optimize(addCuts)

    ofv = None
    masterY = []
    if (master.status == grb.GRB.status.OPTIMAL):
        ofv = master.getObjective().getValue()
        for j in y:
            if (y[j].x >= 0.9):
                masterY.append(j)

    return {
        'ofv': ofv,
        'y': masterY
    }

def dualSubproblem(
    instance:   "Facility location problem, with n, m, c, d",
    y:          "Given integer solution"
    ) -> "Calculate the dual of subproblem, which is a LP":

    # Read instance ===========================================================
    n = instance['n']
    m = instance['m']
    c = instance['c']
    d = instance['d']

    # Dual subproblem initialize ==============================================
    sub = grb.Model("Dual-LP")

    # fTy =====================================================================
    fTy = 0
    for j in range(m):
        fTy += d[j] * y[j]

    # Decision variables ======================================================
    lam = {}
    for i in range(n):
        lam[i] = sub.addVar(vtype=grb.GRB.CONTINUOUS)
    pi = {}
    for i in range(n):
        for j in range(m):
            pi[i, j] = sub.addVar(vtype=grb.GRB.CONTINUOUS)

    # Constraints =============================================================
    for i in range(n):
        for j in range(m):
            sub.addConstr(lam[i] - pi[i, j] <= c[i, j])
    sub.update()

    # Objective ===============================================================
    sub.setObjective(grb.quicksum(lam[i] - grb.quicksum(pi[i, j] * y[j] for j in range(m)) for i in range(n)) + fTy)

    # Interpret result ========================================================
    sub.modelSense = grb.GRB.MAXIMIZE
    sub.setParam('OutputFlag', 0)
    sub.setParam("InfUnbdInfo", 1)
    sub.optimize()
    # If bounded, return Optimality cuts
    if (sub.status == grb.GRB.status.OPTIMAL):
        subLam = {}
        subPi = {}
        for i in range(n):
            subLam[i] = lam[i].x
        for i in range(n):
            for j in range(m):
                subPi[i, j] = pi[i, j].x
        return {
            'type': "optimality",
            'lam': subLam,
            'pi': subPi
        }
    # If unbounded, return Feasibility cut
    elif (sub.status == grb.GRB.status.UNBOUNDED):
        subLam = {}
        subPi = {}
        ray = sub.UnbdRay
        for i in range(n):
            subLam[i] = ray[i]
        for i in range(n):
            for j in range(m):
                subPi[i, j] = ray[n + i * m + j]
        return {
            'type': "feasibility",
            'lam': subLam,
            'pi': subPi
        }
    elif (sub.status == grb.GRB.status.INFEASIBLE):
        return {
            'type': "infeasible",
            'lam': subLam,
            'pi': subPi
        }
