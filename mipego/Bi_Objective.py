import numpy as np
#import matplotlib
#import matplotlib.pyplot as plt
import copy

def s_metric(solutions,n_left,ref_time=None,ref_loss=None):
    #s-metric is used to define the fitness for each solution using loss and time
    #TODO_CHRIS add fixed ref point functionality
    par = pareto(solutions)
    for i in range(len(solutions)):
        diff = copy.deepcopy(solutions)
        del diff[i]
        diff = pareto(diff)
        #calculate contributed hyper-volume
        solutions[i].fitness = -(hyper_vol(par,solutions)-hyper_vol(diff,solutions))
    #non-paretofront points receive a penalty
    solutions = penalty(solutions)
    #solutions = eps_penalty(solutions,par,n_left)#TODO_CHRIS this might make no sense, seems to push more towards paretofront if n_left is high
    #print('pareto-front:')
    #if len(par) > 0:
    #    for x in par:
    #        print('time: ' + str(x.time) + ' loss: ' + str(x.loss) + ' fit:' + str(x.fitness))
    return solutions

def penalty(solutions):
    for i in range(len(solutions)):
        for j in range(len(solutions)):
            if dominated(solutions[i], [solutions[j]]):
                pen = (1+solutions[i].loss - solutions[j].loss)*(1+solutions[i].time-solutions[j].time)-1
                solutions[i].fitness += pen
    return solutions

def sort_par(par):
    #sort pareto front ascending on loss
    for i in range(len(par)):
        for j in range(i+1, len(par)):
            if (par[i].loss < par[j].loss):
                help = par[i]
                par[i] = par[j]
                par[j] = help
    return par

def hyper_vol(par, solutions,ref_time=None,ref_loss=None):
    #set maximum values in pareto front as reference points
    if len(solutions) == 0:
        return 0.0
    if len(par) == 0:
        return 0.0
    if ref_time == None or ref_loss == None:
        ref_time = solutions[0].time
        ref_loss = solutions[0].loss
        for i in range(1,len(solutions)):
            if (solutions[i].time > ref_time):
                ref_time = solutions[i].time
            if (solutions[i].loss > ref_loss):
                ref_loss = solutions[i].loss
        ref_time += 1.0
        ref_loss += 1.0

    loc_par = copy.deepcopy(par)

    #remove entries in pareto front bigger than the reference point (can only happen if ref_time and ref_loss are given as parameter to function)
    i = 0
    while i < len(loc_par):
        if loc_par[i].time > ref_time or loc_par[i].loss > ref_loss:
            del loc_par[i]
        else:
            i += 1
    #sort pareto front
    loc_par = sort_par(loc_par)
    
    #calculate hypervolume
    if len(loc_par) > 0:
        vol = (ref_loss - loc_par[0].loss) * (ref_time - loc_par[0].time)
        for i in range(1,len(loc_par)):
            vol += (loc_par[i-1].loss - loc_par[i].loss) * (ref_time - loc_par[i].time)
    else:
        vol = 0.0
    return vol

def dominated(point, solutions):
    #returns whether <point> is dominated by the set <solutions>
    for i in range(len(solutions)):
        if point.loss < solutions[i].loss:
            continue
        if point.time < solutions[i].time:
            continue
        if (point.loss == solutions[i].loss) and (point.time == solutions[i].time):
            continue
        return True
    return False

def pareto(solutions):
    #returns the set of non-dominated solutions in <solutions>
    par = []
    for i in range(len(solutions)):
        if not dominated(solutions[i], solutions):
            par.append(solutions[i])
    return par

def eps_penalty(solutions,par,n_left):
    max_time = max([i.time for i in par])
    min_time = min([i.time for i in par])
    max_loss = max([i.loss for i in par])
    min_loss = min([i.loss for i in par])
    d_lamb = np.array([max_time-min_time, max_loss-min_loss])
    m = 2 #CHRIS because we have two objectives
    c = 1-1/(2**m)
    eps = d_lamb/len(par)+c*n_left
    for i in range(len(solutions)):
        for j in range(len(solutions)):
            help_sol = copy.deepcopy(solutions[j])
            help_sol.time -= eps[0]
            help_sol.loss -= eps[1]
            if dominated(solutions[i], [help_sol]):
                pen = (1+solutions[i].loss - help_sol.loss)*(1+solutions[i].time-help_sol.time)-1
                solutions[i].fitness += pen
    
    return solutions

#def test_func_1(x):
#    return x+np.random.rand()
#
#def test_func_2(x):
#    return x**2 + np.random.rand()
#
#TODO_CHRIS Point just for test code
#class Point:
#    def __init__(self,x):
#        self.x = x
#        self.time = test_func_1(self.x)
#        self.loss = test_func_2(self.x)
#        self.fitness = 0.0
#
#def test():
#    n = 100
#    solutions = [0.0] * n
#    for i in range(n):
#        solutions[i] = Point(1-i/n)
#
#    n_left = 100
#    solutions = s_metric(solutions,n_left)
#
#    x = [0.0]*n
#    y = [0.0]*n
#    z = [0.0]*n
#
#    for i in range(n):
#        x[i] = solutions[i].time
#        y[i] = solutions[i].loss
#        z[i] = solutions[i].fitness
#
#    plt.scatter(x,y,c = z)
#    plt.xlabel('time')
#    plt.ylabel('loss')
#    plt.show()
#
#test()
