# evader.py

import numpy as np
import matplotlib.pyplot as plt
from utils import plot_line_segments, line_line_intersection
np.random.seed(2)

class node(object):
    _id = 0
    def __init__(self, loc):
        self.loc = loc
        self._id = node._id
        node._id += 1
        self.parent = None
        self.children = []
        self.T = None
        
    def setP(self, parent):
        self.parent = parent

    def addChild (self, child):
        self.children.append(child)
        
    def setT(self, T):
        self.T = T
        
    def getid(self):
        return self._id
        
class EvaderRRT(object):
    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, obstacles):
    	self.statespace_lo = np.array(statespace_lo)
    	self.statespace_hi = np.array(statespace_hi)
    	self.x_init = np.array(x_init)
    	self.x_goal = np.array(x_goal)
    	self.obstacles = obstacles
    	self.n = 1

    def is_free_motion(self, obstacles, x1, x2):
        motion = np.array([x1, x2])
        for line in obstacles:
            if line_line_intersection(motion, line):
                return False
        return True

    def find_nearest(self, V, x):
        num_states = len(V)
        distances = np.zeros(num_states)
        for i in range(num_states):
            distances[i] = np.linalg.norm(np.array(x)-np.array(V[i].loc))
        idx = np.argmin(distances)
        return idx

    def steer_towards(self, x, y, eps):
        dist = np.linalg.norm(np.array(x) - np.array(y))
        if dist < eps:
            return y
        dx = x[0] - y[0]
        dy = x[1] - y[1]
        cos = dx/dist
        sin = dy/dist
        return np.array([x[0]+eps*cos, x[1]+eps*sin])

    def extend(self,V,x_rand,eps):
        success = False
        x_near = V[self.find_nearest(V, x_rand)]
        x_new = node(self.steer_towards(x_near.loc, x_rand, eps))
        
        if self.is_free_motion(self.obstacles, x_near.loc, x_new.loc):
            V.append(x_new)
            x_new.setT(x_near.T + np.linalg.norm(np.array(x_new.loc)-np.array(x_near.loc)))
            cmin = x_new.T
            zmin = x_near
            Z_near = self.near(V,x_new)
            if Z_near:
                for i in range(len(Z_near)):
                    z_new = self.steer_towards(Z_near[i].loc,x_new.loc,eps)
                    dist = np.linalg.norm(np.array(z_new)-np.array(Z_near[i].loc))
                    if self.is_free_motion(self.obstacles, Z_near[i].loc, z_new) 
                    and np.all(z_new == x_new.loc) and Z_near[i].T+dist < cmin:
                        cmin = Z_near[i].T + dist
                        zmin = Z_near[i]
                        
            x_new.setT(zmin.T + np.linalg.norm(np.array(x_new.loc)-np.array(zmin.loc)))
            x_new.setP(zmin)
            zmin.addChild(x_new)
            
            if Z_near:
                if zmin in Z_near:
                    Z_near.remove(zmin)
                for i in range(len(Z)):
                    z_near = self.steer_towards(x_new.loc,Z_near[i].loc,eps)
                    if np.all(z_near == Z_near[i].loc):
                        dist = np.linalg.norm(np.array(x_new.loc)-np.array(Z_near[i].loc))
                        T_near = Z_near[i].T
                        if self.is_free_motion(self.obstacles, x_new.loc, Z_near[i].loc) and x_new.T+dist < T_near:
                            # "re-wiring"
                            Z_near[i].setP(x_new)
                            x_new.addChild(Z_near[i])
                    
            self.n = self.n+1
            if x_new.loc[1] < x_new.loc[0]-.5:
                self.removeBranch(x_new, V)
                return None
            return x_new
        return None
        
    def near(self, V, x):
        dist = 3.0

        retval = []
        for i in range(self.n):
            if np.linalg.norm(np.array(x.loc)-np.array(V[i].loc)) <= dist:
                retval.append(V[i])
        return retval

    def removeBranch(self, node, V):
        for i in range(len(node.children)):
            if node.children[i] == []:
                try: 
                    V.remove(node)
                    self.n = self.n - 1
                except ValueError:
                    print("node wasn't in V (1)...")
            else:
                try:
                    self.removeBranch(node.children[i], V)
                except ValueError:
                    print("node wasn't in V (2)...")
        try: 
            V.remove(node)
            self.n = self.n - 1
        except ValueError:
            print("node wasn't in V (3)...")
        return


    def plot_tree(self, V, **kwargs):
        plot_line_segments([(V[i].parent.loc, V[i].loc) for i in range(len(V)) if V[i].parent != None], **kwargs)

    def plot_path(self, path, **kwargs):
        path = np.array(path)
        plt.plot(path[:,0], path[:,1], **kwargs)

    def solve(self, eps, max_iters = 1000, goal_bias = 0.05):
        x_init = node(self.x_init)
        x_init.setP(None)
        x_init.setT(0)
        V = [x_init]
        n = 1 # the current size of the RRT (states accessible as V[range(n),:])
        x_goal_hi = self.x_goal[0]+.5
        x_goal_lo = self.x_goal[0]-.5
        y_goal_hi = self.x_goal[1]+.5
        y_goal_lo = self.x_goal[1]-.5

        success = False
        i = 0
        while not success and i <= max_iters:
            i = i + 1
            x_rand = np.random.uniform(self.statespace_lo, self.statespace_hi)
            x_new = self.extend(V,x_rand,eps)
            if (x_new != None):
            	# check near_capture or x_goal
            	if x_new.loc[0] > x_goal_lo and x_new.loc[0] < x_goal_hi:
            		if x_new.loc[1] > y_goal_lo and x_new.loc[1] < y_goal_hi:
            			success = True

        plt.figure()
        plot_line_segments(self.obstacles, color="red", linewidth=2, label="obstacles")
        self.plot_tree(V, color="blue", linewidth=.5, label="RRT tree")
        nodes = np.zeros((self.n,2))
        for i in range(len(V)):
            nodes[i,:] = V[i].loc
            if V[i].loc[0] > x_goal_lo and V[i].loc[0] < x_goal_hi:
            		if V[i].loc[1] > y_goal_lo and V[i].loc[1] < y_goal_hi:
                		goalnode = V[i]
        if success:
            solution_path_node = [goalnode]
            solution_path = [goalnode.loc]
            while np.all(solution_path[0] != self.x_init):
                parent = solution_path_node[0].parent
                solution_path_node = [parent] + solution_path
                solution_path = [parent.loc] + solution_path
            self.plot_path(solution_path, color="green", linewidth=2, label="solution path")

        print(goalnode.T)
        plt.scatter(nodes[:,0], nodes[:,1])
        plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
        plt.annotate(r"$x_{init}$", self.x_init[:2] + [.2, 0], fontsize=16)
        plt.annotate(r"$x_{goal}$", self.x_goal[:2] + [.2, 0], fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

### TESTING

MAZE = np.array([
    ((5, 5), (-5, 5)),
    ((-5, 5), (-5, -5)),
    ((-5,-5), (5, -5)),
    ((5, -5), (5, 5)),
    ((-3, -3), (-3, -1)),
    ((-3, -3), (-1, -3)),
    ((3, 3), (3, 1)),
    ((3, 3), (1, 3)),
    ((1, -1), (3, -1)),
    ((3, -1), (3, -3)),
    ((-1, 1), (-3, 1)),
    ((-3, 1), (-3, 3)),
    ((-1, -1), (1, -3)),
    ((-1, 5), (-1, 2)),
    ((0, 0), (1, 1))
])

NOMAZE = np.array([])

#evader = EvaderRRT([-5,-5], [5,5], [-4,-4], [4,4], MAZE)
evader = EvaderRRT([-2,-2], [2,2], [-1,-1], [2,2], NOMAZE)
evader.solve(1.0, 5000)

plt.show()
