#pursuit_evasion.py

import evader
import pursuer
import numpy as np
import matplotlib.pyplot as plt
from utils import plot_line_segments, line_line_intersection
np.random.seed(2)

class PursuitEvasionGame(object):
    """docstring for ClassName"""
    def __init__(self, evader, pursuer): 
        self.e = evader
        self.p = pursuer

    # def near_capture(self, V, x): # exact same as near
    #     dist = 3.0
    #     near_arr = []
    #     for i in range(len(V)):
    #         if np.linalg.norm(np.array(x.loc)-np.array(V[i].loc)) <= dist:
    #             near_arr.append(V[i])
    #     return near_arr

    # def plot_all_the_things(self, V_e, V_p):
    #     plt.figure()
    #     plot_line_segments(evader.obstacles, color="red", linewidth=2, label="obstacles")
    #     evader.plot_tree(V_e, color="blue", linewidth=.5, label="RRT tree")
    #     nodes = np.zeros((evader.n,2))
    #     for i in range(len(V_e)):
    #         nodes[i,:] = V_e[i].loc
    #         if evader.is_safe(X_goal,V_e[i]):
    #             goalnode = V_e[i]
    #     if success:
    #         solution_path_node = [goalnode]
    #         solution_path = [goalnode.loc]
    #         while np.all(solution_path[0] != evader.x_init):
    #             parent = solution_path_node[0].parent
    #             solution_path_node = [parent] + solution_path
    #             solution_path = [parent.loc] + solution_path
    #         evader.plot_path(solution_path, color="green", linewidth=2, label="solution path")

    #     print(goalnode.T)
    #     plt.scatter(nodes[:,0], nodes[:,1])
    #     plt.scatter([evader.x_init[0], evader.x_goal[0]], [evader.x_init[1], evader.x_goal[1]], color="green", s=30, zorder=10)
    #     plt.annotate(r"$x_{init}$", evader.x_init[:2] + [.2, 0], fontsize=16)
    #     plt.annotate(r"$x_{goal}$", evader.x_goal[:2] + [.2, 0], fontsize=16)
    #     plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)
    #     plt.show()

    def solve(self, max_iters, eps = 1.0):

        goal = False

        x_init_e = evader.node(self.e.x_init)
        x_init_e.setP(None)
        x_init_e.setT(0)

        V_e = [x_init_e] # init "graph"

        margin = 0.5 # create "X-goal"
        x_goal_hi = self.e.x_goal[0]+margin
        x_goal_lo = self.e.x_goal[0]-margin
        y_goal_hi = self.e.x_goal[1]+margin
        y_goal_lo = self.e.x_goal[1]-margin
        X_goal = [x_goal_lo, y_goal_lo, x_goal_hi, y_goal_hi]

        i = 0
        while not goal and i <= max_iters:
            i = i + 1
            x_rand_e = np.random.uniform(self.e.ss_lo, self.e.ss_hi)
            x_new_e = self.e.extend(V_e, x_rand_e, eps)
            if (x_new_e != None):
                Z_p_near = self.e.near(V_p, x_new_e)
                for z_p_near in Z_p_near:
                    if z_p_near.T < x_new_e.T:
                        self.e.remove_branch(x_new_e)
                if self.e.is_safe(X_goal, x_new_e):
                    success = True
            x_rand_p = np.random.uniform(self.p.ss_lo, self.p.ss_hi)
            x_new_p = self.p.extend(V_p, x_rand_p, eps)
            if (x_new_p != None):
                Z_e_near = self.p.near(V_e, x_new_p)
                for z_e_near in Z_e_near:
                    if x_p_new.T < z_e_near.T:
                        self.p.remove_branch(z_e_near)

        #self.plot_all_the_things(V_e, V_p)

############################ TESTING ################################

NOMAZE = np.array([])

e = evader.EvaderRRT([-2,-2], [2,2], [-1,-1], [1,1], NOMAZE)
p = pursuer.PursuerRRT([-2,-2], [2,2], [-1,1], NOMAZE)
peg = PursuitEvasionGame(e,p)
peg.solve(2000)

plt.show()

# MAZE = np.array([
#     ((5, 5), (-5, 5)),
#     ((-5, 5), (-5, -5)),
#     ((-5,-5), (5, -5)),
#     ((5, -5), (5, 5)),
#     ((-3, -3), (-3, -1)),
#     ((-3, -3), (-1, -3)),
#     ((3, 3), (3, 1)),
#     ((3, 3), (1, 3)),
#     ((1, -1), (3, -1)),
#     ((3, -1), (3, -3)),
#     ((-1, 1), (-3, 1)),
#     ((-3, 1), (-3, 3)),
#     ((-1, -1), (1, -3)),
#     ((-1, 5), (-1, 2)),
#     ((0, 0), (1, 1))
# ])
