import numpy as np
import matplotlib.pyplot as plt
#from dubins import path_length, path_sample
from utils import plot_line_segments, line_line_intersection
np.random.seed(1)
# Represents a motion planning problem to be solved using the RRT algorithm
class RRT(object):

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, obstacles):
        self.statespace_lo = np.array(statespace_lo)    # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = np.array(statespace_hi)    # state space upper bound (e.g., [5, 5])
        self.x_init = np.array(x_init)                  # initial state
        self.x_goal = np.array(x_goal)                  # goal state
        self.obstacles = obstacles                      # obstacle set (line segments)

    # Subject to the robot dynamics, returns whether a point robot moving along the shortest
    # path from x1 to x2 would collide with any obstacles (implemented for you as a "black box")
    # INPUT: (obstacles, x1, x2)
    #   obstacles - list/np.array of line segments ("walls")
    #          x1 - start state of motion
    #          x2 - end state of motion
    # OUTPUT: Boolean True/False
    def is_free_motion(self, obstacles, x1, x2):
        raise NotImplementedError("is_free_motion must be overriden by a subclass of RRT")

    # Given a list of states V and a query state x, returns the index (row) of V such that the
    # steering distance (subject to robot dynamics) from V[i] to x is minimized
    # INPUT: (obstacles, x1, x2)
    #           V - list/np.array of states ("samples")
    #           x - query state
    # OUTPUT: Integer index of nearest point in V to x
    def find_nearest(self, V, x):
        raise NotImplementedError("find_nearest must be overriden by a subclass of RRT")

    # Steers from x towards y along the shortest path (subject to robot dynamics); returns y if
    # the length of this shortest path is less than eps, otherwise returns the point at distance
    # eps along the path from x to y.
    # INPUT: (obstacles, x1, x2)
    #           x - start state
    #           y - target state
    #         eps - maximum steering distance
    # OUTPUT: State (numpy vector) resulting from bounded steering
    def steer_towards(self, x, y, eps):
        raise NotImplementedError("steer_towards must be overriden by a subclass of RRT")

    # Constructs an RRT rooted at self.x_init with the aim of producing a dynamically-feasible
    # and obstacle-free trajectory from self.x_init to self.x_goal.
    # INPUT: (eps, max_iters, goal_bias):
    #         eps - maximum steering distance
    #   max_iters - maximum number of RRT iterations (early termination is possible when a feasible
    #               solution is found)
    #   goal_bias - probability during each iteration of setting x_rand = self.x_goal
    #               (instead of uniformly randomly sampling from the state space)
    # OUTPUT: None officially (just plots), but see the "Intermediate Outputs" descriptions below
    def solve(self, eps, max_iters = 1000, goal_bias = 0.05):
        state_dim = len(self.x_init)

        # V stores the states that have been added to the RRT (pre-allocated at its maximum size
        # since numpy doesn't play that well with appending/extending)
        V = np.zeros((max_iters, state_dim))
        V[0,:] = self.x_init    # RRT is rooted at self.x_init
        n = 1                   # the current size of the RRT (states accessible as V[range(n),:])

        # P stores the parent of each state in the RRT. P[0] = -1 since the root has no parent,
        # P[1] = 0 since the parent of the first additional state added to the RRT must have been
        # extended from the root, in general 0 <= P[i] < i for all i < n
        P = -np.ones(max_iters, dtype=int)

        ## Intermediate Outputs
        # You must update and/or populate:
        #    - V, P, n: the represention of the planning tree
        #    - succcess: whether or not you've found a solution within max_iters RRT iterations
        #    - solution_path: if success is True, then must contain list of states (tree nodes)
        #          [x_init, ..., x_goal] such that the global trajectory made by linking steering
        #          trajectories connecting the states in order is obstacle-free.

        # TODO: fill me in!
        success = False
        i = 0
        while not success and i <= max_iters:
            i = i + 1
            z = np.random.uniform()
            if z < goal_bias :
                x_rand = self.x_goal
            else:
                x_rand = np.random.uniform(self.statespace_lo, self.statespace_hi)
            x_near = V[self.find_nearest(V[:n,:], x_rand),:]
            x_new = self.steer_towards(x_near, x_rand, eps)

            if self.is_free_motion(self.obstacles, x_near, x_new):
                V[n,:] = x_new
                # print V - x_near
                # print np.where(V[:n,:] == x_near)
                P[n] = np.where(V[:n,:] == x_near)[0][0]
                n = n+1
                if np.all(x_new == self.x_goal):
                    success = True


        P = P[:n]
        V = V[:n,:]
        for i in range(len(P)):
            if i < P[i]:
                print "Error"
        # print P
        solution_path = [self.x_goal]
        # print "path"
        while np.all(solution_path[0] != self.x_init):
            idx = np.where(V == solution_path[0])[0][0]
            solution_path = [V[P[idx],:]] + solution_path

        C = 0
        for i in range(1,len(solution_path)):
            C = C + np.linalg.norm(solution_path[i] - solution_path[i-1])
        print C
        # print type(solution_path)
        plt.figure()
        plot_line_segments(self.obstacles, color="red", linewidth=2, label="obstacles")
        self.plot_tree(V, P, color="blue", linewidth=.5, label="RRT tree")
        if success:
            self.plot_path(solution_path, color="green", linewidth=2, label="solution path")
        plt.scatter(V[:n,0], V[:n,1])
        plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
        plt.annotate(r"$x_{init}$", self.x_init[:2] + [.2, 0], fontsize=16)
        plt.annotate(r"$x_{goal}$", self.x_goal[:2] + [.2, 0], fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)


# Represents a geometric planning problem, where the steering solution between two points is a
# straight line (Euclidean metric)
class GeometricRRT(RRT):

    def find_nearest(self, V, x):
        # TODO: fill me in!
        num_states = len(V[:,0])
        distances = np.zeros(num_states)
        for i in range(num_states):
            distances[i] = np.linalg.norm(np.array(x)-np.array(V[i,:]))
        # print distances

        idx = np.argmin(distances)
        # print idx
        return idx


    def steer_towards(self, x, y, eps):
        # TODO: fill me in!
        dist = np.linalg.norm(np.array(x)-np.array(y))
        if dist < eps:
            return y
        dx = x[0] - y[0]
        dy = x[1] - y[1]
        cos = dx/dist
        sin = dy/dist
        return np.array([x[0]+eps*cos, x[1]+eps*sin])
        
    def near(self, V, x):
        n = len(V[:,0])
        dist = np.sqrt(np.log(n)/(np.pi*n))
        retval = []
        for i in range(n):
            if np.linalg.norm(np.array(x)-np.array(V[i,:])) <= dist:
                retval.append(V[i,:])
                
        return np.array(retval)


    def is_free_motion(self, obstacles, x1, x2):
        motion = np.array([x1, x2])
        # print "motion = ", motion
        for line in obstacles:
            if line_line_intersection(motion, line):
                # print "False"
                return False
        # print "True"
        return True

    def plot_tree(self, V, P, **kwargs):
        plot_line_segments([(V[P[i],:], V[i,:]) for i in range(V.shape[0]) if P[i] >= 0], **kwargs)

    def plot_path(self, path, **kwargs):
        path = np.array(path)
        plt.plot(path[:,0], path[:,1], **kwargs)


# Represents a planning problem for the Dubins car, a model of a simple car that moves at constant
# speed forwards and has a limited turning radius. We will use this package:
# https://github.com/AndrewWalker/pydubins/blob/master/dubins/dubins.pyx
# to compute steering distances and steering trajectories. In particular, note the functions
# dubins.path_length and dubins.path_sample (read their documentation at the link above). See
# http://planning.cs.uiuc.edu/node821.html
# for more details on how these steering trajectories are derived.
#class DubinsRRT(RRT):
#
#    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, obstacles, turning_radius):
#        self.turning_radius = turning_radius
#        super(self.__class__, self).__init__(statespace_lo, statespace_hi, x_init, x_goal, obstacles)
#
#    def find_nearest(self, V, x):
#        # TODO: fill me in!
#        num_states = len(V[:,0])
#        distances = np.zeros(num_states)
#        for i in range(num_states):
#            distances[i] = path_length(V[i,:], x, self.turning_radius)
#
#        idx = np.argmin(distances)
#
#        return idx
#
#    def steer_towards(self, x, y, eps):
#        # TODO: fill me in!
#        dist = path_length(x, y, self.turning_radius)
#        if dist < eps:
#            return y
#        path = path_sample(x, y, self.turning_radius, eps)[0]
#
#        return np.asarray(path[1])
#
#    def is_free_motion(self, obstacles, x1, x2, resolution = np.pi/6):
#        pts = path_sample(x1, x2, self.turning_radius, self.turning_radius*resolution)[0]
#        pts.append(x2)
#        for i in range(len(pts) - 1):
#            for line in obstacles:
#                if line_line_intersection([pts[i][:2], pts[i+1][:2]], line):
#                    return False
#        return True
#
#    def plot_tree(self, V, P, resolution = np.pi/24, **kwargs):
#        line_segments = []
#        for i in range(V.shape[0]):
#            if P[i] >= 0:
#                pts = path_sample(V[P[i],:], V[i,:], self.turning_radius, self.turning_radius*resolution)[0]
#                pts.append(V[i,:])
#                for j in range(len(pts) - 1):
#                    line_segments.append((pts[j], pts[j+1]))
#        plot_line_segments(line_segments, **kwargs)
#
#    def plot_path(self, V, resolution = np.pi/24, **kwargs):
#        pts = []
#        for i in range(len(V) - 1):
#            pts.extend(path_sample(V[i], V[i+1], self.turning_radius, self.turning_radius*resolution)[0])
#        plt.plot([x for x, y, th in pts], [y for x, y, th in pts], **kwargs)

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

grrt = GeometricRRT([-5,-5], [5,5], [-4,-4], [4,4], MAZE)
grrt.solve(1.0, 5000)

#drrt = DubinsRRT([-5,-5,0], [5,5,2*np.pi], [-4,-4,0], [4,4,np.pi/2], MAZE, .5)
#drrt.solve(3.0, 1000)

plt.show()