import copy
import heapq
import os
import sys

"""
    A* WITH MANHATTAN DISTANCE HEURISTIC

    node represented as dict with following keys:
        state       3x3 int array
        g           cost to get to this node from starting state
        dirs        array of directions taken from start position

    frontier implemented using python's heapq library. each element of heap is a 3-tuple:
    ( f(node), order, node )
    as such we ensure we always get the node n with the minimum f(n) when we pop from the heap
    presence of order ensures that we never attempt to compare dictionary objects

    we keep track of which states have been visited using a python set. since arrays cannot be
    hashed, we first pre-process the arrays into hashable integers.
"""


U = "UP"
D = "DOWN"
L = "LEFT"
R = "RIGHT"

N = "UNSOLVABLE"
DIRS = [U, D, L, R]

class Puzzle(object):
    def __init__(self, init_state, goal_state):
        # You may add more attributes as necessary
        self.init_state = init_state
        self.goal_state = goal_state
        self.actions = list()
        self.solvable = True
        self.heap = []          # frontier
        self.node_index = 0.0    # number of nodes so far
        self.v_set = set()       # set of visited nodes


    # main method
    def solve(self):
        
        first_node = {
            "state": self.init_state,
            "g": 0,
            "dirs": []
        }

        self.visit(first_node)
        self.hpush(first_node)

        while len(self.heap) > 0:
            cur = self.hpop()
    
            # reached goal state
            if self.state(cur) == self.goal_state:
                return self.dirs(cur)

            # add new nodes to frontier
            for newNode in self.get_array_of_children(cur):
                if self.visited(newNode):
                    continue
                else:
                    self.visit(newNode)
                    self.hpush(newNode)

        # frontier is empty at this point
        # in other words, we've explored every state reachable from start state
        return [N]





    ################################################################################################
    # HEURISTIC FUNCTIONS
    ################################################################################################

    # heuristic
    def h(self, n):
        return self.get_heuristic_score(self.state(n))

    # converts state into a one-dimensional array
    def flatten_state(self, state):
        flattened_state = []
        for i in range(3):
            for j in range(3):
                flattened_state.append(state[i][j])
        return flattened_state

    # heuristic helper
    def get_heuristic_score(self, state):
        """
        Computes the manhattan distance
        Args:
            state (array): 3x3 arrary represents current state
        Returns the value of manhattan distance
        """
        flattened_state = self.flatten_state(state)
        manhattan = 0
        for i, val in enumerate(flattened_state):
            if val:
                row_distance = abs((val-1)%3 - i%3)
                col_distance = abs((val-1)//3 - i//3)
                manhattan = manhattan + row_distance + col_distance
        return manhattan





    ################################################################################################
    # FUNCTIONS THAT INTERFACE WITH SET OF VISITED NODES
    ################################################################################################

    # mark as visited
    def visit(self, n):
        int_form = self.encode_state(self.state(n))
        self.v_set.add(int_form)

    # check if visited
    def visited(self, n):
        int_form = self.encode_state(self.state(n))
        return int_form in self.v_set

    # pre-process state array so it can fit into a python set
    def encode_state(self, s):
        int_form = 0
        
        for i in range(3):
            for j in range(3):
                int_form += s[i][j]
                int_form *= 10

        return int_form





    ################################################################################################
    # EXPLORATION FUNCTIONS
    ################################################################################################

    # gets array of all valid children of this node
    def get_array_of_children(self, n):
        s = self.state(n)
        rv = []

        for d in DIRS:
            newState = self.shift(s, d)

            if newState == -1:
                continue
            else:
                rv.append({
                    "state": newState,
                    "g": self.g(n) + 1,
                    "dirs": self.add_dir(n, d)
                })

        return rv

    # attempt to make a specified move (d) from this state (s)
    # returns the resultant state if legal, -1 if not.
    def shift(self, s, d):
        r = -1
        c = -1
        rv = copy.deepcopy(s)

        for i in range(3):
            for j in range(3):
                if s[i][j] == 0:
                    r = i
                    c = j

        if d == U:
            if r == 2:
                return -1
            else:
                rv[r][c] = rv[r+1][c]
                rv[r+1][c] = 0

        elif d == D:
            if r == 0:
                return -1
            else:
                rv[r][c] = rv[r-1][c]
                rv[r-1][c] = 0

        elif d == L:
            if c == 2:
                return -1
            else:
                rv[r][c] = rv[r][c+1]
                rv[r][c+1] = 0

        elif d == R:
            if c == 0:
                return -1
            else:
                rv[r][c] = rv[r][c-1]
                rv[r][c-1] = 0

        else:
            return -1

        return rv
    # end shift

    # add extra direction to list of directions for n
    def add_dir(self, n, d):
        rv = copy.copy(self.dirs(n))
        rv.append(d)
        return rv

    # get directions taken to get to n from start state
    def dirs(self, n):
        return n["dirs"]

    # get game state at this node
    def state(self, n):
        return n["state"]





    ################################################################################################
    # FUNCTIONS THAT INTERFACE WITH HEAP (PRIORITY QUEUE)
    ################################################################################################

    # push into heap
    def hpush(self, n):
        heapq.heappush(
            self.heap, (
                self.f(n),
                self.node_index,
                n
            )
        )

        self.node_index += 1.0
    
    # pop from heap
    def hpop(self):
        return (heapq.heappop(
            self.heap
        ))[2]






    ################################################################################################
    # OTHER A* FUNCTIONS
    ################################################################################################

    # sum of g(n) and h(n)
    def f(self, n):
        return self.g(n) + self.h(n)

    # total cost so far
    def g(self, n):
        return n["g"]















    ################################################################################################
    # DON'T MODIFY
    ################################################################################################

if __name__ == "__main__":
    # do NOT modify below
    if len(sys.argv) != 3:
        raise ValueError("Wrong number of arguments!")

    try:
        f = open(sys.argv[1], 'r')
    except IOError:
        raise IOError("Input file not found!")

    init_state = [[0 for i in range(3)] for j in range(3)]
    goal_state = [[0 for i in range(3)] for j in range(3)]
    lines = f.readlines()

    i,j = 0, 0
    for line in lines:
        for number in line:
            if '0'<= number <= '8':
                init_state[i][j] = int(number)
                j += 1
                if j == 3:
                    i += 1
                    j = 0

    for i in range(1, 9):
        goal_state[(i-1)//3][(i-1)%3] = i
    goal_state[2][2] = 0

    puzzle = Puzzle(init_state, goal_state)
    ans = puzzle.solve()

    with open(sys.argv[2], 'a') as f:
        for answer in ans:
            f.write(answer+'\n')







