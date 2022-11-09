# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # each item in the stack will consist:
    #   [node, action to reach next node, successors of node]
    path = util.Stack()
    current = [problem.getStartState(), '', ()]
    visited = [current[0]]

    while not problem.isGoalState(current[0]):
        if not len(current[2]):
            current[2] = tuple(problem.getSuccessors(current[0]))

        for node, move, cost in reversed(current[2]):
            if node not in visited:
                current[1] = move
                path.push(current)
                visited.append(node)
                path.push([node, '', ()])
                break

        if path.isEmpty():
            return
        current = path.pop()

    return [x[1] for x in path.list]

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # astar without taking cost into account == bfs
    return aStarSearch(problem, costDependent=False)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # astar with h(x) = 0 == ucs
    return aStarSearch(problem, heuristic=nullHeuristic, costDependent=True)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic, costDependent=True):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # paths will consist:
    #   [node, actions to reach node]
    # paths will use a PriorityQueue if cost dependent else a Queue
    paths = util.Queue()
    visited = []
    start = (problem.getStartState(), [])
    paths.push(start)

    if costDependent:
        paths = util.PriorityQueue()
        costs = util.Counter()
        costs[str(start[0])] += heuristic(start[0], problem)
        paths.push(start, costs[str(start[0])])

    while not paths.isEmpty():
        node, path = paths.pop()

        if problem.isGoalState(node):
            return path

        if not node in visited:
            visited.append(node)
            for nnode, move, cost in problem.getSuccessors(node):
                if not costDependent:
                    paths.push((nnode, path + [move]))
                    continue
                costs[str(nnode)] = costs[str(node)] - heuristic(node, problem)
                costs[str(nnode)] += cost + heuristic(nnode, problem)
                paths.push((nnode, path + [move]), costs[str(nnode)])

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
