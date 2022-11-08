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
    return [s, s, w, s, w, w, s, w]


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
    visitedStates = []
    pathMap = {}  # For each state, store the parent
    solvedPath = []  # Return

    currentState = problem.getStartState()
    queue = util.Stack()  # Last-In-First-Out

    while not problem.isGoalState(currentState):
        # 1) Goal state is not current state, so, explore child nodes, push back to queue
        allNeighbours = problem.getSuccessors(currentState)  # Get Succesors
        for neighbourState in allNeighbours:  # Loop through neighbours
            if neighbourState[0] not in visitedStates:  # Prevents visiting nodes that already have been visited
                queue.push(neighbourState[0])
                pathMap[neighbourState[0]] = currentState, neighbourState[1]  # For each state (key) stores which is the parent and which is the direction parent->child

        # 1) Take top off, scream if the queue is empty (this should never happen if the maze is solvable)
        if queue.isEmpty():
            raise "DFS: Start and end point are not connected"
        else:
            currentState = queue.pop()  # Returns (x, y)
            visitedStates.append(currentState)  # Adds current state to already visited states

    # Return List Initialization
    currentChild = currentState, ""
    while currentChild[0] is not problem.getStartState():
        currentChild = pathMap[currentChild[0]]
        solvedPath.append(currentChild[1])

    solvedPath.reverse()
    return solvedPath


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    visitedStates = []
    pathMap = {}  # For each state, store the parent
    solvedPath = []  # Return

    currentState = problem.getStartState()
    queue = util.Queue()  # First-In-First-Out

    while not problem.isGoalState(currentState):
        # 1) Goal state is not current state, so, explore child nodes, push back to queue
        allNeighbours = problem.getSuccessors(currentState)  # Get Succesors
        for neighbourState in allNeighbours:  # Loop through neighbours
            if neighbourState[0] not in visitedStates:  # Prevents visiting nodes that already have been visited
                queue.push(neighbourState[0])
                pathMap[neighbourState[0]] = currentState, neighbourState[1]  # For each state (key) stores which is the parent and which is the direction parent->child

        # 1) Take top off, scream if the queue is empty
        if queue.isEmpty():
            raise "BFS: Start and end point are not connected"
        else:
            currentState = queue.pop()  # Returns (x, y)
            visitedStates.append(currentState)  # Adds current state to already visited states

    # Return List Initialization
    currentChild = currentState, ""
    while currentChild[0] is not problem.getStartState():
        currentChild = pathMap[currentChild[0]]
        solvedPath.append(currentChild[1])

    solvedPath.reverse()
    return solvedPath


def uniformCostSearch(problem):
    """Search the node of the least total cost first."""
    "*** YOUR CODE HERE ***"
    StatePQ = problem.getStartState()
    Goal: list = []
    goalReached = False
    states = util.PriorityQueue()
    states.push((StatePQ, []) ,0)
    while not states.isEmpty():
        xy, direcetion = states.pop()
        if problem.isGoalState(xy):
            return direcetion
        if xy not in Goal:
            successors = problem.getSuccessors(xy)
            for succ in successors:
                Sates = succ[0]
                if Sates not in Goal:
                    directions = succ[1]
                    newCost = direcetion + [directions]
                    states.push((Sates, direcetion + [directions]), problem.getCostOfActions(newCost))
            Goal.append(xy)
    return direcetion
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    goal: list = []
    goalReached = False
    StatePQ = util.PriorityQueue() #queue has maze((x,y), dir, cost)
    alreadyVisited = {} #it knows the visited route
    mazePath = {}
    mazeCost = {}
    nextStep = 0
    StatePQ.push((problem.getStartState(), ' ', 0), 0)
    while not goalReached:
        xy, dir, cost = StatePQ.pop()
        alreadyVisited[xy] = dir #xy get the next direction
        if problem.isGoalState(xy): #this position is the goal
            nextStep = xy #this is the position this moment
            goalReached = True
            break
        for i in problem.getSuccessors(xy): #for every succesor xy
            if i[0] not in alreadyVisited.keys(): #this position is not visited: i[0] is (x,y) of succesor
                totalcost = cost +i[2] #totalcost is the sum of the all costes
                heuristicCost = totalcost + heuristic(i[0], problem) # heuristic + totalcost by A*
                if not (i[0] in mazeCost.keys() and mazeCost[i[0]] <= totalcost):
                    mazeCost[i[0]] = totalcost #cost to get to the goalpoint
                    mazePath[i[0]] = xy # moment point to get to the succesor
                    StatePQ.push((i[0],i[1],totalcost), heuristicCost) # push xy, dir and totalcost on the stack
    while nextStep in mazePath.keys(): #follow the path
        goal.append(alreadyVisited[nextStep]) #put the action (NWSE) to get the goal list
        currentStep = mazePath[nextStep] #get currstep out of the succesor
        nextStep = currentStep
    goal.reverse() #reverse to get the right sequence
    return goal
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
