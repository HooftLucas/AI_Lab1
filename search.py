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
    # Abbreviating Directions
    from game import Directions
    # NORTH, EAST, SOUTH, WEST, STOP, LEFT, REVERSE, RIGHT
    n = Directions.NORTH
    e = Directions.EAST
    s = Directions.SOUTH
    w = Directions.WEST

    """
    Solving Problem
    """
    # ((5, 4), 'South', 1),
    solved: bool = False
    visitedNodes: list = []
    pathMap = {} # For each state, store the parent
    solvedPath = []

    currentState = problem.getStartState() # Get The Starting state
    queue = util.Stack() # Define Queue
    queue.push(currentState)

    while not solved:
        # 1) Take top off
        if queue.isEmpty():
            raise "fuck"
        else:
            currentState = queue.pop()
            visitedNodes.push(currentState)

        # 2) If current
        if problem.isGoalState(currentState):
            solved = True

            currentChild = currentState
            while currentChild is not None:
                solved.push(currentChild[2])
                currentChild = pathMap[currentChild]



        # 3) Explore nodes, push back to queue
        else:
            allNeighbours = problem.getSuccesorStates(currentState) # Get Succesors
            for state in allNeighbours: # Loop and push
                if state not in visitedStates: # Prevents visiting nodes that already have been visited
                    queue.push(state)
                    parentMap[state] = currentState



    # Return List Initialization
    return solvedPath

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    goal: list = [] #list
    goalReached = False
    stateQueue = util.Queue() #deze bevat (x,y), richting en kost
    alreadyVisited = {} # (x,y) : direction van de vorige coordinaten
    mazePath = {} #houd bij welk pad er uitgevoerd is
    nextstep = 0
    stateQueue.push(problem.getStartState(), '')

    while not goalReached:
        xy, direction = stateQueue.pop()
        alreadyVisited[xy] = direction # set the direction where he was in the list
        if problem.isGoalState(xy): #control if the destination is the goal
            nextstep = xy
            goalReached = True
            break #go out the while
        for i in problem.getSuccessors(xy):
            if i[0] not in alreadyVisited.keys() and i[0] not in (j[0] for j in stateQueue.list):
                mazePath[i[0]] = xy
                stateQueue.push((i[0], i[1]))

    while nextstep in mazePath.keys(): # follow the path
        goal.append(alreadyVisited[nextstep])
        currentStep = mazePath[nextstep] #set a new currentstep
        nextstep = currentStep
    goal.reverse()
    return goal

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    goal: list = [] #list
    goalReached = False
    StatePQ = util.PriorityQueue()
    alreadyVisited = {}
    mazepath = {}
    mazecost = {}
    nextstep = 0
    StatePQ.push((problem.getStartState(),'',0),0)
    while not goalReached:
        xy, direction = stateQueue.pop()
        alreadyVisited[xy] = direction # set the direction where he was in the list
        if problem.isGoalState(xy): #control if the destination is the goal
            nextstep = xy
            goalReached = True
            break #go out the while
        for i in problem.getSuccessors(xy):
            if i[0] not in alreadyVisited.keys():
                mazePath[i[0]] = xy
                stateQueue.push((i[0], i[1]))

    while nextstep in mazePath.keys(): # follow the path
        goal.append(alreadyVisited[nextstep])
        currentStep = mazePath[nextstep] #set a new currentstep
        nextstep = currentStep
    goal.reverse()
    return goal

    while not goalReached:
        xy, direction = stateQueue.pop()
        alreadyVisited[xy] = direction # set the direction where he was in the list
        if problem.isGoalState(xy): #control if the destination is the goal
            nextstep = xy
            goalReached = True
            break #go out the while
        for i in problem.getSuccessors(xy):
            if i[0] not in alreadyVisited.keys() and i[0] not in (j[0] for j in stateQueue.list):
                mazePath[i[0]] = xy
                stateQueue.push((i[0], i[1]))

    while nextstep in mazePath.keys(): # follow the path
        goal.append(alreadyVisited[nextstep])
        currentStep = mazePath[nextstep] #set a new currentstep
        nextstep = currentStep
    goal.reverse()
    return goal

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
