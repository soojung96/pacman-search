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
    # problem:
    # A search problem defines the state space, start state, goal test, successor
    # function and cost function.  This search problem can be used to find paths
    # to a particular point on the pacman board.
    #
    # The state space consists of (x,y) positions in a pacman game.
    #
    # Note: this search problem is fully specified; you should NOT change it.
    #
    # problem.getSuccessors:
    # Returns successor states, the actions they require, and a cost of 1.
    #
    #  As noted in search.py:
    #      For a given state, this should return a list of triples,
    #  (successor, action, stepCost), where 'successor' is a
    #  successor to the current state, 'action' is the action
    #  required to get there, and 'stepCost' is the incremental
    #  cost of expanding to that successor

    closed_set = []
    fringe = util.Stack()
    startState = problem.getStartState()


    fringe.push((startState, []))
    #while fringe still has nodes to expand on
    while not fringe.isEmpty():
        #pop the fringe
        node = fringe.pop()
        #actions you will expand on
        expandedActions = node[1]
        #state you are expanding
        currentState = node[0]

        #before expanding the state, need to check if state is in the
        #closed set
        if problem.isGoalState(currentState):
            return expandedActions
        if currentState not in closed_set:
            #need to check if currentState is goal state before expanding
            successors = problem.getSuccessors(currentState)
            #need to add state to cclosed_set
            closed_set.append(currentState)
            for successor in successors:
                successorState = successor[0]
                successorAction = successor[1]
                actions = expandedActions + [successorAction]
                fringe.push((successorState, actions))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    closed_set = []
    fringe = util.Queue()
    startState = problem.getStartState()

    fringe.push((startState, []))
    #while fringe still has nodes to expand on
    while not fringe.isEmpty():
        #pop the fringe
        node = fringe.pop()
        #actions you will expand on
        expandedActions = node[1]
        #state you are expanding
        currentState = node[0]

        if problem.isGoalState(currentState):
            return expandedActions
        if currentState not in closed_set:
            successors = problem.getSuccessors(currentState)
            #need to add state to cclosed_set
            closed_set.append(currentState)
            for successor in successors:
                successorState = successor[0]
                successorAction = successor[1]
                actions = expandedActions + [successorAction]
                fringe.push((successorState, actions))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    closed_set = []
    fringe = util.PriorityQueue()
    startState = problem.getStartState()

    fringe.push((startState, [], 0), 0)
    #while fringe still has nodes to expand on
    while not fringe.isEmpty():
        #pop the fringe
        node = fringe.pop()
        #actions you will expand on
        expandedActions = node[1]
        #state you are expanding
        currentState = node[0]
        #cost of moving to that states
        cost = node[2]

        #before expanding the state, need to check if state is in the
        #closed set
        if problem.isGoalState(currentState):
            return expandedActions
        if currentState not in closed_set:
            #need to check if currentState is goal state before expanding
            successors = problem.getSuccessors(currentState)
            #need to add state to cclosed_set
            closed_set.append(currentState)
            for successor in successors:
                successorState = successor[0]
                successorAction = successor[1]
                successorCost = successor[2]
                actions = expandedActions + [successorAction]
                totalCost = cost + successorCost
                fringe.push((successorState, actions, totalCost), totalCost)

def nullHeuristic(state, problem=None):
    """
    A function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    closed_set = []
    fringe = util.PriorityQueue()
    startState = problem.getStartState()

    fringe.push((startState, [], 0), heuristic(startState, problem))
    #while fringe still has nodes to expand on
    while not fringe.isEmpty():
        #pop the fringe
        node = fringe.pop()
        #actions you will expand on
        expandedActions = node[1]
        #state you are expanding
        currentState = node[0]
        #cost of moving to that states
        cost = node[2]

        #before expanding the state, need to check if state is in the
        #closed set
        if problem.isGoalState(currentState):
            return expandedActions
        if currentState not in closed_set:
            #need to check if currentState is goal state before expanding
            successors = problem.getSuccessors(currentState)
            #need to add state to cclosed_set
            closed_set.append(currentState)
            for successor in successors:
                successorState = successor[0]
                successorAction = successor[1]
                successorCost = successor[2]
                actions = expandedActions + [successorAction]
                totalCost = cost + successorCost
                heuristicIncludedCost = totalCost + heuristic(successorState, problem)
                fringe.push((successorState, actions, totalCost), heuristicIncludedCost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
