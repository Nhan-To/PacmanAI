import util

class SearchProblem:

    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def getSuccessors(self, state):
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    startingNode = problem.getStartState()
    if problem.isGoalState(startingNode):
        return []
    
    myPQueue = util.PriorityQueue()
    paths = util.PriorityQueue()
    is_visited = []
    path = []
    myPQueue.push(startingNode, 0)
    paths.push([], 0)

    while not myPQueue.isEmpty():
        current_state = myPQueue.pop()
        current_path = paths.pop()
        if current_state in is_visited:
            continue
        is_visited.append(current_state)
        if problem.isGoalState(current_state):
            path = current_path
            break
        successors = problem.getSuccessors(current_state)
        for successor in successors:
            if successor[0] not in is_visited:
                path_cost = problem.getCostOfActions(current_path + [successor[1]])
                myPQueue.push(successor[0], path_cost)
                paths.push(current_path + [successor[1]], path_cost)
    return path


def nullHeuristic(state, problem=None):
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    startingNode = problem.getStartState()
    if problem.isGoalState(startingNode):
        return []

    myPQueue = util.PriorityQueue()
    myPQueue.push((startingNode, [], 0), 0)
    visitedNodes = []

    while (not myPQueue.isEmpty()):
        (curNode, curPath, prevCost)  = myPQueue.pop()

        if problem.isGoalState(curNode):
                return curPath
        
        if (curNode not in visitedNodes):
            visitedNodes.append(curNode)
            
            for (nextNode, nextAction, cost) in problem.getSuccessors(curNode):
                if (nextNode not in visitedNodes):
                    myPQueue.push((nextNode, curPath + [nextAction], prevCost + cost), prevCost + cost + heuristic(nextNode, problem))


astar = aStarSearch
ucs = uniformCostSearch
