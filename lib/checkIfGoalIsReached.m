function isReached = checkIfGoalIsReached(planner, goalState, newState)

isReached = false;
threshold = 5;          % either 0.5 or 3

if planner.StateSpace.distance(newState, goalState) < threshold  
        isReached = true;
end

end

