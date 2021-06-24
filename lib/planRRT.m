classdef planRRT < handle
    
    properties (Constant, Access = protected)
        GoalReached = 1
        MaxIterationsReached = 2
        MaxNumTreeNodesReached = 3
        MotionInCollision = 4
        InProgress = 5
        GoalReachedButContinue = 6
        ExistingState = 7
        Unassigned = 8
        
    end
    
    properties
        StateSpace
        StateValidator
        MaxNumTreeNodes
        MaxIterations
        MaxConnectionDistance
        GoalBias
        GoalReachedFcn
    end    
    
    properties (Access = protected)
        CurrentGoalState
    end
    
    methods
        
        function obj = planRRT(stateSpace, stateValidator)
            obj.StateSpace = stateSpace;
            obj.StateValidator = stateValidator;
            obj.MaxNumTreeNodes = 1e4;
            obj.MaxIterations = 1e4;
            obj.MaxConnectionDistance = 0.3;
            obj.GoalBias = 0.05;
            obj.GoalReachedFcn = @checkIfGoalIsReached;
        end
        
        function [pathObj, solnInfo] = plan(obj, start, goal)
            obj.CurrentGoalState = goal;
            tentativeGoalIds = [];
            
            % create search tree
            treeInternal = nav.algs.internal.SearchTree(start, obj.MaxNumTreeNodes);
            
            if obj.GoalReachedFcn(obj, start, goal)
                pathObj = navPath(obj.StateSpace, [startState; goalState]);
                solnInfo = struct();
                solnInfo.IsPathFound = true;
                solnInfo.ExitFlag = obj.GoalReached;
                solnInfo.NumNodes = 1;
                solnInfo.NumIterations = 0;
                solnInfo.TreeData = [startState;...
                                     nan(1,obj.StateSpace.NumStateVariables); ...
                                     startState;...
                                     goalState;...
                                     nan(1,obj.StateSpace.NumStateVariables)];
                return;
            end
            
            pathFound = false;
            statusCode = obj.Unassigned;
            numIterations = 0;
            
            for k = 1:obj.MaxIterations
                randState = getRandomSample(obj);               % choose a random sample
                if rand() < obj.GoalBias
                    randState = goal;                           % probability of choosing the exact goal state
                end
                
                [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal);
                if statusCode == obj.GoalReached
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId];
                    numIterations = k;
                    break;
                end
                
                if statusCode == obj.GoalReachedButContinue
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId];
                end
                
                if statusCode == obj.MaxNumTreeNodesReached
                    numIterations = k;
                    break;
                end  
                              
            end 
            
            if numIterations == 0
                numIterations = obj.MaxIterations;
            end
            
            treeData = treeInternal.inspect();
            numNodes = treeInternal.getNumNodes();
            
            exitFlag = statusCode;
            if statusCode > obj.MotionInCollision
                exitFlag = obj.MaxIterationsReached;
            end
            
            if pathFound
                costBest = inf;
                idBest = -1;
                for j = 1:length(tentativeGoalIds)
                    nid = tentativeGoalIds(j);
                    c = treeInternal.getNodeCostFromRoot(nid);
                    if c < costBest
                        idBest = nid;
                        costBest = c;
                    end
                end
                pathStates = treeInternal.tracebackToRoot(idBest);
                pathObj = navPath(obj.StateSpace, flip(pathStates'));
            else
                pathObj = navPath(obj.StateSpace);
            end
            
            solnInfo = struct;
            solnInfo.IsPathFound = pathFound;
            solnInfo.ExitFlag = exitFlag;
            solnInfo.NumNodes = numNodes;
            solnInfo.NumIterations = numIterations;
            solnInfo.TreeData = treeData';
        end
        
        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            statusCode = obj.InProgress;
            newNodeId = nan;
            idx = treeInternal.nearestNeighbor(randState);
            nearestNode = treeInternal.getNodeState(idx);
            
            newState = randState;
            d = obj.StateSpace.distance(randState, nearestNode);
            if d > obj.MaxConnectionDistance
                newState = obj.StateSpace.interpolate(nearestNode, randState, obj.MaxConnectionDistance/d);
            end
            
            if ~obj.StateValidator.isMotionValid(nearestNode, newState)
                statusCode = obj.MotionInCollision;
                return;
            end
            
            newNodeId = treeInternal.insertNode(newState, idx);
            if obj.GoalReachedFcn(obj, obj.CurrentGoalState, newState)
                statusCode = obj.GoalReached;
                return;
            end
            
            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return;
            end
            
        end
        
        function randState = getRandomSample(obj)
            randState = obj.StateSpace.sampleUniform();
        end
 
    end
    
end


