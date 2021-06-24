classdef planRRTStar < planRRT
    properties
        BallRadiusConstant       
        ContinueAfterGoalReached
        FindOptimalPath
        OptimalPath
        MaxTime
    end
    
    properties (Constant, Access = protected)
        MaxTimeElapsed = 9;
    end
    
    methods
        function obj = planRRTStar(stateSpace, stateValidator)
            obj@planRRT(stateSpace, stateValidator);
            
            obj.BallRadiusConstant = 50000;             
            obj.ContinueAfterGoalReached = false;
            obj.MaxTime = nan;
        end
        
        function [pathObj, solnInfo] = plan(obj, start, goal)
            tic;
            obj.CurrentGoalState = goal;
            tentativeGoalIds = [];
            
            % create search tree
            treeInternal = nav.algs.internal.SearchTree(start, obj.MaxNumTreeNodes);
            treeInternal.setBallRadiusConstant(obj.BallRadiusConstant);
            treeInternal.setMaxConnectionDistance(obj.MaxConnectionDistance);
            
            if obj.GoalReachedFcn(obj, start, goal)
                pathObj = navPath(obj.StateSpace, [start; goal]);
                solnInfo = struct();
                solnInfo.IsPathFound = true;
                solnInfo.ExitFlag = obj.GoalReached;
                solnInfo.NumNodes = 1;
                solnInfo.NumIterations = 0;
                solnInfo.TreeData = [startState;...
                                     nan(1,obj.StateSpace.NumStateVariables); ...
                                     start;...
                                     goal;...
                                     nan(1,obj.StateSpace.NumStateVariables)];
                solnInfo.ElapsedTime = toc;
                solnInfo.PathDistance = obj.StateSpace.distance(start, goal);
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
               
                elapsedTime = toc;
                if elapsedTime > obj.MaxTime 
                    statusCode = obj.MaxTimeElapsed;
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
            if statusCode > obj.MotionInCollision && statusCode < obj.Unassigned
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
                pathLength = treeInternal.getNodeCostFromRoot(idBest);
            else
                pathObj = navPath(obj.StateSpace);
                pathLength = nan;
            end
            
            solnInfo = struct;
            solnInfo.IsPathFound = pathFound;
            solnInfo.ExitFlag = exitFlag;
            solnInfo.NumNodes = numNodes;
            solnInfo.NumIterations = numIterations;
            solnInfo.TreeData = treeData';
            solnInfo.ElapsedTime = elapsedTime;
            solnInfo.PathDistance = pathLength;
        end
        
        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            statusCode = obj.InProgress;
            newNodeId = nan;
            
            idx = treeInternal.nearestNeighbor(randState);
            nearestNode = treeInternal.getNodeState(idx);
            costNN = treeInternal.getNodeCostFromRoot(idx);
            
            d = obj.StateSpace.distance(nearestNode, randState);
            if d < 1e-10
                statusCode = obj.ExistingState;
                return;
            end
            
            newState = randState;
            if d > obj.MaxConnectionDistance
                newState = obj.StateSpace.interpolate(nearestNode, newState, obj.MaxConnectionDistance/d);
                d = obj.MaxConnectionDistance;
            end
            costNew = costNN + d;
            
            if ~obj.StateValidator.isMotionValid(nearestNode, newState)
                statusCode = obj.MotionInCollision;
                return;
            end
            
            nearIndices = treeInternal.near(newState);
            costMin = costNew;
            idMin = -1;
            
            for j = 1:length(nearIndices)
                idNear = nearIndices(j);
                stateNear = treeInternal.getNodeState(idNear);
                costNear = treeInternal.getNodeCostFromRoot(idNear);
                
                costTentative = costNear + obj.StateSpace.distance(stateNear, newState);
                if costMin > costTentative && obj.StateValidator.isMotionValid(stateNear, newState)
                    costMin = costTentative;
                    idMin = idNear;
                end
            end
            
            if idMin >= 0
                nearIndices = nearIndices(nearIndices ~= idMin);
            else
                idMin = idx;
            end
            
            idNew = treeInternal.insertNode(newState, idMin);
            newNodeId = idNew;
            
            for k = 1:length(nearIndices)
                idNear = nearIndices(k);
                stateNear = treeInternal.getNodeState(idNear);
                costNear = treeInternal.getNodeCostFromRoot(idNear);
                
                if costNear > costNew + obj.StateSpace.distance(stateNear, newState) && obj.StateValidator.isMotionValid(stateNear, newState)
                    rewireStatus = treeInternal.rewire(idNear, idNew);
                end
            end
            
            if obj.GoalReachedFcn(obj, obj.CurrentGoalState, newState)
                if obj.ContinueAfterGoalReached
                    statusCode = obj.GoalReachedButContinue;
                else
                    statusCode = obj.GoalReached;
                end
                return;
            end
            
            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return;
            end
        end
    end
end
