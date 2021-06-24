classdef planPSORRTStar < planRRTStar

   properties
        InertiaWeight
        InertiaDamping
        LearningCoefficients
        PopulationSize
        Iterations
        CostFunction
   end
    
    methods
        function obj = planPSORRTStar(stateSpace, stateValidator)
            obj@planRRTStar(stateSpace, stateValidator)
            
            obj.InertiaWeight = 0.5;
            obj.InertiaDamping = 0.3;
            obj.LearningCoefficients = [0.1 0.1];
            obj.PopulationSize = 15;
            obj.Iterations = 4;
            obj.CostFunction = @sphere2D;
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
                solnInfo.TreeData = [start;...
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
                samples = getRandomSample(obj);               % choose a random sample
                validIdx = obj.StateValidator.isStateValid(samples) == 1;
                validStates = samples(validIdx, :);
                randState = validStates(randi(size(validStates, 1)), :);

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
        
        function randState = getRandomSample(obj)
            sampleList = obj.StateSpace.sampleUniform(obj.PopulationSize);
            position = sampleList(:, 1:2);
            orientations = sampleList(:, 3);
            velocity = zeros(obj.PopulationSize, 2);
            
            cost = costUpdate(position, obj.CostFunction, obj.CurrentGoalState);
            
            pBest.Cost = cost;
            pBest.Position = position;
            pBest = pBestUpdate(position, cost, pBest);

            [minCost, idx] = min(cost);
            gBest.Cost = minCost;
            gBest.Position = position(idx, :);
            
            c1 = obj.LearningCoefficients(1);
            c2 = obj.LearningCoefficients(2);
            w = obj.InertiaWeight;
            wDamping = obj.InertiaDamping;
            iterations = obj.Iterations;
            
            for i = 1:iterations
                velocity = velocityUpdate(position, velocity, pBest, gBest, w, c1, c2);
                position = position + velocity;
                cost = costUpdate(position, obj.CostFunction, obj.CurrentGoalState);

                gBest = gBestUpdate(position, cost, gBest);
                pBest = pBestUpdate(position, cost, pBest);
                
                w = w * wDamping;
            end
            pBestPos = pBest.Position;
            [~, sortIdx] = sort(cost);
            pBestPos = pBestPos(sortIdx, :);
            randState = [pBestPos, orientations];

            
            function newCost = costUpdate(position, costFunction, goal)
                newCost = zeros(size(position, 1), 1);
                for j = 1:size(position, 1)
                    newCost(j) = costFunction(position(j, 1), position(j, 2), goal);
                end
            end

            function newVelocity = velocityUpdate(position, velocity, pBest, gBest, w, c1, c2)
               newVelocity = zeros(size(velocity));
               for k = 1:size(velocity, 1)
                   newVelocity(k, 1) = w*velocity(k, 1) + c1*rand*(pBest.Position(k, 1) - position(k, 1)) + c2*rand*(gBest.Position(1) - position(k, 1));
                   newVelocity(k, 2) = w*velocity(k, 2) + c1*rand*(pBest.Position(k, 2) - position(k, 2)) + c2*rand*(gBest.Position(2) - position(k, 2));
               end
            end

            function gBest = gBestUpdate(position, cost, gBest)
                [cMin, index] = min(cost);
                if cMin < gBest.Cost 
                    gBest.Cost = cMin;
                    gBest.Position = position(index, :);
                end
            end

            function newPBest = pBestUpdate(position, cost, pBest)
               costMat = pBest.Cost;
               posMat = pBest.Position;
               for l = 1:size(costMat, 1)
                   if costMat(l) > cost(l)
                       costMat(l) = cost(l);
                       posMat(l, :) = position(l, :);
                   end
               end
               newPBest.Cost = costMat;
               newPBest.Position = posMat;
            end
        end
        
    end
    
end