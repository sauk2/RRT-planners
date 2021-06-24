classdef planQRRTStar < planRRTStar

    properties
        ParentDepthSearch
        ParentDepthRewire
    end
    properties(Access = protected)
        ParentNodeIds
    end
    
    methods
        function obj = planQRRTStar(stateSpace, stateValidator)
            obj@planRRTStar(stateSpace, stateValidator);
            
            obj.ParentDepthSearch = 2;
            obj.ParentDepthRewire = 1;
            obj.ParentNodeIds = containers.Map('KeyType', 'double', 'ValueType', 'double');
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
            
            
            for n = 1:obj.ParentDepthSearch
                if obj.ParentNodeIds.isKey(idMin)
                    idParent = obj.ParentNodeIds(idMin);
                    stateParent = treeInternal.getNodeState(idParent);
                    costParent = treeInternal.getNodeCostFromRoot(idParent);
                    
                    costTentativeParent = costParent + obj.StateSpace.distance(stateParent, newState);
                    if costMin > costTentativeParent && obj.StateValidator.isMotionValid(stateParent, newState)
                        costMin = costTentativeParent;
                        idMin = idParent;
                    end
                end
            end
     
            idNew = treeInternal.insertNode(newState, idMin);
            obj.ParentNodeIds(idNew) = idMin;
            
            newNodeId = idNew;
            
            for k = 1:length(nearIndices)
                idNear = nearIndices(k);
                stateNear = treeInternal.getNodeState(idNear);
                costNear = treeInternal.getNodeCostFromRoot(idNear);
                
                idNew = newNodeId;
                
                if costNear > costNew + obj.StateSpace.distance(stateNear, newState) && obj.StateValidator.isMotionValid(stateNear, newState)
%                     for l = 1:obj.ParentDepthRewire
%                         if obj.ParentNodeIds.isKey(idNew)
%                             idParentRewire = obj.ParentNodeIds(idNew);
%                             stateParentRewire = treeInternal.getNodeState(idParentRewire);
%                             costParentRewire = treeInternal.getNodeCostFromRoot(idParentRewire);
% 
%                             costTentativeParentRewire = costParentRewire + obj.StateSpace.distance(stateNear, stateParentRewire);
% 
%                             if costNear > costTentativeParentRewire && obj.StateValidator.isMotionValid(stateNear, stateParentRewire)
%                                 costNear = costTentativeParentRewire;
%                                 idNew = idParentRewire;
%                             end
%                         end
%                     end
                    
                    rewireStatus = treeInternal.rewire(idNear, idNew);
                    obj.ParentNodeIds(idNear) = idNew;
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