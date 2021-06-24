classdef planPRRTStar < planRRTStar
    properties
        K
        ObservationDistance
        LearningRate
    end
    
    methods
        function obj = planPRRTStar(stateSpace, stateValidator)
            obj@planRRTStar(stateSpace, stateValidator)
            
            obj.K = 5;
            obj.ObservationDistance = 0;
            obj.LearningRate = 0.01;
        end
        
        function randState = getRandomSample(obj)
            randState = obj.StateSpace.sampleUniform();
            
            lidar = rangeSensor;
            [ranges, ~] = lidar(randState, obj.StateValidator.Map);
            
            
            for i = 1:obj.K
                
                FAtt = -2*(obj.CurrentGoalState' - randState');
                
%                 if isempty(find(ranges < obj.ObservationDistance, 1))
%                     return;
%                 else
                    randState = (randState' - obj.LearningRate*FAtt)';
%                 end
            end
        end
    end
end