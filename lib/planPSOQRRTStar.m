classdef planPSOQRRTStar < planPSORRTStar & planQRRTStar 

    methods
        function obj = planPSOQRRTStar(stateSpace, stateValidator)
            obj@planPSORRTStar(stateSpace, stateValidator)
            obj@planQRRTStar(stateSpace, stateValidator)
        end
                
    end
end