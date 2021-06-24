classdef planPQRRTStar < planPRRTStar & planQRRTStar

    methods
        function obj = planPQRRTStar(stateSpace, stateValidator)
            obj@planPRRTStar(stateSpace, stateValidator);
            obj@planQRRTStar(stateSpace, stateValidator)
        end

    end
end
