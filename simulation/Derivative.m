classdef Derivative < matlab.System
    properties (Access = private)
        prev_time;
        prev_val;
        prev_der;
    end
    methods (Access = protected)
        function setupImpl(obj)
            obj.prev_time = 0;
            obj.prev_val = 0;
            obj.prev_der = 0;
        end

        function der = stepImpl(obj, val)
            curr_time = getCurrentTime(obj);
            dt = curr_time - obj.prev_time;

            if curr_time == 0
                der = obj.prev_der;
                obj.prev_val = val;
            elseif dt > 0
                der = (val - obj.prev_val) / dt;
                obj.prev_val = val;
                obj.prev_time = curr_time;
                obj.prev_der = der;
            else
                der = obj.prev_der; % Use previous derivative if dt == 0
            end
        end
    end
end