classdef MovingAverage < matlab.System
    properties (Nontunable)
        window_size = 10; % Fixed window size
    end
    properties (Access = protected)
        window; % Pre-allocate storage
        index = 0; % Circular index for replacing old values
        count = 0; % Count of received samples
    end
    methods (Access = protected)
        function setupImpl(obj)
            obj.window = zeros(1, obj.window_size); % Preallocate the buffer
        end

        function avg = stepImpl(obj, input)
            % Update the circular buffer
            obj.index = mod(obj.index, obj.window_size) + 1;
            obj.window(obj.index) = input;

            % Update count (handles first `window_size` elements correctly)
            obj.count = min(obj.count + 1, obj.window_size);

            % Compute moving average
            avg = mean(obj.window(1:obj.count));
        end
    end
end