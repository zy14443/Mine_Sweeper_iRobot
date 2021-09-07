classdef TimerObj < handle
    properties
        theTimer
        Frequency   % Frequency at which the timer triggers
    end
    events
        TimerEvent
    end
    methods
        function this = TimerObj(freq)
            this.Frequency = freq;
            this.theTimer = timer( ...
                'ExecutionMode', 'fixedRate', ...
                'Period', 1 / freq, ...
                'TimerFcn', @(src, evt) this.notify( 'TimerEvent' ) );
            start(this.theTimer);
        end
        function delete(this)
            stop(this.theTimer);
            delete(this.theTimer);
        end
    end
end