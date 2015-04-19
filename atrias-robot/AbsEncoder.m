% Easy-to-use absolute encoder decoding block

classdef AbsEncoder < MedullaEncoder
	properties
		% Units per tick
		unitsPerTick = 0

		% Calibration value (ticks)
		calibTicks = 0

		% Calibration location
		calibVal = 0

		% Maximum acceptable position
		maxPos = inf

		% Minimum acceptable position
		minPos = -inf

		% Maximum acceptable speed
		maxSpd = inf
	end

	methods (Access = protected)
		function [pos, vel, isValid] = stepImpl(this, ticks, counter, timestamp)
			% Forward the work up to MedullaEncoder
			[pos, vel, isValid] = stepImpl@super(...
				ticks,                  ...
				counter,                ...
				timestamp,              ...
				int64(this.calibTicks), ...
				calibVal,               ...
				counter ~= 0,           ...
				this.unitsPerTick,      ...
				this.minPos,            ...
				this.maxPos,            ...
				-this.maxSpd,           ...
				this.maxSpd);
		end
	end
end
