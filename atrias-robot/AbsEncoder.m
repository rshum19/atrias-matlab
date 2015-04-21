% Easy-to-use absolute encoder decoding block

classdef AbsEncoder < MedullaEncoder
	properties
		% Units per tick
		unitsPerTick = 0

		% Calibration value (ticks)
		calibTicks = 0

		% Calibration location
		calibVal = 0

		% Unwrapping modulus
		unwrapMod@int64 = int64(0)

		% Drop repeated values
		dropRepeatVals = false

		% Minimum acceptable position
		minPos = -inf

		% Maximum acceptable position
		maxPos = inf

		% Maximum acceptable speed
		maxSpd = inf
	end

	methods (Access = protected)
		% Check for repeated values, if enabled
		function trust = trustData(this, newState)
			% Like Encoder's trust Data, we have guards which return
			% early if any issues are encountered
			trust = false;

			% Check for duplicate values, if enabled
			if this.dropRepeatVals && (newState.rawTicks == this.curState.rawTicks)
				return
			end

			% Forward up to the superclass's trustData() function
			trust = trustData@MedullaEncoder(this, newState);
		end

		function [pos, vel, isValid] = stepImpl(this, ticks, counter, timestamp)
			% Forward the work up to MedullaEncoder
			[pos, vel, isValid] = stepImpl@MedullaEncoder(...
				this,                   ...
				ticks,                  ...
				counter,                ...
				timestamp,              ...
				int64(this.calibTicks), ...
				this.calibVal,          ...
				counter ~= 0,           ...
				this.unitsPerTick,      ...
				this.unwrapMod,         ...
				this.minPos,            ...
				this.maxPos,            ...
				-this.maxSpd,           ...
				this.maxSpd);
		end
	end
end
