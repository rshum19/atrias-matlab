% Easy-to-use incremental encoder decoding block

classdef IncEncoder < MedullaEncoder
	properties
		% Units per tick
		unitsPerTick = 0

		% Unwrapping modulus
		unwrapMod@int64 = int64(2^16)
	end

	methods (Access = protected)
		function [pos, vel] = stepImpl(this, ticks, counter, timestamp, absPos, absIsValid)
			% Forward the work to the MedullaEncoder class
			[pos, vel] = stepImpl@MedullaEncoder(...
				this,                         ...
				ticks,                        ...
				counter,                      ...
				timestamp,                    ...
				int64(ticks),                 ...
				absPos,                       ...
				absIsValid && (counter ~= 0), ...
				this.unitsPerTick,            ...
				this.unwrapMod,               ...
				-inf,                         ...
				inf,                          ...
				-inf,                         ...
				inf);
		end
	end
end
