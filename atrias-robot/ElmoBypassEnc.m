% For encoders that are nominally connected to the Elmo
% amplifiers but may be bypassed through a Medulla

classdef ElmoBypassEnc < MedullaEncoder
	properties
		% Units per encoder tick
		unitsPerTick = 0

		% Medulla encoder unwrapping modulus
		unwrapMod = 0

		% Encoder routed through Medulla
		bypass@logical = false
	end

	methods (Access = protected)
		function [pos, vel] = stepImpl(this, elmoTicks, medullaTicks, counter, timestamp, calibVal, calibTrig)
			if this.bypass
				[pos, vel] = stepImpl@MedullaEncoder(...
					medullaTicks, counter, timestamp, ticks, calibVal, calibTrig, this.unitsPerTick, this.unwrapMod, -inf, inf, -inf, inf);
			else
				[pos, vel] = stepImpl@Encoder(elmoTicks, this.sample_time, ticks, calibVal, calibTrig, this.unitsPerTick, 0, inf, -inf, inf, -inf);
			end
		end
	end
end
