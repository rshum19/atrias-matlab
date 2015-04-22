% For encoders that are nominally connected to the Elmo
% amplifiers but may be bypassed through a Medulla

classdef ElmoBypassEnc < MedullaEncoder
	properties
		% Units per encoder tick
		unitsPerTick = 0

		% Medulla encoder unwrapping modulus
		unwrapMod = 2^16

		% Encoder routed through Medulla
		bypass@logical = false
	end

	methods (Access = protected)
		function [pos, vel] = stepImpl(this, elmoTicks, medullaTicks, counter, timestamp, calibVal, calibTrig)
			if this.bypass
				[pos, vel] = stepImpl@MedullaEncoder(...
					this, medullaTicks, counter, timestamp, medullaTicks, calibVal, calibTrig, this.unitsPerTick, this.unwrapMod, -inf, inf, -inf, inf);
			else
				[pos, vel] = stepImpl@Encoder(this, elmoTicks, this.sample_time, elmoTicks, calibVal, calibTrig, this.unitsPerTick, int64(0), -inf, inf, -inf, inf);
			end
		end
	end
end
