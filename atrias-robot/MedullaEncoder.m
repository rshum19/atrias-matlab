% Simulink block to handle Medulla-based encoder decoding.
% This is expected to be subclassed instead of used directly,
% similar to Encoder, and is written in a similarly extensible
% manner.

classdef MedullaEncoder < Encoder
	properties
		% Sample time
		sample_time = .001

		% Medulla counter modulus
		counterMod = 256

		% Medulla clock frequency
		medullaFreq = 32e6
	end

	properties (Access = protected)
		% Previous Medulla counter output
		prevCntr = 0

		% Previous Medulla timestamp output
		prevTs = 0
	end

	methods (Access = protected)
		function [pos, vel, isValid] = stepImpl(this, ticks, counter, timestamp, calibTicks, calibVal, calibTrig, unitsPerTick, unwrapMod, minPos, maxPos, minVel, maxVel)
			% Compute the delta time value for this iteration
			dt = this.sample_time * mod(double(counter) - this.prevCntr, this.counterMod) + (double(timestamp) - this.prevTs) / this.medullaFreq;

			% Update the stored values used in the above calculation
			this.prevCntr = double(counter);
			this.prevTs   = double(timestamp);

			% Forward the remaining work to the Encoder class
			[pos, vel, isValid] = stepImpl@Encoder(this, ticks, dt, calibTicks, calibVal, calibTrig, unitsPerTick, unwrapMod, minPos, maxPos, minVel, maxVel);
		end
	end
end
