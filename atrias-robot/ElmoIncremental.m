% Simulink block for decoding an incremental encoder
% connected to an Elmo amplifier.
%
% This requires calibration information from an absolute encoder
% on the same axis, and will calibrate as soon as the absValid
% input becomes true.
%
% Inputs:
%     incTicks  Current Position reading from the Elmo
%     calibVal  Calibration location, from an absolute encoder
%     calibTrig Signal to trigger calibration -- when this becomes true, the calibration occurs.

classdef ElmoIncremental < Encoder
	properties
		% Output units per tick
		unitsPerTick = 0

		% Sample time
		sample_time = 0.001
	end

	methods (Access = protected)
		function [pos, vel] = stepImpl(this, incTicks, calibVal, calibTrig)
			[pos, vel] = stepImpl@super(incTicks, this.sample_time, incTicks, calibVal, calibTrig, this.unitsPerTick, 0, inf, -inf, inf, -inf);
		end
	end
end
