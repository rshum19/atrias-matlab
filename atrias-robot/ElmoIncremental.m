% Simulink block for decoding an incremental encoder
% connected to an Elmo amplifier.
%
% This requires calibration information from an absolute encoder
% on the same axis, and will calibrate as soon as the absValid
% input becomes true.

% Input ports:
%     incTicks  Current Position reading from the Elmo
%     calibVal  Calibration location, from an absolute encoder
%     calibTrig Signal to trigger calibration -- when this becomes true, the calibration occurs.

classdef ElmoIncremental < Encoder
	methods (Access = protected)
		function [pos, vel] = stepImpl(this, incTicks, calibVal, calibTrig)
			
		end
	end
end
