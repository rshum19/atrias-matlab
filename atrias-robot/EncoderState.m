% Structure for encompassing the instantaneous state of
% an encoder. Used to keep updating sane within Encoder

classdef EncoderState < handle
	properties
		% Current position (output units)
		pos = 0

		% Current velocity (output units)
		vel = 0

		% Current tick count relative to the calibration location
		posTicks@int64 = int64(0)
	end
end
