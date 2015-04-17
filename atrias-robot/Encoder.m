% MATLAB System (Simulink block) for decoding an ATRIAS Renishaw encoder.
%
% This is currently specialized to the Renishaw encoders on ATRIAS,
% but could be easily extended to handle the robot's other encoders.
%
% This includes a filter for rejecting erroneous outputs from the encoders.
% If used on a different encoder type, this filter should be made optional
% (or modified, if used for the absolute encoders on the hips).
%
% This does not un-wrap the encoder values. If used for any other encoder,
% un-wrapping must be implemented! See Encoder.m for a clean way to do this.

classdef RSEncoder < matlab.System
	properties
		% Calibration position
		calibPos@double

		% Calibration Ticks
		calibTicks@double

		% Units per tick
		unitsPerTick@double

		% Minimum acceptable position
		minPos@double

		% Maximum acceptable position
		maxPos@double

		% Maximum acceptable speed
		maxSpd@double

		% Medulla counter modulus (wrapping value)
		counterMod = 256

		% Medulla timer frequency (Hz)
		medullaFreq = 32e6;

		% Cycle delta time (sample time)
		sampleTime@double
	end

	properties (Access = protected)
		% Whether or not this object has been initialized (i.e.
		% whether the first valid data has been received).
		isInitialized = false

		% Most recent known-valid values
		pos = 0
		vel = 0

		% Previous cycle values. Not necessarily from valid cycles!
		prevCounter = 0
		prevTs      = 0

		% Delta time (towards the next velocity calculation)
		dt = 0
	end

	methods (Access = protected)
		% Decodes a given tick count into the corresponding position.
		function pos = decodePos(this, ticks)
			pos = this.calibPos + this.unitsPerTick * (ticks - this.calibTicks);
		end

		% Determine whether the given new position and velocity are "trustworthy"
		function trust = trustData(this, newPos, newVel)
			trust = false;

			% Throw out duplicate values, as the Medulla latches
			% values when the encoder sets an error flag
			if newPos == this.pos
				return
			end

			% Throw out out-of-range positions
			if newPos < this.minPos || newPos > this.maxPos
				return
			end

			% Throw out out-of-range velocities
			if abs(newVel) > this.maxSpd
				return
			end

			% Verify that the velocity is finite
			% (a nonfinite velocity could occur (dt == 0))
			if ~isfinite(newVel)
				return
			end

			% We've made it through all the checks, so trust the new value
			trust = true;
		end

		% Initialization routine. This gets run each iteration until initialization is complete.
		function initialize(this, newTicks, newCounter, newTs)
			% Return early (don't initialize) if we have yet to receive data
			if newCounter == 0
				return
			end

			% Compute the current position for use in validation and initialization
			pos = this.decodePos(newTicks);

			% Check that the received position is sane. If not, don't initialize yet
			% (otherwise the filter will latch the bad value).
			if ~this.trustData(pos, 0)
				return
			end

			% We just received the first cycle of data! Initialize
			% all uninitialized members
			this.isInitialized = true;
			this.pos           = pos;
		end

		% Update routine. This gets run each cycle after initialization is complete
		function update(this, newTicks, newCounter, newTs)
			% Update the delta time and counter value
			this.dt = this.dt + this.sampleTime * mod(newCounter - this.prevCounter, this.counterMod) + (newTs - this.prevTs) / this.medullaFreq;

			% Compute the new position from the given data,
			% and use it to calculate the velocity as well
			newPos = this.decodePos(newTicks);
			newVel = (newPos - this.pos) / this.dt;

			% Ignore the new data if it's not trustworthy
			if ~this.trustData(newPos, newVel)
				return
			end

			% Store the new position and velocity
			this.pos = newPos;
			this.vel = newVel;

			% Reset the delta time for the next velocity calculation
			this.dt = 0;
		end

		function [pos, vel] = stepImpl(this, newTicks, newCounter, newTs)
			% If we're initialized, perform a normal update. Otherwise,
			% run the initialization routine
			if this.isInitialized
				this.update(newTicks, newCounter, newTs);
			else
				this.initialize(newTicks, newCounter, newTs);
			end

			% Update the "previous cycle" values
			this.prevCounter = newCounter;
			this.prevTs      = newTs;

			% Set our outputs
			pos = this.pos;
			vel = this.vel;
		end
	end
end
