% Generic Simulink block for a variety of encoder types.
% Although this is directly usable, it is expected that the user will subclass
% this block to specify settings for specific sensor types and to extend
% the capabilities of this block.
%
% This includes a (optional) filter for rejecting erroneous outputs from the encoders.
%
% Most of the settings have been made inputs so they can be changed on the fly
% by the Simulink model or set by a subclass.

classdef Encoder < matlab.System
	properties
		% Upper raw value (0 for no unwrap)
		maxRawCnt@int64 = 0

		% Lower raw value (0 for no unwrap)
		minRawCnt@int64 = 0

		% Initial position output
		initPosOut@double = 0

		% Initial velocity output
		initVelOut@double = 0
	end

	properties (Logical = true)
		% Check for non-finite pos or vel
		checkInf@logical = false
	end

	properties (Access = protected)
		% Whether or not a calibration has been performed.
		calibrated = false

		% Most recent known-valid values
		pos = 0
		vel = 0

		% Current position relative to the calibration position, in ticks
		posTicks@int64 = 0

		% Accumulated delta time (toward the next velocity calculation)
		dt = 0

		% Data filter properties. These are set at each iteration by
		% stepImpl. These are properties, rather than passed in,
		% to aid subclasses in overriding trustData()
		minPos = -inf % Minimum acceptable position
		maxPos =  inf % Maximum acceptable position
		maxVel =  inf % Maximum acceptable velocity
		minVel = -inf % Maximum acceptable velocity
	end

	methods (Access = protected)
		% Constructor; does some basic initialization
		function this = Encoder
			% Copy over the initial position and velocity values
			this.pos = this.initPosOut;
			this.vel = this.initVelOut;
		end

		% Determine whether the given new position and velocity are "trustworthy"
		function trust = trustData(this, newPos, newVel)
			% We'll have a series of guards that return early if they detect
			% an issue with the data. At the end, if we pass all guards,
			% we set this to true.
			trust = false;

			% Throw out out-of-range positions
			if newPos < this.minPos || newPos > this.maxPos
				return
			end

			% Throw out out-of-range velocities
			if newVel < this.minVel || newVel > this.maxVel
				return
			end

			% Verify that the new data is finite (if the check is enabled)
			if this.checkInf && (~isfinite(newPos) || ~isfinite(newVel))
				return
			end

			% We've made it through all the checks, so trust the new value
			trust = true;
		end

		% Simulink's update function. Called at each model iteration.
		% See the top of this file for parameter documentation
		function [pos, vel, isValid] = stepImpl(this, ticks, dt, calibTicks, calibVal, calibTrig, minPos, maxPos, minVel, maxVel)
			% Copy over the filter settings
			this.minPos = minPos;
			this.maxPos = maxPos;
			this.minVel = minVel;
			this.maxVel = maxVel;

			% Run the calibration routine, if the calibration trigger has been specified
			if calibTrig
				this.calibrate(ticks, calibTicks, calibVal)
			end

			% Compute a new position iff calibration's already been done.
			if this.calibrated
				this.update(ticks, dt)
			end

			% Set our outputs
			pos     = this.pos;
			vel     = this.vel;
			isValid = this.calibrated;
		end
	end
end
