% Generic Simulink block for a variety of encoder types.
% Although this is directly usable, it is expected that the user will subclass
% this block to specify settings for specific sensor types and to extend
% the capabilities of this block.
%
% This includes a (optional) filter for rejecting erroneous outputs from the encoders.
%
% Most of the settings have been made inputs so they can be changed on the fly
% by the Simulink model or set by a subclass.
%
% Inputs:
%     ticks        Current raw encoder reading
%     dt           Delta time since the last encoder reading
%     calibTicks   Calibration location in ticks
%     calibVal     Output position at the calibration location.
%     calibTrig    Setting this to true triggers calibration to occur
%     unitsPerTick The number of output units per tick, for linear encoders
%     minPos       Minimum acceptable position. -inf to disable minimum position check
%     maxPos       Maximum acceptable position. +inf to disable maximum position check
%     minVel       Minimum acceptable velocity. -inf to disable minimum velocity check
%     maxVel       Maximum acceptable velocity. +inf to disable maximum velocity check
%
% Outputs:
%     pos     Current position
%     vel     Current velocity
%     isValid Whether the output position and velocity are valid yet

classdef Encoder < matlab.System
	properties
		% Unwrapping modulus (0 for no unwrap)
		unwrapMod = 0

		% Initial position output
		initPosOut@double = 0

		% Initial velocity output
		initVelOut@double = 0

		% Check for non-finite pos or vel
		checkInf@logical = false

		% Calibrate only once
		calibOnce@logical = true
	end

	properties (Access = protected)
		% Whether or not a calibration has been performed.
		calibrated = false

		% Most recent known-valid values
		pos = 0
		vel = 0

		% Current position relative to the calibration position, in ticks
		posTicks@int64 = int64(0)

		% Last iteration's tick count, for differentiation.
		prevTicks@int64 = int64(0)

		% Calibration location
		lastCalibVal = 0

		% Accumulated delta time (toward the next velocity calculation)
		dt = 0

		% Output position units per tick (for linear encoders)
		posUnitsPerTick@double = 0

		% Data filter properties. These are set at each iteration by
		% stepImpl. These are properties, rather than passed in,
		% to aid subclasses in overriding trustData()
		filtMinPos = -inf % Minimum acceptable position
		filtMaxPos =  inf % Maximum acceptable position
		filtMaxVel =  inf % Maximum acceptable velocity
		filtMinVel = -inf % Maximum acceptable velocity
	end

	methods
		% Constructor; does some basic initialization
		function this = Encoder
			% Copy over the initial position and velocity values
			this.pos = this.initPosOut;
			this.vel = this.initVelOut;
		end
	end

	methods (Access = protected)
		% Determine whether the given new position and velocity are "trustworthy"
		function trust = trustData(this, newPos, newVel)
			% We'll have a series of guards that return early if they detect
			% an issue with the data. At the end, if we pass all guards,
			% we set this to true.
			trust = false;

			% Throw out out-of-range positions
			if newPos < this.filtMinPos || newPos > this.filtMaxPos
				return
			end

			% Throw out out-of-range velocities
			if newVel < this.filtMinVel || newVel > this.filtMaxVel
				return
			end

			% Verify that the new data is finite (if the check is enabled)
			if this.checkInf && (~isfinite(newPos) || ~isfinite(newVel))
				return
			end

			% We've made it through all the checks, so trust the new value
			trust = true;
		end

		% Unwraps a delta value -- decodes a possibly wrapped tick count difference
		% into the range [-unwrapMod/2, unwrapMod/2)
		function out = unwrapTicks(this, ticks)
			% Don't unwrap if unwrapping is disabled
			if this.unwrapMod <= 0
				out = ticks;
				return
			end

			% The unwrapped tick count must be in the above range and equivalent
			% to the input modulo this.unwrapMod.
			% These two restrictions give a unique solution. This line guarantees
			% that these restrictions hold, and therefore gives the desired solution
			out = mod(ticks + (this.unwrapMod/2), this.unwrapMod) - (this.unwrapMod/2);
		end

		% Processes the given position. This takes in a tick count relative to the calibration
		% location and returns a position. May be overridden for nonlinear encoders
		function pos = decodePos(this, ticks, calibVal)
			pos = calibVal + this.posUnitsPerTick * ticks;
		end

		% Checks if we should calibrate on this cycle.
		% This will only be called when calibTrig is true.
		function out = shouldCalib(this, ticks, calibTicks, calibVal)
			% Make sure we're not recalibrating unless configured to do so
			if this.calibOnce && this.calibrated
				out = false;
				return
			end

			% Compute the position given the current tick count and calibration location.
			% Assumes the encoder is within half a (modular) rotation of the calibration rotation.
			pos = this.decodePos(this.unwrapTicks(ticks - calibTicks), calibVal);

			% Calibrate iff the data is trustworthy.
			% We ignore the velocity because we don't have enough information
			% to compute it yet.
			out = this.trustData(pos, 0);
		end

		% Calibration function. Check if we need to calibrate,
		% calibrate, then record that we've calibrated.
		function calibrate(this, ticks, calibTicks, calibVal)
			% Quit of we aren't calibrating this cycle
			if ~this.shouldCalib(ticks, calibTicks, calibVal)
				return
			end

			% Compute the position relative to the calibration location
			this.posTicks = this.unwrapTicks(ticks - calibTicks);

			% Copy over other calibration values
			this.lastCalibVal = calibVal;

			% Record that calibration has occurred
			this.calibrated = true;
		end

		% Main update routine. Handles position and velocity computation
		function update(this, ticks, dt)
			% Compute the position delta and new position tick count
			dticks = this.unwrapTicks(ticks - this.prevTicks);
			newPosTicks = this.posTicks + dticks;

			% Update the delta time for the velocity calculation
			this.dt = this.dt + dt;

			% Compute the new position and velocity
			pos = this.decodePos(newPosTicks, this.lastCalibVal);
			vel = (pos - this.pos) / this.dt;

			% Check if it's acceptable. If not, quit early (throwing out the data)
			if ~this.trustData(pos, vel)
				return
			end

			% Store the new values, and reset the delta time for the next iteration
			this.posTicks = newPosTicks;
			this.pos      = pos;
			this.vel      = vel;
			this.dt       = 0;
		end

		% Simulink's update function. Called at each model iteration.
		% See the top of this file for parameter documentation.
		% This should be overrode and called by subclasses
		function [pos, vel, isValid] = stepImpl(this, ticks, dt, calibTicks, calibVal, calibTrig, unitsPerTick, minPos, maxPos, minVel, maxVel)
			% Copy over the filter settings
			this.filtMinPos = minPos;
			this.filtMaxPos = maxPos;
			this.filtMinVel = minVel;
			this.filtMaxVel = maxVel;

			% Copy over miscellaneous settings
			this.posUnitsPerTick = unitsPerTick;

			% Compute a new position iff calibration's already been done.
			if this.calibrated
				this.update(ticks, dt)
			end

			% Run the calibration routine, if the calibration trigger has been specified
			if calibTrig
				this.calibrate(ticks, calibTicks, calibVal)
			end

			% Set our outputs
			pos     = this.pos;
			vel     = this.vel;
			isValid = this.calibrated;

			% Copy over the previous values for differentiation purposes
			this.prevTicks = ticks;
		end
	end
end
