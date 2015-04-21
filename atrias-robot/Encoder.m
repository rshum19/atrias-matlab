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
%     unwrapMod    Modulus for encoder unwrapping. Set to 0 to disable unwrapping
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

		% Current "state" -- most recent known valid values
		curState@EncoderState

		% Calibration location
		calibLoc = 0

		% Accumulated delta time (toward the next velocity calculation)
		dt = 0

		% Output position units per tick (for linear encoders)
		posUnitsPerTick@double = 0

		% Unwrapping modulus (0 for no unwrap)
		encUnwrapMod@int64 = int64(0)

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
			this.curState     = EncoderState;
			this.curState.pos = this.initPosOut;
			this.curState.vel = this.initVelOut;
		end
	end

	methods (Access = protected)
		% Determine whether the given new position and velocity are "trustworthy"
		function trust = trustData(this, newState)
			% We'll have a series of guards that return early if they detect
			% an issue with the data. At the end, if we pass all guards,
			% we set this to true.
			trust = false;

			% Throw out out-of-range positions
			if newState.pos < this.filtMinPos || newState.pos > this.filtMaxPos
				return
			end

			% Throw out out-of-range velocities
			if newState.vel < this.filtMinVel || newState.vel > this.filtMaxVel
				return
			end

			% Verify that the new data is finite (if the check is enabled)
			if this.checkInf && (~isfinite(newState.pos) || ~isfinite(newState.vel))
				return
			end

			% We've made it through all the checks, so trust the new value
			trust = true;
		end

		% Unwraps a delta value -- decodes a possibly wrapped tick count difference
		% into the range [-unwrapMod/2, unwrapMod/2)
		function out = unwrapTicks(this, ticks)
			% Don't unwrap if unwrapping is disabled
			if this.encUnwrapMod <= 0
				out = ticks;
				return
			end

			% The unwrapped tick count must be in the above range and equivalent
			% to the input modulo this.unwrapMod.
			% These two restrictions give a unique solution. This line guarantees
			% that these restrictions hold, and therefore gives the desired solution
			out = mod(ticks + (this.encUnwrapMod/2), this.encUnwrapMod) - (this.encUnwrapMod/2);
		end

		% Processes the given position. This takes in a tick count relative to the calibration
		% location and returns a position. May be overridden for nonlinear encoders
		function pos = decodePos(this, ticks, calibVal)
			pos = calibVal + this.posUnitsPerTick * double(ticks);
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
			calibState     = EncoderState;
			calibState.pos = this.decodePos(this.unwrapTicks(ticks - calibTicks), calibVal);

			% Calibrate iff the data is trustworthy.
			% We ignore the velocity because we don't have enough information
			% to compute it yet.
			out = this.trustData(calibState);
		end

		% Calibration function. Check if we need to calibrate,
		% calibrate, then record that we've calibrated.
		function calibrate(this, ticks, calibTicks, calibVal)
			% Quit of we aren't calibrating this cycle
			if ~this.shouldCalib(ticks, calibTicks, calibVal)
				return
			end

			% Compute the position relative to the calibration location
			this.curState.posTicks = this.unwrapTicks(ticks - calibTicks);

			% Copy over other calibration values
			this.calibLoc = calibVal;

			% Record that calibration has occurred
			this.calibrated = true;
		end

		% Main update routine. Handles position and velocity computation
		function update(this, ticks, dt)
			% Create a new state object for this iteration's computed position and velocity
			newState = EncoderState;

			% Compute the position delta and new position tick count
			dticks = this.unwrapTicks(ticks - this.curState.posTicks);
			newState.posTicks = this.curState.posTicks + dticks;

			% Update the delta time for the velocity calculation
			this.dt = this.dt + dt;

			% Compute the new position and velocity
			newState.pos = this.decodePos(newState.posTicks, this.calibLoc);
			newState.vel = (newState.pos - this.curState.pos) / this.dt;

			% Check if it's acceptable. If not, quit early (throwing out the data)
			if ~this.trustData(newState)
				return
			end

			% Store the new values and reset the delta time for the next iteration
			this.curState = newState;
			this.dt       = 0;
		end

		% Simulink's update function. Called at each model iteration.
		% See the top of this file for parameter documentation.
		% This should be overrode and called by subclasses
		function [pos, vel, isValid] = stepImpl(this, ticks, dt, calibTicks, calibVal, calibTrig, unitsPerTick, unwrapMod, minPos, maxPos, minVel, maxVel)
			% Copy over the filter settings
			this.filtMinPos = minPos;
			this.filtMaxPos = maxPos;
			this.filtMinVel = minVel;
			this.filtMaxVel = maxVel;

			% Copy over miscellaneous settings
			this.encUnwrapMod    = unwrapMod;
			this.posUnitsPerTick = unitsPerTick;

			% Compute a new position iff calibration's already been done.
			if this.calibrated
				this.update(int64(ticks), dt)
			end

			% Run the calibration routine, if the calibration trigger has been specified
			if calibTrig
				this.calibrate(ticks, calibTicks, calibVal)
			end

			% Set our outputs
			pos     = this.curState.pos;
			vel     = this.curState.vel;
			isValid = this.calibrated;
		end
	end
end
