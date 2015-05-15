classdef Controller < matlab.System
%CONTROLLER Controller superclass.

  % PUBLIC PROPERTIES =====================================================
  properties (Logical = true)
    % Simulation flag
    isSim@logical = false
  end % properties

  properties
    % PS3 controller joysticks
    ps3Axes@double = zeros(1,4)
    % PS3 controller buttons
    ps3Buttons@double = zeros(1,17)
    % Torque limit (N*m)
    u_lim@double = 600
  end % properties

  % PROTECTED PROPERTIES ==================================================
  properties (Access = protected)
    % PS3 controller object
    ps3@PS3Controller
    % Run time (s)
    runTime@double = 0
    % Controller run state
    isRun@logical = false
  end % properties

  % CONSTANT PROPERTIES ===================================================
  properties (Constant = true, Hidden = true)
    % Sample interval time (s)
    sampleInterval = 0.001
  end % properties

  % ABSTRACT METHODS ======================================================
  methods (Abstract = true)
    [] = userSetup(obj)
    [u] = userStep(obj, q, dq)
    [userOut] = userOutput(obj)
  end % methods

  % PROTECTED METHODS =====================================================
  methods (Access = protected)
    function setupImpl(obj)
    %SETUPIMPL Initialize system object.

      % Initialize PS3 controller interface
      obj.ps3 = PS3Controller;

      % Run controller specific initialization
      obj.userSetup;
    end % setupImpl

    function [eStop, u, userOut] = stepImpl(obj, q, dq)
    %STEPIMPL System output and state update equations.

      % Initialize control input
      u = zeros(1,6);

      % Simulation overrides
      if obj.isSim
        % Coordinate transformation for simulator
        q(9) = -q(9);
        dq(9) = -dq(9);

        % Always run controller
        obj.isRun = true;
      end % if

      % Update PS3 controller
      obj.ps3.update(obj.ps3Axes, obj.ps3Buttons);

      % Parse start button data
      if obj.ps3.start.clickDuration > 1;
        obj.isRun = true;
      elseif obj.ps3.start.isPressed;
        obj.isRun = false;
      end % if

      % Parse PS button data
      if obj.ps3.ps.isPressed
        eStop = true;
      else
        eStop = false;
      end % if

      % Run controller logic
      if obj.isRun
        % Update run time
        obj.runTime = obj.runTime + obj.sampleInterval;

        % Run controller
        u = obj.userStep(q, dq);

        % Enable ramp up sequence
        if ~obj.isSim
          % Slowly increase torque limit
          u_lim = obj.u_lim*clamp(obj.runTime/2, 0, 1);

          % Limit torque commands
          u = clamp(u, -u_lim, u_lim);
        end % if

      else
        % Reset run time
        obj.runTime = 0;

        % Run controller specific initialization
        obj.userSetup;

        % Damping gains
        kd_leg = 150;
        kd_hip = 50;

        % Leg actuator torques computed to behave like virtual dampers
        u([1 2 4 5]) = (0 - dq([2 4 6 8]))*kd_leg;
        u([3 6]) = (0 - dq([9 10]))*kd_hip;
      end % if

      % Limit torque commands
      u = clamp(u, -obj.u_lim, obj.u_lim);

      % Simulation overrides
      if obj.isSim
        % Coordinate transformation for simulator
        u = -u;
        u(3) = -u(3);
      end % if

      % User output
      userOut = obj.userOutput;
    end % stepImpl
  end % methods
end % classdef

%% LOCAL FUNCTIONS ========================================================

function b = clamp(a, lim1, lim2)
  %CLAMP Clamp value between two bounds.

  % Find which limit is min and max
  a_min = min(lim1, lim2);
  a_max = max(lim1, lim2);

  % Clamp value between limits
  b = max(min(a, a_max), a_min);
end % clamp

function [y, dy] = cubic(x1, x2, y1, y2, dy1, dy2, x, dx)
  %CUBIC Cubic interpolation between values.

  % Limit range since curve fit is only valid within range
  x = clamp(x, x1, x2);

  % Interpolate
  a0 = 2*(y1 - y2) + (dy1 + dy2)*(x2 - x1);
  a1 = y2 - y1 - dy1*(x2 - x1) - a0;
  a2 = dy1*(x2 - x1);
  a3 = y1;
  s = (x - x1)/(x2 - x1);
  y = a0*s^3 + a1*s^2 + a2*s + a3;
  dy = dx*(-3*a0*(x - x1)^2/(x1 - x2)^3 + 2*a1*(x - x1)/(x1 - x2)^2 - a2/(x1 - x2));
end % cubic

function s = scaleFactor(f, tl, tu)
  %SCALEFACTOR Compute scalar (0 to 1) representing forces in leg.

  s = (clamp(f, tl, tu) - tl)/(tu - tl);
end % scaleFactor
