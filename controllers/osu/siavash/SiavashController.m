classdef SiavashController < matlab.System %& matlab.system.mixin.Propagates
%CONTROLLER Controller superclass. 

  % PUBLIC PROPERTIES =====================================================
  properties (Logical = true)
    % Simulation flag
    isSim@logical = true
  end % properties

  properties
    % PS3 Controller Joysticks
    ps3Axes@double = zeros(1,4)
    % PS3 Controller Buttons
    ps3Buttons@double = zeros(1,17)
    % Torque Limit (N*m)
    u_lim@double = 600;
  end % properties

  % PROTECTED PROPERTIES ==================================================
  properties (Access = protected)
    % PS3 Controller
    ps3@PS3Controller
    % Run Time (s)
    runTime@double = 0
    % Controller run state
    isRun@logical = false
    
    sampleRate@double = 0.001;


  end % properties

  % CONSTANT PROPERTIES ===================================================
%   properties (Constant = true)
%     % Sample Rate (s)
%     sampleRate = 0.001
%   end % properties

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
%         if obj.isRun
%             obj.sampleRate=q(14)-obj.runTime;
%         end
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
        obj.runTime = obj.runTime + obj.sampleRate;

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



