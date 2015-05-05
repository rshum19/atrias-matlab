classdef TorqueController < Controller
%TORQUECONTROLLER Motor torque debugging controller.
%
% Copyright 2015 Mikhail S. Jones

  % PUBLIC METHODS ========================================================
  methods
    function userSetup(obj)
    %USERSETUP Initialize system object.
    
    end % userSetup

    function userOut = userOutput(obj)
    %USEROUTPUT User output function.

      % Null output
      userOut = 0;
      
      % Force controller to run
      obj.isRun = true;
    end % userOutput

    function u = userStep(obj, q, dq)
    %USERSTEP System output and state update equations.

      % Initialize motor commands
      u_r_B = 0; u_r_A = 0; u_r_h = 0; u_l_B = 0; u_l_A = 0; u_l_h = 0;

      % Get selected motor from PS3 controller trigger button combos
      combo = [obj.ps3.l1.value, obj.ps3.l2.value, obj.ps3.r1.value, obj.ps3.r2.value];

      % Get torque command from PS3 left joy stick
      u_cmd = obj.u_lim*obj.ps3.leftStickY;

      % Interpret motor torque command from button combo
      if all(combo == [1 0 0 0])
        u_l_A = u_cmd;
      elseif all(combo == [0 1 0 0])
        u_l_B = u_cmd;
      elseif all(combo == [1 1 0 0])
        u_l_h = u_cmd;
      elseif all(combo == [0 0 1 0])
        u_r_A = u_cmd;
      elseif all(combo == [0 0 0 1])
        u_r_B = u_cmd;
      elseif all(combo == [0 0 1 1])
        u_r_h = u_cmd;
      end % if

      % Set motor torque command vector
      u = [u_r_B u_r_A u_r_h u_l_B u_l_A u_l_h];
    end % userStep
  end % methods
end % classdef
