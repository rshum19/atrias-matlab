classdef TrackingController < Controller
%TRACKINGCONTROLLER Motor tracking debugging controller.
%
% Copyright 2015 Mikhail S. Jones

  % PUBLIC PROPERTIES =====================================================
  properties
    % Leg P Gain (N*m/rad)
    kp_leg@double = 3000
    % Leg D Gain (N*m*s/rad)
    kd_leg@double = 150
    % Hip P Gain (N*m/rad)
    kp_hip@double = 2000
    % Hip D Gain (N*m*s/rad)
    kd_hip@double = 75
    % Leg length amplitude (m)
    l_A@double = 0.1
    % Leg angle amplitude (rad)
    q_A@double = 0.2
    % Coulomb friction coefficient
    f_c@double = 0
    % Viscous friction coefficient
    f_v@double = 0
  end % properties
  
  % PROTECTED PROPERTIES ==================================================
  properties (Access = protected)
    % Speed
    s@double = 0
    % Time
    t@double = 0;
    % Output
    output@double = zeros(1,8)
  end % properties
  
  % CONSTANT PROPERTIES ===================================================
  properties (Constant = true)
    % Leg mass (kg)
    m_leg = 20.35
    % Gravity
    g = 9.81
    % Hip length (m)
    l_h = 0.1831
  end % properties
  
  % PUBLIC METHODS ========================================================
  methods
    function userSetup(obj)
    %USERSETUP Initialize system object.

      % Reset variables
      obj.s = 0;
      obj.t = 0;
    end % userSetup

    function userOut = userOutput(obj)
    %USEROUTPUT User output function.

      % Null output
      userOut = obj.output;
    end % userOutput

    function u = userStep(obj, q, dq)
    %USERSTEP System output and state update equations.
     
      % Parse PS3 controller
      if obj.ps3.up.isPressed
        obj.s = obj.s + 1;
      elseif obj.ps3.down.isPressed
        obj.s = obj.s - 1;
      elseif obj.isSim
        obj.s = 10;
      end % if
      
      % Advance counter
      obj.t = obj.t + obj.sampleInterval*obj.s;
      
      % Left leg state variables
      q_l_mA = q(8); dq_l_mA = dq(8); q_l_mB = q(6); dq_l_mB = dq(6);
      q_l_lA = q(7); dq_l_lA = dq(7); q_l_lB = q(5); dq_l_lB = dq(5);
      q_l_h = q(10); dq_l_h = dq(10);
      
      % Right leg state variables
      q_r_mA = q(4); dq_r_mA = dq(4); q_r_mB = q(2); dq_r_mB = dq(2);
      q_r_lA = q(3); dq_r_lA = dq(3); q_r_lB = q(1); dq_r_lB = dq(1);
      q_r_h = q(9); dq_r_h = dq(9);
        
      % Hip PD controller with feed-forward gravity compensation
      u_l_h = obj.m_leg*obj.g*obj.l_h + ...
        (0 - q_l_h)*obj.kp_hip + ...
        (0 - dq_l_h)*obj.kd_hip;
      u_r_h = -obj.m_leg*obj.g*obj.l_h + ...
        (0 - q_r_h)*obj.kp_hip + ...
        (0 - dq_r_h)*obj.kd_hip;

      % Target left leg actuator positions and velocities
      q_l_mA_tgt = real(pi + obj.q_A*sin(obj.t) - acos(0.8 + obj.l_A*cos(obj.t)));
      q_l_mB_tgt = real(pi + obj.q_A*sin(obj.t) + acos(0.8 + obj.l_A*cos(obj.t)));
      dq_l_mA_tgt = obj.s*real(obj.q_A*cos(obj.t) - (obj.l_A*sin(obj.t))/sqrt(1 - (0.8 + obj.l_A*cos(obj.t))^2));
      dq_l_mB_tgt = obj.s*real(obj.q_A*cos(obj.t) + (obj.l_A*sin(obj.t))/sqrt(1 - (0.8 + obj.l_A*cos(obj.t))^2));
      
      % Right leg target positions
      q_r_mA_tgt = real(pi - obj.q_A*sin(obj.t) - acos(0.8 - obj.l_A*cos(obj.t)));
      q_r_mB_tgt = real(pi - obj.q_A*sin(obj.t) + acos(0.8 - obj.l_A*cos(obj.t)));
      dq_r_mA_tgt = obj.s*real(-obj.q_A*cos(obj.t) + (obj.l_A*sin(obj.t))/sqrt(1 - (0.8 - obj.l_A*cos(obj.t))^2));
      dq_r_mB_tgt = obj.s*real(-obj.q_A*cos(obj.t) - (obj.l_A*sin(obj.t))/sqrt(1 - (0.8 - obj.l_A*cos(obj.t))^2));
      
      % Left leg PD controller with feed-forward friction compensation
      u_l_A = sign(dq_l_mA_tgt)*(obj.f_v*abs(dq_l_mA_tgt) + obj.f_c) + ...
        (q_l_mA_tgt - q_l_mA)*obj.kp_leg + ...
        (dq_l_mA_tgt - dq_l_mA)*obj.kd_leg;
      u_l_B = sign(dq_l_mB_tgt)*(obj.f_v*abs(dq_l_mB_tgt) + obj.f_c) + ...
        (q_l_mB_tgt - q_l_mB)*obj.kp_leg + ...
        (dq_l_mB_tgt - dq_l_mB)*obj.kd_leg;

      % Right leg PD controller with feed-forward friction compensation
      u_r_A = sign(dq_r_mA_tgt)*(obj.f_v*abs(dq_r_mA_tgt) + obj.f_c) + ...
        (q_r_mA_tgt - q_r_mA)*obj.kp_leg + ...
        (dq_r_mA_tgt - dq_r_mA)*obj.kd_leg;
      u_r_B = sign(dq_r_mB_tgt)*(obj.f_v*abs(dq_r_mB_tgt) + obj.f_c) + ...
        (q_r_mB_tgt - q_r_mB)*obj.kp_leg + ...
        (dq_r_mB_tgt - dq_r_mB)*obj.kd_leg;

      % Set motor torque command vector
      u = [u_r_B u_r_A u_r_h u_l_B u_l_A u_l_h];
      
      % Output tracking errors
      obj.output = [...
        q_l_mA_tgt - q_l_mA, ...
        q_l_mB_tgt - q_l_mB, ...
        q_r_mA_tgt - q_r_mA, ...
        q_r_mB_tgt - q_r_mB, ...
        dq_l_mA_tgt - dq_l_mA, ...
        dq_l_mB_tgt - dq_l_mB, ...
        dq_r_mA_tgt - dq_r_mA, ...
        dq_r_mB_tgt - dq_r_mB];
    end % userStep
  end % methods
end % classdef
