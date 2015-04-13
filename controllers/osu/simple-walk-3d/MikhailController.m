classdef MikhailController < Controller
%MIKHAILCONTROLLER Mikhail's controller.
%
% Copyright 2015 Mikhail S. Jones

  % PUBLIC PROPERTIES =====================================================
  properties (Logical = true)
    % Trigger based on time
    isTimeTrig@logical = true
    % Trigger based on force
    isForceTrig@logical = true
  end % properties

  properties
    % Time invariant trigger threshold
    timeThres@double = 1
    % Force trigger threshold
    forceThres@double = 0.25
    % Leg trajectory velocity scaling factor
    dt_gain@double = 0.35
    % Leg P Gain (N*m/rad)
    kp_leg@double = 3000
    % Leg D Gain (N*m*s/rad)
    kd_leg@double = 150
    % Hip P Gain (N*m/rad)
    kp_hip@double = 2200
    % Hip D Gain (N*m*s/rad)
    kd_hip@double = 70
    % Torso Gain Scaling
    s_torso@double = 0.5
    % Swing Leg Gain Scalaing
    s_leg@double = 0.5
    % Lower Torque Threshold (N*m)
    thres_lo@double = 40
    % Upper Torque Threshold (N*m)
    thres_hi@double = 80
    % Filter Time Constant (s)
    tau_c@double = 0.12
    % Step Duration (s)
    t_step@double = 0.35
    % Leg Extension Gain (m)
    l_ext_gain@double = 0.02
    % X Velocity Feed-Forward Gain
    dx_gain@double = 0.18
    % X Velocity Error P Gain
    dx_err_p_gain@double = 0.1
    % X Velocity Error D Gain
    dx_err_d_gain@double = 0.1
    % Y Velocity Feed-Forward Gain
    dy_gain@double = 0.2
    % Y Velocity Error P Gain
    dy_err_p_gain@double = 0.1
    % Y Velocity Error D Gain
    dy_err_d_gain@double = 0.1
    % Hip Offset (m)
    y0_offset@double = 0.14
    % Hip Offset Gain
    y0_gain@double = 0.03
  end % properties

  % PROTECTED PROPERTIES ==================================================
  properties (Access = protected)
    % Length of stance leg at switch
    l_leg_last@double = 0.91
    % Storage for passing signals to output function
    tmp@double = zeros(1,2)
    % Time since last step (s)
    t@double = 0
    % Gait mode
    gaitMode@GaitMode
    % Current stance leg (1 == Left, -1 == Right)
    stanceLeg@double = 1
    % Nominal Leg Length (m)
    l0_leg@double = 0.91
    % Swing Leg Retraction (m)
    l_ret@double = 0.2
    % X Center of Mass Offset (m)
    x_offset@double = 0.015
    % Y Center of Mass Offset (m)
    y_offset@double = 0.075
    % Stance toe X position of last step (m)
    x_st_e@double = 0
    % Swing toe X position of last step (m)
    x_sw_e@double = 0
    % Estimated X position (m)
    x_est@double = 0
    % Estimated Y position (m)
    y_est@double = 0
    % Estimated X velocity (m/s)
    dx_est@double = 0
    % Estimated Y velocity (m/s)
    dy_est@double = 0
    % Estimated X velocity of last step (m/s)
    dx_est_e@double = 0
    % Estimated Y velocity of last step (m/s)
    dy_est_e@double = 0
    % Estimated average Y velocity of last step (m/s)
    dy_est_avg@double = 0
    % Target X velocity (m/s)
    dx_tgt@double = 0
    % Target Y velocity (m/s)
    dy_tgt@double = 0
  end % properties

  % CONSTANT PROPERTIES ===================================================
  properties (Constant = true)
    % Center of mass offset trim increment (m)
    trimIncrement = 0.005
    % Leg rotational spring constant (N*m/rad)
    ks_leg = 2950
    % Torso mass (kg)
    m_torso = 22.2
    % Leg mass (kg)
    m_leg = 20.35
    % Total mass (kg)
    m_total = 62.9
    % Gravity
    g = 9.81
  end % properties

  % PUBLIC METHODS ========================================================
  methods
    function userSetup(obj)
    %USERSETUP Initialize system object.

      % Reset objects
      obj.gaitMode = GaitMode.Normal;

      % Reset parameters
      obj.t = 0;
      obj.x_st_e = 0;
      obj.x_sw_e = 0;
      obj.x_est = 0;
      obj.y_est = 0;
      obj.dx_est = 0;
      obj.dy_est = 0;
      obj.dx_est_e = 0;
      obj.dy_est_e = 0;
      obj.dy_est_avg = 0;
      obj.dx_tgt = 0;
      obj.dy_tgt = 0;
      obj.l_leg_last = obj.l0_leg;
    end % userSetup

    function userOut = userOutput(obj)
    %USEROUTPUT User output function.

      userOut = [obj.t, obj.tmp, obj.l_leg_last, ...
        obj.x_est, obj.y_est, ...
        obj.dx_est, obj.dy_est, ...
        obj.dx_tgt, obj.dy_tgt];
    end % userOutput

    function u = userStep(obj, q, dq)
    %USERSTEP System output and state update equations.

      % Initialize control input
      u = zeros(1,6);

      % Update stance timer
      obj.t = obj.t + obj.sampleRate;

      % Parse PS3 controller data
      obj.parsePS3Controller;

      % Setup stance and swing indexes
      if obj.stanceLeg == 1 % Left
        leg_m = [8 6 4 2]; leg_l = [7 5 3 1]; leg_u = [5 4 2 1];
        hip_m = [10 9]; hip_u = [6 3];
      else % Right
        leg_m = [4 2 8 6]; leg_l = [3 1 7 5]; leg_u = [2 1 5 4];
        hip_m = [9 10]; hip_u = [3 6];
      end % if

      % Cartesian position of toes relative to hip in world frame
      x_st = sum(sin(q(13) + q(leg_l(1:2)))/2);
      y_st = sum(cos(q(13) + q(leg_l(1:2)))/2);
      x_sw = sum(sin(q(13) + q(leg_l(3:4)))/2);
      y_sw = sum(cos(q(13) + q(leg_l(3:4)))/2);

      % Forward kinematic lengths
      l_l = cos(q(leg_l(2))/2 - q(leg_l(1))/2);
      dl_l = -sin(q(leg_l(1))/2 - q(leg_l(2))/2)*(dq(leg_l(1))/2 - q(leg_l(2))/2);
      l_h = 0.1831*obj.stanceLeg;
      l_t = 0.334*obj.m_leg/obj.m_total;

      % Estimate CoM velocities assuming stance leg is fixed on the ground
      dx = -mean(cos(q(13) + q(leg_l(1:2))).*(dq(13) + dq(leg_l(1:2)))) + l_t*cos(q(13))*dq(13);
      dy = (l_l*cos(q(hip_m(1)) + q(11)) - l_h*sin(q(hip_m(1)) + q(11)))*(dq(hip_m(1)) + dq(11)) + l_t*cos(q(11))*dq(11);

      % Scaling factors representing the magnitude of force in each leg
      s_st = scaleFactor(obj.ks_leg*mean(abs(q(leg_m(1:2)) - q(leg_l(1:2)))), obj.thres_lo, obj.thres_hi);
      s_sw = scaleFactor(obj.ks_leg*mean(abs(q(leg_m(3:4)) - q(leg_l(3:4)))), obj.thres_lo, obj.thres_hi);

      % Update CoM velocity estimates based on confidence leg is on ground
      if s_st >= 1
        % Compute smoothing factor
        alpha = obj.sampleRate/(obj.tau_c + obj.sampleRate);

        % Filter velocity estimate, ignoring bad (large) values
        obj.dx_est = obj.dx_est + alpha*(dx - obj.dx_est)*(abs(dx) < 3);
        obj.dy_est = obj.dy_est + alpha*(dy - obj.dy_est)*(abs(dy) < 1);
      end % if

      % Update CoM position estimates
      obj.x_est = obj.x_est + obj.dx_est*obj.sampleRate;
      obj.y_est = obj.y_est + obj.dy_est*obj.sampleRate;

      % Stance leg push-off is proportional to desired speed and error
      l_ext = clamp(obj.l_ext_gain*abs(obj.dx_tgt), 0, 0.96 - obj.l0_leg)*(sign(obj.dx_tgt) == sign(obj.dx_est));

      % Step length is proportional to current velocity
      l_step = clamp(...
        obj.dx_est*obj.dx_gain + ...
        (obj.dx_est - obj.dx_tgt)*obj.dx_err_p_gain + ...
        (obj.dx_est - obj.dx_est_e)*obj.dx_err_d_gain - ...
        l_t*sin(q(13)) + ...
        obj.x_offset*obj.m_leg/obj.m_total, ...
        -0.4, 0.4);

      % Define a time variant parameter
      s = clamp(obj.t/obj.t_step, 0, Inf);
      ds = obj.dt_gain/obj.t_step;

      % Swing leg retraction policy
      [l_sw, dl_sw] = cubic_interp([0, 0.5, 1], [obj.l0_leg, obj.l0_leg - obj.l_ret, obj.l0_leg], [0, 0, 0], s, ds);

      % Swing leg swing policy
      [d_sw, dd_sw] = cubic_interp([0, 0.7], [obj.x_sw_e, l_step], [0, 0], s, ds);
      r_sw = pi/2 + real(acos(d_sw/l_sw)) - q(13);
      dr_sw = - dq(13) - (dd_sw/l_sw - (dl_sw*d_sw)/l_sw^2)/sqrt(1 - d_sw^2/l_sw^2);

      % Target swing leg actuator positions
      q_sw = r_sw + [-1; 1]*real(acos(l_sw));

      % Target swing leg actuator velocities
      dq_sw = dr_sw + [1; -1]*dl_sw/sqrt(1 - l_sw^2);

      % Swing leg actuator torques from PD controller
      u(leg_u(3:4)) = obj.s_leg*((q_sw - q(leg_m(3:4)))*obj.kp_leg + (dq_sw - dq(leg_m(3:4)))*obj.kd_leg);

      % Stance leg push off policy (extend leg after mid stance linearly)
      [l_st, dl_st] = linear_interp([0, 0.5, 1], [obj.l_leg_last, obj.l_leg_last, obj.l_leg_last + l_ext], s, ds);

      % Target stance leg actuator positions
      q_st = mean(q(leg_l(1:2))) + [-1; 1]*real(acos(l_st));

      % Target stance leg actuator velocities
      dr_st = [1; 1]*mean(dq(leg_l(1:2)));
      dq_st = dr_st + [1; -1]*dl_st/sqrt(1 - l_st^2);

      % Stance leg actuator torques from PD controller
      u(leg_u(1:2)) = (q_st - q(leg_m(1:2)))*obj.kp_leg + (dq_st - dq(leg_m(1:2)))*obj.kd_leg;

      % Add additional torque commands to leg actuators to stabilize torso
      % scaled based on the "force" felt in the leg
      u(leg_u(1:2)) = u(leg_u(1:2)) + ...
        s_st*obj.s_torso*(q(13)*obj.kp_leg + dq(13)*obj.kd_leg);
      u(leg_u(3:4)) = u(leg_u(3:4)) + ...
        s_sw*obj.s_torso*(q(13)*obj.kp_leg + dq(13)*obj.kd_leg);

      % Lateral foot placement policy
      d = - (obj.y0_offset - obj.y0_gain*abs(obj.dx_tgt))*obj.stanceLeg - ...
        obj.dy_est*obj.dy_gain - ...
        ((obj.dy_est + obj.dy_est_e)/2 - obj.dy_tgt)*obj.dy_err_p_gain - ...
        ((obj.dy_est + obj.dy_est_e)/2 - obj.dy_est_avg)*obj.dy_err_d_gain - ...
        obj.y_offset*obj.m_leg/obj.m_total - ...
        l_t*sin(q(11));

      % Inverse kinematics
      L = sqrt(l_l^2 + l_h^2);
      q1 = real(asin(d/L));
      q2 = real(asin(-l_h/L));
      q_h = clamp(q1 - q2 - q(11), -0.13*obj.stanceLeg, 0.35*obj.stanceLeg);
      dq_h = -real((d*l_l*dl_l)/(sqrt(1 - d^2/(l_h^2 + l_l^2))*(l_h^2 + l_l^2)^(3/2))) - real((l_h*l_l*dl_l)/(sqrt(1 - l_h^2/(l_h^2 + l_l^2))*(l_h^2 + l_l^2)^(3/2))) - dq(11);

      % Hip feed-forward torque for gravity compensation
      if s_st > 0 || s_sw > 0
        u(hip_u) = obj.m_leg*obj.g*abs(l_h).*[obj.stanceLeg; -obj.stanceLeg];
      end % if

      % Swing leg PD controller
      u(hip_u(2)) = u(hip_u(2)) + ...
        s*(1 - s_sw)*(q_h - q(hip_m(2)))*obj.kp_hip + (dq_h - dq(hip_m(2)))*obj.kd_hip;

      % Torso stabilization weighted PD controller
      u(hip_u) = u(hip_u) + ...
        [s_st s_sw].*obj.s_torso.*(q(11)*obj.kp_hip + dq(11)*obj.kd_hip);

      % Detect when swing leg force exceeds stance leg force
      if obj.isForceTrig*(s_sw > obj.forceThres && s > 0.9) || obj.isTimeTrig*(s >= obj.timeThres)
        % Switch stance legs
        obj.stanceLeg = -obj.stanceLeg;

        % Exit conditions
        obj.x_st_e = x_sw;
        obj.x_sw_e = x_st;
        obj.dx_est_e = obj.dx_est;
        obj.dy_est_avg = (obj.dy_est + obj.dy_est_e)/2;
        obj.dy_est_e = obj.dy_est;
        obj.l_leg_last = l_sw;

        % Reset time since last switch
        obj.t = 0;
      end % if

      % Pass signals to output function
      obj.tmp = [obj.ks_leg*mean(abs(q(leg_m(1:2)) - q(leg_l(1:2)))), obj.ks_leg*mean(abs(q(leg_m(3:4)) - q(leg_l(3:4))))];
    end % userStep

    function parsePS3Controller(obj)
    %PARSEPS3CONTROLLER

      % Parse gait specific tweaks
      if obj.ps3.cross.isPressed; obj.gaitMode = GaitMode.Stealth;
      elseif obj.ps3.circle.isPressed; obj.gaitMode = GaitMode.Normal;
      elseif obj.ps3.triangle.isPressed; obj.gaitMode = GaitMode.Dynamic;
      elseif obj.ps3.square.isPressed; obj.gaitMode = GaitMode.Hop;
      end % if

      switch obj.gaitMode
      case GaitMode.Stealth
        obj.l0_leg = 0.93;
        obj.l_ret = 0.12;
        t_c = 1; dx_max = 0.1; dy_max = 0.1;
      case GaitMode.Dynamic
        obj.l0_leg = 0.91;
        obj.l_ret = 0.3;
        t_c = 3; dx_max = 0.5; dy_max = 0.2;
      % case GaitMode.Hop
      %   obj.l0_leg = 0.91;
      %   obj.l_ret = 0.15;
      %   t_c = 1; dx_max = 0.2; dy_max = 0.2;
      otherwise % GaitMode.Normal
        obj.l0_leg = 0.91;
        obj.l_ret = 0.2;
        t_c = 3; dx_max = 0.75; dy_max = 0.2;
      end % switch

      % Parse center of mass trimming
      if obj.ps3.up.isPressed
        % Trim forward
        obj.x_offset = obj.x_offset - obj.trimIncrement;
      elseif obj.ps3.right.isPressed
        % Trim right
        obj.y_offset = obj.y_offset - obj.trimIncrement;
      elseif obj.ps3.down.isPressed
        % Trim backward
        obj.x_offset = obj.x_offset + obj.trimIncrement;
      elseif obj.ps3.left.isPressed
        % Trim left
        obj.y_offset = obj.y_offset + obj.trimIncrement;
      end % if

      % Parse right lower trigger
      if obj.ps3.r2.value;
        dx_max = 2*dx_max;
      end % if

      % Parse left joystick data (X Velocity)
      dx_cmd = dx_max*clamp(obj.ps3.leftStickY, -1, 1);

      % Parse right joystick data (Y Velocity)
      dy_cmd = dy_max*clamp(obj.ps3.rightStickX, -1, 1);

      % Simulation overrides
      if obj.isSim
        dx_cmd = 0; dy_cmd = 0;
        % dx_cmd = 1.5*round(sin(obj.runTime*2*pi/15));
        % dx_cmd = 0.25*(floor(obj.runTime/5));
        % dx_cmd = 1.5*(obj.runTime > 2);
      end % if

      % Compute smoothing factor
      alpha_x = obj.sampleRate/(t_c + obj.sampleRate);
      alpha_y = obj.sampleRate/(t_c/3 + obj.sampleRate);

      % Filter target velocity commands
      obj.dx_tgt = obj.dx_tgt + alpha_x*(dx_cmd - obj.dx_tgt);
      obj.dy_tgt = obj.dy_tgt + alpha_y*(dy_cmd - obj.dy_tgt);
    end % parsePS3Controller
  end % methods
end % classdef

%% LOCAL FUNCTIONS ========================================================

function s = scaleFactor(f, tl, tu)
%SCALEFACTOR Compute scalar (0 to 1) representing forces in leg.

  s = (clamp(f, tl, tu) - tl)/(tu - tl);
end % scaleFactor
