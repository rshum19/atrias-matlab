classdef MikhailController < Controller
%MIKHAILCONTROLLER Mikhail's controller.
%
% Copyright 2015 Mikhail S. Jones

  % PUBLIC PROPERTIES =====================================================
  properties
    % Step Duration (s)
    t0_step@double = 0.35
    % Step Duration Gain
    t_gain@double = 0.02
    % Double support phase overlap
    s_overlap@double = 0.37
    % Stance Leg P Gain (N*m/rad)
    kp_st_leg@double = 3200
    % Stance Leg D Gain (N*m*s/rad)
    kd_st_leg@double = 220
    % Swing Leg P Gain (N*m/rad)
    kp_sw_leg@double = 3000
    % Swing Leg D Gain (N*m*s/rad)
    kd_sw_leg@double = 150
    % Hip P Gain (N*m/rad)
    kp_hip@double = 1800
    % Hip D Gain (N*m*s/rad)
    kd_hip@double = 60
    % Left A Motor Torque Scaling Factor
    s_l_A@double = 0.9
    % Left B Motor Torque Scaling Factor
    s_l_B@double = 1.17
    % Right A Motor Torque Scaling Factor
    s_r_A@double = 1.08
    % Right B Motor Torque Scaling Factor
    s_r_B@double = 1.13
    % Lower Force Threshold (N)
    thres_lo@double = 125
    % Upper Force Threshold (N)
    thres_hi@double = 250
    % Velocity Filter Time Constant (s)
    tau@double = 0.12
    % X Velocity Feed-Forward Gain
    dx_gain@double = 0.18
    % X Velocity Error P Gain
    dx_err_p_gain@double = 0.1
    % X Velocity Error D Gain
    dx_err_d_gain@double = 0.1
    % Y Velocity Feed-Forward Gain
    dy_gain@double = 0.22
    % Y Velocity Error P Gain
    dy_err_p_gain@double = 0.1
    % Y Velocity Error D Gain
    dy_err_d_gain@double = 0.1
    % Hip Offset (m)
    y0_offset@double = 0.15
    % Hip Offset Gain
    y0_gain@double = 0.02
    % Nominal Leg Length (m)
    l0_leg@double = 0.9
    % Swing Leg Retraction (m)
    l_ret@double = 0.2
    % Leg Extension Gain (m)
    l_ext_gain@double = 0.018

    % Use impulse regulation
    isImpulseControl@logical = false
    % Use cubic hip trajectory
    isHipTrajectory@logical = false
    % Use time scaled hip gains
    isTimeScaledHipGains@logical = true
    % Use zero torque stance control on swing leg
    isZeroSwingLegTorque@logical = false
  end % properties

  % PROTECTED PROPERTIES ==================================================
  properties (Access = protected)
    % Current stance leg (1 == Left, -1 == Right)
    stanceLeg@double = 1
    % Gait mode
    gaitMode@GaitMode
    % Time since last step (s)
    t@double = 0
    % Heading (rad)
    q_yaw@double = 0
    % X Center of Mass Offset (m)
    x_offset@double = 0.003
    % Y Center of Mass Offset (m)
    y_offset@double = -0.014
    % Z Center of Mass Offset (m)
    z_offset@double = 0.1179
    % Estimated X position (m)
    x_est@double = 0
    % Estimated Y position (m)
    y_est@double = 0
    % Estimated X velocity (m/s)
    dx_est@double = 0
    % Estimated Y velocity (m/s)
    dy_est@double = 0
    % Estimated average Y velocity of last step (m/s)
    dy_est_avg@double = 0
    % Target X velocity (m/s)
    dx_tgt@double = 0
    % Target Y velocity (m/s)
    dy_tgt@double = 0
    % Target yaw velocity (rad/s)
    dq_yaw_tgt@double = 0
    % Estimated X velocity of last step (m/s)
    dx_est_last@double = 0
    % Estimated Y velocity of last step (m/s)
    dy_est_last@double = 0
    % Length of stance leg at switch (m)
    l_st_last@double = 0.9
    % Length rate of stance leg at switch (m/s)
    dl_st_last@double = 0
    % Length of swing leg at switch (m)
    l_sw_last@double = 0.9
    % Length rate of swing leg at switch (m/s)
    dl_sw_last@double = 0
    % Stance toe X position of last step (m)
    x_st_last@double = 0
    % Stance toe X velocity of last step (m/s)
    dx_st_last@double = 0
    % Swing toe X position of last step (m)
    x_sw_last@double = 0
    % Swing toe X velocity of last step (m/s)
    dx_sw_last@double = 0
    % Swing toe Y position of last step (m)
    y_sw_last@double = 0
    % Swing toe Y velocity of last step (m/s)
    dy_sw_last@double = 0
    % Storage for passing signals to output function
    output@double = zeros(1,16)

    % Horizontal impulse on body during current step
    Jx@double = 0
    % Vertical impulse on body during current step
    Jz@double = 0
  end % properties

  % CONSTANT PROPERTIES ===================================================
  properties (Constant = true, Hidden = true)
    % Center of mass offset trim increment (m)
    trimIncrement = 0.001
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
    % Hip length
    l_h = 0.1831
    % Leg motor coulomb friction
    f_c = 0
    % Leg motor viscous friction
    f_v = 0
  end % properties

  % PUBLIC METHODS ========================================================
  methods
    function userSetup(obj)
    %USERSETUP Initialize system object.

      % Reset objects
      obj.gaitMode = GaitMode.Triangle;

      % Reset parameters
      obj.t = 0;
      obj.x_est = 0;
      obj.y_est = 0;
      obj.dx_est = 0;
      obj.dy_est = 0;
      obj.dy_est_avg = 0;
      obj.dx_tgt = 0;
      obj.dy_tgt = 0;
      obj.dq_yaw_tgt = 0;
      obj.dx_est_last = 0;
      obj.dy_est_last = 0;
      obj.x_st_last = 0;
      obj.dx_st_last = 0;
      obj.x_sw_last = 0;
      obj.dx_sw_last = 0;
      obj.y_sw_last = 0;
      obj.dy_sw_last = 0;
      obj.l_st_last = obj.l0_leg;
      obj.dl_st_last = 0;
      obj.l_sw_last = obj.l0_leg;
      obj.dl_sw_last = 0;
      obj.Jx = 0;
      obj.Jz = 0;
    end % userSetup

    function userOut = userOutput(obj)
    %USEROUTPUT User output function.

      userOut = [...
        obj.output, ...
        obj.Jz/100, ...
        obj.Jx/100, ...
        ...obj.x_est, obj.y_est, ...
        obj.dx_est, obj.dy_est, ...
        obj.dx_tgt, obj.dy_tgt];
    end % userOutput

    function u = userStep(obj, q, dq)
    %USERSTEP System output and state update equations.

      % Update stance duration timer
      obj.t = obj.t + obj.sampleInterval;

      % Parse PS3 controller data
      obj.parsePS3Controller;

      % Relabel leg state variables in terms of stance and swing
      if obj.stanceLeg == 1 % Left
        q_st_mA = q(8); dq_st_mA = dq(8); q_st_mB = q(6); dq_st_mB = dq(6);
        q_st_lA = q(7); dq_st_lA = dq(7); q_st_lB = q(5); dq_st_lB = dq(5);
        q_st_h = q(10); dq_st_h = dq(10);
        q_sw_mA = q(4); dq_sw_mA = dq(4); q_sw_mB = q(2); dq_sw_mB = dq(2);
        q_sw_lA = q(3); dq_sw_lA = dq(3); q_sw_lB = q(1); dq_sw_lB = dq(1);
        q_sw_h = q(9); dq_sw_h = dq(9);
      else % Right
        q_st_mA = q(4); dq_st_mA = dq(4); q_st_mB = q(2); dq_st_mB = dq(2);
        q_st_lA = q(3); dq_st_lA = dq(3); q_st_lB = q(1); dq_st_lB = dq(1);
        q_st_h = q(9); dq_st_h = dq(9);
        q_sw_mA = q(8); dq_sw_mA = dq(8); q_sw_mB = q(6); dq_sw_mB = dq(6);
        q_sw_lA = q(7); dq_sw_lA = dq(7); q_sw_lB = q(5); dq_sw_lB = dq(5);
        q_sw_h = q(10); dq_sw_h = dq(10);
      end % if

      % Relabel torso state variables
      q_yaw = q(12); dq_yaw = dq(12);
      q_pitch = q(13); dq_pitch = dq(13);
      q_roll = q(11); dq_roll = dq(11);

      % Store heading for use in hold position mode
      obj.q_yaw = q_yaw;

      % Hip lengths
      l_st_h = obj.l_h*obj.stanceLeg;
      l_sw_h = -obj.l_h*obj.stanceLeg;

      % Compute hip to center of mass distances and velocities
      x_t = obj.x_offset*cos(q_pitch) + obj.z_offset*cos(q_roll)*sin(q_pitch) + obj.y_offset*sin(q_pitch)*sin(q_roll);
      y_t = obj.y_offset*cos(q_roll) - obj.z_offset*sin(q_roll);
      dx_t = 0;%obj.z_offset*cos(q_pitch)*cos(q_roll)*dq_pitch - obj.x_offset*sin(q_pitch)*dq_pitch + obj.y_offset*cos(q_pitch)*sin(q_roll)*dq_pitch + obj.y_offset*cos(q_roll)*sin(q_pitch)*dq_roll - obj.z_offset*sin(q_pitch)*sin(q_roll)*dq_roll;
      dy_t = 0;%-(obj.z_offset*cos(q_roll) + obj.y_offset*sin(q_roll))*dq_roll;

      % Compute toe to center of mass distances and velocities
      dx = (sin(q_pitch)*sin(q_st_lA)*dq_pitch)/2 - (cos(q_pitch)*cos(q_st_lB)*dq_st_lB)/2 - (cos(q_pitch)*cos(q_st_lA)*dq_st_lA)/2 + (sin(q_pitch)*sin(q_st_lB)*dq_pitch)/2 - obj.x_offset*sin(q_pitch)*dq_pitch + obj.z_offset*cos(q_pitch)*cos(q_roll)*dq_pitch + obj.y_offset*cos(q_pitch)*sin(q_roll)*dq_pitch + obj.y_offset*cos(q_roll)*sin(q_pitch)*dq_roll - obj.z_offset*sin(q_pitch)*sin(q_roll)*dq_roll - (cos(q_pitch)*cos(q_roll)*cos(q_st_h)*cos(q_st_lA)*dq_pitch)/2 - (cos(q_pitch)*cos(q_roll)*cos(q_st_h)*cos(q_st_lB)*dq_pitch)/2 + (cos(q_pitch)*cos(q_st_lA)*sin(q_roll)*sin(q_st_h)*dq_pitch)/2 + (cos(q_pitch)*cos(q_st_lB)*sin(q_roll)*sin(q_st_h)*dq_pitch)/2 + (cos(q_roll)*cos(q_st_lA)*sin(q_pitch)*sin(q_st_h)*dq_roll)/2 + (cos(q_st_h)*cos(q_st_lA)*sin(q_pitch)*sin(q_roll)*dq_roll)/2 + (cos(q_roll)*cos(q_st_lB)*sin(q_pitch)*sin(q_st_h)*dq_roll)/2 + (cos(q_st_h)*cos(q_st_lB)*sin(q_pitch)*sin(q_roll)*dq_roll)/2 + (cos(q_roll)*cos(q_st_lA)*sin(q_pitch)*sin(q_st_h)*dq_st_h)/2 + (cos(q_st_h)*cos(q_st_lA)*sin(q_pitch)*sin(q_roll)*dq_st_h)/2 + (cos(q_roll)*cos(q_st_lB)*sin(q_pitch)*sin(q_st_h)*dq_st_h)/2 + (cos(q_st_h)*cos(q_st_lB)*sin(q_pitch)*sin(q_roll)*dq_st_h)/2 + (cos(q_roll)*cos(q_st_h)*sin(q_pitch)*sin(q_st_lA)*dq_st_lA)/2 + (cos(q_roll)*cos(q_st_h)*sin(q_pitch)*sin(q_st_lB)*dq_st_lB)/2 - l_st_h*cos(q_pitch)*cos(q_roll)*sin(q_st_h)*dq_pitch - l_st_h*cos(q_pitch)*cos(q_st_h)*sin(q_roll)*dq_pitch - l_st_h*cos(q_roll)*cos(q_st_h)*sin(q_pitch)*dq_roll - l_st_h*cos(q_roll)*cos(q_st_h)*sin(q_pitch)*dq_st_h - (sin(q_pitch)*sin(q_roll)*sin(q_st_h)*sin(q_st_lA)*dq_st_lA)/2 - (sin(q_pitch)*sin(q_roll)*sin(q_st_h)*sin(q_st_lB)*dq_st_lB)/2 + l_st_h*sin(q_pitch)*sin(q_roll)*sin(q_st_h)*dq_roll + l_st_h*sin(q_pitch)*sin(q_roll)*sin(q_st_h)*dq_st_h;
      dy = l_st_h*cos(q_roll)*sin(q_st_h)*dq_roll - obj.y_offset*sin(q_roll)*dq_roll - (cos(q_st_lA)*sin(q_roll)*sin(q_st_h)*dq_roll)/2 - (cos(q_st_lB)*sin(q_roll)*sin(q_st_h)*dq_roll)/2 - (cos(q_st_lA)*sin(q_roll)*sin(q_st_h)*dq_st_h)/2 - (cos(q_st_lB)*sin(q_roll)*sin(q_st_h)*dq_st_h)/2 - (cos(q_roll)*sin(q_st_h)*sin(q_st_lA)*dq_st_lA)/2 - (cos(q_st_h)*sin(q_roll)*sin(q_st_lA)*dq_st_lA)/2 - (cos(q_roll)*sin(q_st_h)*sin(q_st_lB)*dq_st_lB)/2 - (cos(q_st_h)*sin(q_roll)*sin(q_st_lB)*dq_st_lB)/2 - obj.z_offset*cos(q_roll)*dq_roll + l_st_h*cos(q_st_h)*sin(q_roll)*dq_roll + l_st_h*cos(q_roll)*sin(q_st_h)*dq_st_h + l_st_h*cos(q_st_h)*sin(q_roll)*dq_st_h + (cos(q_roll)*cos(q_st_h)*cos(q_st_lA)*dq_roll)/2 + (cos(q_roll)*cos(q_st_h)*cos(q_st_lB)*dq_roll)/2 + (cos(q_roll)*cos(q_st_h)*cos(q_st_lA)*dq_st_h)/2 + (cos(q_roll)*cos(q_st_h)*cos(q_st_lB)*dq_st_h)/2;

      % Horizontal ground reaction forces computed from spring deflections
      fx_st = 2*obj.ks_leg*(sin(q_pitch + q_st_lB)*(q_st_mA - q_st_lA) - sin(q_pitch + q_st_lA)*(q_st_mB - q_st_lB))/sin(q_st_lA - q_st_lB);
      fx_sw = 2*obj.ks_leg*(sin(q_pitch + q_sw_lB)*(q_sw_mA - q_sw_lA) - sin(q_pitch + q_sw_lA)*(q_sw_mB - q_sw_lB))/sin(q_sw_lA - q_sw_lB);

      % Vertical ground reaction forces computed from spring deflections
      fz_st = 2*obj.ks_leg*(cos(q_pitch + q_st_lB)*(q_st_mA - q_st_lA) - cos(q_pitch + q_st_lA)*(q_st_mB - q_st_lB))/sin(q_st_lA - q_st_lB);
      fz_sw = 2*obj.ks_leg*(cos(q_pitch + q_sw_lB)*(q_sw_mA - q_sw_lA) - cos(q_pitch + q_sw_lA)*(q_sw_mB - q_sw_lB))/sin(q_sw_lA - q_sw_lB);

      % Scaling factors representing a normalized vertical GRF
      s_st = scaleFactor(fz_st, obj.thres_lo, obj.thres_hi);
      s_sw = scaleFactor(fz_sw, obj.thres_lo, obj.thres_hi);

      % Horizontal ground impulse
      obj.Jx = obj.Jx + s_st*fx_st*obj.sampleInterval + s_sw*fx_sw*obj.sampleInterval;

      % Vertical ground impulse
      obj.Jz = obj.Jz + s_st*fz_st*obj.sampleInterval + s_sw*fz_sw*obj.sampleInterval;

      % Compute smoothing factor based on confidence leg is on the ground
      alpha = s_st*obj.sampleInterval/(obj.tau + obj.sampleInterval);

      % Filter velocity estimate, ignoring bad (large) values
      obj.dx_est = obj.dx_est + alpha*(dx - obj.dx_est)*(abs(dx) < 4);
      obj.dy_est = obj.dy_est + alpha*(dy - obj.dy_est)*(abs(dy) < 1);

      % Update CoM position estimates
      obj.x_est = obj.x_est + obj.sampleInterval*(cos(obj.q_yaw)*obj.dx_est + sin(obj.q_yaw)*obj.dy_est);
      obj.y_est = obj.y_est + obj.sampleInterval*(sin(obj.q_yaw)*obj.dx_est + cos(obj.q_yaw)*obj.dy_est);

      % Step duration adjusted discretely step by step
      t_step = obj.t0_step - obj.t_gain*abs(obj.dx_est_last);

      % Define a time variant parameter normalized between 0 and 1
      s = clamp(obj.t/t_step, 0, 1);
      ds = 1/t_step;

      % Double support phase overlap adjusted discretely step by step
      s0 = cubic_interp(abs(obj.dx_est_last), 0, ...
        [0, 1.5], ...
        [obj.s_overlap, 0], ...
        [0, 0]);

      % Compute Stance Leg Target Positions -------------------------------

      % Stance leg extension policy for energy injection
      l_ext = obj.l_ext_gain*abs(obj.dx_tgt)*(sign(obj.dx_tgt) == sign(obj.dx_est));

      % Clamp stance leg length to avoid mechanical limits
      l_ext = clamp(l_ext, 0, 0.96 - obj.l0_leg);

      % TODO: test yaw control on robot
      l_yaw = clamp(-obj.dq_yaw_tgt*obj.stanceLeg*abs(obj.dx_est_last), 0, 0.02);

      % Target stance leg length (cubic extension in second half of stance)
      [l_st, dl_st] = cubic_interp(s, ds, ...
        [0, s0, 0.4, 1], ...
        [obj.l_st_last, obj.l0_leg + l_yaw, obj.l0_leg + l_yaw, obj.l0_leg + l_ext], ...
        [obj.dl_st_last, 0, 0, 0]/ds);

      % Impulse regulation to avoid excessive energy injection
      if obj.isImpulseControl
        % Compute target impulse assuming linear increase through stance
        Jz_tgt = s*obj.m_total*obj.g*t_step;

        % Varying leg length to regulate impulse
        l_st = l_st + clamp(0.002*(Jz_tgt - obj.Jz), -0.02, 0);
      end % if

      % Target stance leg angle and velocity (zero hip torque)
      q_st = mean([q_st_lA q_st_lB]);
      dq_st = mean([dq_st_lA dq_st_lB]);

      % Target stance leg actuator angles and velocities
      q_st_mA_tgt = real(q_st - acos(l_st));
      q_st_mB_tgt = real(q_st + acos(l_st));
      dq_st_mA_tgt = real(dq_st + dl_st/sqrt(1 - l_st^2));
      dq_st_mB_tgt = real(dq_st - dl_st/sqrt(1 - l_st^2));

      % Compute Swing Leg Target Positions --------------------------------

      % Target swing leg length and velocity
      [l_sw, dl_sw] = cubic_interp(s, ds,...
        [0, 0.5 + s0/2, 1 + s0], ...
        [obj.l_sw_last, obj.l0_leg - obj.l_ret, obj.l0_leg + l_yaw], ...
        [obj.dl_sw_last, 0, 0]/ds);

      % Target swing leg foot placement policy
      x_sw_tgt = obj.dx_gain*obj.dx_est + ...
        obj.dx_err_p_gain*(obj.dx_est - obj.dx_tgt) + ...
        obj.dx_err_d_gain*(obj.dx_est - obj.dx_est_last);

      % Smooth clamp target swing leg foot placement
      x_sw_tgt = atans(x_sw_tgt, 1, 1); % TODO: tune this with leg extension

      % Target swing leg cartesian position and velocity
      [x_sw, dx_sw] = cubic_interp(s, ds, ...
        [0, 0.8], ...
        [obj.x_sw_last, x_sw_tgt], ...
        [obj.dx_sw_last, -obj.dx_est]/ds*(obj.gaitMode == GaitMode.Triangle));

      % Target swing leg angle and velocity
      % TODO: Yaw control works better with this disabled, might yaw less during normal operation with it enabled
      if s_sw > 0 && obj.isZeroSwingLegTorque
        q_sw = mean([q_sw_lA q_sw_lB]);
        dq_sw = mean([dq_sw_lA dq_sw_lB]);
      else
        q_sw = real(pi - q_pitch - asin((x_sw + x_t)/l_sw));
        dq_sw = - real(((dx_sw + dx_t)/l_sw - ((x_sw + x_t)*dl_sw)/l_sw^2)/sqrt(1 - (x_sw + x_t)^2/l_sw^2)) - dq_pitch;
      end % if

      % Target swing leg actuator angles and velocities
      q_sw_mA_tgt = real(q_sw - acos(l_sw));
      q_sw_mB_tgt = real(q_sw + acos(l_sw));
      dq_sw_mA_tgt = real(dq_sw + dl_sw/sqrt(1 - l_sw^2));
      dq_sw_mB_tgt = real(dq_sw - dl_sw/sqrt(1 - l_sw^2));

      % Target swing leg foot placement policy
      y_sw_tgt = -obj.stanceLeg*(obj.y0_offset - obj.y0_gain*abs(obj.dx_est)) + ...
        obj.dy_gain*obj.dy_est + ...
        obj.dy_err_p_gain*((obj.dy_est + obj.dy_est_last)/2 - obj.dy_tgt) + ...
        obj.dy_err_d_gain*((obj.dy_est + obj.dy_est_last)/2 - obj.dy_est_avg);

      % Smooth clamp target swing leg foot placement
      % y_sw_tgt = atans(y_sw_tgt, 1, 0.5);

      % Target swing leg cartesian position and velocity
      if obj.isHipTrajectory
        % TODO: test ground speed matching velocity terms
        [y_sw, dy_sw] = cubic_interp(s, ds, ...
          [0, 0.8], ...
          [obj.y_sw_last, y_sw_tgt], ...
          0*[obj.dy_sw_last, -obj.dy_est]/ds);
      else
        y_sw = y_sw_tgt;
        dy_sw = 0;
      end % if

      % Target swing hip angle and velocity
      L = sqrt(obj.l0_leg^2 + l_sw_h^2);
      q_sw_h_tgt = real(asin((y_sw + y_t)/L) - asin(l_sw_h/L) - q_roll);
      dq_sw_h_tgt = real((dy_sw + dy_t)/(L*sqrt(1 - (y_sw + y_t)^2/L^2)) - dq_roll);

      % Clamp swing hip angles to avoid mechanical limits
      q_sw_h_tgt = clamp(q_sw_h_tgt, -0.13*obj.stanceLeg, 0.35*obj.stanceLeg);

      % Compute Control Inputs --------------------------------------------

      % Scale stance leg gains based on force
      kp = s_st*obj.kp_st_leg + (1 - s_st)*obj.kp_sw_leg;
      kd = s_st*obj.kd_st_leg + (1 - s_st)*obj.kd_sw_leg;

      % Stance leg actuator torques from PD controller
      u_st_A = obj.f_c*sign(dq_st_mA_tgt) + obj.f_v*dq_st_mA_tgt + kp*(q_st_mA_tgt - q_st_mA) + kd*(dq_st_mA_tgt - dq_st_mA);
      u_st_B = obj.f_c*sign(dq_st_mB_tgt) + obj.f_v*dq_st_mB_tgt + kp*(q_st_mB_tgt - q_st_mB) + kd*(dq_st_mB_tgt - dq_st_mB);

      % Scale swing leg gains based on force
      kp = s_sw*obj.kp_st_leg + (1 - s_sw)*obj.kp_sw_leg;
      kd = s_sw*obj.kd_st_leg + (1 - s_sw)*obj.kd_sw_leg;

      % Swing leg actuator torques from PD controller
      u_sw_A = obj.f_c*sign(dq_sw_mA_tgt) + obj.f_v*dq_sw_mA_tgt + kp*(q_sw_mA_tgt - q_sw_mA) + kd*(dq_sw_mA_tgt - dq_sw_mA);
      u_sw_B = obj.f_c*sign(dq_sw_mB_tgt) + obj.f_v*dq_sw_mB_tgt + kp*(q_sw_mB_tgt - q_sw_mB) + kd*(dq_sw_mB_tgt - dq_sw_mB);

      % Torso stabilization PD controller scaled based on leg force
      u_st_A = u_st_A + s_st*(obj.kp_st_leg*(q_pitch - 0) + obj.kd_st_leg*(dq_pitch - 0));
      u_st_B = u_st_B + s_st*(obj.kp_st_leg*(q_pitch - 0) + obj.kd_st_leg*(dq_pitch - 0));
      u_sw_A = u_sw_A + s_sw*(obj.kp_st_leg*(q_pitch - 0) + obj.kd_st_leg*(dq_pitch - 0));
      u_sw_B = u_sw_B + s_sw*(obj.kp_st_leg*(q_pitch - 0) + obj.kd_st_leg*(dq_pitch - 0));

      % Hip feed-forward torque for gravity compensation
      u_st_h = max(s_st, s_sw)*obj.m_leg*obj.g*l_st_h;
      u_sw_h = max(s_st, s_sw)*obj.m_leg*obj.g*l_sw_h;

      % Swing leg hip PD controller
      if obj.isTimeScaledHipGains
        u_sw_h = u_sw_h + s*(1 - s_sw)*obj.kp_hip*(q_sw_h_tgt - q_sw_h) + obj.kd_hip*(dq_sw_h_tgt - dq_sw_h);
      else
        u_sw_h = u_sw_h + (1 - s_sw)*obj.kp_hip*(q_sw_h_tgt - q_sw_h) + obj.kd_hip*(dq_sw_h_tgt - dq_sw_h);
      end % if

      % Torso stabilization PD controller scaled based on leg force
      u_st_h = u_st_h + s_st*(obj.kp_hip*(q_roll - 0) + obj.kd_hip*(dq_roll - 0));
      u_sw_h = u_sw_h + s_sw*(obj.kp_hip*(q_roll - 0) + obj.kd_hip*(dq_roll - 0));

      % Detect when time or force thresholds have been reached
      if s >= 1
        % Switch stance legs
        obj.stanceLeg = -obj.stanceLeg;

        % Store exit conditions
        obj.x_st_last = x_sw;
        obj.dx_st_last = dx_sw;
        obj.x_sw_last = l_st*sin(q_pitch + q_st) - x_t;
        obj.dx_sw_last = -obj.dx_est;
        obj.y_sw_last = L*sin(q_roll + q_st_h + asin(l_st_h/L)) - y_t;
        obj.dy_sw_last = -obj.dy_est;
        obj.l_st_last = l_sw;
        obj.dl_st_last = dl_sw;
        obj.l_sw_last = l_st;
        obj.dl_sw_last = dl_st;
        obj.dy_est_avg = (obj.dy_est + obj.dy_est_last)/2;
        obj.dx_est_last = obj.dx_est;
        obj.dy_est_last = obj.dy_est;

        % Reset parameters
        obj.t = 0;
        obj.Jx = 0;
        obj.Jz = 0;
      end % if

      % Compute actual leg states
      l_st_m = cos((q_st_mA - q_st_mB)/2);
      dl_st_m = -(sin((q_st_mA - q_st_mB)/2).*(dq_st_mA - dq_st_mB))/2;
      l_st_l = cos((q_st_lA - q_st_lB)/2);
      dl_st_l = -(sin((q_st_lA - q_st_lB)/2).*(dq_st_lA - dq_st_lB))/2;
      q_st_m = (q_st_mA + q_st_mB)/2;
      dq_st_m = (dq_st_mA + dq_st_mB)/2;
      q_st_l = (q_st_lA + q_st_lB)/2;
      dq_st_l = (dq_st_lA + dq_st_lB)/2;
      l_sw_m = cos((q_sw_mA - q_sw_mB)/2);
      dl_sw_m = -(sin((q_sw_mA - q_sw_mB)/2).*(dq_sw_mA - dq_sw_mB))/2;
      l_sw_l = cos((q_sw_lA - q_sw_lB)/2);
      dl_sw_l = -(sin((q_sw_lA - q_sw_lB)/2).*(dq_sw_lA - dq_sw_lB))/2;
      q_sw_m = (q_sw_mA + q_sw_mB)/2;
      dq_sw_m = (dq_sw_mA + dq_sw_mB)/2;
      q_sw_l = (q_sw_lA + q_sw_lB)/2;
      dq_sw_l = (dq_sw_lA + dq_sw_lB)/2;

      % Store signals in object for logging
      obj.output = [l_st, l_st_m, l_st_l, q_st, q_st_m, q_st_l, ...
        l_sw, l_sw_m, l_sw_l, q_sw, q_sw_m, q_sw_l, ...
        q_sw_h_tgt, q_sw_h, ...
        s_st*fz_st/100, s_sw*fz_sw/100];

      % Set commanded torque vector
      if obj.stanceLeg == 1 % Left
        u = [obj.s_r_B*u_sw_B, obj.s_r_A*u_sw_A, u_sw_h, ...
            obj.s_l_B*u_st_B, obj.s_l_A*u_st_A, u_st_h];
      else % Right
        u = [obj.s_l_B*u_st_B, obj.s_l_A*u_st_A, u_st_h, ...
            obj.s_r_B*u_sw_B, obj.s_r_A*u_sw_A, u_sw_h];
      end % if
    end % userStep

    function parsePS3Controller(obj)
    %PARSEPS3CONTROLLER

      % Parse D-Pad for velocity trimming
      if obj.ps3.up.isPressed
        % Trim forward
        obj.x_offset = obj.x_offset - obj.trimIncrement;
      elseif obj.ps3.right.isPressed
        % Trim right
        obj.y_offset = obj.y_offset + obj.trimIncrement;
      elseif obj.ps3.down.isPressed
        % Trim backward
        obj.x_offset = obj.x_offset + obj.trimIncrement;
      elseif obj.ps3.left.isPressed
        % Trim left
        obj.y_offset = obj.y_offset - obj.trimIncrement;
      end % if

      % Parse gait modes
      if obj.ps3.cross.isPressed; obj.gaitMode = GaitMode.Cross;
      elseif obj.ps3.circle.isPressed; obj.gaitMode = GaitMode.Circle;
      elseif obj.ps3.triangle.isPressed; obj.gaitMode = GaitMode.Triangle;
      elseif obj.ps3.square.isPressed; obj.gaitMode = GaitMode.Square;
      end % if

      % Parse select button for reset home
      if obj.ps3.select.isPressed; obj.x_est = 0; obj.y_est = 0; end % if

      % Parse left joystick data for X Velocity
      dx_cmd = clamp(obj.ps3.leftStickY, -1, 1);

      % Parse right joystick data for Y Velocity
      dy_cmd = clamp(obj.ps3.rightStickX, -1, 1);

      % Parse left joystick data for yaw Velocity
      dq_yaw_cmd = clamp(obj.ps3.leftStickX, -1, 1);

      % Set gait mode specific tweaks
      switch obj.gaitMode
      case GaitMode.Cross
        % Hold position mode
        dx_cmd = clamp(-0.2*(cos(obj.q_yaw)*obj.x_est + sin(obj.q_yaw)*obj.y_est), -0.4, 0.4);
        dy_cmd = clamp(-0.2*(cos(obj.q_yaw)*obj.y_est - sin(obj.q_yaw)*obj.x_est), -0.2, 0.2);
        dq_yaw_cmd = 0;

      case GaitMode.Circle
        % Robust stand/walk mode
        dx_cmd = 0.75*dx_cmd;
        dy_cmd = 0.2*dy_cmd;
        dq_yaw_cmd = 0.02*dq_yaw_cmd;

      case GaitMode.Triangle
        % Fast walk/run mode
        dx_cmd = 1.5*dx_cmd;
        dy_cmd = 0.2*dy_cmd;
        dq_yaw_cmd = 0.02*dq_yaw_cmd;

      otherwise % GaitMode.Square
        % TODO: Hop mode
        dx_cmd = 0.2*dx_cmd;
        dy_cmd = 0.2*dy_cmd;
        dq_yaw_cmd = 0;
      end % switch

      % Simulation overrides
      if obj.isSim
        obj.gaitMode = GaitMode.Triangle;
        % dy_cmd = 0.2*((obj.runTime >= 15) - 2*(obj.runTime >= 30));
        % dx_cmd = 1.5*round(sin(obj.runTime*2*pi/15));
        dx_cmd = clamp(0.5*floor(obj.runTime/5), 0, 3);
        % dq_yaw_cmd = 0.02;
        % dy_cmd = 0.2*(obj.runTime >= 2);
        % dx_cmd = 1.5*(obj.runTime >= 2);
      end % if

      % Parse right lower trigger for turbo mode
      if obj.ps3.r2.value
        dx_cmd = 2*dx_cmd;
      end % if

      % Parse left lower trigger for auto speed regulation mode
      if obj.ps3.l2.value
        if abs(obj.dy_est_avg) >= 0.05
          dx_cmd = dx_cmd/2;
        end % if
      end % if

      % Filter target X velocity command
      alpha_dx = obj.sampleInterval/(2 + obj.sampleInterval);
      obj.dx_tgt = obj.dx_tgt + alpha_dx*(dx_cmd - obj.dx_tgt);

      % Filter target Y velocity command
      alpha_dy = obj.sampleInterval/(0.5 + obj.sampleInterval);
      obj.dy_tgt = obj.dy_tgt + alpha_dy*(dy_cmd - obj.dy_tgt);

      % Filter target yaw velocity command
      alpha_dq_yaw = obj.sampleInterval/(0.5 + obj.sampleInterval);
      obj.dq_yaw_tgt = obj.dq_yaw_tgt + alpha_dq_yaw*(dq_yaw_cmd - obj.dq_yaw_tgt);
    end % parsePS3Controller
  end % methods
end % classdef

%% LOCAL FUNCTIONS ========================================================

function s = scaleFactor(f, tl, tu)
%SCALEFACTOR Compute scalar (0 to 1) representing forces in leg.

  s = (clamp(f, tl, tu) - tl)/(tu - tl);
end % scaleFactor

function y = atans(x, slope, y_lim)
%ATANS Arctangent smooth clamp function

  % Rescale Y limit for arctangent function
  y_lim = y_lim*2/pi;

  % Arctangent scaled to have desired slope at 0 and desired clamp at Inf
  y = y_lim*atan(slope*x/y_lim);
end % atans
