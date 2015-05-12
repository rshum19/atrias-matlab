%MECHANICAL_PARAMS Mechanical properties of ATRIAS for simulation.
%
% Notes:
%   - All quantities are in SI Units

%% ENVIRONMENTAL ==========================================================
g = 9.81;

%% SENSOR NOISE ===========================================================
sensor_noise_variance = (2e-4)^2*ones(13,1);

%% MOTORS =================================================================

% Saggital
k_leg_spring = 2950;
b_leg_spring = 0.075*sqrt(k_leg_spring*0.5*.25^2);
N = 50;
mu_motor = 0.85;
b_rotor = 0*60/(2*pi)*9.725e-5;
i_rotor = 1.5e-3;
i_rotor_moments = [0 0 i_rotor];
i_motor_reflected = i_rotor * N^2;
r_rotor = 0.001;
m_rotor = 1e-5;
r_virtual_gear = N*r_rotor;
m_virtual_gear = 1e-5;
i_virtual_gear = [0 0 1e-5];
m_output_gear = 1e-5;
i_output_gear = [0 0 1e-5];
m_gearing = m_rotor+m_virtual_gear+m_output_gear;
Kt_leg = 0.121;
Kb_leg = 0.121;
R_leg = 0.052;
L_leg = 0.102e-3;
max_voltage_leg = 50;
max_current_leg = 200;
LEG_MTR_MAX_TORQUE = max_current_leg * Kt_leg * N;

% Lateral
r_hip_gearhead = 0.009525;
r_hip_shaft = 0.542925;
N_h = r_hip_shaft / r_hip_gearhead;
mu_hmotor = 0.90;
b_hmotor = 0;
i_hrotor_moments = [0 0 3.193e-5];
max_current_hip = 60;
Kt_hip = 0.184;
HIP_MTR_MAX_TORQUE = max_current_hip*Kt_hip*N_h;

% Friction (Specified after the gear reduction)
breakaway_friction = [25 25 25 25];
coulomb_friction = [20 20 20 20];
viscous_friction = [0.001 0.001 0.001 0.001];
friction_linear_zone = [0.5 0.5 0.5 0.5].*pi/180; % velocity zone before full stiction is applied

%% LEG GEOMETRY ===========================================================
l_seg = 0.5;
l_rad = 0.03;

% Thigh
com_thigh = [0.0160 -0.1822 0];
m_thigh = 0.69;
i_thigh_moments = [0.02 0 0.02];
i_thigh_products = [0 0 0];

% Lower Leg
lowerleg_12 = 0.40;
com_lowerleg = [0 -0.1513 0];
m_lowerleg = 0.4438;
i_lowerleg_moments = [0.01 0 0.01];
i_lowerleg_products = [0 0 0];
l_lowerleg = 0.49;

% Foot
% d_foot_point_front = [0.06350 -0.02515 0];
% d_foot_point_back = [-0.06350 -0.02515 0];
d_foot_point_front = [0.10162 -0.02515 0];
d_foot_point_back = [-0.10162 -0.02515 0];
m_foot = 0.0857;
com_foot = [0 -0.00959 0];
i_foot_moments = [0.00001 0.00006 0.00006];
i_foot_products = [0 0 0];
ankle_stiffness = 1/2*pi;
ankle_equilibrium = 25*pi/180;

% Four Bar Link
com_fblink = [0 -0.1137 0];
m_fblink = 0.46;
i_fblink_moments = [0.01 0 0.01];
i_fblink_products = [0 0 0];

% Shin
shin_12 = 0.40;
com_shin = [0.0215 -0.1503 0];
m_shin = 0.75;
i_shin_moments = [0.02 0 0.02];
i_shin_products = [0 0 0];

% Mechanical Limits Model
k_phi = 5000; % N/m
v_phi_max = 1/60; % rad/s
delta_phi_min = 25.5*pi/180;
delta_phi_max = 160*pi/180;
phi_shin_max = 3*pi/2 + 17.5*pi/180;
phi_shin_min = pi - 45*pi/180;
phi_thigh_max = pi + 45*pi/180;
phi_thigh_min = pi/2 - 17.5*pi/180;
phi_roll_max = pi + 16*pi/180;
phi_roll_min = pi - 9*pi/180;
phi_max_collision = -5*pi/180;

% Hip Shaft
leg_offset = 0.1831;
hshaft_12 = [0 0 leg_offset];
com_hshaft = [0 0.029 leg_offset-0.0149];
m_hshaft = 18; % includes spring plates
i_hshaft_moments = [0.29 0.10 0.27];
i_hshaft_products = 0*[0 0 0.1];

%% BOOM ===================================================================
pitch_mount_angle = 7.2824 * pi/180;

% Boom tube
l_boom = 1.8161;
boom_12 = [0 0 -l_boom];
com_boom = [0 0 -1.33];
m_boom = 4.49;
i_boom_moments = [3.9 3.9 0];
i_boom_products = [0 0 0];

% Yaw shaft
yshaft_12 = [0 0.9906 0];
com_yshaft = [0 1 -0.15];
m_yshaft = 12.89;
i_yshaft_moments = [8 3.3 4.9];
i_yshaft_products = [-3 0 0];

% Boom base
base_12 = [0 0 0];

% Counter weight
counterweight_mass = 0;

%% TORSO ==================================================================
boom_mount_to_hip = 0.318;
boom_mount_to_center = 0.2225;
boom_mount_to_center_diagonal = boom_mount_to_center / cos(pitch_mount_angle);
torso_12 = [0 -boom_mount_to_hip -boom_mount_to_center];
% com_torso = [0 0.016 -0.2225];
com_torso = [0.01 (0.334 - 0.318) (-0.2225 + 0.06)];
m_torso = 22.2;
% i_torso_moments = [1.5 1.5 2.2];
i_torso_moments = [1.2 0.7 1.6];
i_torso_products = [0 0 0];
winch_hook_offset = [0 0.375 -0.2225];
d_hip_com_to_torso_com = boom_mount_to_hip - com_hshaft(2) + com_torso(2);
d_pelvis_to_torso = boom_mount_to_hip + com_torso(2);
d_pelvis_to_IMU = [4.720 18.770 6.327]*2.54/100;

%% OTHER ==================================================================

% Total massess and inertias
num_legs = 2;
m_total = num_legs*(m_hshaft + m_thigh + m_lowerleg + m_shin + m_fblink + 2*m_gearing) + m_torso;
m_leg = m_thigh + m_lowerleg + m_shin + m_fblink;
m_hip = m_hshaft + m_leg;
i_motor = i_motor_reflected + m_leg/2*l_seg^2;
i_leg = m_leg/2*l_seg^2;
d_com = (d_hip_com_to_torso_com*m_torso)/(m_torso + num_legs*m_hshaft);
boom_mount_to_com = boom_mount_to_hip - d_com;
i_robot = i_torso_moments(3) + m_torso*(d_hip_com_to_torso_com - d_com)^2 + num_legs*m_hshaft*d_com^2 + num_legs*i_hshaft_moments(1);

%% INITIAL CONDITIONS =====================================================
initial_r_leg_length = 0.9;
initial_r_leg_angle =  pi;
thigh_initial_r = initial_r_leg_angle - acos(initial_r_leg_length);
shin_initial_r = initial_r_leg_angle + acos(initial_r_leg_length);
initial_l_leg_length = 0.9;
initial_l_leg_angle = pi;
thigh_initial_l = initial_l_leg_angle - acos(initial_l_leg_length);
shin_initial_l = initial_l_leg_angle + acos(initial_l_leg_length);
v_thigh_initial_r =  0;
v_shin_initial_r =  0;

initial_com_height = d_com + initial_r_leg_length + 0.03;
initial_torso_yaw = 0*pi/180;
initial_torso_pitch = 0*pi/180;
initial_torso_roll = 0*pi/180;
initial_pitch_joint_height = initial_com_height + boom_mount_to_com;
initial_hip_roll = -0.15;
initial_dx = 0;

%% CONTACT MODEL ==========================================================

% Vertical ground interaction stiffness
k_gy = m_total*g/0.005; % [N/m]

% Maximum vertical ground relaxation speed
v_gy_max = 0.03; %[m/s]

% Horizontal ground interaction stiffness
k_gx = m_total*g/0.005; % [N/m]

% Maximum horizontal ground relaxation speed
v_gx_max = 0.03; %[m/s]

% Stiction coefficient
mu_stick = 0.9;

% Sliding coefficient
mu_slide = 0.8;

% Slip-stic transition velocity
vLimit = 0.01; %[m/s]
