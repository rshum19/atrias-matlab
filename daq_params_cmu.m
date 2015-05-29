% This function generates:
%       calibration parameters 
%       motor parameters
%
% It is primarily based on http://code.google.com/p/atrias/source/browse/robot_definitions/
%
% Notes:
%   CMU biped = type 1
%   CMU left leg = type 1
%   CMU right leg = type 2
%   1 | A = shin
%   2 | B = thigh
%   leg = shin/thigh encoders
%   trans = motor encoders

% IMU Parameters (location dependent)
imu_latitude = 44.5673031 * pi/180; % DRL latitude
imu_heading = deg2rad(108); % Position for DRL Graf lab. 90 degrees magnetic!!!
%imu_heading = 2.7389; % Position for DRL Covell lab

% Update rates
sample_time = 0.001; % seconds
update_freq = 1/sample_time; % Hz
correctedSampleTime = 0.0009982; % Seconds

% This is the CMU robot
isOsuRobot = false;

% Medulla state parameters
MEDULLA_STATE_IDLE  = 0;
MEDULLA_STATE_RUN   = 2;
MEDULLA_STATE_HALT  = 4;
MEDULLA_STATE_ERROR = 5;
MEDULLA_STATE_RESET = 6;

% Commands sent from the Instrument Panel to the system.
% These become parameters for the Simulink model
gui_enable_cmd  = Simulink.Parameter; gui_enable_cmd.CoderInfo.StorageClass  = 'SimulinkGlobal'; gui_enable_cmd.Value  = 0; % Whether to enable the robot
gui_disable_cmd = Simulink.Parameter; gui_disable_cmd.CoderInfo.StorageClass = 'SimulinkGlobal'; gui_disable_cmd.Value = 0; % Whether to disable the robot
gui_reset_cmd   = Simulink.Parameter; gui_reset_cmd.CoderInfo.StorageClass   = 'SimulinkGlobal'; gui_reset_cmd.Value   = 0; % Whether to reset the robot (after a E-Stop)
gui_exit_cmd    = Simulink.Parameter; gui_exit_cmd.CoderInfo.StorageClass    = 'SimulinkGlobal'; gui_exit_cmd.Value    = 0; % Whether to stop the model's exection (gracefully)
imu_reset_cmd   = Simulink.Parameter; imu_reset_cmd.CoderInfo.StorageClass   = 'SimulinkGlobal'; imu_reset_cmd.Value   = 0; % Whether to reset the IMU system

% Low-level Medulla parameters
MEDULLA_ADC_OFFSET_COUNTS = 172;  % Ticks
MEDULLA_ADC_MAX_VOLTS     = 2.70; % Volts

% Renishaw Absolute 32-bit encoders
LEG_A_CALIB_LOC = pi + 0.305432619099008; % Radians
LEG_B_CALIB_LOC = pi - 0.305432619099008; % Radians

% Ticks to radians constants
BOOM_PITCH_ENCODER_RAD_PER_TICK = 2*pi / (2^17 * 2); % Ticks to rad constant = rad_per_rev / (ticks_per_rev * gear_ratio)
BOOM_ROLL_ENCODER_RAD_PER_TICK = 2*pi / (2^17 * 7); % Ticks to rad constant = rad_per_rev / (ticks_per_rev * gear_ratio)
BOOM_YAW_ENCODER_RAD_PER_TICK = 2*pi / (2^17 * 9.6); % Ticks to rad constant = rad_per_rev / (ticks_per_rev * gear_ratio)
RIGHT_HIP_ABS_ENCODER_RAD_PER_TICK = 2*pi / 2^13; % Ticks to rad constant = rad_per_rev / ticks_per_rev
LEFT_HIP_ABS_ENCODER_RAD_PER_TICK = -2*pi / 2^13; % Ticks to rad constant = rad_per_rev / ticks_per_rev
HIP_INC_ENCODER_RAD_PER_TICK =  2*pi / (2500 * 4 * 57);  % Ticks to rad constant = rad_per_rev / (lines_per_rev * quadrature * gear_ratio)
RIGHT_HIP_INC_ENCODER_RAD_PER_TICK =  -HIP_INC_ENCODER_RAD_PER_TICK;
LEFT_HIP_INC_ENCODER_RAD_PER_TICK =  HIP_INC_ENCODER_RAD_PER_TICK;
LEG_INC_ENCODER_RAD_PER_TICK = 2*pi / (4*3500*50); % rad_per_rev / (decoding style * lines per rev * gear ratio)

% Motor parameters
LEFT_MOTOR_HIP_DIRECTION = 1.0; %The direction for the left hip motor.
RIGHT_MOTOR_HIP_DIRECTION = -1.0; %The direction for the right hip motor.
MTR_MAX_COUNT = 1990; % The maximum commanded amplifier value. This is the maximum value sent to the Medullas for the amplifier command.
LEG_MOTOR_CONSTANT = 0.119;
HIP_MOTOR_CONSTANT = 0.184;
HIP_MTR_GEAR_RATIO = 57;
LEG_MTR_GEAR_RATIO = 50;

HIP_INC_MAX_ENCODER_TICKS = 2^16 - 1; % Max value of the incremental hip encoder (16-bit)
HIP_ABS_MAX_ENCODER_TICKS = 2^13 - 1; % Max value of the absolute hip encoder (13-bit)
BOOM_MAX_ENCODER_TICKS = 2^17 - 1; % Max value of the absolute boom encoders (17-bit)

MTR_MAX_TEMP = 118; % Maximum motor temperature, degrees Celsius

% From biped1_variant_defs.h
% Note: Left and right are swapped for CMU
RIGHT_HIP_CALIB_VAL = 4301;   %Calibration encoder value in ticks (OSU convention)
LEFT_HIP_CALIB_VAL = 1265;  %Calibration encoder value in ticks (OSU convention)
RIGHT_HIP_CALIB_POS = -0.1066;  %Calibration angle in radians
LEFT_HIP_CALIB_POS = 0.0422; %Calibration angle in radians
MTR_MAX_CONT_CURRENT = 120.0; %Maximum continuous amplifier current (RMS Amps)
MTR_MAX_CURRENT = MTR_MAX_CONT_CURRENT*2^0.5; %Maximum motor current for scaling (Amps)
MTR_HIP_MAX_CURRENT = 60.0; %Maximum hip motor current for scaling
BOOM_LENGTH = 2.006; % meters, center of rotation to hip center
BOOM_X_METERS_PER_TICK = -0.00000937522094511213193198; %The meters of boom motion per encoder tick. This is calculated from the boom's length, the number of encoder ticks per encoder revolution, and the gear ratio between the boom and the encoder.
LEG_MTR_MAX_TORQUE = MTR_MAX_CONT_CURRENT*LEG_MOTOR_CONSTANT*LEG_MTR_GEAR_RATIO;
LEG_MTR_MAX_CONT_TORQUE = MTR_MAX_CONT_CURRENT*LEG_MOTOR_CONSTANT*LEG_MTR_GEAR_RATIO;
LEG_MTR_MAX_VELOCITY = 393.755 / LEG_MTR_GEAR_RATIO; % Rad/s
HIP_MTR_MAX_TORQUE = MTR_HIP_MAX_CURRENT*HIP_MOTOR_CONSTANT*HIP_MTR_GEAR_RATIO;
LEG_CURRENT_LIMIT = MTR_MAX_CONT_CURRENT; % Maximum motor current for testing
HIP_CURRENT_LIMIT = MTR_HIP_MAX_CURRENT;  % Maximum motor current for testing

% Right Leg (A/B flipped) (TRAN == motor)
LEG1_LEG_A_CALIB_VAL  = 262986707; %Calibration encoder value in ticks
LEG1_TRAN_A_CALIB_VAL = 197479830; %Calibration encoder value in ticks
LEG1_LEG_B_CALIB_VAL  = 263740968; %Calibration encoder value in ticks
LEG1_TRAN_B_CALIB_VAL = 199010960; %Calibration encoder value in ticks

LEG1_LEG_A_RAD_PER_CNT  = -9.8039216e-09; %Ticks to rad constant
LEG1_TRAN_A_RAD_PER_CNT = -9.8039216e-09; %Ticks to rad constant
LEG1_LEG_B_RAD_PER_CNT  =  9.8039216e-09; %Ticks to rad constant
LEG1_TRAN_B_RAD_PER_CNT =  9.8039216e-09; %Ticks to rad constant

LEG1_MOTOR_A_DIRECTION = -1.0; 
LEG1_MOTOR_B_DIRECTION = 1.0;

% Left Leg (A/B flipped) (TRAN == motor)
LEG2_LEG_A_CALIB_VAL  = 173855185; %Calibration encoder value in ticks
LEG2_TRAN_A_CALIB_VAL = 197730500; %Calibration encoder value in ticks
LEG2_LEG_B_CALIB_VAL  = 264966140; %Calibration encoder value in ticks
LEG2_TRAN_B_CALIB_VAL = 142254400; %Calibration encoder value in ticks

LEG2_LEG_A_RAD_PER_CNT  =  9.8039216e-09; %Ticks to rad constant
LEG2_TRAN_A_RAD_PER_CNT = -9.8039216e-09; %Ticks to rad constant
LEG2_LEG_B_RAD_PER_CNT  =  9.8039216e-09; %Ticks to rad constant
LEG2_TRAN_B_RAD_PER_CNT = -9.8039216e-09; %Ticks to rad constant

LEG2_MOTOR_A_DIRECTION = -1.0;
LEG2_MOTOR_B_DIRECTION = 1.0;

% Soft limits for motor positions
HARD_MOTOR_POSITION_LIMITS_UPPER = [3.60; 4.78; 2.47]; % A B retraction
HARD_MOTOR_POSITION_LIMITS_LOWER = [1.51; 2.68; 0.50]; % A B extension
limExt = 0.1;
MOTOR_POSITION_LIMITS_UPPER = [HARD_MOTOR_POSITION_LIMITS_UPPER(1:2)-limExt; HARD_MOTOR_POSITION_LIMITS_UPPER(3)-0.3];
MOTOR_POSITION_LIMITS_LOWER = [HARD_MOTOR_POSITION_LIMITS_LOWER(1:2)+limExt; HARD_MOTOR_POSITION_LIMITS_LOWER(3)+0.15];

% params for DAQ functions
max_motor_velocity = 7.88; % rad/s
max_hip_velocity   = 5;    % rad/s

% Parameters related to incremental encoder decoding
INC_ENC_RAD_PER_TICK = 2*pi/14000/LEG_MTR_GEAR_RATIO;
LEG_INC_ENCODER_DIRECTION_RIGHT_BACK = -1.0;
LEG_INC_ENCODER_DIRECTION_RIGHT_FRONT = 1.0;
LEG_INC_ENCODER_DIRECTION_LEFT_BACK = -1.0;
LEG_INC_ENCODER_DIRECTION_LEFT_FRONT = 1.0;
MEDULLA_TIMER_FREQ = 32e6;
