clear; close all;

%% load local flight data
clear;

% homeDir = getenv('HOME');
% addpath(genpath(fullfile(homeDir,'tails_plotters/src/')));

% p = parselog('~/LOGS/cyclone2v1_pprz/20241030_valken_ewoud/144/24_10_30__16_27_37_SD.data'); log_nbr = '144';
% p = parselog('~/LOGS/20241030_valken_ewoud/145/24_10_30__16_45_37_SD.data'); log_nbr = '145';
p = parselog('~/LOGS/cyclone2v1_pprz/20241030_valken_ewoud/148/24_10_30__17_27_57_SD.data'); log_nbr = '148';

% p = parselog('~/LOGS/20241211_valken_vaggelis/161/24_12_11__15_21_45_SD.data'); log_nbr = '161';

% p = parselog('~/LOGS/20241220_cybezoo_tuning/0221/22_05_01__01_59_46_SD.data'); log_nbr = '0221';
% p = parselog('~/LOGS/20241220_cybezoo_tuning/0222/22_05_01__01_59_46_SD.data'); log_nbr = '0222';

% p = parselog('~/LOGS/20250108_mavlab_G_testing/0228/25_01_08__12_46_39_SD.data'); log_nbr = '0228';

% p = parselog('~/LOGS/20250116_cyberzoo_training/0253/25_01_16__17_55_45_SD.data'); log_nbr = '0253';

% p = parselog('~/LOGS/20250117_valken_first_succ_manual/0254/25_01_17__14_22_01_SD.data'); log_nbr = '0254';
% p = parselog('~/LOGS/cyclone2v1_pprz/20250117_valken_first_succ_manual/0257/25_01_17__15_36_58_SD.data'); log_nbr = '0257';

% p = parselog('~/LOGS/20250122_mavlab_elrs/0299/25_01_22__13_55_07_SD.data'); log_nbr = '0299';

% p = parselog('~/LOGS/20250128_cybezoo_first_nav/0351/25_01_27__06_21_40_SD.data'); log_nbr = '0351';

% p = parselog('~/LOGS/20250217_cyberzoo_cutoff_freq/0392/25_04_01__08_56_31_SD.data'); log_nbr = '0392';

% p = parselog('~/LOGS/20250218_cyberzoo_phi/0394/25_04_02__08_10_52_SD.data'); log_nbr = '0394';

% p = parselog('~/LOGS/20250303_cyberzoo_nav_long/0397/25_03_10__06_28_16_SD.data'); log_nbr = '0397';
% p = parselog('~/LOGS/20250303_cyberzoo_nav_long/0398/25_03_10__07_35_43_SD.data'); log_nbr = '0398';

% p = parselog('~/LOGS/20250303_valken_nav/0403/25_03_03__15_00_21_SD.data'); log_nbr = '0403';

% p = parselog('~/LOGS/20250305_cyberzoo_HOV_C/0415/25_03_12__09_03_08_SD.data'); log_nbr = '0415';
% p = parselog('~/LOGS/20250305_cyberzoo_HOV_C/0416/25_03_12__09_09_42_SD.data'); log_nbr = '0416';

% p = parselog('~/LOGS/cyclone2v1_pprz/20250307_valken_spiral/0418/22_05_01__01_59_46_SD.data'); log_nbr = '0418';
% p = parselog('~/LOGS/20250307_valken_spiral/0420/25_03_07__16_26_20_SD.data'); log_nbr = '0420';

% p = parselog('~/LOGS/20250319_delft_ekf/0426/25_03_19__16_52_56_SD.data'); log_nbr = '0426';
% p = parselog('~/LOGS/20250319_delft_ekf/0427/25_03_19__17_03_32_SD.data'); log_nbr = '0427';

% p = parselog('~/LOGS/20250320_cyberzoo_Gmatrix/0430/22_05_01__01_59_46_SD.data'); log_nbr = '0430';
% p = parselog('~/LOGS/20250320_cyberzoo_Gmatrix/0431/25_03_20__09_51_07_SD.data'); log_nbr = '0431';
% p = parselog('~/LOGS/20250320_cyberzoo_Gmatrix/0433/22_05_01__01_59_46_SD.data'); log_nbr = '0433';
% p = parselog('~/LOGS/20250320_cyberzoo_Gmatrix/0440/25_03_22__08_42_46_SD.data'); log_nbr = '0440';
% p = parselog('~/LOGS/20250320_cyberzoo_Gmatrix/0443/25_03_22__09_02_58_SD.data'); log_nbr = '0443';

% p = parselog('~/LOGS/20250407_OJF/0061/25_04_07__18_32_42_SD_no_GPS.data'); log_nbr = '0061';
% p = parselog('~/LOGS/20250407_OJF/0062/25_04_07__19_28_56_SD_no_GPS.data'); log_nbr = '0062';

% p = parselog('~/LOGS/0061/25_04_07__18_32_42_SD_no_GPS.data'); log_nbr = 'temp';
ac_data = p.aircrafts.data;

%% Imu scaled
figure('Name','IMU Scaled');
plot_imu_scaled(ac_data, true, 1);

%% Euler angles
figure('Name','Euler ZXY');
plot_eul(ac_data,'ZXY');

%% actuators
figure('Name', 'Actuators');
plot_actuators(ac_data);

%% STAB data
figure('Name','Stab Attitude');
plot_stab_attitude(ac_data);

%% RC controls
figure('Name', 'RC controls');
plot_rc_controls(ac_data);

%% Speed (negative speed --> zero)
figure('Name', 'Speed');
plot_speed(ac_data);

%% Effectiveness Matrix
figure('Name','Effectiveness Matrix'); 
plot_eff_mat(ac_data);

%% Optitrack
figure('Name','Optitrack');
plot_optitrack(ac_data);

%% Guidance (position plotting still uses carrot signal; check if needs fixing)
figure('Name', 'Guidance INDI Hybrid');
plot_guidance_indi_hybrid(ac_data);

%% rotorcraft fp
figure('Name','Rotorcraft FP');
plot_rotorcraft_fp(ac_data, p.aircrafts.motors_on);

%%
% figure;
% plot_rotorcraft_status(ac_data)

%% Energy
figure('Name', 'Energy'); 
plot_energy(ac_data);

%% VISUALIZE FLIGHT
vis(ac_data,20);

%% load sim logs
MATLABBASE = '~/MATLAB/';

addpath(fullfile(MATLABBASE, 'paparazzi_log_parsing/matlab/math'));
addpath(fullfile(MATLABBASE, 'paparazzi_log_parsing/matlab/tools'));
addpath(fullfile(MATLABBASE, 'paparazzi_log_parsing/matlab/plotters'));

filename = '~/pprz_sim_logs/ZXY_20241204-155134.csv';
data = readtable(filename, 'Delimiter', ',');
ac_data = struct();

ac_data.AHRS_REF_QUAT.timestamp = data.timestamp;
ac_data.AHRS_REF_QUAT.ref_qi = data.ref_qi;
ac_data.AHRS_REF_QUAT.ref_qx = data.ref_qx;
ac_data.AHRS_REF_QUAT.ref_qy = data.ref_qy;
ac_data.AHRS_REF_QUAT.ref_qz = data.ref_qz;
ac_data.AHRS_REF_QUAT.body_qi = data.qi;
ac_data.AHRS_REF_QUAT.body_qx = data.qx;
ac_data.AHRS_REF_QUAT.body_qy = data.qy;
ac_data.AHRS_REF_QUAT.body_qz = data.qz;

ac_data.ROTORCRAFT_RADIO_CONTROL.timestamp = data.timestamp;
ac_data.ROTORCRAFT_RADIO_CONTROL.roll = data.rc_roll;
ac_data.ROTORCRAFT_RADIO_CONTROL.pitch = data.rc_pitch;
ac_data.ROTORCRAFT_RADIO_CONTROL.yaw = data.rc_yaw;
ac_data.ROTORCRAFT_RADIO_CONTROL.throttle = data.rc_throttle;
ac_data.ROTORCRAFT_RADIO_CONTROL.mode = data.rc_mode;

%% add here older logs loading commands so that the top of the script does not overflow

% p = parselog('~/LOGS/20250129_cyberzoo_asq/0362/25_01_28__02_44_18_SD.data'); log_nbr = '0362';
% p = parselog('~/LOGS/20250129_cyberzoo_asq/0363/25_01_28__03_01_01_SD.data'); log_nbr = '0363';
% p = parselog('~/LOGS/20250129_cyberzoo_asq/0364/25_01_28__03_18_38_SD.data'); log_nbr = '0364';
% p = parselog('~/LOGS/20250129_cyberzoo_asq/0365/25_01_28__03_22_29_SD.data'); log_nbr = '0365';
% p = parselog('~/LOGS/20250129_cyberzoo_asq/0368/25_01_28__05_03_03_SD.data'); log_nbr = '0368';
% p = parselog('~/LOGS/20250129_cyberzoo_asq/0369/25_01_28__05_10_20_SD.data'); log_nbr = '0369';
% p = parselog('~/LOGS/20250129_cyberzoo_asq/0370/25_01_28__05_37_23_SD.data'); log_nbr = '0370';

% p = parselog('~/LOGS/20250130_cyberzoo_nav_cont/0374/25_01_28__23_03_22_SD.data'); log_nbr = '0374';
% p = parselog('~/LOGS/20250130_cyberzoo_nav_cont/0375/25_01_29__00_13_45_SD.data'); log_nbr = '0375';
% p = parselog('~/LOGS/20250130_cyberzoo_nav_cont/0376/25_01_29__00_32_32_SD.data'); log_nbr = '0376';
% p = parselog('~/LOGS/20250130_cyberzoo_nav_cont/0379/25_02_21__03_29_26_SD.data'); log_nbr = '0379';
% p = parselog('~/LOGS/20250130_cyberzoo_nav_cont/0385/25_02_21__06_09_34_SD.data'); log_nbr = '0385';
% p = parselog('~/LOGS/20250130_cyberzoo_nav_cont/0386/25_02_21__06_21_47_SD.data'); log_nbr = '0386';