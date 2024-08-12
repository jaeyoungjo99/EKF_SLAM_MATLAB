close all; clc; clear;

addpath('common_function');
addpath('filter_folder');
%% simul
position_noise = 0.5;
heading_noise_deg = 5;
[GT_val, sensor_val, input_val] = simul(position_noise, heading_noise_deg);

%% init
Pval = 10000;
P0 = [Pval   0     0 
        0   Pval   0 
        0    0    Pval];

Qvar = 0.001;
Q = [Qvar    0     0 
        0   Qvar   0 
        0    0    Qvar];

R_var = 5;
R = [R_var   0     0 
        0   R_var  0
        0    0    R_var];
%% EKF
[ekf_estimated_state, ekf_error] = ekf(P0, Q, R, GT_val, sensor_val, input_val);

[ekf_abs_mean_error] = calMeanError('EKF', ekf_error);

%% UKF
% UKF Parameters
    alpha = 0.5;
    beta = 2;
    kappa = 20;
[ukf_estimated_state, ukf_error] = ukf(P0, Q, R, GT_val, sensor_val, input_val, alpha, beta, kappa);

[ukf_abs_mean_error] = calMeanError('UKF', ukf_error);

%% EIF
[eif_estimated_state, eif_error] = eif(P0, Q, R, GT_val, sensor_val, input_val);

[eif_abs_mean_error] = calMeanError('EIF', eif_error);

%% Error 비교
visualize_error_result(ekf_error, ukf_error, eif_error, ekf_abs_mean_error, ukf_abs_mean_error, eif_abs_mean_error);