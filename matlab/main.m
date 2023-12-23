clear;clc;close all;

%% Parameters

f_c = 77e9;                 % frequency [Hz]
c = 3e8;                    % speed of light [m/s]
lambda = c / f_c;           % wavelength [m]

range_max = 200;            % maximum range [m]
range_res = 1;            % range resolution [m]

v_max = 230 * 1000/3600;    % maximum speed of vehicle [m/s]


%% Waveform-related parameters

rtt_max = 2 * range_max / c;        % maximum round trip time [s]
sweep_time = 5.5 * rtt_max;
bw = c / range_res / 2;             % bandwidth [Hz]

sweep_slope = bw / sweep_time;

% determine sampling rate

fr_max = sweep_slope * rtt_max;     % maximum beat frequency due to range
fd_max = 2*v_max/lambda;            % maximum beat frequency due to doppler shift
fb_max = fr_max + fd_max;

fs = max(2*fb_max, bw);             % sampling rate [# sample/s]

% print waveform paramters
fprintf('---- System parameters ----\n');
fprintf('Carrier frequency: %.2f [GHz]\n', f_c / 1e9);
fprintf('Maximum target range: %.2f [m]\n', range_max);
fprintf('Range resolution: %.2f [m]\n', range_res);
fprintf('Maximum target speed: %.2f [km/h]\n', v_max * 3.6);
fprintf('Sweep time: %.2f [us]\n', sweep_time * 1e6);
fprintf('Sweep bandwidth: %.2f [MHz]\n', bw / 1e6);
fprintf('Maximum beat frequency: %.2f [Mhz]\n', fb_max / 1e6);
fprintf('Sampling rate: %.2f [Mhz]\n', fs / 1e6);
fprintf('---------------------------\n');


%% Plot signal

sampling_interval = 1/fs;               % sampling interval [s]

t = 0:sampling_interval:sweep_time;
x_base = cos(2*pi*(sweep_slope/2 * t.^2));  % baseband FMCW signal

plot(t, x_base);
title('Baseband FMCW waveform');
xlabel('time [s]');
ylabel('Amplitude [normalized]');


%% Deploy target

ego = struct('location', [0, 0, 0], ...
             'velocity', [0, 0, 0]);

pt1 = struct('location', [20, 50, 0], ...
             'velocity', [0, 60*1e3/3.6e3, 0]);
pt2 = struct('location', [20, 50, 0], ...
             'velocity', [0, 60*1e3/3.6e3, 0]);


%pt3...




