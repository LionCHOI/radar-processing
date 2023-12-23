clear;clc;close all;
addpath('library\');

flag_show_parameters = 1;
flag_show_target = 1;       

%% Parameters

% constant
c = 3e8;        % speed of light [m/s]

% waveform parameters
wave_param = struct('f_c', 77e9, ...                % center frequency [Hz]
                    'range_max', 200, ...           % maximum detectable range [m]
                    'range_res', 1, ...             % range resolution [m]
                    'v_max', 230/3.6, ...           % maximum detectable velocity [m/s]
                    'sweep_fact', 6, ...            % sweep time factor (5~6)
                    'range_fft_size', 512, ...      % range fft size
                    'doppler_fft_size', 256, ...    % doppler fft size
                    'chirp_guard_time', 0);         % time between consecutive chirps [s]

% antenna radiation pattern
default_rad_pattern = struct('type', 'patch',...        % 'isotropic' or 'patch'
                             'antenna_gain', 8, ...     % antenna gain [dB]
                             'v_3dB_width', 65, ...     % vertical 3dB beamwidth [deg]
                             'v_SLA', 30, ...           % vertical sidelobe attenuation [dB]
                             'h_3dB_width', 65, ...     % horizontal 3dB beamwidth [deg]
                             'h_SLA', 30);              % horizontal sidelobe attenuation [dB]

% antenna parameters (Rx)
ant_params_rx = struct('type', 'planar', ...            % 'planar' or 'custom'
                       'pattern', default_rad_pattern, ...
                       'v_n_elements', 1, ...           % # vertical elements
                       'h_n_elements', 12, ...          % # horizontal elements
                       'v_spacing', 0.5, ...            % vertical antenna spacing [lambda]
                       'h_spacing', 0.5);               % horizontal antenna spacing [lambda]

% assume that Tx has the same radiation pattern as Rx, but only one antenna
ant_params_tx = ant_params_rx;
ant_params_tx.v_n_elements = 1;
ant_params_tx.h_n_elements = 1;


%% Compute additional waveform parameters

wave_param.lambda = c / wave_param.f_c;                         % wavelength [m]
wave_param.rtt_max = 2*wave_param.range_max / c;                % maximum round trip time [s]
wave_param.sweep_time = wave_param.sweep_fact * wave_param.rtt_max;    % sweep time [s]
wave_param.bw = c / wave_param.range_res / 2;                   % bandwidth [Hz]
wave_param.sweep_slope = wave_param.bw / wave_param.sweep_time; % sweep slope [Hz/s]

% determine sampling rate
fr_max = wave_param.sweep_slope * wave_param.rtt_max;   % maximum beat frequency due to range
fd_max = 2*wave_param.v_max / wave_param.lambda;        % maximum beat frequency due to doppler shift
wave_param.fb_max = fr_max + fd_max;                    % maximum beat frequency [Hz]           

wave_param.fs = max(2*wave_param.fb_max, wave_param.bw);  % sampling rate [# sample/s]
wave_param.ts = 1/wave_param.fs;    % sampling time [s]

% maximum # of samples taken in a chirp
wave_param.max_samples_per_chirp = floor((wave_param.sweep_time-wave_param.rtt_max)/wave_param.ts);


%% print waveform paramters
if flag_show_parameters
    fprintf('---- Waveform parameters ----\n');
    fprintf('Carrier frequency: %.2f [GHz]\n', wave_param.f_c / 1e9);
    fprintf('Maximum target range: %.2f [m]\n', wave_param.range_max);
    fprintf('Range resolution: %.2f [m]\n', wave_param.range_res);
    fprintf('Maximum target speed: %.2f [km/h]\n', wave_param.v_max * 3.6);
    fprintf('Sweep time: %.2f [us]\n', wave_param.sweep_time * 1e6);
    fprintf('Sweep bandwidth: %.2f [MHz]\n', wave_param.bw / 1e6);
    fprintf('Maximum beat frequency: %.2f [Mhz]\n', wave_param.fb_max / 1e6);
    fprintf('Sampling rate: %.2f [Mhz]\n', wave_param.fs / 1e6);
    fprintf('Sampling interval: %f [us]\n', wave_param.ts * 1e6)
    fprintf('Maximum samples per chirp: %d\n', wave_param.max_samples_per_chirp);
    fprintf('Range FFT size: %d\n', wave_param.range_fft_size);
    if wave_param.max_samples_per_chirp < wave_param.range_fft_size
        fprintf(2, '[WARNING] range FFT size is greater than maximum available samples\n');
    end        
    fprintf('Doppler FFT size: %d\n', wave_param.doppler_fft_size);
    fprintf('Total time for %d chirps: %f [us]\n', wave_param.doppler_fft_size, wave_param.doppler_fft_size*(wave_param.sweep_time+wave_param.chirp_guard_time)*1e6);
    fprintf('----------------------------\n');
end


%% Deploy targets in the reference coordinate system (ENU system)
% i.e., x, y, z axes point East, North, and Up directions, respectively

% align radar coordinate
quat = [1, 0, 0, 0];
quat = singleAxisRotation(quat, 90, 0, 0);
quat = singleAxisRotation(quat, 0, 180, 0);

ego = struct('location', [0, 0, 0], ...
             'velocity', [0, 0, 0], ...
             'radar_orientation', quat);        % orientation (quaternion) of radar module

% deploy point targets

pt1 = struct('location', [0, 20, 0], ...
             'velocity', [0, -10, 0], ...
             'SNR_dB', 20);
pt2 = struct('location', [10, 40, 0], ...
             'velocity', [0, 20, 0], ...
             'SNR_dB', 20);
pt3 = struct('location', [10, 30, 0], ...
             'velocity', [0, 30, 0], ...
             'SNR_dB', 20);
pt4 = struct('location', [-10, 40, 0], ...
             'velocity', [0, 30, 0],...
             'SNR_dB', 20);
pt5 = struct('location', [50, 10, 0], ...
             'velocity', [0, 60, 0],...
             'SNR_dB', 20);

pts = [pt1, pt2, pt3, pt4 ,pt5];                % pt array


if flag_show_target
    figure; hold on;

    % prepare radar elements
    lambda = wave_param.lambda;
    h_spacing = ant_params_rx.h_spacing;
    n_h = ant_params_rx.h_n_elements;
    v_spacing = ant_params_rx.v_spacing;
    n_v = ant_params_rx.v_n_elements;
    
    radar_loc = (h_spacing*(0: n_h-1)) + 1i*(v_spacing * (0:n_v-1)');
    radar_loc = lambda * radar_loc(:);

    radar_loc_3D = [real(radar_loc), imag(radar_loc), zeros(length(radar_loc),1)];
    radar_boresight_dir = [0, 0, 1];

    % transform radar elements to ENU coordinates
    rot_mat = getRotationMatrixFromVector(ego.radar_orientation);
    radar_loc_3D_enu = rot_mat * radar_loc_3D';
    radar_boresight_dir_enu = rot_mat * radar_boresight_dir';

    for k=1:length(radar_loc)
        h = plot3(radar_loc_3D_enu(1,k), radar_loc_3D_enu(2,k), radar_loc_3D_enu(3,k), 'rx');
        h = line([radar_loc_3D_enu(1,k), radar_loc_3D_enu(1,k)+radar_boresight_dir_enu(1)],...
            [radar_loc_3D_enu(2,k), radar_loc_3D_enu(2,k)+radar_boresight_dir_enu(2)],...
            [radar_loc_3D_enu(3,k), radar_loc_3D_enu(3,k)+radar_boresight_dir_enu(3)]);
    end         

    % plot targets
    dt = 0.1;       % time for velocity plot
    for pt = pts
        curr_loc = pt.location;
        curr_velo = pt.velocity;

        h = plot3(curr_loc(1), curr_loc(2), curr_loc(3), 'rx');
        h.MarkerSize = 10;
        h.LineWidth = 2;
        
        h = line([curr_loc(1), curr_loc(1)+curr_velo(1)*dt],...
            [curr_loc(2), curr_loc(2)+curr_velo(2)*dt],...
            [curr_loc(3), curr_loc(3)+curr_velo(3)*dt]);
        h.Color = 'k';
        h.LineWidth = 2;
    end
end


%% range-doppler FFT

[fft_1D, fft_2D, Rx_sig] = cal_range_doppler_fft(ego, pts, wave_param, ant_params_tx, ant_params_rx);

%% angle

angle_range = 0:0.05:180;
pt_unit_dir_vecs = [cos(angle_range'*pi/180) sin(angle_range'*pi/180) zeros(length(angle_range),1)];
[~, s_vec_rx] = getSteeringVector(ant_params_rx, pt_unit_dir_vecs, ego.radar_orientation);
range_angle = zeros(wave_param.range_fft_size/2, 2);

% caculation 
angle_result = reshape(fft_1D,[wave_param.doppler_fft_size^2, ant_params_rx.h_n_elements * ant_params_rx.v_n_elements]);
angle_result = angle_result * conj(s_vec_rx);
angle_result = reshape(angle_result,[wave_param.doppler_fft_size,wave_param.doppler_fft_size,length(angle_range)]);

frequency_range = wave_param.fs*(0:(wave_param.range_fft_size/2)-1)/wave_param.range_fft_size;
range = beat2range(frequency_range', wave_param.sweep_slope);

plot_angle(range, angle_range, wave_param, angle_result);

%% Doppler-FFT

frequency_velocity = (0:(wave_param.range_fft_size/2)-1)/wave_param.range_fft_size;
velocity = lambda*frequency_velocity/2/wave_param.sweep_time;
velocity = linspace(-max(velocity),max(velocity),256); 
Range_Doppler_fft = abs(fft_2D(:,:,1));

figure('Name', 'range-doppler map');
surf(velocity,range, Range_Doppler_fft);
shading interp % shading flat --> 점들의 집합이라서 검게 표현된다. --> interpolation!
colorbar;
xlim([-wave_param.v_max,wave_param.v_max])
ylim([0,wave_param.range_max])
view(0,90)
ylabel('range [m]');
xlabel('velocity [m/s]');

CFAR_2D(Range_Doppler_fft, wave_param, velocity, range);

%% Music AL
Rx_cov = pagemtimes(Rx_sig, pagectranspose(Rx_sig));        % 마지막 차원을 기준으로 행렬곱을 취한다. (3차원 행렬)
Rx_cov = mean(Rx_cov, 3);                                   % 마지막 차원을 기준으로 평균을 취한다. --> RX_cov
% Rx_cov = (Rx_sig*Rx_sig')/wave_param.doppler_fft_size;    % Rx_sig가 2차원인 경우 
[doas,spec,specang] = musicdoa(Rx_cov, 5);              
num_angle = length(doas);
test_doas_val = zeros([1, num_angle]);
real_doas = zeros([1, num_angle]);
for i=1:num_angle 
    test_doas_val(i) = 10*log10(spec(doas(i)+91));
    real_doas(i) = atan2(pts(i).location(1), pts(i).location(2))*180/pi;
end
figure("Name","Music")
plot(specang,10*log10(spec))
hold on;
plot(doas, test_doas_val, 'ro');
plot(real_doas, max(test_doas_val), 'bx');
xlabel('Arrival Angle (deg)')
ylabel('Magnitude (dB)')
title('MUSIC Spectrum')
grid
disp(sort(doas));
disp(sort(real_doas));


