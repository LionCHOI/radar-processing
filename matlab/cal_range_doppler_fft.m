function [fft_1D, fft_2D, Rx_sig_3D] = cal_range_doppler_fft(ego, pts, wave_param, ant_params_tx, ant_params_rx)

% this funciton perform range/doppler fft
% inputs:


%% Extract essential parameters

c = 3e8;

f_c = wave_param.f_c;                       % center freq
rtt_max = wave_param.rtt_max;   
sweep_time = wave_param.sweep_time;
sweep_slope = wave_param.sweep_slope;
fs = wave_param.fs;
ts = wave_param.ts;                         % sampling time
fb_max = wave_param.fb_max;
range_fft_size = wave_param.range_fft_size;
doppler_fft_size = wave_param.doppler_fft_size;
chirp_guard_time = wave_param.chirp_guard_time;


%% Preparation

% compute relative position and velocity
n_pt = length(pts);

pt_ranges = nan(n_pt, 1);           % range
pt_velos = nan(n_pt, 1);            % radial velocity
pt_unit_dir_vecs = nan(n_pt, 3);    % unit direction vectors

for k=1:n_pt
    rel_pos = pts(k).location - ego.location;
    curr_unit_dir_vec = rel_pos / norm(rel_pos);

    curr_radial_velo = curr_unit_dir_vec * pts(k).velocity' - curr_unit_dir_vec * ego.velocity';

    pt_ranges(k) = norm(rel_pos);       % store results
    pt_velos(k) = curr_radial_velo;
    pt_unit_dir_vecs(k, :) = curr_unit_dir_vec;
end

% compute sampling index/time
sample_start_idx = ceil(rtt_max / ts);      % 
sample_idx = sample_start_idx : floor(sweep_time / ts);
if length(sample_idx) >= range_fft_size
    sample_idx = sample_idx(1:range_fft_size);  
else
    fprintf(2, '[WARNING] Insufficient sample size. Range FFT size will be reduced from %d to %d\n', range_fft_size, length(sample_idx));
    range_fft_size = length(sample_idx);
end
sample_time = sample_idx * ts;


% compute # rx antennas
if strcmp(ant_params_rx.type, 'planar')    % # rx antennas
    n_antenna = ant_params_rx.h_n_elements * ant_params_rx.v_n_elements;
else
    error([ant_params_rx.type, ' is not supported yet']);
end


% compute steering vectors
[gain_tx, s_vec_tx] = getSteeringVector(ant_params_tx, pt_unit_dir_vecs, ego.radar_orientation);
[gain_rx, s_vec_rx] = getSteeringVector(ant_params_rx, pt_unit_dir_vecs, ego.radar_orientation);


%% Range FFT (1D)

fft_1D = zeros(range_fft_size/2, doppler_fft_size, n_antenna);          % placeholder for range FFT
tx_sig = cos(2*pi*f_c*sample_time + 0.5*sweep_slope*sample_time.^2);    % transmit signal

inter_chirp_time = sweep_time + chirp_guard_time;

% 변경 시작
sum_set = zeros([12, 5]);       % 노이즈가 섞인 값 (확인용)
noise_set = zeros([12, 5]);     % 노이즈 값 (확인용)
unit_num_antenna = 4;
noise_diff_module_set = zeros([1, n_antenna/unit_num_antenna]);% + 2*pi*rand(n_antenna/unit_num_antenna);
noise_diff_module_set(1) = 0;       % 첫 번쨰는 0
% 변경 끝

for chirp_idx = 1:doppler_fft_size

    % [Note] it takes too much time to compute rx signal and apply lowpass later
    % we compute dechirp signal manually

%     dechirp_sig = zeros(range_fft_size, n_antenna);
    dechirp_sig = randn(range_fft_size, n_antenna);         % add noise with the power of 1

    fast_time = 512;
    slow_time = 100;
    time_setting = zeros(fast_time, slow_time);

    Rx_sig_3D = zeros(n_antenna, fast_time, slow_time);     %% 1/sqrt(2) * (randn(n_antenna, fast_time, slow_time) + 1i*randn(n_antenna, fast_time, slow_time));
    Rx_sig_3D = 1/sqrt(2) * (randn(n_antenna, fast_time, slow_time) + 1i*randn(n_antenna, fast_time, slow_time));
    Rx_sig_2D = zeros(n_antenna, fast_time);    %% 1/sqrt(2) * (randn(n_antenna, fast_time) + 1i*randn(n_antenna, fast_time));
    Rx_sig_2D = 1/sqrt(2) * (randn(n_antenna, fast_time) + 1i*randn(n_antenna, fast_time));

    for pt_idx = 1:n_pt
        curr_range = pt_ranges(pt_idx) + pt_velos(pt_idx) * inter_chirp_time * (chirp_idx-1);
        curr_td = 2 * curr_range / c;
        curr_delay = sample_time - curr_td;
        curr_SNR_dB = pts(pt_idx).SNR_dB;
        curr_SNR_linear = 10.^(curr_SNR_dB/20);

        for ant_idx = 1:n_antenna
            
            % noise setting start
            if rem((ant_idx),4) == 1        % choose noise
                seq_antenna = floor((ant_idx-1)/4) + 1;
                noise_diff_module = noise_diff_module_set(seq_antenna);
            end
            % noise setting terminate

            curr_ant_phase = angle(s_vec_rx(ant_idx, pt_idx)) + noise_diff_module;          % phase from antenna array
            
            % adding noise start
            sum_set(ant_idx,pt_idx) = curr_ant_phase;       % value mixed noise (for check)
            noise_set(ant_idx,pt_idx) = noise_diff_module;  % only noise (for check)

            if curr_ant_phase == 0 && (ant_idx == 1)
                ant_ang = 0;
                prev_ant_phase = curr_ant_phase;
            elseif (curr_ant_phase>prev_ant_phase) && (ant_idx == 2)
                ant_ang = (curr_ant_phase - prev_ant_phase)/2/pi;
            elseif (prev_ant_phase>curr_ant_phase) && (ant_idx == 2)
                ant_ang = (prev_ant_phase - curr_ant_phase)/2/pi;
            end
           % adding noise terminate

           % Rx_sig setting start
            fast_sample_time = linspace(0,wave_param.sweep_time,fast_time);    % sample_time;
            slow_sample_time = (1:slow_time)*wave_param.sweep_time + fast_sample_time(2); 
            time_setting(:, 1) = transpose(fast_sample_time);   % first chirp
            for i = 2:slow_time     % from second chirp
                time_setting(:, i) = transpose(fast_sample_time + slow_sample_time(i-1)); 
            end
            % 3D Rx_sig
            sig_tmp = curr_SNR_linear*s_vec_rx(ant_idx, pt_idx)*exp(1i*(2*pi*(sweep_slope*curr_td*(time_setting)+f_c*curr_td)-pi*sweep_slope*curr_td^2));
            Rx_sig_3D(ant_idx, :, :) = Rx_sig_3D(ant_idx, :, :) + reshape(sig_tmp, [1, fast_time, slow_time]);

            % 2D Rx_sig
            Rx_sig_2D(ant_idx, :) = Rx_sig_2D(ant_idx, :) +  curr_SNR_linear*s_vec_rx(ant_idx, pt_idx)*exp(1i*(2*pi*(sweep_slope*curr_td*(fast_sample_time)+f_c*curr_td)-pi*sweep_slope*curr_td^2));
            % Rx_sig setting terminate

            dechirp_sig(:, ant_idx) = dechirp_sig(:, ant_idx) + curr_SNR_linear*0.5*transpose(cos(2*pi*(f_c*curr_td + sweep_slope*curr_td*(curr_delay) - 0.5*sweep_slope*curr_td^2) + curr_ant_phase));
        end
    end

    % dechirp process
    for ant_idx = 1:n_antenna
        curr_range_fft = fft(dechirp_sig(:, ant_idx));
        fft_1D(1:range_fft_size/2, chirp_idx, ant_idx) = curr_range_fft(1:range_fft_size/2);
        
        if chirp_idx==1 && ant_idx == 1
            plot_fft(fft_1D(:,:,1), wave_param);
        end
    end
end

%% Doppler FFT (2D)
fft_2D = zeros(size(fft_1D));
for ant_idx = 1:n_antenna
    fft_2D(:,:,ant_idx) = fft(fft_1D(:,:,ant_idx), [], 2);
    fft_2D(:,:,ant_idx) = fftshift(fft_2D(:,:,ant_idx), 2);
end

end


%% make the 1st fft for range
function plot_fft(FFT_1D, wave_param)
    
    FFT_1D = abs(FFT_1D)/wave_param.range_fft_size;             % amplitude
    FFT_1D = FFT_1D(1:wave_param.range_fft_size/2);             % remove the other part value

    f = wave_param.fs*(0:(wave_param.range_fft_size/2)-1)/wave_param.range_fft_size;             % sampled freaquency
    r = beat2range(f', wave_param.sweep_slope);    % beat to range function (1D row is only available for position f)

    figure('Name', 'Range from First FFT');
    plot(r, FFT_1D);                      
    axis([0 wave_param.range_max 0 2]);
    xlabel('range[m]');
    ylabel('amplitude');
end