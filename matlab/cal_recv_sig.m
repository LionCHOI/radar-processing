function FFT_1D = cal_recv_sig(fc, c, fs, rtt_max, fr_max, sampling_interval, sweep_time, sweep_slope, Nd, Nr)
    
%% Deploy target

ego = struct('location', [0, 0, 0], ...
             'velocity', [0, 0, 0]);

pt1 = struct('location', [20, 0, 0], ...
             'velocity', [-40, 0, 0]);
pt2 = struct('location', [40, 0, 0], ...
             'velocity', [45, 0, 0]);
pt3 = struct('location', [80, 0, 0], ...
             'velocity', [-50, 0, 0]);
pt4 = struct('location', [160, 0, 0], ...
             'velocity', [-49, 0, 0]);
pt5 = struct('location', [200, 0, 0], ...
             'velocity', [48, 0, 0]);

pt = [pt1, pt2, pt3, pt4 ,pt5];                         % pt array

[target_range, target_velocity] = cal_dist(ego, pt);    % calculation distance & velocity
 

%% calcutlation receive signal

sampling_start_idx = rtt_max / sampling_interval;                           % sampling start index
t = (sampling_start_idx:sampling_start_idx + Nr - 1) * sampling_interval;   % sampling time  

FFT_1D =  zeros(Nr, Nd);                % fft

for col = 1:Nd
    Rx = zeros(1,Nr);                                   % received signal
    Tx = cos(2*pi*((fc*t) + (sweep_slope*t.^2)/2));     % transmit signal

        % calculation Rx loop
        for each_pt = 1:length(pt)            
            r_t = target_range(each_pt) + target_velocity(each_pt) * (t + sweep_time*col);% distance (**consider accumulation)
            td = 2 * r_t / c;           % delay time (= distance/c)
            delay = t - td;             % time part of receive signal
            Rx = Rx + cos(2*pi*(fc*delay + sweep_slope * (delay.^2)/2));   % set receive signal
        end

    Mix_sig = lowpass(Tx.*Rx, fr_max, fs);   % mix signal
    FFT_1D(:,col) = fft(Mix_sig, Nr);        % 1st FFT

    % plot fft 
    if col == 1
        plot_fft(FFT_1D(:,col), Nr, sweep_slope, fs);
    end
end
end

%% calculation distance & velocity
function [distance, velocity] = cal_dist(ego, pt)

distance = zeros(1, length(pt));
velocity = zeros(1, length(pt));

for axis=1:length(pt)
    
    % paramter 
    pos_relative = ego.location - pt(axis).location;    % relateive location
    if pos_relative(1) == 0
        if pos_relative(2) < 0
            theta = pi/2;
        elseif pos_relative(2) > 0
            theta = -pi/2;
        else
            theta = pi/4; % pt position is same with ego
        end    
    else
        theta = atan(pos_relative(2)/pos_relative(1));   % relative angle
    end

    % result
    distance(axis) = norm(pos_relative);               % distance
    velocity(axis) = (pt(axis).velocity(1)-ego.velocity(1))*cos(theta) + ...
        (pt(axis).velocity(2)-ego.velocity(2))*cos(pi/2-theta); %velocity (2D (x,y))
end
end

%% make the 1st fft for range
function plot_fft(FFT_1D, Nr, sweep_slope, fs)
    
    FFT_1D = abs(FFT_1D);               % amplitude
    FFT_1D = FFT_1D(1:Nr/2);            % remove the other part value

    f = fs*(0:(Nr/2)-1)/Nr;             % sampled freaquency
    r = beat2range(f', sweep_slope);    % beat to range function (1D row is only available for position f)

    figure('Name', 'Range from First FFT');
    plot(r, FFT_1D);                      
    axis([0 200 0 150]);
    xlabel('range[m]');
    ylabel('amplitude');
end