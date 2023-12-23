function [A_total, steering_vec] = getSteeringVector(antenna_params, ray_vectors_ref_frame, quat)

%% Antenna parameter

v_n = antenna_params.v_n_elements;
h_n = antenna_params.h_n_elements;
v_spacing = antenna_params.v_spacing;
h_spacing = antenna_params.h_spacing;

radiation_pattern = antenna_params.pattern;

antenna_gain = radiation_pattern.antenna_gain;
v_SLA = radiation_pattern.v_SLA;
h_SLA = radiation_pattern.h_SLA;
v_3dB_width = radiation_pattern.v_3dB_width;
h_3dB_width = radiation_pattern.h_3dB_width;


%% Transform ray vectors to local coordinate frame

% consider downtilt angle (rotate along y axis)

rotMat = getRotationMatrixFromVector(quat);

ray_vectors_local_frame = rotMat' * ray_vectors_ref_frame';
ray_vectors_local_frame = ray_vectors_local_frame';

x_ref = ray_vectors_local_frame(:,1);
y_ref = ray_vectors_local_frame(:,2);
z_ref = ray_vectors_local_frame(:,3);


%% Antenna gain 

AOD = atan2(z_ref, x_ref);
ZOD = atan2(z_ref, y_ref);

A_v = -12 * ((AOD * 180/pi - 90) / h_3dB_width).^2;
A_v(A_v < - v_SLA) = -v_SLA;

A_h = -12 * ((ZOD * 180/pi - 90) / v_3dB_width).^2;
A_h(A_h < - h_SLA) = -h_SLA;

A_total = A_v + A_h;
A_total(A_total < -h_SLA) = -h_SLA;
A_total = A_total + antenna_gain;


%% Steering vector

antenna_loc = (0:h_spacing:h_spacing*(h_n-1))' + 1i*(0:v_spacing:v_spacing*(v_n-1));
antenna_loc = antenna_loc(:) - antenna_loc(1);

antenna_loc = [real(antenna_loc), imag(antenna_loc), zeros(length(antenna_loc), 1)];

n_antenna_elements = size(antenna_loc, 1);
n_rays = size(ray_vectors_local_frame, 1);

steering_vec = zeros(n_antenna_elements, n_rays);

for k=1:n_rays
    phase_diff = 2*pi * antenna_loc * transpose(ray_vectors_local_frame(k,:));
    steering_vec(:, k) = exp(-1i*phase_diff);
end

% for k=1:n_antenna_elements
%     phase_diff = ray_vectors_local_frame(:,1) .* (ones(n_rays, 1) * real(antenna_loc(k)));
%     phase_diff = phase_diff + ray_vectors_local_frame(:,2) .* (ones(n_rays, 1) * imag(antenna_loc(k)));
%     steering_vec(k, :) = exp(1i*2*pi*phase_diff);
% end

%% End