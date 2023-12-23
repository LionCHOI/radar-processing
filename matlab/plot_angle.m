function plot_angle(r, angle, wave_param, fft_value)

scale = 4;

theta = angle;
x = r.*cos(theta/180*pi);
y = r.*sin(theta/180*pi);
result = reshape(fft_value(:,1,:), 256, []);
figure('Name','angle-doppler map')
surf(x,y,abs(result));
shading interp % shading flat --> 점들의 집합이라서 검게 표현된다. --> interpolation!
colorbar;

xlabel('x')
ylabel('y')
zlabel('z')

xlim([-wave_param.range_max/scale,wave_param.range_max/scale])
ylim([0,wave_param.range_max/scale])
colorbar;
view(0,90)
grid minor;
end