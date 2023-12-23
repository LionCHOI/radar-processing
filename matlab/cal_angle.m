function angle = cal_angle(FFT_2D)

%% make a steering vector
f_c = 77e9;
c = 3e8;
lambda = c / f_c; 

% steervec = zeros(1, 10);

%total = size_res(1)*size_res(2);
result = zeros(201,1);

for ang=-100:1:100
    index = ang + 101;
    for i=1:10
        d = 1.8*lambda;
        steervec = exp(1j*2*pi/lambda*(i-1)*d*sin(ang));
        for j=1:1024
            for k=1:256
                result(index) = result(index) + conj(steervec)*FFT_2D(j,k);
            end
        end
    end
end 
angle = max(result);

sv_imag = zeros(201,1);
sv_real = zeros(201,1);

figure('Name', 'angle')
for i=1:201
    sv_real(i) = real(result(i));
    sv_imag(i) = imag(result(i));
    result(i) = atan(sv_imag(i)/sv_real(i));
end
x_angle = -100:1:100;
plot(x_angle , result)
%% 결과를 어떻게 출력해야 할지 모르겠다. --> 차원이 너무 높다.
end

%{
elementPos = (0:lambda:10*lambda);  % space distances of antena [m]
c = physconst('LightSpeed');        % The Light Speed (3e8)
fc = 1e9;                           % received wave frequency [Hz]
lam = c/fc;                         % the wavelength [m]
ang = [0;0];                       % 45º azimuth and 0º elevation
sv = steervec(elementPos/lam, ang);

res_angle = Mix .* conj(sv);


sv_real = real(sv);
sv_imag = imag(sv);
angle = atan(sv_imag/sv_real);

%}