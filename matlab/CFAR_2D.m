function CFAR_2D(FFT_2D, wave_param, velocity, range)

Tr = 2;
Td = 1;
Gr = 2; 
Gd = 1;
offset = 3.3;

noise_level = zeros(1,1);

dB_Doppler_fft = 10*log10(FFT_2D);
dB_Doppler_fft = dB_Doppler_fft/max(max(dB_Doppler_fft));
 
for i = Tr+Gr+1 : wave_param.range_fft_size/2-(Gr+Tr) % over range
    for j = Td+Gd+1 : wave_param.doppler_fft_size-(Gd+Td) % over doppler
        noise_level = zeros(1,1);
        for p = i-(Tr+Gr): i+ (Tr+Gr)
            for q = j-(Td+Gd): j+(Td+Gd)
                if (abs(i-p)> Gr ||abs(j-q)>Gd)
                    noise_level = noise_level+ db2pow(dB_Doppler_fft(p,q));
                end
            end

        end
        threshold = pow2db(noise_level/(2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1));
        threshold = threshold + offset;

        CUT= dB_Doppler_fft(i,j);
        if (CUT<threshold)
            dB_Doppler_fft(i,j)=0;
        else 
            dB_Doppler_fft(i,j)= 1; % max_T
%             disp(i);
%             disp(j);
        end

        
    end
end

dB_Doppler_fft(dB_Doppler_fft~=0 & dB_Doppler_fft~=1) = 0;

figure('Name', 'CFAR Range-Doppler map')
surf(velocity,range, dB_Doppler_fft);
shading interp % shading flat --> 점들의 집합이라서 검게 표현된다. --> interpolation!
colorbar;
xlim([-wave_param.v_max,wave_param.v_max])
ylim([0,wave_param.range_max])
view(0,90)
ylabel('range [m]');
xlabel('velocity [m/s]');

end