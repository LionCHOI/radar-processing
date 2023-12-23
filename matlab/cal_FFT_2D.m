function RDM = cal_FFT_2D(Mix, Nr, Nd)

%% Range doppler response
sig_fft2 = fft2(Mix, Nr, Nd);                       

sig_fft2 = sig_fft2(1:Nr/2, 1:Nd);                  % remove other part of distance  
sig_fft2 = fftshift (sig_fft2);                     % shift

RDM = abs(sig_fft2);
RDM = 10*log10(RDM);                                % 로그 스케일


doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400); % scale을 보정하기


%% Range doppler response CDFT --> 노이즈를 제거하는 부분인데 잘 모르겠다.

Tr = 8;
Td = 6;

Gr = 4;
Gd = 4;

offset = 1.4;

noise_level = zeros(1,1);

num_cells = 2*(Td+Gd+1)*2*(Tr+Gr+1)- (Gr*Gd-1);

RDM = RDM/max(max(RDM));

    for i = (Tr + Gr + 1):((Nr / 2) - (Gr + Tr))
        for j = (Td + Gd + 1):(Nd - (Gd + Td))
            for p = (i - (Tr + Gr)):(i + (Tr + Gr))
                for q = (j - (Td + Gd)):(j + (Td + Gd))
                    if (abs(i - p) > Gr || abs(j - q) > Gd)
                        noise_level = noise_level + db2pow(RDM(p,q));
                    end
                end
            end
       
        threshold = pow2db(noise_level/num_cells);

        threshold = threshold + offset;

        CUT = RDM(i,j);
        if (CUT > threshold)
            RDM(i,j) = 1;
        else
            RDM(i,j) = 0;
        end
            
        noise_level = zeros(1,1);
        end
    end

RDM(RDM~=0 & RDM~=1) = 0; 

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.

figure('Name', 'Range Doppler map');
contour(doppler_axis,range_axis,RDM,1);
colorbar;
xlabel('doppler')
ylabel('range ')
grid on;
grid minor;
xlim([-100, 100]);
ylim([-200, 200]);
end
