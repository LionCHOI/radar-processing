p=100;% number of Snapshots
fs=10^7; %sampling Frequency
fc=10^6 ;%center Frequency of narrow band sources
M=10;%Number of array elements
N=5; %Number of sources
s_var=1; %variance of the amplitude of the sources
s=sqrt(1)*randn(N,p).*exp(j*(2*pi*fc*repmat([1:p]/fs,N,1)));

doa=[20;50;85;110;145]; % DOAs
c_speed=3*10^8 ; % speed of light
dist=150; %antenna spacing
A=zeros(M,N); %To create a matrix with M row and N column
for k=1:N
 A(:,k)=exp(-j*2*pi*fc*dist*cosd(doa(k))*(1/c_speed)*[0:M-1]'); %NOTE:
end
noisecoeff=1; %variance of added noise
x=A*s+sqrt(noisecoeff)*randn(M,p);

R=(x*x')/p; %Empirical covariance of the antenna data 

[doas,spec,specang] = musicdoa(R, N);
figure("Name","Music")
plot(specang+90,10*log10(spec))
grid 