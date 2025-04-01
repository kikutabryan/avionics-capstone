%% Low Pass Filter for Position
% Filter specs
Fs = 500;
Fc = 10;
N = 100;

Wn = Fc / (Fs / 2);

b = fir1(N, Wn);
fvtool(b, 1, 'Fs', Fs);

save('posFilterCoeffs.mat', 'b');