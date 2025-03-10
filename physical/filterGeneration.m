% T = 0.01;
% fs = 1 / T;
% fcut = 10;
% fc = fcut / fs;

N = 41; % Filter length
% Generate a Blackman window of size N
b = 0.42 - 0.5 * cos(2 * pi * (0:N-1) / (N-1)) + 0.08 * cos(4 * pi * (0:N-1) / (N-1));
% Normalize the window so the sum of all elements is 1
b = b / sum(b);

freqz(b, 1, 1024); % Plot frequency response
save('FIR_coefficients.mat', 'b');