x = imu.gyro(1,:);
% x = pose.quaternion(2,:);
% x = imu.accel(3,:);
fs = 200;

y = fft(x);

n = length(x);
f = (0:n-1)*(fs/n);
power = abs(y).^2/n;

y0 = fftshift(y);
f0 = (-n/2:n/2-1)*(fs/n);
power0 = abs(y0).^2/n;

figure(10), clf;
plot(f0,power0);
xlabel('Frequency [Hz]'); ylabel('Power');

% https://www.analog.com/media/en/analog-dialogue/volume-46/number-3/articles/analyzing-frequency-response-of-inertial-mems.pdf