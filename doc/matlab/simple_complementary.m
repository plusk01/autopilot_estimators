%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Linear Complementary Filtering Example
%
%   - no dynamics: \hat{x}(s) = x(s) + F1(s)\mu1(s) + F2(s)\mu2(s)
%   - known frequency content of disturbance (i.e., high or low)
%
% c.f., Appendix A Mahony et al. 2008
%
% Parker Lusk
% 20 June 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;

% should export figure
exfig = 1;

% timing
Ts = 0.001;
tend = 3;
t = 0:Ts:(tend-Ts);

% true signal
fx = 5; % [Hz]
x = cos(2*pi*fx*t);

% noise sources
fmu1 = 20*fx;
mu1 = 0.5*sin(2*pi*fmu1*t);% + 2*cos(2*pi*2*fmu1*t);
fmu2 = 0.05*fx;
mu2 = 2*sin(2*pi*fmu2*t);% + 0.3*cos(2*pi*0.9*fmu2*t);

% measurements
y1 = x + mu1;
y2 = x + mu2;

% estimates
fc1 = 1; fc2 = 10;
[xhat1, F1_1, F2_1] = estimate(t,y1,y2,fc1);
[xhat2, F1_2, F2_2] = estimate(t,y1,y2,fc2);

figure(1), clf;
subplot(211); hold on; grid on;
% title('Measurements of x(t)');
plot(t,y1); xlabel('t [s]');
ylabel('y_1(t)');
subplot(212);
plot(t,y2); hold on; grid on;
ylabel('y_2(t)'); xlabel('t [s]');
if exfig
    set(gcf, 'Color', 'w');
    export_fig('../figures/scf_meas.pdf','-dCompatibilityLevel=1.5');
end

figure(2), clf;
subplot(211); hold on; grid on;
% title('Estimate');
plot(t,xhat1,'LineWidth',1.5);
plot(t,x,'k--','LineWidth',1);
ylabel(['xhat (f_c = ' num2str(fc1) ')']); xlabel('t [s]');
subplot(212); hold on; grid on;
plot(t,xhat2,'LineWidth',1.5);
plot(t,x,'k--','LineWidth',1);
ylabel(['xhat (f_c = ' num2str(fc2) ')']); xlabel('t [s]');
if exfig
    set(gcf, 'Color', 'w');
    export_fig('../figures/scf_est.pdf','-dCompatibilityLevel=1.5');
end

figure(3), clf;
subplot(211);
h = bodeplot(F1_1,F2_1,{0.5,500});
title('');
p = getoptions(h);
p.FreqUnits = 'Hz';
p.PhaseVisible = 'off';
setoptions(h,p);
hold on; grid on;
ax = findobj(gcf,'type','axes'); magax = ax(2);
xyaxis = axis(magax);
plot(magax,[fx fx], xyaxis(3:4), 'k','LineWidth',2);
subplot(212);
h = bodeplot(F1_2,F2_2,{0.5,500});
title('');
p = getoptions(h);
p.FreqUnits = 'Hz';
p.PhaseVisible = 'off';
setoptions(h,p);
hold on; grid on;
ax = findobj(gcf,'type','axes'); magax = ax(2);
xyaxis = axis(magax);
plot(magax,[fx fx], xyaxis(3:4), 'k','LineWidth',2);
if exfig
    set(gcf, 'Color', 'w');
    export_fig('../figures/scf_bode.pdf','-dCompatibilityLevel=1.5');
end

function [xhat, F1, F2] = estimate(t, y1, y2, fc)
% build filters
wc = 2*pi*fc;
tau = 1/wc;
F1 = tf(1,[tau 1]); % lpf
F2 = tf([tau 0],[tau 1]); % hpf

% estimate signal
xhat1 = lsim(F1,y1,t);
xhat2 = lsim(F2,y2,t);
xhat = xhat1 + xhat2;
end