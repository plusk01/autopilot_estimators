%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROSflight Estimator Comparison
%
%
% Parker Lusk
% 21 May 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc;
set(0,'DefaultLineLineWidth',1); % 0.5
exfig = 0;

% -------------------------------------------------------------------------
% Setup

% Load flight data
[imu, state, pose, frame] = readACLBag('HX02', 'bags/acl/imu_hx02.bag');
% [imu, state, pose, frame] = readMiscBag('bags/rosflight_bias_in_estimator_with_ground_truth.bag');
% [imu, state, pose, frame] = readRFBag('bags/acl/unity.bag');
% [imu, state, pose, frame] = readRFBag('bags/rrg/rosflight_noNaN_aggressive.bag', 1);

% timing - use IMU as clock
ts = 0;
te = Inf;
Ns = find(imu.t>=ts,1);
Ne = find(imu.t>=te,1);
if isempty(Ne), Ne = length(imu.t); end
tvec = imu.t(Ns:Ne);

% start and end indices for pose data
pNs = find(pose.t>=ts,1);
pNe = max([find(pose.t>=te,1) length(pose.t)]);

% start and end indices for state data
sNs = find(state.t>=ts,1);
sNe = max([find(state.t>=te,1) length(state.t)]);

figure(1), clf;
subplot(211);
title('Raw IMU Data');
plot(imu.t(Ns:Ne), imu.gyro(:,Ns:Ne));
legend('X','Y','Z');
grid on; ylabel('Gyro [rad/s]');
subplot(212);
plot(imu.t(Ns:Ne), imu.accel(:,Ns:Ne));
grid on; ylabel('Accel [m/s/s]'); xlabel('Time [s]');
% if exfig
%     set(gcf, 'Color', 'w');
%     export_fig('doc/figures/rawimu.pdf','-dCompatibilityLevel=1.5');
% end

% -------------------------------------------------------------------------
%% Filter Setup

kp = 0.5; ki = 0.01; margin = 0.1; acc_LPF_alpha = 0.8;
useExtAtt = 1; extAttRate = 100; extAttDropAfter = Inf;
useAcc = 1;

% ratio of imu sample rate to extAtt sample rate
freqRat = (length(tvec)/max(tvec))/extAttRate;

%
% Mahony Filter
%

filter = MahonyAHRS(kp, ki, 1, margin, acc_LPF_alpha);
filter.frame = frame;

% initialize with quat from vicon
filter = filter.setQ(pose.quaternion(:,pNs));
% filter = filter.setQ(Q.fromRPY(0.0,0.0,0.9).q);

qmahony = zeros(4,length(tvec));
qmahony(:,1) = filter.getQ();

halfe = zeros(3,length(tvec));
mahonyintegral = zeros(3,length(tvec));

%
% ROSflight
%

rf = rosflight.ROSflight();
rf.frame = frame;
rf.setParam('FILTER_USE_ACC', useAcc);
rf.setParam('FILTER_ACCMARGIN', margin);
rf.setParam('FILTER_QUAD_INT', 0);
rf.setParam('FILTER_MAT_EXP', 0);
rf.setParam('FILTER_KP_ACC', kp);
rf.setParam('FILTER_KP_EXT', 1.5);
rf.setParam('FILTER_KI', ki);
rf.setParam('ACC_LPF_ALPHA', acc_LPF_alpha);
rf.setParam('GYROXY_LPF_ALPHA', 0.0);
rf.setParam('GYROZ_LPF_ALPHA', 0.0);
rf.setParam('GYRO_X_BIAS', 0);
rf.setParam('GYRO_Y_BIAS', 0);
rf.setParam('GYRO_Z_BIAS', 0);
rf.setTime(tvec(1));
rf.run();

rfstate.q = zeros(4,length(tvec)); rfstate.q(1,1) = 1;
rfstate.RPY = zeros(3,length(tvec));
rfstate.omega = zeros(3,length(tvec));
rfstate.t = zeros(1,length(tvec)); rfstate.t(1) = tvec(1);

%
% ROSflight (MATLAB implementation of estimator)
%

rfestimator = RFEstimator(rf);
rfestimator.frame = frame;

% initialize with quat from vicon
% rfestimator = rfestimator.setQ(pose.quaternion(:,pNs));
% rfestimator = rfestimator.setQ(Q.fromRPY(0.0,0.0,0).q);

qrfe = zeros(4,length(tvec));
qrfe(:,1) = rfestimator.getQ();

wacc = zeros(3,length(tvec));
walt = zeros(3,length(tvec));
rfbias = zeros(3,length(tvec));

% -------------------------------------------------------------------------
% Main Loop

% estimator period over time
dts = zeros(1,length(tvec)-1);

% vis handle for attitude
viz = 0;
if viz == 1
    figure(10), clf; hold on;
    hViz = viz(Q.Identity, Q.Identity, Q.Identity, zeros(3,1), margin, frame, []);
end

extAttFlags = zeros(1,length(tvec));

for i = 2:length(tvec)
    t = tvec(i);
    dt = t - tvec(i-1);
    dts(i-1) = dt;
    
    % extra indexing because we are using a window of the IMU data
    j = Ns+i-1;
    
    % find the time-sync'd index of the pose data
    pidx = find(pose.t>=t,1);
    if isempty(pidx), pidx = 0; end
    
    % simulate external attitude drop out
    if t>=extAttDropAfter, pidx = 0; end
    
    % Check if external attitude is yawed
%     if pidx ~=0
%         RPY = quat2eul(pose.quaternion(:,pidx)', 'XYZ');
%         if rad2deg(RPY(3))>10 || rad2deg(RPY(3))<-10
%             pidx = 0;
%         end
%     end
    
    %
    % MATLAB Implementation of Mahony Filter
    %
    
    filter = filter.updateIMU(imu.gyro(:,j), imu.accel(:,j), dt);
    qmahony(:,i) = filter.getQ();
    halfe(:,i) = filter.halfe;
    mahonyintegral(:,i) = [filter.integralFBx;filter.integralFBy;filter.integralFBz];
    
    %
    % ROSflight
    %
    
    extAttFlag = mod(i,round(freqRat)) == 0;
    if extAttRate == Inf, extAttFlag = 1; end
    
    if pidx ~= 0 && useExtAtt == 1 && extAttFlag == 1
        extAttq = pose.quaternion(:,pidx);
        rf.extAttCorrection(extAttq);
    end
    
    rf.setIMU(imu.gyro(:,j), imu.accel(:,j));
    rf.setTime(t);
    rf.run(); rf.run();
    [q, rpy, omega, ~] = rf.getState();
    rfstate.q(:,i) = q;
    rfstate.RPY(:,i) = rpy;
    rfstate.omega(:,i) = omega;
    rfstate.t(i) = t;
    
    %
    % ROSflight MATLAB estimator
    %
    
    extAttFlag = mod(i,round(freqRat)) == 0;
    if extAttRate == Inf, extAttFlag = 1; end
    
    if pidx ~= 0 && useExtAtt == 1 && extAttFlag == 1
        extAttFlags(i) = 1;
        extAttq = pose.quaternion(:,pidx);
        rfestimator = rfestimator.extAttCorrection(extAttq, t);
    end
    
    rfestimator = rfestimator.updateIMU(imu.gyro(:,j), imu.accel(:,j), dt);
    qrfe(:,i) = rfestimator.getQ();
    wacc(:,i) = [1 0 0; 0 -1 0; 0 0 -1]*rfestimator.wacc;
    walt(:,i) = [1 0 0; 0 -1 0; 0 0 -1]*rfestimator.wacc_alt;
    rfbias(:,i) = [1 0 0; 0 -1 0; 0 0 -1]*(-1)*rfestimator.bias;
    
    
    % Visualize attitude and accelerometer measurement
    if viz == 1 && pidx ~= 0 && mod(i,1000) == 0
        hViz = viz(qrfe(:,i), qmahony(:,i), pose.quaternion(:,pidx), ...
                        rfestimator.accel_LPF, margin, frame, hViz);
        drawnow;
    end
end

% -------------------------------------------------------------------------

figure(2), clf;
subplot(411); grid on; ylabel('qw'); hold on; legend;
plot(pose.t(pNs:pNe),pose.quaternion(1,pNs:pNe),'DisplayName','VICON');
% plot(state.t(sNs:sNe),state.quat(1,sNs:sNe),'DisplayName','Onboard');
% plot(tvec,qmahony(1,:),'DisplayName','Mahony');
plot(rfstate.t, rfstate.q(1,:),'DisplayName','ROSflight');
plot(tvec, qrfe(1,:),'DisplayName','ROSflight MATLAB');
subplot(412); grid on; ylabel('qx'); hold on;
plot(pose.t(pNs:pNe),pose.quaternion(2,pNs:pNe));
% plot(state.t(sNs:sNe),state.quat(2,sNs:sNe));
% plot(tvec,qmahony(2,:));
plot(rfstate.t, rfstate.q(2,:));
plot(tvec, qrfe(2,:));
subplot(413); grid on; ylabel('qy'); hold on;
plot(pose.t(pNs:pNe),pose.quaternion(3,pNs:pNe));
% plot(state.t(sNs:sNe),state.quat(3,sNs:sNe));
% plot(tvec,qmahony(3,:))
plot(rfstate.t, rfstate.q(3,:));
plot(tvec, qrfe(3,:));
subplot(414); grid on; ylabel('qz'); hold on;
plot(pose.t(pNs:pNe),pose.quaternion(4,pNs:pNe));
% plot(state.t(sNs:sNe),state.quat(4,sNs:sNe));
% plot(tvec,qmahony(4,:));
plot(rfstate.t, rfstate.q(4,:));
plot(tvec, qrfe(4,:));
xlabel('Time [s]');

% convert quat to RPY
seq = 'XYZ';
mahonyRPY = quat2eul(qmahony', seq)*180/pi;
viconRPY = quat2eul(pose.quaternion(:,pNs:pNe)', seq)*180/pi;
stateRPY = quat2eul(state.quat(:,sNs:sNe)', seq)*180/pi;
rfRPY = quat2eul(rfstate.q', seq)*180/pi;
rfeRPY = quat2eul(qrfe', seq)*180/pi;

figure(3), clf;
subplot(311); grid on; ylabel('R'); hold on; legend;
plot(pose.t(pNs:pNe),viconRPY(:,1),'DisplayName','VICON');
% plot(state.t(sNs:sNe),stateRPY(:,1),'DisplayName','Onboard');
% plot(tvec,mahonyRPY(:,1),'DisplayName','Mahony');
plot(rfstate.t,rfRPY(:,1),'DisplayName','ROSflight');
% plot(rfstate.t,rfstate.RPY(1,:)*180/pi,'DisplayName','ROSflight (internal)');
plot(tvec,rfeRPY(:,1),'DisplayName','ROSflight MATLAB');
subplot(312); grid on; ylabel('P'); hold on;
plot(pose.t(pNs:pNe),viconRPY(:,2));
% plot(state.t(sNs:sNe),stateRPY(:,2));
% plot(tvec,mahonyRPY(:,2));
plot(rfstate.t,rfRPY(:,2));
% plot(rfstate.t,rfstate.RPY(2,:)*180/pi);
plot(tvec,rfeRPY(:,2));
subplot(313); grid on; ylabel('Y'); hold on;
plot(pose.t(pNs:pNe),viconRPY(:,3));
% plot(state.t(sNs:sNe),stateRPY(:,3));
% plot(tvec,mahonyRPY(:,3));
plot(rfstate.t,rfRPY(:,3));
% plot(rfstate.t,rfstate.RPY(3,:)*180/pi);
plot(tvec,rfeRPY(:,3));
xlabel('Time [s]');
if exfig
    set(gcf, 'Color', 'w');
    export_fig('doc/figures/estrpy.pdf','-dCompatibilityLevel=1.5');
end

I = find(extAttFlags==1);
scatter(tvec(I),repmat(-7,length(I),1),2)

figure(4), clf;
subplot(211); grid on; ylabel('Mahony [Accel Only]'); hold on;
plot(tvec, 2*halfe); legend('X','Y','Z')
subplot(212); grid on; ylabel('ROSflight'); hold on;
plot(tvec, wacc);
% subplot(313); grid on; ylabel('walt'); hold on;
% plot(tvec, walt);
xlabel('time [s]');
if exfig
    set(gcf, 'Color', 'w');
    export_fig('doc/figures/werr.pdf','-dCompatibilityLevel=1.5');
end

figure(5), clf;
subplot(211); grid on; ylabel('Mahony [Accel Only]'); hold on;
plot(tvec, mahonyintegral); legend('X','Y','Z'); %title('Integral Feedback (Bias)');
xyaxis = axis;
subplot(212); grid on; ylabel('ROSflight'); hold on;
plot(tvec, rfbias);
axis(xyaxis);
xlabel('time [s]');
if exfig
    set(gcf, 'Color', 'w');
    export_fig('doc/figures/estbias.pdf','-dCompatibilityLevel=1.5');
end

function v = clamp(v,u)
    if v>u, v=u; end
    if v<-u,v=-u; end
end