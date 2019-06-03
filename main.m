%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROSflight Estimator Comparison
%
%
% Parker Lusk
% 21 May 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc;

% -------------------------------------------------------------------------
% Setup

% Load flight data
[imu, state, pose, frame] = readACLBag('HX05', 'bags/acl/imu.bag');
% [imu, state, pose, frame] = readRFBag('bags/acl/unity_e1xtatt.bag');
% [imu, state, pose, frame] = readRFBag('bags/rrg/rosflight_some_yaw.bag', 1);

% generate IMU
% keepYaw = 1;
% imu = genIMU(imu, pose, keepYaw);

% timing - use IMU as clock
ts = 0;
te = Inf;
Ns = find(imu.t>=ts,1);
Ne = max([find(imu.t>=te,1) length(imu.t)]);
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

% subplot(313);
% plot(imu.t(Ns:Ne), vecnorm(imu.accel(:,Ns:Ne),2,1));
% grid on; ylabel('Accel [m/s/s]'); xlabel('Time [s]');

% -------------------------------------------------------------------------
% Filter Setup

%
% Mahony Filter
%
kp = 0.5; ki = 0.1; margin = 0.2; acc_LPF_alpha = 0.8;
filter = MahonyAHRS(kp, ki, 1, margin, acc_LPF_alpha);
filter.frame = frame;

% initialize with quat from vicon
filter = filter.setQ(pose.quaternion(:,pNs));

qmahony = zeros(4,length(tvec));
qmahony(:,1) = filter.getQ();

halfe = zeros(3,length(tvec));
mahonyintegral = zeros(3,length(tvec));

%
% ROSflight
%

rf = rosflight.ROSflight();
rf.frame = frame;
rf.setParam('FILTER_USE_ACC', 1);
rf.setParam('FILTER_QUAD_INT', 0);
rf.setParam('FILTER_MAT_EXP', 0);
rf.setParam('FILTER_KP', kp);
rf.setParam('FILTER_KI', ki);
rf.setParam('ACC_LPF_ALPHA', acc_LPF_alpha);
rf.setParam('GYROXY_LPF_ALPHA', 0.0);
rf.setParam('GYROZ_LPF_ALPHA', 0.0);
rf.setParam('GYRO_X_BIAS', 0);
rf.setParam('GYRO_Y_BIAS', 0);
rf.setParam('GYRO_Z_BIAS', 0);
rf.setParam('FILTER_KP_COR', 10);
rf.setTime(tvec(1));
rf.run();

rfstate.q = zeros(4,length(tvec)); rfstate.q(1,1) = 1;
rfstate.RPY = zeros(3,length(tvec));
rfstate.omega = zeros(3,length(tvec));
rfstate.t = zeros(1,length(tvec)); rfstate.t(1) = tvec(1);

%
% ROSflight (MATLAB implementation of estimator)
%

rfestimator = RFEstimator(rf, margin);
rfestimator.frame = frame;

% initialize with quat from vicon
% rfestimator = rfestimator.setQ(pose.quaternion(:,pNs));

qrfe = zeros(4,length(tvec));
qrfe(:,1) = rfestimator.getQ();

wacc = zeros(3,length(tvec));
walt = zeros(3,length(tvec));
rfbias = zeros(3,length(tvec));

% -------------------------------------------------------------------------
% Main Loop

% estimator period over time
dts = zeros(1,length(tvec)-1);

for i = 2:length(tvec)
    t = tvec(i);
    dt = t - tvec(i-1);
    dts(i-1) = dt;
    
    % extra indexing because we are using a window of the IMU data
    j = Ns+i-1;
    
    % find the time-sync'd index of the pose data
    pidx = find(pose.t>=t,1);
%     if abs(pose.t(pidx)-t)>0.001, pidx = 0; end % within 1 ms tolerance
%     pidx = 0;
    
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
    
    if pidx ~= 0
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
    
    if pidx ~= 0
        extAttq = pose.quaternion(:,pidx);
        rfestimator = rfestimator.extAttCorrection(extAttq);
    end
    
    rfestimator = rfestimator.updateIMU(imu.gyro(:,j), imu.accel(:,j), dt);
    qrfe(:,i) = rfestimator.getQ();
    wacc(:,i) = [1 0 0; 0 -1 0; 0 0 -1]*rfestimator.wacc;
    walt(:,i) = [1 0 0; 0 -1 0; 0 0 -1]*rfestimator.wacc_alt;
    rfbias(:,i) = [1 0 0; 0 -1 0; 0 0 -1]*(-1)*rfestimator.bias;
    
end

% -------------------------------------------------------------------------

figure(2), clf;
subplot(411); grid on; ylabel('qw'); hold on; legend;
plot(pose.t(pNs:pNe),pose.quaternion(1,pNs:pNe),'DisplayName','VICON');
plot(state.t(sNs:sNe),state.quat(1,sNs:sNe),'DisplayName','Onboard');
plot(tvec,qmahony(1,:),'DisplayName','Mahony');
% plot(rfstate.t, rfstate.q(1,:),'DisplayName','ROSflight');
plot(tvec, qrfe(1,:),'DisplayName','ROSflight MATLAB');
subplot(412); grid on; ylabel('qx'); hold on;
plot(pose.t(pNs:pNe),pose.quaternion(2,pNs:pNe));
plot(state.t(sNs:sNe),state.quat(2,sNs:sNe));
plot(tvec,qmahony(2,:));
% plot(rfstate.t, rfstate.q(2,:));
plot(tvec, qrfe(2,:));
subplot(413); grid on; ylabel('qy'); hold on;
plot(pose.t(pNs:pNe),pose.quaternion(3,pNs:pNe));
plot(state.t(sNs:sNe),state.quat(3,sNs:sNe));
plot(tvec,qmahony(3,:))
% plot(rfstate.t, rfstate.q(3,:));
plot(tvec, qrfe(3,:));
subplot(414); grid on; ylabel('qz'); hold on;
plot(pose.t(pNs:pNe),pose.quaternion(4,pNs:pNe));
plot(state.t(sNs:sNe),state.quat(4,sNs:sNe));
plot(tvec,qmahony(4,:));
% plot(rfstate.t, rfstate.q(4,:));
plot(tvec, qrfe(4,:));
xlabel('Time [s]');

% convert quat to RPY
mahonyRPY = quat2eul(qmahony', 'XYZ')*180/pi;
viconRPY = quat2eul(pose.quaternion(:,pNs:pNe)', 'XYZ')*180/pi;
stateRPY = quat2eul(state.quat(:,sNs:sNe)', 'XYZ')*180/pi;
rfRPY = quat2eul(rfstate.q', 'XYZ')*180/pi;
rfeRPY = quat2eul(qrfe', 'XYZ')*180/pi;

figure(3), clf;
subplot(311); grid on; ylabel('R'); hold on; legend;
plot(pose.t(pNs:pNe),viconRPY(:,1),'DisplayName','VICON');
plot(state.t(sNs:sNe),stateRPY(:,1),'DisplayName','Onboard');
plot(tvec,mahonyRPY(:,1),'DisplayName','Mahony');
plot(rfstate.t,rfRPY(:,1),'DisplayName','ROSflight');
% plot(rfstate.t,rfstate.RPY(1,:)*180/pi,'DisplayName','ROSflight (internal)');
plot(tvec,rfeRPY(:,1),'DisplayName','ROSflight MATLAB');
subplot(312); grid on; ylabel('P'); hold on;
plot(pose.t(pNs:pNe),viconRPY(:,2));
plot(state.t(sNs:sNe),stateRPY(:,2));
plot(tvec,mahonyRPY(:,2));
plot(rfstate.t,rfRPY(:,2));
% plot(rfstate.t,rfstate.RPY(2,:)*180/pi);
plot(tvec,rfeRPY(:,2));
subplot(313); grid on; ylabel('Y'); hold on;
plot(pose.t(pNs:pNe),viconRPY(:,3));
plot(state.t(sNs:sNe),stateRPY(:,3));
plot(tvec,mahonyRPY(:,3));
plot(rfstate.t,rfRPY(:,3));
% plot(rfstate.t,rfstate.RPY(3,:)*180/pi);
plot(tvec,rfeRPY(:,3));
xlabel('Time [s]');

% figure(4), clf;
% subplot(311); grid on; ylabel('halfe'); hold on;
% plot(tvec, halfe); legend('X','Y','Z')
% subplot(312); grid on; ylabel('wacc'); hold on;
% plot(tvec, wacc);
% subplot(313); grid on; ylabel('walt'); hold on;
% plot(tvec, walt);
% 
% figure(5), clf;
% subplot(211); grid on; ylabel('MahonyAHRS'); hold on;
% plot(tvec, mahonyintegral); legend('X','Y','Z'); title('Integral Feedback');
% subplot(212); grid on; ylabel('ROSflight'); hold on;
% plot(tvec, rfbias);