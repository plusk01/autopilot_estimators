function [imu, state, pose, frame] = readRFBag(bagpath, varargin)
%READBAG Extracts messages into matrices without using custom msg defn

% Data is in FRD
frame = 'FRD';

% RRG unity sim uses `/true_pose` as PoseStampted ground truth.
% rosflight_unity sim uses `/rosflight_unity/truth`.
if nargin == 2, isRRG = varargin{1}; else, isRRG = false; end

bag = rosbag(bagpath);

% Get IMU messages
bagsel = select(bag, 'Topic', '/imu/data');
msgs = readMessages(bagsel,'DataFormat','struct'); imuMsg = [msgs{:}];
tsec = header([imuMsg.Header]);
tmp = [imuMsg.AngularVelocity]; gX = [tmp.X]; gY = [tmp.Y]; gZ = [tmp.Z];
gyro = [gX' gY' gZ']';
tmp = [imuMsg.LinearAcceleration]; aX = [tmp.X]; aY = [tmp.Y]; aZ = [tmp.Z];
accel = [aX' aY' aZ']';
imu = struct('t',tsec,'gyro',gyro,'accel',accel);

% Get pose messages
if isRRG, truthTopic = '/true_pose';
else, truthTopic = '/rosflight_unity/truth'; end
bagsel = select(bag, 'Topic', truthTopic);
msgs = readMessages(bagsel,'DataFormat','struct');
poseStampedMsg = [msgs{:}]; poseMsg = [poseStampedMsg.Pose];
tsec = header([poseStampedMsg.Header]);
tmp = [poseMsg.Position]; pX = [tmp.X]; pY = [tmp.Y]; pZ = [tmp.Z];
pos = [pX' pY' pZ']';
tmp = [poseMsg.Orientation]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
quat = [qW' qX' qY' qZ']';
pose = struct('t',tsec,'position',pos,'quaternion',quat);

% Get state messages
bagsel = select(bag, 'Topic', '/attitude');
msgs = readMessages(bagsel,'DataFormat','struct'); stateMsg = [msgs{:}];
tsec = header([stateMsg.Header]);
tmp = [stateMsg.Attitude]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
quat = [qW' qX' qY' qZ']';
tmp = [stateMsg.AngularVelocity]; wX = [tmp.X]; wY = [tmp.Y]; wZ = [tmp.Z];
omega = [wX' wY' wZ']';
state = struct('t',tsec,'quat',quat,'omega',omega);

% time sync
mint = min([min(state.t), min(pose.t), min(imu.t)]);
state.t = state.t - mint;
pose.t = pose.t - mint;
imu.t = imu.t - mint;

end

function t = header(headerMsg)
stamp = [headerMsg.Stamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
end

function t = statestamp(stateMsg)
stamp = [stateMsg.StateStamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
end