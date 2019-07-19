function [imu, state, pose, frame] = readRFBag(bagpath)
%READBAG Extracts messages into matrices without using custom msg defn

% Data is in FRD
frame = 'FRD';

bag = rosbag(bagpath);

% Get IMU messages
bagsel = select(bag, 'Topic', '/imu/data'); %'interpolate_imu/imu'
msgs = readMessages(bagsel,'DataFormat','struct'); imuMsg = [msgs{:}];
[tsec, seq] = header([imuMsg.Header]);
tmp = [imuMsg.AngularVelocity]; gX = [tmp.X]; gY = [tmp.Y]; gZ = [tmp.Z];
gyro = [gX' gY' gZ']';
tmp = [imuMsg.LinearAcceleration]; aX = [tmp.X]; aY = [tmp.Y]; aZ = [tmp.Z];
accel = [aX' aY' aZ']';
imu = struct('t',tsec,'seq',seq,'gyro',gyro,'accel',accel);

% Get pose messages
bagsel = select(bag, 'Topic', '/rosflight_unity/truth');
bagsel = select(bag, 'Topic', '/multirotor/truth/NED');
msgs = readMessages(bagsel,'DataFormat','struct');
if length(msgs)>0
    poseStampedMsg = [msgs{:}]; poseMsg = [poseStampedMsg.Pose];
    poseMsg = [poseMsg.Pose];
    [tsec, seq] = header([poseStampedMsg.Header]);
    tmp = [poseMsg.Position]; pX = [tmp.X]; pY = [tmp.Y]; pZ = [tmp.Z];
    pos = [pX' pY' pZ']';
    tmp = [poseMsg.Orientation]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
    quat = [qW' qX' qY' qZ']';
    pose = struct('t',tsec,'seq',seq,'position',pos,'quaternion',quat);
else
    quat = zeros(4,1); quat(1,:) = 1;
    pos = zeros(3,1);
    tsec = 0; seq = 0;
    pose = struct('t',tsec,'seq',seq,'position',pos,'quaternion',quat); 
end

% Get state messages
bagsel = select(bag, 'Topic', '/attitude');
msgs = readMessages(bagsel,'DataFormat','struct'); stateMsg = [msgs{:}];
[tsec, seq] = header([stateMsg.Header]);
tmp = [stateMsg.Attitude]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
quat = [qW' qX' qY' qZ']';
tmp = [stateMsg.AngularVelocity]; wX = [tmp.X]; wY = [tmp.Y]; wZ = [tmp.Z];
omega = [wX' wY' wZ']';
state = struct('t',tsec,'seq',seq,'quat',quat,'omega',omega);

% time sync
mint = min([min(state.t), min(pose.t), min(imu.t)]);
state.t = state.t - mint;
pose.t = pose.t - mint;
imu.t = imu.t - mint;

end

function [t, seq] = header(headerMsg)
stamp = [headerMsg.Stamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
seq = [headerMsg.Seq];
end

function t = statestamp(stateMsg)
stamp = [stateMsg.StateStamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
end