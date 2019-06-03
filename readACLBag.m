function [imu, state, pose, frame] = readACLBag(veh, bagpath)
%READBAG Extracts messages into matrices without using custom msg defn

% Data is in FLU
frame = 'FLU';

t = @(topic) ['/' veh '/' topic];

bag = rosbag(bagpath);

% Get IMU messages
bagsel = select(bag, 'Topic', t('imu'));
msgs = readMessages(bagsel,'DataFormat','struct'); imuMsg = [msgs{:}];
tsec = header([imuMsg.Header]);
tmp = [imuMsg.Gyro]; gX = [tmp.X]; gY = [tmp.Y]; gZ = [tmp.Z];
gyro = [gX' gY' gZ']';
tmp = [imuMsg.Accel]; aX = [tmp.X]; aY = [tmp.Y]; aZ = [tmp.Z];
accel = [aX' aY' aZ']';
imu = struct('t',tsec,'gyro',gyro,'accel',accel);

% Get pose messages
bagsel = select(bag, 'Topic', t('pose'));
msgs = readMessages(bagsel,'DataFormat','struct');
poseStampedMsg = [msgs{:}]; poseMsg = [poseStampedMsg.Pose];
tsec = header([poseStampedMsg.Header]);
tmp = [poseMsg.Position]; pX = [tmp.X]; pY = [tmp.Y]; pZ = [tmp.Z];
pos = [pX' pY' pZ']';
tmp = [poseMsg.Orientation]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
quat = [qW' qX' qY' qZ']';
pose = struct('t',tsec,'position',pos,'quaternion',quat);

% Get state messages
bagsel = select(bag, 'Topic', t('state'));
msgs = readMessages(bagsel,'DataFormat','struct'); stateMsg = [msgs{:}];
tsec = header([stateMsg.Header]); tstate = statestamp(stateMsg);
tmp = [stateMsg.Pos]; pX = [tmp.X]; pY = [tmp.Y]; pZ = [tmp.Z];
pos = [pX' pY' pZ']';
tmp = [stateMsg.Vel]; vX = [tmp.X]; vY = [tmp.Y]; vZ = [tmp.Z];
vel = [vX' vY' vZ']';
tmp = [stateMsg.Quat]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
quat = [qW' qX' qY' qZ']';
tmp = [stateMsg.W]; wX = [tmp.X]; wY = [tmp.Y]; wZ = [tmp.Z];
omega = [wX' wY' wZ']';
state = struct('t',tsec,'tstate',tstate,'pos',pos,'vel',vel,...
               'quat',quat,'omega',omega);

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