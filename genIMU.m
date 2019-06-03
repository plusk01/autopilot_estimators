function imu = genIMU(imu, pose, keepYaw)

% numerically differentiate position to get v, a of body w.r.t world
dt = mean(gradient(pose.t));
vel = gradient(pose.position) ./ dt;
a = gradient(vel) ./ dt;

% imudt = mean(gradient(imu.t));
gyro = zeros(3,length(a));
accel = zeros(3,length(a));

for i = 1:length(a)
    R_BW = Q(pose.quaternion(:,i)).toRotm()';
    accel_body = R_BW * (a(:,i) - [0;0;1]*9.80665);
    
    % calculate angular velocity of body w.r.t world, in body frame
    % https://gamedev.stackexchange.com/a/147417
    if i < length(a), j = i+1; else, j = i; end
    R_BBprime = R_BW * Q(pose.quaternion(:,j)).toRotm();
    Q_BBprime = Q.fromRotm(R_BBprime);
    [u, theta] = Q_BBprime.toAxisAngle();
    
    if theta > pi, theta = theta - 2*pi; end
    angvel_body = (theta / dt) * (u/norm(u));
    
    % make some noise!
    accel(:,i) = AccelerometerNoiseModel(accel_body);
    gyro(:,i) = GyroNoiseModel(angvel_body);
end

imu.t = pose.t;
imu.accel = accel;
imu.gyro = gyro;

end

function accel = AccelerometerNoiseModel(accel)
acc_stddev = 0.001;
eta = randn(3,1);
accel = accel + acc_stddev*eta;
end

function gyro = GyroNoiseModel(gyro)
gyro_stddev = 0.001;
eta = randn(3,1);
gyro = gyro + gyro_stddev*eta;
end