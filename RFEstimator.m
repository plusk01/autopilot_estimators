classdef RFEstimator
    %RFESTIMATOR MATLAB implementation of ROSflight Mahony filter
    %   Betaflight and MahonyAHRS uses DCM / quat. ROSflight uses just
    %   quaternions.
    
    properties
        % parameters
        kp = 0.5;
        ki = 0.1;
        
        % whether or not to use accelerometer data
        useAcc = 1;
        
        % integration schemes
        quadInt = 0; % quadratic gyro integration
        expMat = 0; % use exponential map for quaternion integration
        
        % How close should the accel be to just gravity for us to use the
        % accelerometer measurement?
        margin = 0.2;
        
        % spin rate limit: see betaflight imu.c
        SPIN_RATE_LIMIT = deg2rad(2000000);
        
        % accelerometer alpha for LPF
        acc_LPF_alpha = 0;
        accel_LPF = [0;0;-9.80665]; % (n.b. FRD body)
        
        % gyro LPF
        gyroXY_LPF_alpha = 0;
        gyroZ_LPF_alpha = 0;
        gyro_LPF = [0;0;0];
        
        % gyro quadratic integration terms
        w1 = [0;0;0]; w2 = [0;0;0];
        
        % This implementation uses the FRD coordinate frame.
        % If the incoming data is in FLU and you want the outgoing data to
        % be in FLU, change this to FLU.
        frame = 'FRD';
        
        % internal state
        q = [1 0 0 0];
        bias = [0;0;0];
        
        % accel-based correction term
        wacc = [0;0;0];
        wacc_alt = [0;0;0]; % alternate (MahonyAHRS) correction term
        
        % external attitude correction
        extAttKp = 10;
        extAttq = [1 0 0 0];
        extAttFlag = 0;
        extAttSlerpFlag = 0;
        extAttDt = 0;
        extAttTimeLast = 0;
        extAttCount = 0;
    end
    properties (SetAccess = private, Hidden = true)
        % rotation of a FRD body w.r.t a FLU body (rotates FRD into FLU)
        R_flu_frd = [1 0 0; 0 -1 0; 0 0 -1];
    end
    
    methods
        function obj = RFEstimator(rf, margin)
            %MAHONYAHRS Construct an instance of this class
            %   Detailed explanation goes here
            obj.margin = margin;
            
            obj.useAcc = rf.getParam('FILTER_USE_ACC');
            obj.quadInt = rf.getParam('FILTER_QUAD_INT');
            obj.expMat = rf.getParam('FILTER_MAT_EXP');
            obj.kp = rf.getParam('FILTER_KP');
            obj.ki = rf.getParam('FILTER_KI');
            obj.acc_LPF_alpha = rf.getParam('ACC_LPF_ALPHA');
            obj.gyroXY_LPF_alpha = rf.getParam('GYROXY_LPF_ALPHA');
            obj.extAttKp = rf.getParam('FILTER_KP_COR');
        end
        
        function obj = extAttCorrection(obj, quat, t)
            obj.extAttq = quat;
            obj.extAttq(2:4) = obj.rotIn(quat(2:4));
            obj.extAttFlag = 1;
            
            dt = t - obj.extAttTimeLast;
            n = obj.extAttCount;
            obj.extAttDt = 1/(n+1) * (n*obj.extAttDt + dt);
            
            % for next time
            obj.extAttTimeLast = t;
            obj.extAttCount = obj.extAttCount + 1;
        end
        
        function obj = updateIMU(obj, gyro, accel, dt)
            %UPDATEIMU Use accel and gyro to update attitude
            %   Gyro expected in rad/s
            
            % Make sure coordinate frames are consistent
            gyro = obj.rotIn(gyro);
            accel = obj.rotIn(accel);
            
            % LPF raw sensor data
            obj = obj.runLPF(accel, gyro);
            accel = obj.accel_LPF;
            gyro = obj.gyro_LPF;
            
            % bounds for accel mag check. In a perfect world (i.e., with no
            % gyro bias/noise) we would just integrate gyro measurements to
            % get the current attitude. Unfortunately, this is not so.
            % Thus, we use accelerometer to help correct roll/pitch angles,
            % but we can only do this if we are not accelerating.
            % The take-away: Use accelerometer sparingly.
            lowerbound = (1-obj.margin)*9.80665;
            upperbound = (1+obj.margin)*9.80665;
            
            % attitude correction term using accelerometer
            w_acc = [0;0;0];
            
            % flags for which measurement was used
            usedAccel = 0;
            usedExtAtt = 0;
            
            % compute feedback only if accelerometer measurement valid
            if obj.useAcc && ...
                    lowerbound < norm(accel) && norm(accel) < upperbound
                
                % Normalize accel measurement
                a = accel/norm(accel);
                
%                 Get the quaternion (R_BW) from accelerometer
                q_acc_inv = Q.fromTwoVectors([0;0;-1], a).q;
                
                % Get the error quaternion between observer and q
                q_tilde = Q(q_acc_inv).mult(obj.q).q;
                
                % Correction term
                w_acc(1) = -2*q_tilde(1)*q_tilde(2);
                w_acc(2) = -2*q_tilde(1)*q_tilde(3);
%                 w_acc(3) = 0; % don't correct unobservable z
%                 w_acc(3) = -2*q_tilde(1)*q_tilde(4);

                obj.wacc = w_acc;

%                 w_acc = 0.1*Q.boxminus(q_acc_inv, obj.q)';
%                 w_acc(3) = 0;
                
                [u, theta] = Q(q_tilde).toAxisAngle();
                w_acc = theta*u;
%                 w_acc(3) = 0;

obj.wacc_alt = w_acc;

                w_acc = 2*obj.calcMahonyCorrection(a);
%                 w_acc(3) = 0;

                R = Q(obj.q).toRotm();
                w_acc = cross(R(3,:)', a);

%                 R = Q(obj.q).toRotm();
%                 w_acc = cross(R(3,:)', a);
%                 err(3) = 0;

%                 obj.wacc_alt = w_acc;

                usedAccel = 1;
            else                
                fprintf('Skipped! accel mag: %.2f\n', norm(accel));
            end
            
            % Use external attitude correction
            useSLERP = 0;
            if useSLERP && obj.extAttFlag == 1
                obj.extAttFlag = 0;
                obj.extAttSlerpFlag = 1;
            end
            if obj.extAttFlag == 1
                obj.extAttFlag = 0;
                
                q_tilde = Q(Q.inv(obj.q)).mult(obj.extAttq).q;
                
                % Correction term
                err = 2*q_tilde(1)*q_tilde(2:4)';

                % rosflight way
%                 err = Q.boxminus(obj.q, obj.extAttq);
%                 err = rosflight.utils.boxminus(obj.extAttq, obj.q);
                
%                 [u, theta] = Q(q_tilde).toAxisAngle();
%                 err = theta*u;
                
                % gross way
%                 err = obj.q(2:4) - obj.extAttq(2:4);

%                 Rtilde = Q(obj.q).toRotm()'*Q(obj.extAttq).toRotm();
%                 Pa = 0.5*(Rtilde - Rtilde.');
%                 err = [Pa(3,2);Pa(1,3);Pa(2,1)];
                
                % betaflight/Leishman vector way
                R = Q(obj.q).toRotm();
                Rext = Q(obj.extAttq).toRotm();
                err = -cross(R(3,:)', Rext(3,:)');
%                 err(3) = 0;
                
                err2 = -cross(R(1,:)', Rext(1,:)');
%                 err2(1) = 0;
                err = err + err2;
                
                err3 = -cross(R(2,:)', Rext(2,:)');
%                 err3(2) = 0;
                err = err + err3;

                obj.wacc_alt = err;

                w_acc = [0;0;0];
                w_acc = w_acc + obj.extAttKp*err;
                fprintf('[MATLAB] extAtt err: %.4f, %.4f, %.4f\n', err(1), err(2), err(3));
                
                usedExtAtt = 1;
            end
            
            % integrate biases from accel feedback
            if norm(gyro) < obj.SPIN_RATE_LIMIT
                % only integrate if not spinning too fast
                obj.bias(1) = obj.bias(1) - obj.ki*w_acc(1)*dt;%obj.extAttDt;
                obj.bias(2) = obj.bias(2) - obj.ki*w_acc(2)*dt;%obj.extAttDt;
%                 obj.bias(3) = 0;
                obj.bias(3) = obj.bias(3) - obj.ki*w_acc(3)*dt;%obj.extAttDt;
            end
            
            % handle gyro measurements
            if obj.quadInt
                wbar = (obj.w2/-12) + obj.w1*(8/12) + gyro*(5/12);
                obj.w2 = obj.w1;
                obj.w1 = gyro;
            else
                wbar = gyro;
            end
            
%             wbar = zeros(3,1);
%             wbar(3) = 0;
            
            % select gain
            kp = obj.kp;
            if usedExtAtt == 1, kp = 1.5; end

            % build the composite omega vector for kinematic propagation
            wfinal = wbar - obj.bias + kp*w_acc;
%             wfinal = wbar + obj.kp*w_acc;
            
            % propagate dynamics (only if we've moved)
            if norm(wfinal) > 0
                p = wfinal(1); q = wfinal(2); r = wfinal(3);
                
                w = obj.q(1); x = obj.q(2); y = obj.q(3); z = obj.q(4);
                
                if obj.expMat
                    % matrix exponential approximation
                    t1 = cos(norm(wfinal)*dt/2);
                    t2 = 1/norm(wfinal) * sin(norm(wfinal)*dt/2);
                    qhat = Q.Identity;
                    qhat(1) = t1*w + t2*(-p*x - q*y - r*z);
                    qhat(2) = t1*x + t2*( p*w + r*y - q*z);
                    qhat(3) = t1*y + t2*( q*w - r*x + p*z);
                    qhat(4) = t1*z + t2*( r*w + q*x - p*y);
                else
                    % Euler integration
                    qdot = Q.Identity;
                    qdot(1) = 0.5 * (-p*x - q*y - r*z);
                    qdot(2) = 0.5 * ( p*w + r*y - q*z);
                    qdot(3) = 0.5 * ( q*w - r*x + p*z);
                    qdot(4) = 0.5 * ( r*w + q*x - p*y);
                    qhat = obj.q + qdot*dt;
                end
                obj.q = qhat/norm(qhat);
                
                if obj.extAttSlerpFlag == 1
                    obj.q = Q(obj.q).slerp(obj.extAttq, 0.05).q;
                    obj.extAttSlerpFlag = 0;
                end
            end
        end
        
        function q = getQ(obj)
            q = obj.q;
            q(2:4) = obj.rotOut(obj.q(2:4));
        end
        
        function obj = setQ(obj, q)
            obj.q = reshape(q,1,4);
            obj.q(2:4) = obj.rotIn(obj.q(2:4));
        end
    end
    
    methods (Access = private)
        function halfe = calcMahonyCorrection(obj, accel)
            % alternative accel correction term (from MahonyAHRS)
            
            accel = obj.R_flu_frd*accel;
            quat  = [obj.q(1);obj.R_flu_frd*obj.q(2:4)']';
            
            ax = accel(1); ay = accel(2); az = accel(3);
            q0 = quat(1); q1 = quat(2); q2 = quat(3); q3 = quat(4);
            
            % Estimated direction of gravity
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5 + q3 * q3;

            % Error is sum of cross product between estimated
            % and measured direction of gravity
            halfex = ay * halfvz - az * halfvy;
            halfey = az * halfvx - ax * halfvz;
            halfez = ax * halfvy - ay * halfvx;
            
            halfe = [halfex;halfey;halfez];
            
            halfe = obj.R_flu_frd'*halfe;
        end
        function obj = runLPF(obj, accel, gyro)
            alpha = obj.acc_LPF_alpha;
            obj.accel_LPF = alpha*obj.accel_LPF + (1-alpha)*accel;
            
            alpha = obj.gyroXY_LPF_alpha;
            obj.gyro_LPF(1:2) = alpha*obj.gyro_LPF(1:2) + (1-alpha)*gyro(1:2);
            
            alpha = obj.gyroZ_LPF_alpha;
            obj.gyro_LPF(3) = alpha*obj.gyro_LPF(3) + (1-alpha)*gyro(3);
        end
        
        function v = rotIn(this, v)
            %ROTIN RFEstimator uses FRD. Make the incoming data be FRD.

            oldshape = size(v);
            v = reshape(v, 3, 1);

            if strcmp(this.frame, 'FRD')
              v = v; % data is already in FRD
            elseif strcmp(this.frame, 'FLU')
              v = this.R_flu_frd'*v;
            else
                error(['Frame ''' this.frame ''' not defined']);
            end

            v = reshape(v, oldshape);
        end

        function v = rotOut(obj, v)
            %ROTIN RFEstimator uses FRD. Make the outgoing data fit the user's.

            oldshape = size(v);
            v = reshape(v, 3, 1);

            if strcmp(obj.frame, 'FRD')
              v = v; % data is already in FRD
            elseif strcmp(obj.frame, 'FLU')
              v = obj.R_flu_frd*v;
            else
                error(['Frame ''' obj.frame ''' not defined']);
            end

            v = reshape(v, oldshape);
        end
    end
end

