classdef MahonyAHRS
    %MAHONYAHRS Mahony filter: quat-based nonlinear complementary filter
    %   Based on Madgwick's implementation of Mahony's AHRS filter. See:
    %   github.com/PaulStoffregen/MahonyAHRS/blob/master/src/MahonyAHRS.cpp
    
    properties
        % parameters
        kp = 0.5;
        ki = 0.5;
        
        % whether or not to use accelerometer data
        useAcc = 1;
        
        % How close should the accel be to just gravity for us to use the
        % accelerometer measurement?
        margin = 0.1;
        
        % accelerometer alpha for LPF
        acc_LPF_alpha = 0;
        accel_LPF = [0;0;9.80665];
        
        % gyro LPF
        gyroXY_LPF_alpha = 0;
        gyroZ_LPF_alpha = 0;
        gyro_LPF = [0;0;0];
        
        % This implementation uses the FLU coordinate frame.
        % If the incoming data is in FRD and you want the outgoing data to
        % be in FRD, change this to FRD.
        frame = 'FLU';
        
        % internal state
        q = [1 0 0 0];
        integralFBx = 0;
        integralFBy = 0;
        integralFBz = 0;
        
        % accel-based correction term
        halfe = [0;0;0];
    end
    properties (SetAccess = private, Hidden = true)
        % rotation of a FRD body w.r.t a FLU body (rotates FRD into FLU)
        R_flu_frd = [1 0 0; 0 -1 0; 0 0 -1]; 
    end
    
    methods
        function obj = MahonyAHRS(kp, ki, useAcc, margin, acc_LPF_alpha)
            %MAHONYAHRS Construct an instance of this class
            %   Detailed explanation goes here
            obj.kp = kp;
            obj.ki = ki;
            obj.useAcc = useAcc;
            obj.margin = margin;
            obj.acc_LPF_alpha = acc_LPF_alpha;
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
            
            % for convenience
            twoKi = 2 * obj.ki; twoKp = 2 * obj.kp;
            gx = gyro(1); gy = gyro(2); gz = gyro(3);
            ax = accel(1); ay = accel(2); az = accel(3);
            q0 = obj.q(1); q1 = obj.q(2); q2 = obj.q(3); q3 = obj.q(4);
            
            % compute feedback only if accelerometer measurement valid
            % (avoids NaN in accelerometer normalisation)
%             if obj.useAcc && ~(ax==0 && ay==0 && az==0)
            if obj.useAcc && (1-obj.margin)*9.80665 < norm(accel) ...
                            && norm(accel) < (1+obj.margin)*9.80655
                
                % Normalize accel measurement
                ax = ax/norm(accel);
                ay = ay/norm(accel);
                az = az/norm(accel);
                
                % Estimated direction of gravity
                halfvx = q1 * q3 - q0 * q2;
                halfvy = q0 * q1 + q2 * q3;
                halfvz = q0 * q0 - 0.5 + q3 * q3;
                
                % Error is sum of cross product between estimated
                % and measured direction of gravity
                halfex = ay * halfvz - az * halfvy;
                halfey = az * halfvx - ax * halfvz;
                halfez = ax * halfvy - ay * halfvx;

                % omega from paper
%                 zb = [ax;ay;az];
%                 e1 = [1;0;0];
%                 yb = cross(zb,e1)/norm(cross(zb,e1));
%                 xb = cross(yb,zb)/norm(cross(yb,zb));
%                 Ry = [xb yb zb];
% %                 Ry = Q(Q.fromTwoVectors([0;0;1], zb).q).toRotm();
%                 Rtilde = Q(obj.q).toRotm().'*Ry;
%                 Pa = 0.5*(Rtilde - Rtilde.');
%                 halfe = 0.5 * [Pa(3,2);Pa(1,3);Pa(2,1)];
% %                 halfex = halfe(1); halfey = halfe(2); halfez = halfe(3);
                
                obj.halfe = [halfex;halfey;halfez];
                
                if twoKi > 0
                    % integral error scaled by ki
                    obj.integralFBx = obj.integralFBx + twoKi*halfex*dt;
                    obj.integralFBy = obj.integralFBy + twoKi*halfey*dt;
                    obj.integralFBz = obj.integralFBz + twoKi*halfez*dt;
                else
                    % prevent integral windup
                    obj.integralFBx = 0;
                    obj.integralFBy = 0;
                    obj.integralFBz = 0;
                end
                
                % Apply proportional feedback
                gx = gx + twoKp * halfex;
                gy = gy + twoKp * halfey;
                gz = gz + twoKp * halfez;
            else
                fprintf('Skipped! accel mag: %.2f\n', norm(accel));
            end
            
            % apply integral feedback (n.b. MahonyAHRS does this wrong,
            % betaflight moved it here, correctly so)
            gx = gx + obj.integralFBx;
            gy = gy + obj.integralFBy;
            gz = gz + obj.integralFBz;

            % Integrate rate of change of quaternion
            gx = gx * (0.5*dt); % pre-multiply common factors
            gy = gy * (0.5*dt);
            gz = gz * (0.5*dt);
            qa = q0;
            qb = q1;
            qc = q2;
            q0 = q0 + (-qb * gx - qc * gy - q3 * gz);
            q1 = q1 + (qa * gx + qc * gz - q3 * gy);
            q2 = q2 + (qa * gy - qb * gz + q3 * gx);
            q3 = q3 + (qa * gz + qb * gy - qc * gx);
            
            % Normalize quaternion
            obj.q = [q0 q1 q2 q3];
            obj.q = obj.q/norm(obj.q);
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
        function obj = runLPF(obj, accel, gyro)
            alpha = obj.acc_LPF_alpha;
            obj.accel_LPF = alpha*obj.accel_LPF + (1-alpha)*accel;
            
            alpha = obj.gyroXY_LPF_alpha;
            obj.gyro_LPF(1:2) = alpha*obj.gyro_LPF(1:2) + (1-alpha)*gyro(1:2);
            
            alpha = obj.gyroZ_LPF_alpha;
            obj.gyro_LPF(3) = alpha*obj.gyro_LPF(3) + (1-alpha)*gyro(3);
        end
        
        function v = rotIn(this, v)
            %ROTIN MahonyAHRS uses FLU. Make the incoming data be FLU.

            oldshape = size(v);
            v = reshape(v, 3, 1);

            if strcmp(this.frame, 'FLU')
              v = v; % data is already in FLU
            elseif strcmp(this.frame, 'FRD')
              v = this.R_flu_frd*v;
            else
                error(['Frame ''' this.frame ''' not defined']);
            end

            v = reshape(v, oldshape);
        end

        function v = rotOut(obj, v)
            %ROTIN MahonyAHRS uses FLU. Make the outgoing data fit the user's.

            oldshape = size(v);
            v = reshape(v, 3, 1);

            if strcmp(obj.frame, 'FLU')
              v = v; % data is already in FLU
            elseif strcmp(obj.frame, 'FRD')
              v = obj.R_flu_frd'*v;
            else
                error(['Frame ''' obj.frame ''' not defined']);
            end

            v = reshape(v, oldshape);
        end
    end
end

