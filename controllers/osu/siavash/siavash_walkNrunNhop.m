classdef  siavash_walkNrunNhop < Controller
    % Siavash's controller.
    %
    % Copyright 2015 Siavash Rezazadeh
    
    % PUBLIC PROPERTIES =====================================================
    properties (Logical = true)
        % Trigger based on time
        isTimeTrig@logical = true
        % Trigger based on force
        isForceTrig@logical = true
    end % properties
    
    properties
        sT@double=[1.1 1.02 1.1 0.88];  %Torque constant correction coef. (sT)
        k_fp_h_v@double=.23;             % Lateral feedforward gain (k_fp_h_v)
        k_fp_l_v@double=.26;            % Sagittal feedforward gain (k_fp_h_v)
        
        k_fp_h_e@double=.07;             % Lateral proportional gain (k_fp_h_e)
        k_fp_l_e@double=.05;            % Sagittal proportional gain (k_fp_l_e)
        T0@double=.35;                  % Step period (T0)
        k_time@double=0.015;            % Period reduction gain (k_time)
        kDx@double=.1;                    % Foot placement forward derivative gain (kDx)
        kDy@double=0;                    % Foot placement lateral derivative gain (kDy)
        
        xss@double=0;                   % Nominal sagittal toe to toe distance  (xss)
        yss@double=.18;                 % Nominal lateral toe to toe distance (yss)
        
        ys0v@double=0;               % CoM Correction factor (ys0v)
        
        kv_yss@double=0.015;            % Lateral deviation correction factor (kv_yss)
        kv_h=0.005;                     % Lateral gain correction factor (kv_h)
        kv_l=0.005;                     % Sagittal gain correction factor (kv_l)
        
        lret@double=.2;                 % Swing leg retraction (lret)
        
        l0@double=.9;                   % Nominal leg length (l0)
        
        
        kp_leg@double = 3000;    % Leg motor proportional gain (N*m/rad)
        kd_leg@double = 120;     % Leg motor differential gain (N*m*s/rad)
        kp_hip@double = 2000;    % Hip motor proportional gain (N*m/rad)
        kd_hip@double = 75;      % Hip motor differential gain (N*m*s/rad)
        
        legRetPerc@double=40;    % Percentage of time for retraction (legRetPerc)
        k_Energy@double=.021;     % Energy injection gain (k_Energy)
        
        ll_dev@double=.09;       % Stepdown pushoff correction (ll_dev)
        
        k_hop@double=1;          % Scaling for aerial phase (k_hop)
        
         
        mu_s@double=0.5;          %Static friction coefficient (mu_s)
        
        isAngular@logical=false; % Use angular momentum control in flight
        isYawTraj@logical=true; % Use yaw trajectory minimizer
        torso_ff@double=1;
    end % properties
    
    % PROTECTED PROPERTIES ==================================================
    properties (Access = protected)
        
        gaitMode@GaitMode_S
        
        out@double=zeros(1,22); %output vector
        
        
        xs0@double=0;           % Forward CoM trim
        ys0@double=0;           % Lateral CoM trim
        
        lpo@double=0*.06;       % pushoff (input)
        lpo_p@double=0;         % previous pushoff
        
        
        %initations
        q22d0@double=pi/2;
        q23d0@double=acos(.9);
        q21d0@double=acos(.9);
        q24d0@double=pi/2;
        dq21d0@double=0;
        dq22d0@double=0;
        dq23d0@double=0;
        dq24d0@double=0;
        
        uh0@double=0;
        
        dx_des@double=0;
        dy_des@double=0;
        
        f2f0@double=0;
        y_fp@double=0;
        y_fp2@double=0;
        lF@double=0;
        rF@double=0;
        
        
        q_hip_d_s@double=0;
        
        
        RefLeg@double=1;
        
        
        t0@double=0;
        ti@double=0;
        
        x@double=0;
        y@double=0;
        
        
        
        dx@double=0;
        dy@double=0;
        dz@double=0;
        dx_uf@double=0;
        dy_uf@double=0;
        dz_uf@double=0;
        
        
        dxp@double=0;
        dyp@double=0;
        dypp@double=0;
        
        fc2_chk@double=zeros(1,15);
        
    end % properties
    
    % CONSTANT PROPERTIES ===================================================
    properties (Constant = true)
        % Center of mass offset trim increment (m)
        trimIncrement = 0.002;
        % Leg rotational spring constant (N*m/rad)
        k = 2950
        % Torso mass (kg)
        m_torso = 22.2
        % Leg mass (kg)
        m_leg = 20.35
        % Total mass (kg)
        m_total = 62.9
        % Gravity
        g = 9.81
        
        d=0.1831; %leg offset, lateral
        %torso com from the center of the hip
        yG=-0.023;
        zG=0.110887850467290;
        % yG=0;
        % zG=0.110887850467290;
        
        RIGHT=1;
        LEFT=2;
        
    end % properties
    
    % PUBLIC METHODS ========================================================
    methods
        function userSetup(obj)
            %USERSETUP Initialize system object.
            
            % Reset objects
            obj.gaitMode = GaitMode_S.Stand;
            
            % Reset parameters
            
            
            
            obj.lpo_p=0;
            obj.q22d0=pi/2;
            obj.q23d0=acos(.9);
            obj.q24d0=pi/2;
            obj.q21d0=acos(.9);
            obj.dq22d0=0;
            obj.dx_des=0;
            obj.dy_des=0;
            
            obj.uh0=0;
            
            obj.f2f0=0;
            obj.y_fp=0;
            obj.y_fp2=0;
            obj.T0=.35;
            
            obj.lF=0;
            obj.rF=0;
            
            obj.q_hip_d_s=0;
            
            
            
            obj.RefLeg=1;
            
            
            obj.t0=0;
            obj.ti=0;
            
            obj.uh0=0;
            
            obj.dx_des=0;
            obj.dy_des=0;
            
            obj.x=0;
            obj.y=0;
            
            
            obj.dx=0;
            obj.dy=0;
            obj.dz=0;
            obj.dx_uf=0;
            obj.dy_uf=0;
            obj.dz_uf=0;
            obj.dxp=0;
            obj.dyp=0;
            obj.dypp=0;
            obj.fc2_chk=zeros(size(obj.fc2_chk));
        end % userSetup
        
        function userOut = userOutput(obj)
            %USEROUTPUT User output function.
            userOut = obj.out;
            
        end % userOutput
        
        function u = userStep(obj, q, dq)
            %USERSTEP System output and state update equations.
            
            % Initialize input vector to zeros
            u = zeros(1,6); 
            
            % Parse PS3 controller data
            obj.parsePS3Controller;
            
            % Leg center to hip pivot
            d=obj.d;
            % Lateral center of mass w.r.t. hip pivot
            yG=obj.yG;
            % Vertical center of mass w.r.t. hip pivot
            zG=obj.zG;
            % Nominal leg length
            l0=obj.l0;
            % Touchdown leg length
            l0sw=l0;
            % Torso pitch kp and kd
            kpT=obj.k;
            kdT=200;
            % Torso control gain
            k_torso=1;
            % Desired roll angle
            roll_d= -1*pi/180;
            
            if obj.gaitMode == GaitMode_S.Stand
                mu_s=.7;
            else
                mu_s=obj.mu_s;
            end
           
            
            % right and left forces in the leg length direction
            rF=-obj.k*((q(2)-q(4))-(q(1)-q(3)))/sin((q(1)-q(3))/2);
            lF=-obj.k*((q(6)-q(8))-(q(5)-q(7)))/sin((q(5)-q(7))/2);
            % Ignore force spikes
            if abs(rF-obj.rF)>200
                rF=obj.rF;
            end
            if abs(lF-obj.lF)>200
                lF=obj.lF;
            end
            % Store for next step
            obj.rF=rF;
            obj.lF=lF;
            
            % Rename parameters
            pitch=q(13);
            dpitch=dq(13);
            
            roll=q(11);
            droll=dq(11);
            
            yaw=0;
            dyaw=0;
            
            % Hip angles
            phiR=q(9);
            phiL=q(10);
            dphiR=dq(9);
            dphiL=dq(10);
            
            % Absolute leg angles
            if obj.RefLeg == obj.RIGHT
                % 1,2 == Stance leg
                % 3,4 == Trailing leg or swing leg
                % All angles are in world coordinates
                % Leg angles
                theta1=pitch+q(3)-pi/2;
                theta2=pitch+q(1)-pi/2;
                theta3=pitch+q(7)-pi/2;
                theta4=pitch+q(5)-pi/2;
                % Motor angles
                thetam1=pitch+q(4)-pi/2;
                thetam2=pitch+q(2)-pi/2;
                thetam3=pitch+q(8)-pi/2;
                thetam4=pitch+q(6)-pi/2;
                % Leg velocities
                dtheta1=dpitch+dq(3);
                dtheta2=dpitch+dq(1);
                dtheta3=dpitch+dq(7);
                dtheta4=dpitch+dq(5);
                % Motor velocities
                dthetam1=dpitch+dq(4);
                dthetam2=dpitch+dq(2);
                dthetam3=dpitch+dq(8);
                dthetam4=dpitch+dq(6);
                
            else % Left leg
                theta3=pitch+q(3)-pi/2;
                theta4=pitch+q(1)-pi/2;
                theta1=pitch+q(7)-pi/2;
                theta2=pitch+q(5)-pi/2;
                thetam3=pitch+q(4)-pi/2;
                thetam4=pitch+q(2)-pi/2;
                thetam1=pitch+q(8)-pi/2;
                thetam2=pitch+q(6)-pi/2;
                dtheta3=dpitch+dq(3);
                dtheta4=dpitch+dq(1);
                dtheta1=dpitch+dq(7);
                dtheta2=dpitch+dq(5);
                dthetam3=dpitch+dq(4);
                dthetam4=dpitch+dq(2);
                dthetam1=dpitch+dq(8);
                dthetam2=dpitch+dq(6);
                
            end %if
            
            % Stance and swing leg angles
            theta=(theta1+theta2)/2;
            theta_sw=(theta3+theta4)/2;
            % Stance and swing leg length
            ll1=cos((theta2-theta1)/2);
            ll2=cos((theta4-theta3)/2);
            dll1=-1/2*(dtheta2-dtheta1)*sin((theta2-theta1)/2);
            
            % Height of the hip pivot w.r.t. each foot
            zR = cos(roll) * sin(pitch) * cos(-(pitch+q(1)-pi/2) / 0.2e1 + (pitch+q(3)-pi/2) / 0.2e1) * cos((pitch+q(3)-pi/2) / 0.2e1 + (pitch+q(1)-pi/2) / 0.2e1) + d * sin(roll) * cos(phiR) + d * cos(roll) * cos(pitch) * sin(phiR) - cos(-(pitch+q(1)-pi/2) / 0.2e1 + (pitch+q(3)-pi/2) / 0.2e1) * sin((pitch+q(3)-pi/2) / 0.2e1 + (pitch+q(1)-pi/2) / 0.2e1) * sin(roll) * sin(phiR) + cos(-(pitch+q(1)-pi/2) / 0.2e1 + (pitch+q(3)-pi/2) / 0.2e1) * sin((pitch+q(3)-pi/2) / 0.2e1 + (pitch+q(1)-pi/2) / 0.2e1) * cos(roll) * cos(pitch) * cos(phiR);
            zL = cos(roll) * sin(pitch) * cos(-(pitch+q(5)-pi/2) / 0.2e1 + (pitch+q(7)-pi/2) / 0.2e1) * cos((pitch+q(7)-pi/2) / 0.2e1 + (pitch+q(5)-pi/2) / 0.2e1) - d * sin(roll) * cos(phiL) - d * cos(roll) * cos(pitch) * sin(phiL) - cos(-(pitch+q(5)-pi/2) / 0.2e1 + (pitch+q(7)-pi/2) / 0.2e1) * sin((pitch+q(7)-pi/2) / 0.2e1 + (pitch+q(5)-pi/2) / 0.2e1) * sin(roll) * sin(phiL) + cos(-(pitch+q(5)-pi/2) / 0.2e1 + (pitch+q(7)-pi/2) / 0.2e1) * sin((pitch+q(7)-pi/2) / 0.2e1 + (pitch+q(5)-pi/2) / 0.2e1) * cos(roll) * cos(pitch) * cos(phiL);
            if obj.RefLeg==obj.RIGHT
                z1=zR;
                z2=zL;
            else
                z1=zL;
                z2=zR;
            end
            
            % Total time
            obj.ti = obj.ti + obj.sampleInterval;
            % Stance time
            t=obj.ti-obj.t0;
            % Commanded stance time
            T=obj.T0-obj.k_time*(abs(obj.dx_des)-1.4)*(abs(obj.dx_des)>1.4);
            
            % Pushoff for hopping
            % rate limit commanded pushoff
            lpo=clamp(obj.lpo, obj.lpo_p-.01/1*obj.sampleInterval, obj.lpo_p+.01/1*obj.sampleInterval);
            obj.lpo_p=lpo; % keep for next step
            % Smooth and clamp pushoff
            lpo=spline3(t-.4*T, .25*T, 0, 0, lpo*(1-1.5*(abs(obj.dx)-.2)*(abs(obj.dx)>.2)), 0);
            lpo=clamp(lpo,0,.07);
            
            % Pushoff for walking/running forward
            lpo_v=clamp(obj.k_Energy*(abs(obj.dx_des))^1*(sign(obj.dx)==sign(obj.dx_des))+...
                .05*(-1)^obj.RefLeg*sign(obj.dy)*(abs(obj.dy)-.4)*(abs(obj.dy)>.4)-...
                .05*(abs(obj.dx)-abs(obj.dx_des))*(abs(obj.dx)>abs(obj.dx_des))...
                +0*.25*obj.k_Energy*(abs(obj.dx_des)-abs(obj.dx))+0*.05*obj.k_Energy*(abs(obj.dxp)-abs(obj.dx))...
                ,-.01,.07);
            
            % total pushoff
            deltaLx= lpo+(lpo_v)*(t>T/2);
            
            
            
            % Stance force
            F_l=2*obj.k*(((theta2-theta1)/2)-((thetam2-thetam1)/2))/sqrtc(1-cos((theta2-theta1)/2)^2);
            % Swing force
            F_l2=2*obj.k*(((theta4-theta3)/2)-((thetam4-thetam3)/2))/sqrtc(1-cos((theta4-theta3)/2)^2);
            
            % Compute confidence factors for velocity calculations
            RefLegF3=obj.RefLeg;
            RefLegF4=true;
            f_switch=180+abs(obj.dx_des)*40;
            if t<(T/2)
                if obj.RefLeg==obj.LEFT && lF<400 && rF>f_switch
                    RefLegF3=obj.RIGHT;
                elseif obj.RefLeg==obj.RIGHT && rF<400 && lF>f_switch
                    RefLegF3=obj.LEFT;
                end
                if F_l<400 && F_l2<f_switch
                    RefLegF4=false;
                end
            else
                if obj.RefLeg==obj.LEFT && lF<f_switch && rF>400
                    RefLegF3=obj.RIGHT;
                elseif obj.RefLeg==obj.RIGHT && rF<f_switch && lF>400
                    RefLegF3=obj.LEFT;
                end
                if F_l<f_switch && F_l2<400
                    RefLegF4=false;
                end
            end
            
            % Force thresholds for torso control
            f_th=220; % Full control
            f_th0=120; % No control
            % Torso control scaling factors
            % Scaling factor for stance
            if F_l<f_th0
                fc=0;
            elseif F_l<f_th
                fc=(F_l-f_th0)/(f_th-f_th0);
            else
                fc=1;
            end
            % Scaling factor for swing
            if F_l2<f_th0
                fc2=0;
            elseif F_l2<f_th
                fc2=(F_l2-f_th0)/(f_th-f_th0);
            else
                fc2=1;
            end
            % Scaling factor during flight
            if F_l<(f_th0/2)
                fc_h=0;
            elseif F_l<((f_th+f_th0)/2)
                fc_h=(F_l-f_th0/2)/(f_th/2);
            else
                fc_h=1;
            end
            
             
            
            % Find the center of mass velocity given the robot state
            if RefLegF3==obj.RIGHT
                % Jacobian from robot state to center of mass velocity
                JvGR=zeros(3,6);
                JvGR(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * cos(roll) * cos(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (cos(yaw) * cos(roll) * sin(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(yaw) * cos(roll) * yG + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * zG; -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - sin(yaw) * cos(roll) * yG + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * zG; 0;];
                JvGR(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * sin(roll) * cos(phiR) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(yaw) * sin(roll) * sin(phiR) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + sin(yaw) * sin(roll) * yG + sin(yaw) * cos(roll) * cos(pitch) * zG; -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * sin(roll) * cos(phiR) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (cos(yaw) * sin(roll) * sin(phiR) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(yaw) * sin(roll) * yG - cos(yaw) * cos(roll) * cos(pitch) * zG; -sin(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(roll) * cos(phiR) - sin(roll) * cos(pitch) * sin(phiR)) * d + (-cos(roll) * sin(phiR) - sin(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + cos(roll) * yG - sin(roll) * cos(pitch) * zG;];
                JvGR(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * zG; -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * zG; cos(roll) * cos(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * sin(phiR) * d - cos(roll) * sin(pitch) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * zG;];
                JvGR(:,4)=[(sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (sin(yaw) * cos(roll) * cos(phiR) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (-cos(yaw) * cos(roll) * cos(phiR) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * d + (-sin(roll) * cos(phiR) - cos(roll) * cos(pitch) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
                JvGR(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
                JvGR(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
                % Center of mass velocity
                vG=JvGR*[dyaw, droll, dpitch, dphiR, dq(3), dq(1)]';
           
            else % Left leg stance
                % Jacobian from robot state to center of mass velocity
                JvGL=zeros(3,6);
                JvGL(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * cos(roll) * cos(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (cos(yaw) * cos(roll) * sin(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(yaw) * cos(roll) * yG + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * zG; -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - sin(yaw) * cos(roll) * yG + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * zG; 0;];
                JvGL(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * sin(roll) * cos(phiL) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(yaw) * sin(roll) * sin(phiL) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + sin(yaw) * sin(roll) * yG + sin(yaw) * cos(roll) * cos(pitch) * zG; -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * sin(roll) * cos(phiL) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (cos(yaw) * sin(roll) * sin(phiL) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(yaw) * sin(roll) * yG - cos(yaw) * cos(roll) * cos(pitch) * zG; -sin(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(roll) * cos(phiL) - sin(roll) * cos(pitch) * sin(phiL)) * d + (-cos(roll) * sin(phiL) - sin(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * yG - sin(roll) * cos(pitch) * zG;];
                JvGL(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * zG; -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * zG; cos(roll) * cos(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * sin(pitch) * sin(phiL) * d - cos(roll) * sin(pitch) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * zG;];
                JvGL(:,4)=[-(sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (sin(yaw) * cos(roll) * cos(phiL) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (-cos(yaw) * cos(roll) * cos(phiL) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * d + (-sin(roll) * cos(phiL) - cos(roll) * cos(pitch) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
                JvGL(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
                JvGL(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
                % Center of mass velocity
                vG=JvGL*[dyaw, droll, dpitch, dphiL, dq(7), dq(5)]';
            end %if
            
            % Unfiltered velocity of the center of mass
            % Previous
            dx_uf_p=obj.dx_uf;
            dy_uf_p=obj.dy_uf;
            dz_uf_p=obj.dz_uf;
            % Current
            obj.dx_uf=vG(1);
            obj.dy_uf=vG(2);
            obj.dz_uf=vG(3);
            % Ignore spikes in velocity data
            if RefLegF4
                if abs(obj.dx_uf-dx_uf_p)>2*obj.sampleInterval/.001
                    obj.dx_uf=dx_uf_p;
                end
                if abs(obj.dy_uf-dy_uf_p)>2*obj.sampleInterval/.001
                    obj.dy_uf=dy_uf_p;
                end
                if abs(obj.dz_uf-dz_uf_p)>2*obj.sampleInterval/.001
                end
            else
                obj.dx_uf=dx_uf_p;
                obj.dy_uf=dy_uf_p;
                obj.dz_uf=dz_uf_p;
            end
            
            % Filtered CoM velocities
            %             obj.dx=0.01485*dx_uf_p+0.9851*obj.dx; %tf: 1/(s/15+1)
            %             obj.dy=0.01485*dy_uf_p+0.9851*obj.dy;
            %             obj.dz=0.01485*dz_uf_p+0.9851*obj.dz;
            
            obj.dx=0.01094*dx_uf_p+0.9891*obj.dx; %tf: 1/(s/11+1)
            obj.dy=0.01094*dy_uf_p+0.9891*obj.dy;
            obj.dz=0.01094*dz_uf_p+0.9891*obj.dz;
            
            % Update center of mass position
            obj.x=obj.x+obj.dx*obj.sampleInterval;
            obj.y=obj.y+obj.dy*obj.sampleInterval;
            
            % Time scaling factor for position and velocity control of the
            % swing hip
            c_h_p=clamp(t/T,0,1);
            c_h_d=c_h_p;
            
            % Correct foot placements gains based on pushoff and velocity
            % Hip gains for velocity and error
            k_fp_h_v=obj.k_fp_h_v-2*obj.kv_h*abs(obj.dx_des)-1.5*lpo;
            k_fp_h_e=obj.k_fp_h_e - obj.kv_h*abs(obj.dx_des) +0*lpo;
            % Leg gains
            k_fp_l_v=obj.k_fp_l_v + .02*(obj.gaitMode == GaitMode_S.Stand) -4*obj.kv_l*abs(obj.dx_des)+1*lpo;
            k_fp_l_e=obj.k_fp_l_e+0*obj.kv_l*abs(obj.dx_des);
            
            % Trim based on velocity and pushoff
            if obj.dx_des>0
                ys0=obj.ys0 + obj.ys0v*(obj.dx_des) + .2*lpo;
            else
                ys0=obj.ys0 + .5*obj.ys0v*(obj.dx_des) + .2*lpo;
            end
            
            % Scaling factor for leg PD gains based on pushoff and velocity
            if t<.1*T
                s_u=spline3(t, .1*T, .5, 0, 1, 0); 
            elseif t<.5*T
                s_u=spline3(t-.1*T, .4*T, 1, 0, 1, 0); 
            elseif t<.8*T
                s_u=spline3(t-.5*T, .3*T, 1, 0, 1+.05*abs(obj.dx_des)+10*lpo, 0);
            elseif t<T
                s_u=spline3(t-.8*T, .2*T, 1+.05*abs(obj.dx_des)+10*lpo, 0, .5, 0);
            else
                s_u=.5;
            end
            
            % Sagittal toe to toe distance
            xs= obj.xs0+obj.xss*(-1)^obj.RefLeg;
            % Lateral toe to toe distance
            yss = obj.yss-obj.kv_yss*abs(obj.dx_des)+clamp(1.3*(.8-ll1)*(ll1<.8),0,.15)*(obj.gaitMode==GaitMode_S.Obstacle);
            ys=-(-1)^obj.RefLeg*yss+yG+ys0;
            
            % Foot placement w.r.t. the hip pivot
            x_fp=[k_fp_l_v*obj.dx; k_fp_h_v*obj.dy]+[k_fp_l_e; k_fp_h_e].*[1;1].*[obj.dx-obj.dx_des; obj.dy-obj.dy_des]+[xs;ys]+...
                (obj.gaitMode ~= GaitMode_S.Stand)*(obj.gaitMode ~= GaitMode_S.Obstacle)*obj.kDy*[0;1*spline3(t,T,-.5,0,1,0)].*[0;1.3*obj.dy-obj.dypp]+...
                (obj.gaitMode == GaitMode_S.Stand)*obj.kDx*[1;0].*[obj.dx-obj.dxp;0];%*(obj.gaitMode~=GaitMode_S.Obstacle);
            
            % Extend leg if step length exceeded
            % Foot placement w.r.t. the stance foot
            x_f2f_d = x_fp(1)-(cos(theta2)+cos(theta1))/2;
            step_max=4*.4+clamp(.13*(abs(obj.dx_des)-1)*(abs(obj.dx_des)>1)+.15*(abs(obj.dx)-abs(obj.dx_des))*(abs(obj.dx)>abs(obj.dx_des)),0,.45); % maximum step length
            if abs(x_f2f_d)>step_max
                l0sw=clamp(l0sw+.1*(abs(x_f2f_d)-step_max),l0sw,.96); % longer swing leg if needed
            end
            
            % Retraction percentage
            T1=obj.legRetPerc*T/100;
            % Stop initial spline
            T2=80*T/100;
            % Start extending
            T3=45/100*T;
            % Stop extending
            T4=T;

            % Leg length splines
            % Minimum leg length based on forward velocity
            lmin=clamp(l0-obj.lret...
                -.02*(obj.gaitMode == GaitMode_S.Stand)...
                -.08*(obj.gaitMode == GaitMode_S.Obstacle)...
                -.03*abs(obj.dx_des)+0*1.7*lpo-0*.1*(obj.ti>14),.5,.8);
            
            if t<=T1 % Retract
                [q23d,dq23d]=spline3(t,T1,obj.q23d0,obj.dq23d0,acos(lmin),0);
            elseif t<=T3 % Hold
                q23d=acos(lmin);
                dq23d=0;
            elseif t<=T4 % Extend
                q23d=spline3(t-T3,T4-T3,acos(lmin),0,acos(l0sw),0);
                dq23d=0;
            else % Hold
                q23d=acos(l0sw);
                dq23d=0;
            end %if
            
            q23d=acos(clamp(cos(q23d)-0*2*(.82-ll1)*(ll1<.82),cos(q23d),.6)); %(obj.gaitMode==GaitMode_S.Obstacle)
            
            % Delay leg swing angle spline
            tsw0=.15*T-clamp(.05*abs(obj.dx_des),0,.15)*T;
            % Place foot forward (leg angle)
            [x_f2f_t,dx_f2f_t] = spline3(t-tsw0, T2-tsw0, obj.f2f0, 0, x_f2f_d, 0);
            % Final foot point
            obj.y_fp = x_fp(2);
            % If stance leg length is greater than swing leg length
            if (ll1>cos(q23d))
                % Calculate leg angle based on stance leg length
                q24d=acos(clamp((x_f2f_t+(cos(theta2)+cos(theta1))/2)/ll1,-1,1));
                dq24d=-((dx_f2f_t-(sin(theta2)*dtheta2+sin(theta1)*dtheta1)/2))/ll1/sqrt(1-cos(q24d)^2);
            else
                % Calculate leg angle based on swing leg length
                q24d=acos(clamp((x_f2f_t+(cos(theta2)+cos(theta1))/2)/cos(q23d),-1,1));
                dq24d=-((dx_f2f_t-(sin(theta2)*dtheta2+sin(theta1)*dtheta1)/2)*cos(q23d)+sin(q23d)*dq23d*(x_f2f_t+(cos(theta2)+cos(theta1))/2))/cos(q23d)^2/sqrt(1-cos(q24d)^2);
            end %if
            % Overwrite velocity if angle was clamped
            if cos(q24d)==1 || cos(q24d)==-1
                dq24d=0;
            end
            
            % Stance leg control
          
            % Initially
            if t<(.75*T)
                % Hold nominal leg length
                [q21d,dq21d]=spline3(t,.5*T,obj.q21d0,obj.dq21d0,acos(l0sw),0); 
            elseif t<T
                % Follow pushoff policy
                [q21d,dq21d]=spline3(t-(.5+0*.1*obj.dx_des)*T,(.5-0*.1*obj.dx_des)*T,acos(l0sw),0,acos(l0+deltaLx),0);
            else
                % Hold max leg length
                q21d=acos(l0+deltaLx);
                dq21d=0;
            end %if
            % Sanity check
            q21d=clamp(q21d,acos(.97),acos(.7));
%             if obj.test1
%                 q21d=clamp(acos(cos(q21d)-.5*(.8-ll1)*(ll1<.8)),acos(.97),acos(.8));
%             elseif obj.test2
%                 q21d=clamp(q21d,acos(clamp(ll1+.09,.8,1)),acos(ll1-.09));
%             end
            
            if obj.gaitMode==GaitMode_S.Obstacle
                q21d=clamp(q21d,acos(clamp(ll1+obj.ll_dev,.8,.97)),acos(clamp(ll1-obj.ll_dev,.8,.97)));
            end
             
            % desired torso pitch
            qTc=0;
            dqTc=0;
            ddqTc=0;
            
            % Torso pitch control using feedback linearization, stance and swing legs
            uT=1*clamp(-0*(obj.kp_leg*(q24d-(thetam3+thetam4)/2)+obj.kd_leg*(dq24d-(dthetam3+dthetam4)/2))/50+0*F_l*.1*cos(theta-pitch)/(1+.1/ll1*sin(theta-pitch))+ddqTc+k_torso*kdT*(dqTc-dpitch)+k_torso*kpT*(qTc-pitch),-mu_s*fc*F_l,mu_s*fc*F_l);%+(u3+u4)/n/IT;
            uT2=1*clamp(0*F_l2*.1*cos(theta_sw-pitch)/(1+.1/ll2*sin(theta_sw-pitch))+ddqTc+k_torso*kdT*(dqTc-dpitch)+k_torso*kpT*(qTc-pitch),-mu_s*fc2*F_l2,mu_s*fc2*F_l2);%+(u3+u4)/n/IT;
            q22d=-uT/obj.k+(theta1+theta2)/2;
            dq22d=(dtheta1+dtheta2)/2;
            
            % PD control of legs
            u([2 1 5 4])=obj.kp_leg*[s_u s_u .5 .5].*...
                [1 1 1 1].*([(q22d-q21d-thetam1) (q22d+q21d-thetam2) (q24d-q23d-thetam3) (q24d+q23d-thetam4)-(1-fc)*fc2*uT2/obj.k])+...
                obj.kd_leg*[1 1 1 1].*([dq22d-dq21d dq22d+dq21d dq24d-dq23d dq24d+dq23d]-[dthetam1 dthetam2 dthetam3 dthetam4])...
                +obj.torso_ff*obj.k/2*(theta1+theta2-thetam1-thetam2)*[fc fc 0 0];
            
            % Transformation if the other leg is on the ground
            if obj.RefLeg==obj.LEFT
                u([2 1 5 4])=u([5 4 2 1]);
            end
            
            % Correct torques based on motor torque scaling factors
            if ~obj.isSim
                u([1 2 4 5])=obj.sT.*u([1 2 4 5]);
            end
            
            
            % Time to switch between angular momentum control to position control
            t_s_i=10*deltaLx*T;
            
            % Initial hip angle for position control
            obj.q_hip_d_s=0;
            
            % Hip position control
            % If swing leg length is smaller than stance leg
            if ll1>cos(q23d)
                % Calculate hip angle based on foot placement
                q_hip_d=clamp(asin(clamp(obj.y_fp/sqrt(ll1^2+obj.d^2),-1,1))...
                    +atan2(obj.d,ll1)*(-1)^obj.RefLeg-roll,.13*(-1)^obj.RefLeg,-.28*(-1)^obj.RefLeg);
                qh1d0=clamp(asin(clamp(obj.y_fp2/sqrt(ll1^2+obj.d^2),-1,1))...
                    -atan2(obj.d,ll1)*(-1)^obj.RefLeg-roll,-.13*(-1)^obj.RefLeg,.28*(-1)^obj.RefLeg);
                dq_hip_d=0;
            else % if swing leg length is greater than stance leg
                q_hip_d=clamp(asin(clamp(obj.y_fp/sqrt(cos(q23d)^2+obj.d^2),-1,1))...
                    +atan2(obj.d,cos(q23d))*(-1)^obj.RefLeg-roll,.13*(-1)^obj.RefLeg,-.28*(-1)^obj.RefLeg);
                qh1d0=clamp(asin(clamp(obj.y_fp2/sqrt(cos(q23d)^2+obj.d^2),-1,1))...
                    -atan2(obj.d,cos(q23d))*(-1)^obj.RefLeg-roll,-.13*(-1)^obj.RefLeg,.28*(-1)^obj.RefLeg);
                dq_hip_d=obj.y_fp*cos(q23d)*sin(q23d)*dq23d*(cos(q23d)^2+obj.d^2)^(-3/2)/...
                    sqrt(1-obj.y_fp^2/(cos(q23d)^2+obj.d^2))+...
                    obj.d*sin(q23d)*dq23d/(cos(q23d)^2+obj.d^2);
            end %if
            
            if obj.isYawTraj
                % Smooth hip angle
                q_hip_d=clamp((obj.isAngular*(c_h_p-1)+1)*spline3(t-t_s_i, T-(~obj.isAngular)*t_s_i, obj.q_hip_d_s, 0, q_hip_d, 0),.13*(-1)^obj.RefLeg,-.28*(-1)^obj.RefLeg);
            end
            
            % Torso roll control
            if obj.RefLeg==obj.RIGHT
                % Stance leg
                % PD control on roll considering friction
                u30=clamp( (obj.kp_hip*(k_torso*fc*(roll-roll_d))+...
                    obj.kd_hip*(k_torso*fc*droll)),-mu_s*fc*F_l,mu_s*fc*F_l);
                % Including hopping
                u(3)=u30...
                    +obj.k_hop*(1-c_h_p)*(1-fc_h)*(obj.kp_hip*(qh1d0-q(9))+obj.kd_hip*(-dq(9)))...
                    -40*(1-cos(2*pi/T*t))/2;
                
                % Swing leg
                % Use swing leg to help roll control
                if  t>=t_s_i || ~obj.isAngular
                    u(6)=spline3(t,.1*T,obj.uh0,0,((q_hip_d-q(10))*(1-fc2)*c_h_p+k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                        ((dq_hip_d - dq(10))*(1-fc2)*c_h_d+k_torso*fc2*droll)*obj.kd_hip,0);
                else % Angular momentum control
                    
                    if (dq_hip_d>0 && q(10)>.1) || (dq_hip_d<0 && q(10)<-.1)
                        dq_hip_d=0;
                    else
                        dq_hip_d=clamp(4*droll,-4,4);
                    end %if
                    
                    u(6)=((-.03-q(10))*(1-fc2)*c_h_p+k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                        ((dq_hip_d - dq(10))*(1-fc)*(1-fc2)*c_h_d+k_torso*fc2*droll)*obj.kd_hip;
                    
                end %if
                
                % Output parameters
                e_h=q_hip_d-q(10);
                deltaq1=(qh1d0-q(9));

            else
                % Stance leg
                % PD control on roll considering friction
                u60=clamp( ( obj.kp_hip*(k_torso*fc*(roll-roll_d))+ obj.kd_hip*(k_torso*fc*droll)),-mu_s*fc*F_l,mu_s*fc*F_l);
                % Including hopping
                u(6)=u60...
                    +obj.k_hop*(1-c_h_p)*(1-fc_h)*(obj.kp_hip*(qh1d0-q(10))+obj.kd_hip*(-dq(10)))...
                    +40*(1-cos(2*pi/T*t))/2;
                
                % Swing leg
                % Use swing leg to help roll control
                if  t>=t_s_i || ~obj.isAngular
                    u(3)=spline3(t,.1*T,obj.uh0,0,((q_hip_d-q(9))*(1-fc2)*c_h_p+k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                        ((dq_hip_d - dq(9))*(1-fc2)*c_h_d+k_torso*fc2*droll)*obj.kd_hip,0);
                else % Angular momentum control
                    
                    if (dq_hip_d>0 && q(10)>.1) || (dq_hip_d<0 && q(10)<-.1)
                        dq_hip_d=0;
                    else
                        dq_hip_d=clamp(4*droll,-4,4);
                    end %if
                    
                    u(3)=((.03-q(9))*(1-fc2)*c_h_p+k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                        ((dq_hip_d - dq(10))*(1-fc)*(1-fc2)*c_h_d+k_torso*fc2*droll)*obj.kd_hip;
                    
                end %if
                
                % Output parameters
                e_h=q_hip_d-q(9);
                deltaq1=(qh1d0-q(9));
                
            end %if
            
            % Limit absolute torque commands
            u = clamp(u, -obj.u_lim, obj.u_lim);
            
            % Robot output variables
                        obj.out=[obj.x obj.y vG(1:2)' q21d  q22d  q23d ...  %1-10
                            q24d q_hip_d x_f2f_t obj.y_fp fc t lpo ...          %11-19
                            obj.RefLeg obj.dx obj.dy e_h deltaq1 qh1d0... %20-27
                            obj.dx_des obj.dy_des ]; %obj.runTime] %
            % Simulation output variables
%             obj.out=[obj.x obj.y vG(1:2)' q21d dq21d q22d dq22d q23d dq23d...  %1-10
%                 q24d dq24d q_hip_d x_f2f_t obj.y_fp fc fc2 t T...          %11-19
%                 obj.RefLeg obj.dx_uf obj.dy_uf obj.dx obj.dy e_h deltaq1 qh1d0... %20-27
%                 obj.dx_des obj.dy_des lpo obj.sampleInterval obj.q_hip_d_s obj.rF obj.lF dx_uf_p dy_uf_p u x_fp' deltaLx zR zL ll1 ll2 dll1]; %obj.runTime] %
%             
            % Update Swing leg force check
            obj.fc2_chk=[obj.fc2_chk(2:end) fc2];
            ind=find(obj.fc2_chk==1);
            
            % Poincare section; reset parameters
            if ((obj.ti-obj.t0)>=T) || ((t>T/2)&&((cos(q23d)-ll1)>.04) && (length(ind)>(.9*length(obj.fc2_chk))) && (obj.gaitMode==GaitMode_S.Obstacle)) %
                % Initial time
                obj.t0=obj.ti;
                % Initial sagittal foot to foot distance
                obj.f2f0=-1/2*(cos(theta4)+cos(theta3)-cos(theta2)-cos(theta1));
                % Switch feet
                if obj.RefLeg==obj.RIGHT
                    obj.RefLeg=obj.LEFT;
                    obj.uh0=u(3);
                else
                    obj.RefLeg=obj.RIGHT;
                    obj.uh0=u(6);
                end %if
                % Previous lateral foot placement
                obj.y_fp2=obj.y_fp;
                % Inital leg angle
                obj.q22d0=q24d;
                obj.q21d0=q23d;
                obj.q23d0=q21d;
                obj.q24d0=q22d;
                % Initial leg velocities
                obj.dq21d0=dq23d;
                obj.dq22d0=dq24d;
                obj.dq23d0=dq21d;
                obj.dq24d0=dq22d;
                % Center of mass lateral velocity - two steps before
                obj.dypp=obj.dyp;
                % Previous center of mass velocities
                obj.dxp=obj.dx;
                obj.dyp=obj.dy;
            end %if
            
        end % userStep
        
        
        function parsePS3Controller(obj)
            %PARSEPS3CONTROLLER
            
            % Parse gait specific tweaks
            if obj.ps3.cross.isPressed
                obj.gaitMode = GaitMode_S.Stand;
            elseif obj.ps3.circle.isPressed
                obj.gaitMode = GaitMode_S.Normal;
            elseif obj.ps3.triangle.isPressed
                obj.gaitMode = GaitMode_S.Dynamic;
            elseif obj.ps3.square.isPressed
                obj.gaitMode = GaitMode_S.Obstacle;
            end % if
            
            switch obj.gaitMode
                case GaitMode_S.Dynamic
                    
                    t_c = 3; dx_max = 1.55; dy_max = 0.35; obj.lpo=0;
                case GaitMode_S.Obstacle
                    t_c=1.5; dx_max=1;  dy_max = 0.35; obj.lpo=0;
                    
                case GaitMode_S.Stand
                    dx_max=.3; dy_max=.3; t_c=1.5;
                    
                otherwise % GaitMode_S.Normal
                    
                    t_c = 1.5; dx_max = 0.75; dy_max = 0.35; obj.lpo=0;
            end % switch
            
            
            % Parse center of mass trimming
            if obj.ps3.up.isPressed
                % Trim forward
                obj.xs0 = obj.xs0 - obj.trimIncrement;
            elseif obj.ps3.right.isPressed
                % Trim right
                obj.ys0 = obj.ys0 + obj.trimIncrement;
            elseif obj.ps3.down.isPressed
                % Trim backward
                obj.xs0 = obj.xs0 + obj.trimIncrement;
            elseif obj.ps3.left.isPressed
                % Trim left
                obj.ys0 = obj.ys0 - obj.trimIncrement;
            end % if
            
            % Parse right lower trigger
            if obj.ps3.r2.value;
                dx_max = 2*dx_max;
            end % if
            
            % Parse left joystick data (X Velocity)
            dx_cmd = dx_max*clamp(obj.ps3.leftStickY, -1, 1);
            
            % Parse right joystick data (Y Velocity)
            dy_cmd = dy_max*clamp(obj.ps3.rightStickX, -1, 1);
            
            
            % For simulation
            if obj.isSim
                t_c=.3+0*.5*(obj.ti>3)+0*.5*(obj.ti>8);
                dx_cmd = 0*clamp(-.3*floor(obj.ti/2.5),-2.8,2.8) -0*2.8*(obj.ti>1.5)+0*.6*(obj.ti>13)+0*.6*(obj.ti>14)-0*(obj.ti>8); dy_cmd = 0*(obj.ti>2);

            end % if
            
            % Compute smoothing factor
            alpha_x = obj.sampleInterval/(t_c + obj.sampleInterval);
            alpha_y = obj.sampleInterval/(0.5 + obj.sampleInterval);
            
            % Filter target velocity commands
            obj.dx_des = obj.dx_des + alpha_x*(dx_cmd - obj.dx_des);
            obj.dy_des = obj.dy_des + alpha_y*(dy_cmd - obj.dy_des);
        end % parsePS3Controller
    end % methods
end % classdef

%% LOCAL FUNCTIONS ========================================================
function y=sqrtc(x)

if x>0
    y=sqrt(x);
else
    y=0;
end %if
end %sqrtc


function [y,dy]=spline3(t,tf,q0,dq0,q1,dq1)

% Spline start (q0,dq0)
% Spline end (q1, dq1)


if t<=0
    y=q0;
    dy=dq0;
elseif t<tf
    a0=q0;
    a1=dq0;
    a2=(3*(q1-q0)-(2*dq0+dq1)*tf)/tf^2;
    a3=(2*(q0-q1)+(dq0+dq1)*tf)/tf^3;
    
    y=a0+a1*t+a2*t^2+a3*t^3;
    dy=a1+2*a2*t+3*a3*t^2;
    
else
    y=q1;
    dy=dq1;
end %if
end %spline3

