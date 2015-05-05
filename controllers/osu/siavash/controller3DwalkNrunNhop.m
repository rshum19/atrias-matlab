classdef  controller3DwalkNrunNhop < Controller
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
        state@double=1;
        ctrltest@double=0; %ctrltest(0)
        k_fp_h_e@double=.1;
        k_fp_l_e@double=.1;
        T0@double=.35;
        k_time@double=0;
        kI@double=0;
        kD@double=0;
        
        xss@double=0;
        yss@double=.1;
        
        k_fp_h_v@double=.2;
        k_fp_l_v@double=.2;
        a_max@double=.3;
        c_yaw@double=0;
        
        
        
        l0@double=.9;
        
        k_hop@double=1;
        
        kp_leg@double = 2200; % Leg motor proportional gain (N*m/rad)
        kd_leg@double = 120; % Leg motor differential gain (N*m*s/rad)
        kp_hip@double = 2200; % Hip motor proportional gain (N*m/rad)
        kd_hip@double = 75; % Hip motor differential gain (N*m*s/rad)
        
        
    end % properties
    
    % PROTECTED PROPERTIES ==================================================
    properties (Access = protected)
        
        gaitMode@GaitMode_S
        
        out@double=zeros(1,22);
       
        yaw_c@double=0;
       
        xs0@double=0;
        ys0@double=0;
        
        lpo@double=0;
        
        lpoL@double=0;
        lpoR@double=0;
        lpoL_p@double=0;
        lpoR_p@double=0;
        lpo_p@double=0;
        
        q22d0@double=pi/2;
        q23d0@double=acos(.9);
        %     qh1d0=0;
        dx_des@double=0;
        dy_des@double=0;
        
        xId@double=0;
        yId@double=0;
        xd@double=0;
        yd@double=0;
        xI@double=0;
        yI@double=0;
        x_des@double=0;
        
        f2f0@double=0;
        y_fp@double=0;
        y_fp2@double=0;
        T@double=.35;
        x_fpt0@double=[0;0];
        x_f2f@double=[0; 2*.1831];
        lF@double=0;
        rF@double=0;
       
        
        q_hip_d_s@double=0;
        
        Tc@double=1;

        RefLeg@double=1;
        
        
        t0@double=0;
        ti@double=0;
        
        x@double=0;
        y@double=0;
        xc@double=0;
        yc@double=0;
        
        Tcp@double=1;
        xcp@double=0;
        ycp@double=0;
        
        dx@double=0;
        dy@double=0;
        dz@double=0;
        dx_uf@double=0;
        dy_uf@double=0;
        dz_uf@double=0;
        
        
        lret@double=.2;
        
        


    end % properties
    
    % CONSTANT PROPERTIES ===================================================
    properties (Constant = true)
        % Center of mass offset trim increment (m)
        trimIncrement = 0.005;
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
        yG=-(0.2225-0.1577)*22/62;
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
            obj.gaitMode = GaitMode_S.Normal;
            
            % Reset parameters
           
           
            obj.yaw_c=0;
            


	   
            obj.lpoL=0;
            obj.lpoR=0;
            obj.lpoL_p=0;
            obj.lpoR_p=0;
            obj.lpo_p=0;
            obj.q22d0=pi/2;
            obj.q23d0=acos(.9);
            %     qh1d0=0;
            obj.dx_des=0;
            obj.dy_des=0;
            
            obj.xId=0;
            obj.yId=0;
            obj.xd=0;
            obj.yd=0;
            obj.xI=0;
            obj.yI=0;
            obj.x_des=0;
             
            obj.f2f0=0;
            obj.y_fp=0;
            obj.y_fp2=0;
            obj.T=.35;
            obj.x_fpt0=[0;0];
            obj.x_f2f=[0; 2*.1831];
            obj.lF=0;
            obj.rF=0;
           
            obj.q_hip_d_s=0;
            
            
            obj.Tc=1;
            
            obj.RefLeg=1;
            
            
            obj.t0=0;
            obj.ti=0;
            
            
            obj.x=0;
            obj.y=0;
            obj.xc=0;
            obj.yc=0;
            obj.Tcp=1;
            obj.xcp=0;
            obj.ycp=0;
            
            obj.dx=0;
            obj.dy=0;
            obj.dz=0;
            obj.dx_uf=0;
            obj.dy_uf=0;
            obj.dz_uf=0;
        end % userSetup
        
        function userOut = userOutput(obj)
            %USEROUTPUT User output function.
%             if ~obj.isSim
                userOut = obj.out;
%             else
%                 userOut = obj.out(25:26);
%             end
        end % userOutput
        
        function u = userStep(obj, q, dq)
            %USERSTEP System output and state update equations.
            
            
             % Update stance timer
            
%             if obj.isSim 
%                 obj.sampleInterval=q(14)-obj.ti;
%             end
            
%             fprintf('al1: %f\n',obj.sampleInterval)
            
            obj.ti = obj.ti + obj.sampleInterval;
            
            
            t=obj.ti-obj.t0;
            
            d=obj.d;
            yG=obj.yG;
            zG=obj.zG;
            
            
            t_stb=4;
            
            vs=.5;
            
            k_torso=1;
            
            roll_d=-0*1*pi/180  -0*3*pi/180*(obj.RefLeg==2) - 0*4*pi/180*(-1)^obj.RefLeg*sin(pi*t/obj.T)-1*pi/180;
            
            
            yaw_d=0*pi/180;
            
           
            
            
           
            
            % Parse PS3 controller data
            obj.parsePS3Controller;

            
            
            ctrltest=obj.ctrltest+2*(obj.ti>0);
            
            a_max=obj.a_max/max(min(obj.dx_des,1.1),.5);
            
%             T0=clamp(obj.T0-0*.05*(obj.dx_des-obj.vs)*(obj.dx_des>obj.vs),.2,obj.T0);
            
            
            
           
%             if obj.RefLeg==obj.RIGHT
%                 lpo=clamp(obj.lpo,obj.lpoR_p-.01/1.5*obj.sampleInterval,obj.lpoR_p+.01/1.5*obj.sampleInterval);
%                 obj.lpoR_p=lpo;
%             else
%                 lpo=clamp(obj.lpo,obj.lpoL_p-.01/1.5*obj.sampleInterval,obj.lpoL_p+.01/1.5*obj.sampleInterval);
%                 obj.lpoL_p=lpo;
%             end
%             
            lpo=clamp(obj.lpo,obj.lpo_p-.01/1*obj.sampleInterval,obj.lpo_p+.01/1*obj.sampleInterval)+0*.005*(-1)^obj.RefLeg*(obj.ti>(100*obj.lpo));%*(obj.ti>6);
            obj.lpo_p=lpo;
            lpo=lpo+(.02-0*.003*(-1)^obj.RefLeg)*abs(obj.dx_des)*(sign(obj.dx)==sign(obj.dx_des));
            
		    lpo=clamp(lpo,0,.07);


            dx_des0=obj.dx_des;
            
            dx_des=obj.dx_des;
%                        
%             if abs(obj.dx_des)>.1 || abs(obj.dy_des)>.1
%                 dx_des=clamp(obj.dx_des,obj.dx_des_p-obj.a_max*obj.sampleInterval,obj.dx_des_p+obj.a_max*obj.sampleInterval);
%                 dy_des=clamp(obj.dy_des,obj.dy_des_p-obj.a_max*obj.sampleInterval,obj.dy_des_p+obj.a_max*obj.sampleInterval);
%             end
            
            
            
            
            
            
%             if abs(dx_des)<obj.vs
%                 walkCmd=0;
%             else
%                 walkCmd=1;
%             end
            
            
            
%             ldev=.015*(abs(dx_des)-obj.vs)*walkCmd-0*clamp(0.03*q(12),-01,.01)*(-1)^obj.RefLeg*(obj.ti>obj.t_stb);
            
            
            
            l0=obj.l0;%+0*(obj.ti>obj.t_stb)*clamp(0*-.06*q(12)*(-1)^obj.RefLeg-.003*(-1)^obj.RefLeg,-.04,.04);
           
            l0sw=l0;%-.01*(-1)^RefLeg;
            
            lmax0=l0;
            kpT=obj.k;
            kdT=200;
            
            
            
            
            
            
            f_th=220;
            f_th0=120;
            
            
          
           
            
            kl0_y=0*.02;
            kl0_x=0*.02;
            
            
            
            % qTc=0;
            dqTc=0;
            ddqTc=0;
            
            
            
            rF=-obj.k*((q(2)-q(4))-(q(1)-q(3)))/sin((q(1)-q(3))/2);
            lF=-obj.k*((q(6)-q(8))-(q(5)-q(7)))/sin((q(5)-q(7))/2);
            
            if abs(rF-obj.rF)>200
                rF=obj.rF;
            end
            if abs(lF-obj.lF)>200
                lF=obj.lF;
            end
            
            obj.rF=rF;
            obj.lF=lF;
            
            
            
            RefLegF2=obj.RefLeg;
            if obj.RefLeg==obj.LEFT && lF<f_th && rF-lF>0
                RefLegF2=obj.RIGHT;
            elseif obj.RefLeg==obj.RIGHT && rF<f_th && lF-rF>0
                RefLegF2=obj.LEFT;
            end
            
            
            
            
            e_h=0;
            deltaq1=0;
            q22d0o=0;
            q24d=0;
            x_f2f_t=0;
            deltaLx=0;
            lmax=.9;
            dq22d=0;
            x_f2f_t=2*d;
            dq23d=0;
            dq24d=0;
            q22d=0;
            dq21d=0;
            x_fpt=[0;0];
            dx_fpt=[0;0];
            dqd=[0;0;0];
            xFd=0;
            qh1d0=0;
            f2f=2*d;
            xy_st=[0 0];
            phiL=0;
            phiR=0;
            dx=0;
            dy=0;
            q_hip_d=0;
            uT=0;
            q21d=acos(.9);
            q23d=q21d;

            dq_hip_d=0;
            
            
            
            
            
            pitch=q(13);
            dpitch=dq(13);
            yaw=0*q(12);
            dyaw=0*dq(12);
            roll=q(11);
            droll=dq(11);
            
            yaw_a=q(12);
            dyaw_a=dq(12);
            
            phiR=q(9);
            phiL=q(10);
            dphiR=dq(9);
            dphiL=dq(10);
            
           
            
            
            % absolute angles
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
                % Hip angles
                qH1=q(9);
                qH2=q(10);
                dqH1=dq(9);
                qH2=dq(10);
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
                % Hip angles
                qH2=q(9);
                qH1=q(10);
                dqH2=dq(9);
                dqH1=dq(10);
            end
            
            ll1=cos((theta2-theta1)/2);
            ll2=cos((theta4-theta3)/2);
            dll1=-1/2*(dtheta2-dtheta1)*sin((theta2-theta1)/2);
            
            F_l=2*obj.k*(((theta2-theta1)/2)-((thetam2-thetam1)/2))/sqrtc(1-cos((theta2-theta1)/2)^2);
            F_l2=2*obj.k*(((theta4-theta3)/2)-((thetam4-thetam3)/2))/sqrtc(1-cos((theta4-theta3)/2)^2);
            
            F_l3=obj.k/2*((abs(theta2-thetam2))+abs((theta1-thetam1)));
            F_l4=obj.k/2*((abs(theta4-thetam4))+abs((theta3-thetam3)));
            
            
            
            
            
            RefLegF3=obj.RefLeg;
            RefLegF4=true;
            f_switch=180+abs(obj.dx_des)*50;
            if t<(obj.T/2)
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
            
            
            
            
            if F_l<f_th0
                fc=0;
            elseif F_l<f_th
                fc=(F_l-f_th0)/(f_th-f_th0);
            else
                fc=1;
            end
            if F_l2<f_th0
                fc2=0;
            elseif F_l2<f_th
                fc2=(F_l2-f_th0)/(f_th-f_th0);
            else
                fc2=1;
            end
            
            if F_l<(f_th0/2)
                fc_h=0;
            elseif F_l<((f_th+f_th0)/2)
                fc_h=(F_l-f_th0/2)/(f_th/2);
            else
                fc_h=1;
            end
            
            
            
            
            ml_R=cos((q(2)-q(4))/2);
            ml_L=cos((q(6)-q(8))/2);
            ll_R=cos((q(1)-q(3))/2);
            ll_L=cos((q(5)-q(7))/2);
            theta=(theta1+theta2)/2;
            dtheta=(dtheta1+dtheta2)/2;
            theta_sw=(theta3+theta4)/2;
            
       
            
            
            
            xOFR=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
            xOFL=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(roll) * cos(phiL) + cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
            
            xHLFR=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d; cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiL) + cos(roll) * cos(pitch) * sin(phiL)) * d;];
            xHRFL=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d; cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(roll) * cos(phiL) + cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d;];
            
            JvOR=zeros(3,6);
            JvOL=JvOR;
            
            JvOR(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * cos(roll) * cos(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (cos(yaw) * cos(roll) * sin(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); 0;];
            JvOR(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * sin(roll) * cos(phiR) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(yaw) * sin(roll) * sin(phiR) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * sin(roll) * cos(phiR) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (cos(yaw) * sin(roll) * sin(phiR) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -sin(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(roll) * cos(phiR) - sin(roll) * cos(pitch) * sin(phiR)) * d + (-cos(roll) * sin(phiR) - sin(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
            JvOR(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); cos(roll) * cos(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * sin(phiR) * d - cos(roll) * sin(pitch) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
            JvOR(:,4)=[(sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (sin(yaw) * cos(roll) * cos(phiR) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (-cos(yaw) * cos(roll) * cos(phiR) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * d + (-sin(roll) * cos(phiR) - cos(roll) * cos(pitch) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
            JvOR(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
            JvOR(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
            
            JvOL(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * cos(roll) * cos(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (cos(yaw) * cos(roll) * sin(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); 0;];
            JvOL(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * sin(roll) * cos(phiL) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(yaw) * sin(roll) * sin(phiL) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * sin(roll) * cos(phiL) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (cos(yaw) * sin(roll) * sin(phiL) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -sin(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(roll) * cos(phiL) - sin(roll) * cos(pitch) * sin(phiL)) * d + (-cos(roll) * sin(phiL) - sin(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
            JvOL(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); cos(roll) * cos(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * sin(pitch) * sin(phiL) * d - cos(roll) * sin(pitch) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
            JvOL(:,4)=[-(sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (sin(yaw) * cos(roll) * cos(phiL) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (-cos(yaw) * cos(roll) * cos(phiL) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * d + (-sin(roll) * cos(phiL) - cos(roll) * cos(pitch) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
            JvOL(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
            JvOL(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
            
            
            
            
            
            
            if RefLegF3==obj.RIGHT
                
                
                JvGR=zeros(3,6);
                
                JvGR(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * cos(roll) * cos(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (cos(yaw) * cos(roll) * sin(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(yaw) * cos(roll) * yG + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * zG; -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - sin(yaw) * cos(roll) * yG + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * zG; 0;];
                
                JvGR(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * sin(roll) * cos(phiR) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(yaw) * sin(roll) * sin(phiR) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + sin(yaw) * sin(roll) * yG + sin(yaw) * cos(roll) * cos(pitch) * zG; -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * sin(roll) * cos(phiR) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (cos(yaw) * sin(roll) * sin(phiR) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(yaw) * sin(roll) * yG - cos(yaw) * cos(roll) * cos(pitch) * zG; -sin(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(roll) * cos(phiR) - sin(roll) * cos(pitch) * sin(phiR)) * d + (-cos(roll) * sin(phiR) - sin(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + cos(roll) * yG - sin(roll) * cos(pitch) * zG;];
                JvGR(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * zG; -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * zG; cos(roll) * cos(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * sin(phiR) * d - cos(roll) * sin(pitch) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * zG;];
                JvGR(:,4)=[(sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (sin(yaw) * cos(roll) * cos(phiR) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (-cos(yaw) * cos(roll) * cos(phiR) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * d + (-sin(roll) * cos(phiR) - cos(roll) * cos(pitch) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
                
                JvGR(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
                
                JvGR(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
                
                
                vGR=JvGR*[dyaw, droll, dpitch, dphiR, dq(3), dq(1)]';
                vG=vGR;
                
                %position of center of the hip wrt the stance foot
                x_st=-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);
                y_st=-(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);
                
                
                
                
                
                
                
                
                
            else
                
                JvGL=zeros(3,6);
                
                JvGL(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * cos(roll) * cos(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (cos(yaw) * cos(roll) * sin(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(yaw) * cos(roll) * yG + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * zG; -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - sin(yaw) * cos(roll) * yG + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * zG; 0;];
                
                
                JvGL(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * sin(roll) * cos(phiL) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(yaw) * sin(roll) * sin(phiL) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + sin(yaw) * sin(roll) * yG + sin(yaw) * cos(roll) * cos(pitch) * zG; -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * sin(roll) * cos(phiL) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (cos(yaw) * sin(roll) * sin(phiL) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(yaw) * sin(roll) * yG - cos(yaw) * cos(roll) * cos(pitch) * zG; -sin(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(roll) * cos(phiL) - sin(roll) * cos(pitch) * sin(phiL)) * d + (-cos(roll) * sin(phiL) - sin(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * yG - sin(roll) * cos(pitch) * zG;];
                
                JvGL(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * zG; -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * zG; cos(roll) * cos(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * sin(pitch) * sin(phiL) * d - cos(roll) * sin(pitch) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * zG;];
                
                JvGL(:,4)=[-(sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (sin(yaw) * cos(roll) * cos(phiL) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (-cos(yaw) * cos(roll) * cos(phiL) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * d + (-sin(roll) * cos(phiL) - cos(roll) * cos(pitch) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
                
                
                JvGL(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
                
                
                JvGL(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
                
                
                vGL=JvGL*[dyaw, droll, dpitch, dphiL, dq(7), dq(5)]';
                vG=vGL;
                
                
                %position of center of the hip wrt the stance foot
                x_st=-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);
                y_st=-(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);
                
                
                
            end
            
            
           
            
            
            dx_uf_p=obj.dx_uf;
            dy_uf_p=obj.dy_uf;
            dz_uf_p=obj.dz_uf;
            
            
            obj.dx_uf=vG(1);
            obj.dy_uf=vG(2);
            obj.dz_uf=vG(3);
            
            
           
%             if t>(obj.T/2)
%                 k_f_c=1;
%             else
%                 k_f_c=2;
%             end
%             if max(F_l,F_l2)>(k_f_c*f_th)
            if RefLegF4
                obj.yaw_c=obj.yaw_c+dq(12)*obj.sampleInterval;
                if abs(obj.dx_uf-dx_uf_p)>2*obj.sampleInterval/.001
                    obj.dx_uf=dx_uf_p;
                end
                if abs(obj.dy_uf-dy_uf_p)>2*obj.sampleInterval/.001
                    obj.dy_uf=dy_uf_p;
                end
                if abs(obj.dz_uf-dz_uf_p)>2*obj.sampleInterval/.001
%                     obj.dz_uf=dz_uf_p;
                end
            else
                obj.dx_uf=dx_uf_p;
                obj.dy_uf=dy_uf_p;
                obj.dz_uf=dz_uf_p;
            end
            obj.dx=0.01485*dx_uf_p+0.9851*obj.dx; %tf: 1/(s/15+1)
            obj.dy=0.01485*dy_uf_p+0.9851*obj.dy;
            obj.dz=0.01485*dz_uf_p+0.9851*obj.dz;
            
                                  
          
            
%             l0=l0*(1-(kl0_y*(-1)^obj.RefLeg*(obj.dy_des-obj.dy)+kl0_x*sign(pi/2-theta)*(obj.dx_des-obj.dx)));
            lmax=lmax0;
            
            
            
            
            
            if obj.dx_des==0 || obj.kI==0 %|| ~walkCmd
                obj.x=0;
                obj.y=0;
                obj.xc=0;
                obj.yc=0;
                obj.xI=0;
                obj.yI=0;
                obj.xd=0;
                obj.yd=0;
                obj.xId=0;
                obj.yId=0;
                obj.x_des=0;
            else
                obj.x=obj.x+obj.dx*obj.sampleInterval;
                obj.y=obj.y+obj.dy*obj.sampleInterval;
                obj.xc=obj.xc+obj.dx*obj.sampleInterval;
                obj.yc=obj.yc+obj.dy*obj.sampleInterval;
                obj.xI=obj.xI+obj.x*obj.sampleInterval;
                obj.yI=obj.yI+obj.y*obj.sampleInterval;
                obj.xd=obj.xd+dx_des0*obj.sampleInterval;
                obj.yd=obj.yd+obj.dy_des*obj.sampleInterval;
                obj.xId=obj.xId+obj.xd*obj.sampleInterval;
                obj.yId=obj.yId+obj.yd*obj.sampleInterval;
                obj.x_des=obj.x_des+obj.dx_des*obj.sampleInterval;
            end
            
            
            
            obj.T=obj.T0-.015*abs(obj.dx_des);
            c_h_p=clamp(2*(t-obj.T/2)/obj.T,0,1);
            c_h_d=clamp(sqrtc(t/obj.T),0,1);
            c_h=clamp(t/obj.T,0,1);
            % c_h=1;
            c_h_p=c_h;
            c_h_d=c_h;
            
            c_h_v=clamp(2*(t-obj.T/2)/obj.T,0,1);
            
            
%             kyaw_l=-.05*(q(12)-yaw_d);
%             kyaw_x=-.05*(q(12)-yaw_d);
%             
%             kyaw_l=-0*5*.05*yaw_cp;
%             kyaw_x=-0*5*.05*yaw_cp;
            
            kyaw_l=0;
            kyaw_x=0;
            
            lpo_x=0*(.015+kyaw_l)*abs(obj.dx_des)+0*.01*(obj.dx_des-obj.dx)*sign(obj.dx_des);
%             lpo1=clamp(.2*(abs(dx)-abs(dx_des))*(abs(dx)>abs(dx_des)),0,.05);
%             l_yaw=0*clamp(50*x_st*((yaw_c-yaw_cp)+0*dq(12)+0*.2*q(12))*(-1)^obj.RefLeg*(obj.ti>obj.t_stb),-lpo/4,lpo/4);
            l_yaw=0;
            deltaLx=l_yaw+...
                lpo_x*(x_st>0)- lpo_x/2*(x_st<=0)+...
                (lpo-0*.2*(abs(obj.dx)-abs(obj.dx_des))*(abs(obj.dx)>abs(obj.dx_des)))*(t>obj.T/2)+(-0*lpo)*(t<=obj.T/2); %*(dz>0)...%*(dz<0)...
%                 +0*(ldev*sign(x_st)+0*(x_st>0));%*(dx<dx_des)*walkCmd;
            
            
            % k_fp_h_v=k_fp_h_v - clamp(1*max(deltaLx,0),0,.1);
            
            k_fp_h_v=obj.k_fp_h_v - 0*clamp(.03*abs(obj.dx_des)+0*3*max(deltaLx,0),0,.08);
            k_fp_h_e=obj.k_fp_h_e - 0*clamp(.5*.05*abs(obj.dx_des)+.5*max(deltaLx,0),0,.06);
            
            % k_fp_h_v=k_fp_h_v*c_h^3;
            % k_fp_h_e=k_fp_h_e*c_h^3;
            
            k_fp_l_v=obj.k_fp_l_v-0*clamp(.02*abs(obj.dx_des),0,.05);
            k_fp_l_e=clamp(obj.k_fp_l_e-0*.05*abs(obj.dx_des),.05,obj.k_fp_l_e);
            
            
            k_fp_h_v=k_fp_h_v-.02*abs(obj.dx_des)-0*0.5*lpo;
            k_fp_h_e=k_fp_h_e +0*lpo;
            
            
            k_fp_l_v=k_fp_l_v-0*.02*obj.dx_des^2;
            k_fp_l_e=clamp(k_fp_l_e -0*.01*abs(obj.dx_des),.05,k_fp_l_e);
            
            % yss=clamp(yss-.5*max(deltaLx,0)-.01*abs(dx_des),.05,.2);
            % xss=xss+1*(1+0*30*deltaLx)*(0*yaw_c+.3*q(12))*(ti>t_stb);%-.01*(ti>t_stb);
            
            xss=obj.xss-0*.04*abs(obj.dx_des)+0*2*deltaLx;
            
            
            yss = obj.yss-0*.7*lpo-0*.007*abs(obj.dx_des);
            
            ys0=obj.ys0-0*.2*lpo+0*.01*abs(obj.dx_des)-0*.7*deltaLx;
            
            
%             T0=T0-clamp((0*.02+c_yaw*(0*(yaw_d-q(12))-0*yaw_c-2*yaw_cp)*(ti>t_stb))*(-1)^RefLeg,-.04,.04);
           
            
            % roll_d=deltaLx;
            
            % roll_d=clamp(3*yaw_cp,-.15,.15);
%             qTc=0*clamp(c_yaw*(1.5*yaw_cp+0*(1.5*(q(12)-yaw_d)+.005*dq(12))/(1+0*8*deltaLx)),-.15,.15);
            
            % qTc=-.1;
            
            qTc=0;
            xs0=obj.xs0+qTc/10;
            
            % yss=yss-lpo;
            
            xs=0+xs0+xss*(-1)^obj.RefLeg;
            ys=-(-1)^obj.RefLeg*yss+yG+ys0;
            
            
            
            %x_fp wrt the hip when hip angle is zero
            
            % dy_des=dy_des-(xcp/Tcp+xcpp/Tcpp)/2;
            x_fp=[k_fp_l_v*obj.dx; k_fp_h_v*obj.dy]+[k_fp_l_e; k_fp_h_e].*[1;1].*[obj.dx-obj.dx_des; obj.dy-obj.dy_des]+[xs;ys]...
                +[0;0].*[clamp(obj.kI*.05*(1+abs(obj.dx_des))*(obj.x-0*obj.xc-obj.xd),-.05,.05); clamp(2*obj.kI*.05*(1+obj.dy_des)*(obj.y-0*obj.yc-obj.yd),-.05,.05)]...%*walkCmd...
                -obj.kD*[0;0].*clamp([.1*(obj.xcp/obj.Tcp-obj.dx); .1*(obj.ycp/obj.Tcp-obj.dy)],-.05,.05)...%*walkCmd...
                +0*clamp(.02*[0;obj.yI-obj.yId],-.1,.1)+...
                obj.kD*[0;1].*[obj.dx-obj.xcp/obj.Tcp; obj.dy-obj.ycp/obj.Tcp]+...
                obj.kI*[1;0].*[obj.x-obj.x_des; 0];
%              +0*clamp(.0005*[0;1].*[xcp/Tcp+xcpp/Tcpp-2*dx_des; ycp/Tcp+ycpp/Tcpp-2*dy_des],-.02,.02)*walkCmd...
%                 +0*[0; -.015]*walkCmd...
%                 +0*clamp(obj.kD*[0;2].*[0; 0-(.7*obj.yc/max(t,obj.T/2)-.7*obj.ycp/obj.Tcp-0*.3*ycpp/Tcpp)],-.04,.04)*walkCmd...
%                 +0*[0;.5].*clamp([0; ycp+.6*ycpp+0*.3*ycppp],-.05,.05)*(ti>t_stb);%*walkCmd;
            
            % x_fp(1)=x_fp(1)+c_yaw*(-1)^RefLeg;
            
            %% INITIALIZE =========================================================
            
                    
            % Initialize input vector to zeros
            u = zeros(1,6);
            
            
            %% MAIN CONTROLLER ====================================================
            
            switch obj.state
                case 0 % STAND ----------------------------------------------------------
                    
                    
                    u([2 1 5 4])=obj.kp_leg*[2^.5 2^.5 2^.5 2^.5].*([pi-acos(l0) pi+acos(l0) pi-acos(l0) pi+acos(l0)]-[q(4) q(2) q(8) q(6)])+...
                        obj.kd_leg*[2^.5 2^.5 2^.5 2^.5].*(zeros(1,4)-[dq(4) dq(2) dq(8) dq(6)]);
                    
                    
                    u([3 6]) = ([0; 0] - [q(9); q(10)])*obj.kp_hip + (zeros(2,1) - [dq(9); dq(10)])*obj.kd_hip;
                    
                    
                case 1 % WALK --------------------------------------------------------
                    
%                     x_f2f_d = clamp(x_fp(1)-(cos(theta2)+cos(theta1))/2,-.4,.4);
%                     l0sw=clamp(l0sw+0*.01*abs(obj.dx)+0*.01*(obj.dx-obj.dx_des),l0sw,.96);
                    x_f2f_d = x_fp(1)-(cos(theta2)+cos(theta1))/2;
                    step_max=.4+clamp(.1*(abs(obj.dx_des)-1.2)*(abs(obj.dx_des)>1.2),0,.25);
                    if abs(x_f2f_d)>step_max
                        deltaLx=clamp(deltaLx-.2*(abs(x_f2f_d)-step_max),-.06,.96-l0);
                        l0sw=clamp(l0sw+.4*(abs(x_f2f_d)-step_max),l0sw,.96);
%                         l0sw=clamp(l0sw+.025*abs(obj.dx),l0sw,.96);
                        x_f2f_d=step_max*sign(x_f2f_d);
                    end
                    
                     lmin=l0-obj.lret+.02*obj.dx_des+1.5*lpo;
                    
                    
%                     l0sw=clamp(l0sw+.2*(abs(obj.dx)-abs(obj.dx_des))*(abs(obj.dx)>abs(obj.dx_des)),l0sw,.96);
                    
                    
                    
                    if t<=obj.T/2
                        q21d=acos(clamp(l0+deltaLx,.7,.96));
                        dq21d=0;
                    else
                        q21d=acos(clamp(-4*(lmax-l0)/obj.T^2*(t-obj.T/2)^2+4*(lmax-l0)/obj.T*(t-obj.T/2)+l0+deltaLx,.7,.96));
                        dq21d=(8*(lmax-l0)/obj.T^2*(t-obj.T/2)-4*(lmax-l0)/obj.T)/abs(sin(q21d));
                    end
                    
                    
                    
                    [q21d,dq21d]=spline3(t-(.5+0*.1*obj.dx_des)*obj.T,(.5-0*.1*obj.dx_des)*obj.T,acos(l0sw),0,acos(l0+deltaLx),0);
                    
                    
                    uT=clamp(ddqTc+k_torso*kdT*(dqTc-dpitch)+k_torso*kpT*(qTc-pitch),-.7*fc*F_l,.7*fc*F_l);%+(u3+u4)/n/IT;
                    uT2=clamp(ddqTc+k_torso*kdT*(dqTc-dpitch)+k_torso*kpT*(qTc-pitch),-.7*fc2*F_l2,.7*fc2*F_l2);%+(u3+u4)/n/IT;
                    
                    
                    q22d=-uT/obj.k+(theta1+theta2)/2;
                    dq22d=(dtheta1+dtheta2)/2;
                    
                    
                    Tsw=obj.T;
                    
                    if ctrltest==2
                        T1=40*Tsw/100;
                        T1=30*Tsw/100;
                        T2=80*Tsw/100;
                        T2=(80)*Tsw/100;
                        T3=60*Tsw/100;
                        T3=75/100*Tsw;  % smaller T3 make it hop!
                        T4=90*Tsw/100;
                        T4=(100/100-0*8*lpo)*Tsw;
                    else
                        T1=1*Tsw/100;
                        T4=81*Tsw/100;
                        T2=70*Tsw/100;
                        T3=80*Tsw/100;
                    end
                    
                    
                    
                    
                    [x_f2f_t,dx_f2f_t] = spline3(t, T2, obj.f2f0, 0, x_f2f_d, 0);
                    
                                     
                    
                    
                    if t<=T1
                        [q23d,dq23d]=spline3(t,T1,obj.q23d0,0,acos(lmin),0);
                        %             [q23d,dq23d]=spline3(t,T1,acos(l0),0,acos(lmin),0);
                        %             q24d=x_fpt(1)/cos(q23d);
                        %             dq24d=0;
                        
                    elseif t<=T3
                        q23d=acos(lmin);
                        dq23d=0;
                        %             q24d=spline3(t,T3,q240,0,acos(clamp(x_fp(1)/cos(q23d),-1,1)),0);
                        %             dq24d=0;
                    elseif t<=T4
                        [q23d,dq23d]=spline3(t-T3,T4-T3,acos(lmin),0,acos(l0sw),0);
                        %             q24d=spline3(t,T3,q240,0,acos(clamp(x_fp(1)/cos(q23d),-1,1)),0);
                        %             dq24d=0;
                        dq23d=0;
                    else
                        q23d=acos(l0sw);
                        dq23d=0;
                        %             q24d=acos(clamp(x_fpf(1)/l0,-1,1));
                        %             dq24d=0;
                        
                    end
                    
                    %         dq23d=0;
                    
                    %         if t<.7*Tsw %((T3+T4)/2)
                    obj.y_fp = x_fp(2);
                    %         end
                    
                    
                    %         q24d=acos(clamp((x_f2f_t+(cos(theta2)+cos(theta1))/2)/(l0+deltaLx),-1,1));
                    
                    
                    if ll1>cos(q23d)
                        q24d=acos(clamp((x_f2f_t+(cos(theta2)+cos(theta1))/2)/ll1,-1,1));
                        dq24d=-((dx_f2f_t-(sin(theta2)*dtheta2+sin(theta1)*dtheta1)/2))/ll1/sqrt(1-cos(q24d)^2);
                    else
                        q24d=acos(clamp((x_f2f_t+(cos(theta2)+cos(theta1))/2)/cos(q23d),-1,1));
                        dq24d=-((dx_f2f_t-(sin(theta2)*dtheta2+sin(theta1)*dtheta1)/2)*cos(q23d)+sin(q23d)*dq23d*(x_f2f_t+(cos(theta2)+cos(theta1))/2))/cos(q23d)^2/sqrt(1-cos(q24d)^2);
                    end
                    if cos(q24d)==1 || cos(q24d)==-1
                        dq24d=0;
                    end
                    %         dq24d=0;
                    %         if ctrltest==3
                    %             c_h=clamp(2*t/T,0,1);
                    %         else
                    
                    %         end
                    
                    u([2 1 5 4])=obj.kp_leg*[2^.5+.3*abs(obj.dx_des)*sin(pi/obj.T*t) 2^.5+.3*abs(obj.dx_des)*sin(pi/obj.T*t) 1/2^.5 1/2^.5].*...
                        ([1 1 1 1].*([(q22d-q21d) (q22d+q21d) q24d-q23d-fc2*uT2/obj.k q24d+q23d-fc2*uT2/obj.k]-[thetam1 thetam2 thetam3 thetam4])+...
                        0*(1-c_h)*(1-fc_h)*[obj.q22d0-q21d-thetam1 obj.q22d0+q21d-thetam2 0 0])+...
                        obj.kd_leg*[2^.5 2^.5 1/2^.5 1/2^.5].*([dq22d-dq21d dq22d+dq21d dq24d-dq23d dq24d+dq23d]-[dthetam1 dthetam2 dthetam3 dthetam4]);
                    
                    if obj.RefLeg==obj.LEFT
                        u([2 1 5 4])=u([5 4 2 1]);
                    end
                    
                    %         u([2 1])= [0 0]';
                    
                    %% HIP CONTROLLER =====================================================
                    
                    % Hip target position to counteract boom rotation
                    
                    
                    t_s_i=10*deltaLx*obj.T;
                    
                    if (t-obj.sampleInterval)<t_s_i && t>=t_s_i
                        if obj.RefLeg==obj.RIGHT
                            obj.q_hip_d_s=q(10);
                        else
                            obj.q_hip_d_s=q(9);
                        end
                    end
                    
                    
                    obj.q_hip_d_s=0*.03*(-1)^obj.RefLeg;
                    
                    %        y_fp=sign(y_fp)*clamp(abs(y_fp),min(abs(y_fp2),abs(yss)),.6);
%                     obj.y_fp=spline3(t-15*lpo*obj.T, T2+4*lpo*obj.T, -obj.y_fp2, 0, obj.y_fp, 0);
                    %     if ctrltest==4
                    if ll1>cos(q23d)
                        q_hip_d=clamp(asin(clamp(obj.y_fp/sqrt(ll1^2+obj.d^2),-1,1))...
                            +atan2(obj.d,ll1)*(-1)^obj.RefLeg-roll,.13*(-1)^obj.RefLeg,-.28*(-1)^obj.RefLeg);
                        qh1d0=clamp(asin(clamp(obj.y_fp2/sqrt(ll1^2+obj.d^2),-1,1))...
                            -atan2(obj.d,ll1)*(-1)^obj.RefLeg-roll,-.13*(-1)^obj.RefLeg,.28*(-1)^obj.RefLeg);
                        dq_hip_d=0;
                    else
                        q_hip_d=clamp(asin(clamp(obj.y_fp/sqrt(cos(q23d)^2+obj.d^2),-1,1))...
                            +atan2(obj.d,cos(q23d))*(-1)^obj.RefLeg-roll,.13*(-1)^obj.RefLeg,-.28*(-1)^obj.RefLeg);
                        qh1d0=clamp(asin(clamp(obj.y_fp2/sqrt(cos(q23d)^2+obj.d^2),-1,1))...
                            -atan2(obj.d,cos(q23d))*(-1)^obj.RefLeg-roll,-.13*(-1)^obj.RefLeg,.28*(-1)^obj.RefLeg);
                        dq_hip_d=obj.y_fp*cos(q23d)*sin(q23d)*dq23d*(cos(q23d)^2+obj.d^2)^(-3/2)/...
                            sqrt(1-obj.y_fp^2/(cos(q23d)^2+obj.d^2))+...
                            obj.d*sin(q23d)*dq23d/(cos(q23d)^2+obj.d^2);
                    end
                    
                   q_hip_d=clamp((1-c_h_p)*(0*roll+0*.3*dyaw_a*(1)^obj.RefLeg)+c_h_p*spline3(t-t_s_i, obj.T-0*t_s_i, obj.q_hip_d_s, 0, q_hip_d, 0),.13*(-1)^obj.RefLeg,-.28*(-1)^obj.RefLeg); 
                    
                    % Hip feedforward torque for gravity compensation
                    u0_hip = 0*35;
                    u0_hip_sw=0*35;
                    
                    %         fc_h=fc;
                    
                    
                    uhm=min(600,3/abs(cos(theta)));
                    
                    if obj.RefLeg==obj.RIGHT
%                         u0_hip=0*obj.d*obj.m_leg*9.8*cos(q(9));
%                         u0_hip_sw=0*obj.d*obj.m_leg*9.8*cos(q(10));
                        s1=normpdf(pi/2-theta,0,.03)*sqrt(2*pi*.03^2);
                        s2=normpdf(pi/2-theta_sw,0,.03)*sqrt(2*pi*.03^2);
                        s1=1;
                        s2=1;%sin(pi*t/obj.T);
                        u30=clamp( (-u0_hip +obj.kp_hip*(k_torso*fc*(roll-roll_d))+...
                            obj.kd_hip*(k_torso*fc*droll)),-.7*fc*F_l-0*u0_hip,.7*fc*F_l+0*u0_hip);
%                         u30=clamp(u30,-s1*100,s1*100);

                        u(3)=s2*u30...
                            +obj.k_hop*(1-c_h_p)*(1-fc_h)*(obj.kp_hip*(qh1d0-q(9))+obj.kd_hip*(-dq(9)));
                        if  t>=t_s_i
                            u(6)=(1-c_h)*u0_hip_sw +((q_hip_d-q(10))*(1-fc2)*c_h_p+s2*k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                                ((dq_hip_d - dq(10))*(1-fc2)*c_h_d+s2*k_torso*fc2*droll)*obj.kd_hip;
                        else
                            
                            if (dq_hip_d>0 && q(10)>.1) || (dq_hip_d<0 && q(10)<-.1)
                                dq_hip_d=0;
                            else
                                dq_hip_d=clamp(4*droll,-4,4);
                            end
                            
                            u(6)=(1-c_h)*u0_hip_sw +((-.03-q(10))*(1-fc2)*c_h_p+s2*k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                                ((dq_hip_d - dq(10))*(1-fc)*(1-fc2)*c_h_d+s2*k_torso*fc2*droll)*obj.kd_hip;
                        end

                        
                        e_h=q_hip_d-q(10);
                        deltaq1=(qh1d0-q(9));
                        
%                         u(3)=u(3)*((obj.ti<4.18)||(obj.ti>4.35));
%                         u(3)=clamp(u(3),-uhm,uhm);
                    else
%                         u0_hip=0*obj.d*obj.m_leg*9.8*cos(q(10));
%                         u0_hip_sw=0*obj.d*obj.m_leg*9.8*cos(q(9));
                        s1=normpdf(pi/2-theta,0,.03)*sqrt(2*pi*.03^2);
                        s2=normpdf(pi/2-theta_sw,0,.03)*sqrt(2*pi*.03^2);
                        s1=1;
                        s2=1;%sin(pi*t/obj.T);
                        u60=clamp( (u0_hip  + obj.kp_hip*(k_torso*fc*(roll-roll_d))+ obj.kd_hip*(k_torso*fc*droll)),-.7*fc*F_l-0*u0_hip,.7*fc*F_l+0*u0_hip);
%                         u60=clamp(u60,-s1*100,s1*100);
                        u(6)=s2*u60...
                            +obj.k_hop*(1-c_h_p)*(1-fc_h)*(obj.kp_hip*(qh1d0-q(10))+obj.kd_hip*(-dq(10)));
                        if  t>=t_s_i
                            u(3)=-(1-c_h)*u0_hip_sw +((q_hip_d-q(9))*(1-fc2)*c_h_p+s2*k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                                ((dq_hip_d - dq(9))*(1-fc2)*c_h_d+s2*k_torso*fc2*droll)*obj.kd_hip;
                        else
                            
                            if (dq_hip_d>0 && q(10)>.1) || (dq_hip_d<0 && q(10)<-.1)
                                dq_hip_d=0;
                            else
                                dq_hip_d=clamp(4*droll,-4,4);
                            end
                            
                            u(3)=-(1-c_h)*u0_hip_sw + ((.03-q(9))*(1-fc2)*c_h_p+s2*k_torso*fc2*(roll-roll_d))*obj.kp_hip +...
                                ((dq_hip_d - dq(10))*(1-fc)*(1-fc2)*c_h_d+s2*k_torso*fc2*droll)*obj.kd_hip;
                            
                        end
                        
                        
                        e_h=q_hip_d-q(9);
                        deltaq1=(qh1d0-q(9));
                        
%                          u(3)=u(3)*((obj.ti<4.0)||(obj.ti>4.18));
                        
%                         u(6)=clamp(u(6),-uhm,uhm)*((obj.ti<6.45)||(obj.ti>6.61));
%                         u(6)=u(6)*((obj.ti<6.45)||(obj.ti>6.61));
                    end
                                       
                    
                    %%
                    
                    
                otherwise % RELAX -----------------------------------------------------
                    % Leg actuator torques computed to behave like virtual dampers
                    u([2 1 5 4]) = (0 - dq([4 2 8 6]))*obj.kd_leg;
                    u([3 6]) = (0 - dq([9 10]))*obj.kd_hip;
            end % switch
            
            % Limit absolute torque commands
            u = clamp(u, -obj.u_lim, obj.u_lim);
            
            
            obj.out=[obj.x obj.y vG(1:2)' q21d  q22d  q23d ...  %1-10
                q24d q_hip_d x_f2f_t obj.y_fp fc t lpo ...          %11-19
                obj.RefLeg obj.dx obj.dy e_h deltaq1 qh1d0... %20-27
                obj.dx_des obj.dy_des ]; %obj.runTime] %
            
            if (obj.ti-obj.t0)>=obj.T || ((obj.ti-obj.t0)>obj.T*(1/2+1/2+0*clamp(20*lpo,0,0.5)) && RefLegF2~=obj.RefLeg)
                
                obj.Tcp=obj.ti-obj.t0;
                obj.t0=obj.ti;
                obj.f2f0=-1/2*(cos(theta4)+cos(theta3)-cos(theta2)-cos(theta1));
                if obj.RefLeg==obj.RIGHT
                    obj.RefLeg=obj.LEFT;
                else
                    obj.RefLeg=obj.RIGHT;
                end
               
                obj.y_fp2=obj.y_fp;
                obj.q22d0=q24d;
                
                obj.q23d0=q21d;
               
                obj.xcp=obj.xc;
                obj.ycp=obj.yc;
                obj.xc=0;
                obj.yc=0;
                
                obj.yaw_c=0;
            end
            
           
        end % userStep
        
        
        function parsePS3Controller(obj)
            %PARSEPS3CONTROLLER
            
            % Parse gait specific tweaks
            if obj.ps3.cross.isPressed
                obj.gaitMode = GaitMode_S.Normal;
            elseif obj.ps3.circle.isPressed
                obj.gaitMode = GaitMode_S.SemiDyn;
            elseif obj.ps3.triangle.isPressed
                obj.gaitMode = GaitMode_S.Dynamic;
            elseif obj.ps3.square.isPressed
                obj.gaitMode = GaitMode_S.Hop;
            end % if
            
            switch obj.gaitMode
                case GaitMode_S.SemiDyn
%                     obj.l0_leg = 0.93;
%                     obj.l_ret = 0.12;
                    t_c = 1.5; dx_max = 1; dy_max = 0.2; obj.lpo=0;
                case GaitMode_S.Dynamic
                    t_c=.5; dx_max=1;  dy_max = 0.2; obj.lpo=0;
                    
                case GaitMode_S.Hop
                    dx_max=.1; dy_max=.1; t_c=.5;
% 		    obj.trimIncrement=obj.trimIncrement/2;

                otherwise % GaitMode_S.Normal
%                     obj.l0_leg = 0.91;
%                     obj.l_ret = 0.2;
                    t_c = 3; dx_max = 0.75; dy_max = 0.2; obj.lpo=0;
            end % switch
            
            
            if obj.ps3.l1.isPressed && obj.gaitMode==GaitMode_S.Hop
                obj.lpo=obj.lpo + .005;
            end
            if obj.ps3.l2.isPressed && obj.gaitMode==GaitMode_S.Hop
                obj.lpo=obj.lpo - .005;
            end
            
            
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
            
            
            
            
%             if obj.gaitMode == GaitMode_S.Hop
%                 dx_cmd=0;
%                 dy_cmd=0;
%             end
%             
                
            % Simulation overrides    
            if obj.isSim
                t_c=.5+.5*(obj.ti>3)+0*(obj.ti>8);
                dx_cmd = -0*1*(obj.ti>2)-0*1*(obj.ti>3)+0*(obj.ti>8); dy_cmd = 0*(obj.ti>2);
                
                % dx_cmd = 1.5*round(sin(obj.runTime*2*pi/15));
                % dx_cmd = 0.25*(floor(obj.runTime/5));
                % dx_cmd = 1.5*(obj.runTime > 2);
            end % if
            
            % Compute smoothing factor
            alpha_x = obj.sampleInterval/(t_c + obj.sampleInterval);
            alpha_y = obj.sampleInterval/(t_c/3 + obj.sampleInterval);
            
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
end
end


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
end
end


function s = scaleFactor(f, tl, tu)
%SCALEFACTOR Compute scalar (0 to 1) representing forces in leg.

s = (clamp(f, tl, tu) - tl)/(tu - tl);
end % scaleFactor
