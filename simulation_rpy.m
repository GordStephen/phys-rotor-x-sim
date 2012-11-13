 function simulation_rpy
clear;

% ===== Physical Properties ==========

    %  === World Properties ======

        is_drag = 0; % Enable or disable air resistance

    %  === Copter Properties ======

        quad_radius = .25; % Radius of quadrotor [m]

        I_p = 1; % Rotational moments of inertia [kg*m^2]
        I_r = 1;
        I_y = 1;

        b_p = 0.5*is_drag; % Aero drag factors [N*s^2/m^2]
        b_r = 0.5*is_drag;
        b_y = 0.5*is_drag;

        cmd2thrust = @(cmds) cmds; % Function mapping 8-bit ESC input to generated motor thrusts [N]

        cmd2torque = @(cmds) cmds; % Function mapping 8-bit ESC input to generated motor torque [N*m]  
        

% ===== Simulation Setup ==========

    int_error = zeros(1,3);
       
    %  === Static Target Setup ======

        target = [0 0 0]; % Define target positions (rpy xyz)
            
        dcm_target = angle2dcm(target(1),target(2),target(3),'XYZ'); % Calculate target position representation
        right_target = dcm_target*[-quad_radius;0;0]; % Rotate target quadrotor points with target DCM
        left_target = dcm_target*[quad_radius;0;0];
        nose_target = dcm_target*[0;quad_radius;0]; 
        tail_target = dcm_target*[0;-quad_radius;0];

    %  === Initial Conditions Setup ======

        t_start = tic; % Starts simulation clock
        t = toc(t_start);
        
        thrusts = zeros(4,1); % Define initial thrusts and torques
        torques = zeros(3,1);

        ICs = [ % Define initial kinematic conditions
            pi/4 pi/2 pi/6; % pry xyz position
            0 0 0; % pry xyz velocity
            0 0 0
            ];

        current = ICs;

    % ===== Plots Setup ==========
    
        fig1 = figure(1);
        
        subplot(3,2,[1 3 5]);
        axis(.3*[-1 1 -1 1 -1 1]);
        axis equal;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        line([nose_target(1),tail_target(1)],[nose_target(2),tail_target(2)],[nose_target(3),tail_target(3)],'Color','blue','LineStyle',':');
        line ([right_target(1),left_target(1)],[right_target(2),left_target(2)],[right_target(3),left_target(3)],'Color','green','LineStyle',':');
        line([nose_target(1),tail_target(1)],[nose_target(2),tail_target(2)],[0,0],'Color','black','LineStyle',':');
        line ([right_target(1),left_target(1)],[right_target(2),left_target(2)],[0,0],'Color','black','LineStyle',':');
        frame_nose_tail_line = line(nan, nan); set(frame_nose_tail_line,'Color','blue'); 
        frame_right_left_line = line(nan, nan); set(frame_right_left_line,'Color','green');
        shadow_nose_tail_line = line(nan, nan); set(shadow_nose_tail_line,'Color','black','LineStyle','-'); 
        shadow_right_left_line = line(nan, nan); set(shadow_right_left_line,'Color','black','LineStyle','-'); 
        thrust_nose_line = line(nan, nan); set(thrust_nose_line,'Color','red','LineStyle','-');
        thrust_tail_line = line(nan, nan); set(thrust_tail_line,'Color','red','LineStyle','-');
        thrust_right_line = line(nan, nan); set(thrust_right_line,'Color','red','LineStyle','-');
        thrust_left_line = line(nan, nan); set(thrust_left_line,'Color','red','LineStyle','-');

        subplot(3,2,2); 
        title('Pitch (degrees)');
        pitch_line = line(nan, nan); set(pitch_line,'Color','blue'); 
        pitch_dot = line(nan, nan); set(pitch_dot,'Color','green','LineStyle','--'); 
        pitch_dotdot = line(nan, nan); set(pitch_dotdot,'Color','yellow','LineStyle',':');  
        pitch_target = line(nan, nan); set(pitch_target,'Color','black','LineStyle','-.');

        subplot(3,2,4);
        title('Roll (degrees)');
        roll_line = line(nan, nan); set(roll_line,'Color','blue');
        roll_dot = line(nan, nan); set(roll_dot,'Color','green','LineStyle','--'); 
        roll_dotdot = line(nan, nan); set(roll_dotdot,'Color','yellow','LineStyle',':');  
        roll_target = line(nan, nan); set(roll_target,'Color','black','LineStyle','-.');
        
        subplot(3,2,6);
        title('Yaw (degrees)');
        yaw_line = line(nan, nan); set(yaw_line,'Color','blue');
        yaw_dot = line(nan, nan); set(yaw_dot,'Color','green','LineStyle','--');
        yaw_dotdot = line(nan, nan); set(yaw_dotdot,'Color','yellow','LineStyle',':');
        yaw_target = line(nan, nan); set(yaw_target,'Color','black','LineStyle','-.');
        
        function updateplots(current)

            theta = current(1,1:3);
            theta_dot = current(2,1:3);
            theta_dotdot = current(3,1:3);
            
            dcm = angle2dcm(theta(1),theta(2),theta(3),'XYZ'); % Convert calculated ypr angles to directional cosine matrix (DCM)

            nose = dcm*[0;quad_radius;0];  % Rotate + translate quadrotor points with DCM
            tail = dcm*[0;-quad_radius;0];
            right = dcm*[quad_radius;0;0];
            left = dcm*[-quad_radius;0;0];

            nose_thrust = dcm*[0;quad_radius;-thrusts(1)]; % Rotate thrust vector representations with DCM
            tail_thrust = dcm*[0;-quad_radius;-thrusts(2)];
            right_thrust = dcm*[quad_radius;0;-thrusts(3)]; 
            left_thrust = dcm*[-quad_radius;0;-thrusts(4)];

            subplot(3,2,[1 3 5]); % Plot 3D visualization
            title(['t=',num2str(t),'s']);
            set(frame_nose_tail_line,'XData',[nose(1),tail(1)],'YData',[nose(2),tail(2)],'ZData',[nose(3),tail(3)]);
            set(frame_right_left_line,'XData',[right(1),left(1)],'YData',[right(2),left(2)],'ZData',[right(3),left(3)]);
            set(shadow_nose_tail_line,'XData', [nose(1),tail(1)],'YData',[nose(2),tail(2)],'ZData',[-1.2*quad_radius,-1.2*quad_radius]);
            set(shadow_right_left_line,'XData',[right(1),left(1)],'YData',[right(2),left(2)],'ZData',[-1.2*quad_radius,-1.2*quad_radius]);
            set(thrust_nose_line, 'XData',[nose(1),nose_thrust(1)],'YData',[nose(2),nose_thrust(2)],'ZData',[nose(3),nose_thrust(3)]);
            set(thrust_tail_line,'XData',[tail(1),tail_thrust(1)],'YData',[tail(2),tail_thrust(2)],'ZData',[tail(3),tail_thrust(3)]);
            set(thrust_right_line,'XData',[right(1),right_thrust(1)],'YData',[right(2),right_thrust(2)],'ZData',[right(3),right_thrust(3)]);
            set(thrust_left_line, 'XData',[left(1),left_thrust(1)],'YData',[left(2),left_thrust(2)],'ZData',[left(3),left_thrust(3)]);          

            
            subplot(3,2,2); % Plot pitch vs time
            T = get(pitch_line, 'XData');    T = [T t];

            Theta = get(pitch_line, 'YData');    
            Theta = [Theta theta(1)];
            set(pitch_line, 'XData', T, 'YData', Theta);

            % [Uncomment to plot pitch angular velocity]
            %ThetaDot = get(pitch_dot, 'YData');
            %ThetaDot = [ThetaDot theta_dot(1)];
            %set(pitch_dot, 'XData', T, 'YData', ThetaDot);

            % [Uncomment to plot pitch angular acceleration]
            %ThetaDotDot = get(pitch_dotdot, 'YData');
            %ThetaDotDot = [ThetaDotDot theta_dotdot(1)];
            %set(pitch_dotdot, 'XData', T, 'YData', ThetaDotDot);

            ThetaTarget = get(pitch_target, 'YData');
            ThetaTarget = [ThetaTarget target(1)];
            set(pitch_target, 'XData', T, 'YData', ThetaTarget);


            subplot(3,2,4); % Plot roll vs time
            T = get(roll_line, 'XData');    T = [T t];
            Theta = get(roll_line, 'YData');    Theta = [Theta theta(2)];
            set(roll_line, 'XData', T, 'YData', Theta);

            % [Uncomment to plot roll angular velocity]
            %ThetaDot = get(roll_dot, 'YData');    
            %ThetaDot = [ThetaDot theta_dot(2)];
            %set(roll_dot, 'XData', T, 'YData', ThetaDot);

            % [Uncomment to plot roll angular acceleration]
            %ThetaDotDot = get(roll_dotdot, 'YData');
            %ThetaDotDot = [ThetaDotDot theta_dotdot(2)];
            %set(roll_dotdot, 'XData', T, 'YData', ThetaDotDot);

            ThetaTarget = get(roll_target, 'YData');
            ThetaTarget = [ThetaTarget target(2)];
            set(roll_target, 'XData', T, 'YData', ThetaTarget);


            subplot(3,2,6); % Plot yaw vs time
            T = get(yaw_line, 'XData');    T = [T t];
            Theta = get(yaw_line, 'YData');    Theta = [Theta theta(3)];
            set(yaw_line, 'XData', T, 'YData', Theta)

            % [Uncomment to plot yaw angular velocity]
            %ThetaDot = get(yaw_dot, 'YData');
            %ThetaDot = [ThetaDot theta_dot(3)];
            %set(yaw_dot, 'XData', T, 'YData', ThetaDot);

             % [Uncomment to plot yaw angular acceleration]
            %ThetaDotDot = get(yaw_dotdot, 'YData');
            %ThetaDotDot = [ThetaDotDot theta_dotdot(3)];
            %set(yaw_dotdot, 'XData', T, 'YData', ThetaDotDot);

            ThetaTarget = get(yaw_target, 'YData');
            ThetaTarget = [ThetaTarget target(3)];
            set(yaw_target, 'XData', T, 'YData', ThetaTarget); 
            
        end %updateplots

        
% ===== Main Loop ==========

while (ishandle(fig1))
    
    t_prev = t; % Determine current time and timestep
    t = toc(t_start); 
    dt = t-t_prev;
    
    commands = controller(current);
    %commands = commands + signal_noise; % signal_noise could be added here
    
    current = plant(commands,current);
    %current = current + sensor_noise;  % sensor_noise could be added here
    
    updateplots(current);
    pause(0.02);
    
end %while

% ===== Subsystems ==========

function commands = controller(current)

    % PID Parameter Setup
    kp = 5*[4 4 1];
    ki = 1.25*[4 4 1];
    kd = 5*[4 4 1];

    % Error Calculation
    error = target - current(1,:);
    d_error = -current(2,:);
    int_error = int_error + error*dt;    
    
    % PID Control
    corrections = kp.*error + ki.*int_error + kd.*d_error;

    % Generate motor commands from PID corrections
    commands = corr2command(corrections);

    function out = corr2command(corrections)
        nose_thrust = 0.5*corrections(1) + 0.25*corrections(3);
        nose_angle = 0;

        tail_thrust = -0.5*corrections(1) + 0.25*corrections(3);
        tail_angle = 0;

        right_thrust = -0.5*corrections(2) - 0.25*corrections(3);
        right_angle = 0;

        left_thrust = 0.5*corrections(2) - 0.25*corrections(3);
        left_angle = 0;

        out = [
            nose_thrust nose_angle;
            tail_thrust tail_angle;
            right_thrust right_angle;
            left_thrust left_angle
            ];

    end %function corr2command
    
end

function current = plant(commands, prev)

    %Unpack previous physical data
    theta_prev = prev(1,1:3);
    theta_dot_prev = prev(2,1:3);
    theta_dotdot_prev = prev(3,1:3);
      
    torques_prev = torques;
    torques(3) = cmd2torque(commands(1,1))*cos(commands(1,2)) + cmd2torque(commands(2,1))*cos(commands(2,2)) - cmd2torque(commands(3,1))*cos(commands(3,2)) - cmd2torque(commands(4,1))*cos(commands(4,2)); % Determine net system torques resulting from commands
    T_p_dot = (torques(1) - torques_prev(1))/dt;
    T_r_dot = (torques(2) - torques_prev(2))/dt;
    T_y_dot = (torques(3) - torques_prev(3))/dt;
    
    thrusts_prev = thrusts;
    thrusts = cmd2thrust(commands); % Determine motor thrusts generated from commands
    thr_nose_dot = (thrusts(1) - thrusts_prev(1))/dt;
    thr_tail_dot = (thrusts(2) - thrusts_prev(2))/dt;
    thr_right_dot = (thrusts(3) - thrusts_prev(3))/dt;
    thr_left_dot = (thrusts(4) - thrusts_prev(4))/dt;

    theta_dotdot(1) = theta_dotdot_prev(1) + T_p_dot/I_p*dt + (thr_nose_dot - thr_tail_dot)*quad_radius/I_p*dt - b_p*theta_dot_prev(1)^2*sign(theta_dot_prev(1))/I_p; % Aero drag can be incorporated here at some point
    theta_dotdot(2) = theta_dotdot_prev(2) + T_r_dot/I_r*dt + (thr_left_dot - thr_right_dot)*quad_radius/I_r*dt - b_r*theta_dot_prev(2)^2*sign(theta_dot_prev(2))/I_r;
    theta_dotdot(3) = theta_dotdot_prev(3) + T_y_dot/I_y*dt - (b_y*theta_dot_prev(3)^2*sign(theta_dot_prev(3)))/I_y;
    
    theta_dot = theta_dot_prev + theta_dotdot*dt;
    theta = theta_prev + theta_dot*dt + 0.5*theta_dotdot*dt^2;    

    current = [ % Repack current data
        theta;
        theta_dot;
        theta_dotdot;
        ];

end

end %SYSTEM     


