 function SYSTEM
clear;

% ===== Physical Properties ==========

    %  === World Properties ======

        g = 9.81; % Field strength;
        is_drag = 0; % Enable or disable air resistance
        is_grav = 1; % Enable or disable gravity

    %  === Copter Properties ======

        quad_radius = 1; % Radius of quadrotor [m]
        quad_mass = 1.3; % Mass of quadrotor [kg]

        I_p = 1; % Rotational moments of inertia [kg*m^2]
        I_r = 1;
        I_y = 1;

        b_p = 0.5*is_drag; % Aero drag factors [N*s^2/m^2]
        b_r = 0.5*is_drag;
        b_y = 0.5*is_drag;

        cmd2thrust = @(cmds) cmds; % Function mapping 8-bit ESC input to generated motor thrusts [N]

        cmd2torque = @(cmds) cmds; % Function mapping 8-bit ESC input to generated motor torque [N*m]  
        

% ===== Simulation Setup ==========

    int_error = zeros(1,6);
       
    %  === Static Target Setup ======

        target = [0 0 0 -1 0 0]; % Define target positions (rpy xyz)
            
        dcm_target = angle2dcm(target(1),target(2),target(3),'XYZ'); % Calculate target position representation
        right_target = dcm_target*[-quad_radius;0;0] + target(4:6)'; % Rotate target quadrotor points with target DCM
        left_target = dcm_target*[quad_radius;0;0] + target(4:6)';
        nose_target = dcm_target*[0;quad_radius;0] + target(4:6)'; 
        tail_target = dcm_target*[0;-quad_radius;0] + target(4:6)';

    %  === Initial Conditions Setup ======

        t_start = tic; % Starts simulation clock
        t = toc(t_start);
        
        thrusts = zeros(4,1); % Define initial thrusts and torques
        torques = zeros(3,1);

        ICs = [ % Define initial kinematic conditions
            0 0 0 0 0 0; % pry xyz position
            0 0 0 0 0 0; % pry xyz velocity
            0 0 0 0 0 -quad_mass*g*is_grav % pry xyz acceleration - nonzero ICs here mess up acceleration calculations in the plant - keep these at zero (a_z = mg is a good offset)
            ];

        current = ICs;

    % ===== Plots Setup ==========
    
        fig1 = figure(1);
        
        subplot(6,2,[1 3 5 7 9 11]);
        axis(2*[-1 1 -1 1 0 2]);
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

        subplot(6,2,2); 
        title('Pitch (degrees)');
        pitch_line = line(nan, nan); set(pitch_line,'Color','blue'); 
        pitch_dot = line(nan, nan); set(pitch_dot,'Color','green','LineStyle','--'); 
        pitch_dotdot = line(nan, nan); set(pitch_dotdot,'Color','yellow','LineStyle',':');  
        pitch_target = line(nan, nan); set(pitch_target,'Color','black','LineStyle','-.');

        subplot(6,2,4);
        title('Roll (degrees)');
        roll_line = line(nan, nan); set(roll_line,'Color','blue');
        roll_dot = line(nan, nan); set(roll_dot,'Color','green','LineStyle','--'); 
        roll_dotdot = line(nan, nan); set(roll_dotdot,'Color','yellow','LineStyle',':');  
        roll_target = line(nan, nan); set(roll_target,'Color','black','LineStyle','-.');
        
        subplot(6,2,6);
        title('Yaw (degrees)');
        yaw_line = line(nan, nan); set(yaw_line,'Color','blue');
        yaw_dot = line(nan, nan); set(yaw_dot,'Color','green','LineStyle','--');
        yaw_dotdot = line(nan, nan); set(yaw_dotdot,'Color','yellow','LineStyle',':');
        yaw_target = line(nan, nan); set(yaw_target,'Color','black','LineStyle','-.');
        
        subplot(6,2,8); 
        title('X (metres)');
        x_line = line(nan, nan); set(x_line,'Color','blue'); 
        x_dot = line(nan, nan); set(x_dot,'Color','green','LineStyle','--'); 
        x_dotdot = line(nan, nan); set(x_dotdot,'Color','yellow','LineStyle',':');  
        x_target = line(nan, nan); set(x_target,'Color','black','LineStyle','-.');

        subplot(6,2,10);
        title('Y (metres)');
        y_line = line(nan, nan); set(y_line,'Color','blue');
        y_dot = line(nan, nan); set(y_dot,'Color','green','LineStyle','--'); 
        y_dotdot = line(nan, nan); set(y_dotdot,'Color','yellow','LineStyle',':');  
        y_target = line(nan, nan); set(y_target,'Color','black','LineStyle','-.');
        
        subplot(6,2,12);
        title('Z (metres)');
        z_line = line(nan, nan); set(z_line,'Color','blue');
        z_dot = line(nan, nan); set(z_dot,'Color','green','LineStyle','--');
        z_dotdot = line(nan, nan); set(z_dotdot,'Color','yellow','LineStyle',':');
        z_target = line(nan, nan); set(z_target,'Color','black','LineStyle','-.');

        function updateplots(current)

            theta = current(1,1:3);
            theta_dot = current(2,1:3);
            theta_dotdot = current(3,1:3);
            
            position = current(1,4:6);
            velocity = current(2,4:6);
            acceleration = current(3,4:6);

            dcm = angle2dcm(theta(1),theta(2),theta(3),'XYZ'); % Convert calculated ypr angles to directional cosine matrix (DCM)

            nose = dcm*[0;quad_radius;0] + position';  % Rotate + translate quadrotor points with DCM
            tail = dcm*[0;-quad_radius;0]+ position';
            right = dcm*[quad_radius;0;0]+ position';
            left = dcm*[-quad_radius;0;0]+ position';

            nose_thrust = dcm*[0;quad_radius;-thrusts(1)] + position';
            tail_thrust = dcm*[0;-quad_radius;-thrusts(2)] + position';
            right_thrust = dcm*[quad_radius;0;-thrusts(3)] + position'; % Rotate thrust vector representations with DCM
            left_thrust = dcm*[-quad_radius;0;-thrusts(4)] + position';

            subplot(6,2,[1 3 5 7 9 11]); % Plot 3D visualization
            title(['t=',num2str(t),'s']);
            set(frame_nose_tail_line,'XData',[nose(1),tail(1)],'YData',[nose(2),tail(2)],'ZData',[nose(3),tail(3)]);
            set(frame_right_left_line,'XData',[right(1),left(1)],'YData',[right(2),left(2)],'ZData',[right(3),left(3)]);
            set(shadow_nose_tail_line,'XData', [nose(1),tail(1)],'YData',[nose(2),tail(2)],'ZData',[0,0]);
            set(shadow_right_left_line,'XData',[right(1),left(1)],'YData',[right(2),left(2)],'ZData',[0,0]);
            set(thrust_nose_line, 'XData',[nose(1),nose_thrust(1)],'YData',[nose(2),nose_thrust(2)],'ZData',[nose(3),nose_thrust(3)]);
            set(thrust_tail_line,'XData',[tail(1),tail_thrust(1)],'YData',[tail(2),tail_thrust(2)],'ZData',[tail(3),tail_thrust(3)]);
            set(thrust_right_line,'XData',[right(1),right_thrust(1)],'YData',[right(2),right_thrust(2)],'ZData',[right(3),right_thrust(3)]);
            set(thrust_left_line, 'XData',[left(1),left_thrust(1)],'YData',[left(2),left_thrust(2)],'ZData',[left(3),left_thrust(3)]);          

            
            subplot(6,2,2); % Plot ypr angles vs time
            T = get(pitch_line, 'XData');    T = [T t];

            Theta = get(pitch_line, 'YData');    Theta = [Theta theta(1)];
            set(pitch_line, 'XData', T, 'YData', Theta);

            ThetaDot = get(pitch_dot, 'YData');    ThetaDot = [ThetaDot theta_dot(1)];
            set(pitch_dot, 'XData', T, 'YData', ThetaDot);

            ThetaDotDot = get(pitch_dotdot, 'YData');    ThetaDotDot = [ThetaDotDot theta_dotdot(1)];
            %set(pitch_dotdot, 'XData', T, 'YData', ThetaDotDot);

            ThetaTarget = get(pitch_target, 'YData');    ThetaTarget = [ThetaTarget target(1)];
            set(pitch_target, 'XData', T, 'YData', ThetaTarget);


            subplot(6,2,4);
            T = get(roll_line, 'XData');    T = [T t];
            Theta = get(roll_line, 'YData');    Theta = [Theta theta(2)];
            set(roll_line, 'XData', T, 'YData', Theta);

            ThetaDot = get(roll_dot, 'YData');    ThetaDot = [ThetaDot theta_dot(2)];
            set(roll_dot, 'XData', T, 'YData', ThetaDot);

            ThetaDotDot = get(roll_dotdot, 'YData');    ThetaDotDot = [ThetaDotDot theta_dotdot(2)];
            %set(roll_dotdot, 'XData', T, 'YData', ThetaDotDot);

            ThetaTarget = get(roll_target, 'YData');    ThetaTarget = [ThetaTarget target(2)];
            set(roll_target, 'XData', T, 'YData', ThetaTarget);


            subplot(6,2,6);
            T = get(yaw_line, 'XData');    T = [T t];
            Theta = get(yaw_line, 'YData');    Theta = [Theta theta(3)];
            set(yaw_line, 'XData', T, 'YData', Theta)

            ThetaDot = get(yaw_dot, 'YData');    ThetaDot = [ThetaDot theta_dot(3)];
            set(yaw_dot, 'XData', T, 'YData', ThetaDot);

            ThetaDotDot = get(yaw_dotdot, 'YData');    ThetaDotDot = [ThetaDotDot theta_dotdot(3)];
            %set(yaw_dotdot, 'XData', T, 'YData', ThetaDotDot);

            ThetaTarget = get(yaw_target, 'YData');    ThetaTarget = [ThetaTarget target(3)];
            set(yaw_target, 'XData', T, 'YData', ThetaTarget); 
            
            
            subplot(6,2,8); % Plot ypr angles vs time
            T = get(x_line, 'XData');    T = [T t];

            X = get(x_line, 'YData');    X = [X position(1)];
            set(x_line, 'XData', T, 'YData', X);

            XDot = get(x_dot, 'YData');    XDot = [XDot velocity(1)];
            set(x_dot, 'XData', T, 'YData', XDot);

            XDotDot = get(x_dotdot, 'YData');    XDotDot = [XDotDot acceleration(1)];
            %set(x_dotdot, 'XData', T, 'YData', XDotDot);

            XTarget = get(x_target, 'YData');    XTarget = [XTarget target(4)];
            set(x_target, 'XData', T, 'YData', XTarget);


            subplot(6,2,10);
            T = get(y_line, 'XData');    T = [T t];
            Y = get(y_line, 'YData');    Y = [Y position(2)];
            set(y_line, 'XData', T, 'YData', Y);

            YDot = get(y_dot, 'YData');    YDot = [YDot velocity(2)];
            set(y_dot, 'XData', T, 'YData', YDot);

            YDotDot = get(y_dotdot, 'YData');    YDotDot = [YDotDot acceleration(2)];
            %set(y_dotdot, 'XData', T, 'YData', YDotDot);

            YTarget = get(y_target, 'YData');    YTarget = [YTarget target(5)];
            set(y_target, 'XData', T, 'YData', YTarget);


            subplot(6,2,12);
            T = get(z_line, 'XData');    T = [T t];
            Z = get(z_line, 'YData');    Z = [Z position(3)];
            set(z_line, 'XData', T, 'YData', Z)

            ZDot = get(z_dot, 'YData');    ZDot = [ZDot velocity(3)];
            set(z_dot, 'XData', T, 'YData', ZDot);

            ZDotDot = get(z_dotdot, 'YData');    ZDotDot = [ZDotDot acceleration(3)];
            %set(z_dotdot, 'XData', T, 'YData', ZDotDot);

            ZTarget = get(z_target, 'YData');    ZTarget = [ZTarget target(6)];
            set(z_target, 'XData', T, 'YData', ZTarget); 

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
    kp = [.20 20 5 0.1 .1 20];
    ki = [0*5 5 1.25 0 0 10];
    kd = [0*20 20 5 0 0 15];

    % Error Calculation
    error = target - current(1,:);
    d_error = -current(2,:);
    int_error = int_error + error*dt;
    
    % Adjust offset to move to target xy position
    
    offset = zeros(1,6);
    offset(1) = cutoff(-pi/4,pi/4,kp(5)*error(5) + ki(5)*int_error(5) + kd(5)*d_error(5)); % Move in x-axis - tilt up to 45 degrees
    offset(2) = cutoff(-pi/4,pi/4,kp(4)*error(4) + ki(4)*int_error(4) + kd(4)*d_error(4)); % Move in y-axis - tilt up to 45 degrees
        
    error = target - offset - current(1,:);
    int_error = int_error + error*dt;
    
    % PID Control
    corrections = kp.*error + ki.*int_error + kd.*d_error;

    % Generate motor commands from PID corrections
    commands = corr2command(corrections);

    function out = corr2command(corrections) % This is FAR from finished! Need to determine what corrections represent exactly & decide how to tranfer into commands for thrust and angle 
        nose_thrust = 0.5*corrections(1) + 0.25*corrections(3) + 0.25*corrections(6);
        nose_angle = 0;

        tail_thrust = -0.5*corrections(1) + 0.25*corrections(3) + 0.25*corrections(6);
        tail_angle = 0;

        right_thrust = -0.5*corrections(2) - 0.25*corrections(3) + 0.25*corrections(6);
        right_angle = 0;

        left_thrust = 0.5*corrections(2) - 0.25*corrections(3) + 0.25*corrections(6);
        left_angle = 0;

        out = [
            nose_thrust nose_angle;
            tail_thrust tail_angle;
            right_thrust right_angle;
            left_thrust left_angle
            ];

        %out = uint8(out);
    end %function corr2command

    function out = cutoff(min, max, val)
        if (val < min)
            out = min;
        elseif (val > max)
             out = max;
        else
            out = val;            
        end %if
    end % cutoff
    
end

function current = plant(commands, prev)

    %Unpack previous physical data
    theta_prev = prev(1,1:3);
    theta_dot_prev = prev(2,1:3);
    theta_dotdot_prev = prev(3,1:3);
    
    position_prev = prev(1,4:6);
    velocity_prev = prev(2,4:6);
    acceleration_prev = prev(3,4:6);
    
    torques_prev = torques;
    torques(1) = cmd2torque(commands(1,1))*sin(commands(1,2)) + cmd2torque(commands(2,1))*sin(commands(2,2));
    torques(2) = cmd2torque(commands(3,1))*sin(commands(3,2)) + cmd2torque(commands(4,1))*sin(commands(4,2));
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
    
    dcm = angle2dcm(theta(1),theta(2),theta(3),'XYZ'); % Convert calculated ypr angles to directional cosine matrix (DCM)
    %Further per-thruster DCM computation will occur when thrust vectoring incorporated
    
    thr_nose_dot_comp = dcm*[0;0;thr_nose_dot];
    thr_tail_dot_comp = dcm*[0;0;thr_tail_dot];
    thr_right_dot_comp = dcm*[0;0;thr_right_dot];
    thr_left_dot_comp = dcm*[0;0;thr_left_dot];
    
    acceleration = acceleration_prev + (thr_nose_dot_comp' + thr_tail_dot_comp' + thr_right_dot_comp' + thr_left_dot_comp')/quad_mass*dt;
    
    velocity = velocity_prev + acceleration*dt;
    position = position_prev + velocity*dt + 0.5*acceleration*dt^2;

    current = [ % Repack current data
        theta position;
        theta_dot velocity;
        theta_dotdot acceleration;
        ];

end

end %SYSTEM     


