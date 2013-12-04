%%%%%%%%%%%%%%%%
% Section_name %
%%%%%%%%%%%%%%%%

function Vxmax = Max_Speed_Corner(Fy, Total_Mass, g)
%--------------------------------------------------------------------------
%INPUTS
    samples=500;
    radious = linspace (0,samples,samples); % array containing different radious corner [m]
    Vx = zeros(1,samples); % empty array for speed
    yaw_rate = zeros(1,samples); % empty array for yaw_velocity
    Vxmax = 0; % variable to store the Maximum Speed
    ay = (Fy(1,1) + Fy(1,2) + Fy(2,1) + Fy(2,2)) / (Total_Mass * g); % average lateral force of the tires G
    ay = ay * 9.8; %m/s^2
%--------------------------------------------------------------------------
%CALCULATIONS
    for i = 1:length(radious)
        Vx(i) = sqrt(ay * radious(i)) * 3.6; % Km/h
        if Vx(i) > Vxmax
            Vxmax = Vx(i);
        end
        yaw_rate(i) = ay / Vx(i); % Yaw Velocity
    end
%--------------------------------------------------------------------------
%OUTPUTS
%     fig1 = figure;
%     set(fig1,'Name','Corner Speed','NumberTitle','off')
%     plot (Vx, radious);
%     grid on;
%     xlabel('Linear velocity [Km/h]');
%     ylabel('Corner radious [m]');
%     
%     fig2 = figure;
%     set(fig2,'Name','Yaw Rate','NumberTitle','off')
%     plot (yaw_rate, radious);
%     grid on;
%     xlabel('Yaw Rate');
%     ylabel('Corner radious [m]');    
    
    
%--------------------------------------------------------------------------