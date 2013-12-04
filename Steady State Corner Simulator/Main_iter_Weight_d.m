% Performance Simulator v0.1
% Date: 21-11-2013
% Author: César Álvarez Porras (www.cesar-ap.com)

% Script: Calculates maximum lateral force for different values of Weight
% distribution.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                                                     %%%
%%%                         Variables Declaration                       %%%
%%%                                                                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Physics constants
g = 9.81;
pi = 3.14159;

% Scenario
ay = 1.7; %g

% Vehicle
Total_Mass = 1200;%Kg
US_Mass_f = 40;%Kg
US_Mass_r = 46;%Kg
Wheelbase = 2.890;%m
Weight_distribution = [0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.5, 0.51, 0.52, 0.53, 0.54, 0.55];%
track_f = 1.698;%m
track_r = 1.620;%m
hcg = 0.45; %m
hrf = 0; %m
hrr = 0; %m
huf = 0.33; %m
hur = 0.355; %m

% Suspension
MR_wheel_spring_f = 1;%
MR_wheel_spring_r = 1;%
MR_ARB_f = 0.8;%
MR_ARB_r = 0.8;%
K_spring_f = 595000;%Nm
K_spring_r = 507500;%Nm
K_ARB_f = 361375;%Nm
K_ARB_r = 56875;%Nm

% Aerodynamics
air_t = 23;% C degrees
atm_p = 1000;% bar
downforce_coef = 1.8; % at random
drag_coef = 0.835; % at 20 FRH and 40 RRH
area = 1.775;%m^2

%Tire model
K_tire_f = 95000;%Nm
K_tire_r = 95000;%Nm
% K_tire_f = 331800;%Nm
% K_tire_r = 417725;%Nm
c2=[-0.00022, -0.00027];
c1=[2.1, 2.3];
c0=[100, 250];




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                                                     %%%
%%%                               Execution                             %%%
%%%                                                                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Fz_static = zeros(2,2,length(Weight_distribution)); % Empty array of zeros for output Fz_static
Fz = zeros(2,2,length(Weight_distribution)); % Empty array of zeros for output Fz
Fz_downforce = zeros(2,2,length(Weight_distribution)); % Empty array of zeros for output Fz Aerodynamic
Fy = zeros(2,2,length(Weight_distribution)); % Empty array of zeros for output Fy
yaw_moment = zeros(1,length(Weight_distribution)); % Empty array of zeros for output Yaw Moment

for i=1:numel(Weight_distribution)
    Weight_d = Weight_distribution(i); % Current run execution´s Weight distribution

    % Initial calculations
    US_Mass = US_Mass_f + US_Mass_r; % total US Mass
    S_Mass = Total_Mass - US_Mass; % total S Mass
    hs = (Total_Mass * hcg - US_Mass * (huf + hur)/2) / S_Mass; % S Mass CG height
    Total_Mass_f = Total_Mass * Weight_d; %Kg
    Total_Mass_r = Total_Mass * (1 - Weight_d); %Kg 
    S_Mass_f = Total_Mass_f - US_Mass_f; %Kg
    S_Mass_r = Total_Mass_r - US_Mass_r; %Kg
    a = Total_Mass_r / Total_Mass * Wheelbase; %m
    b = Total_Mass_f / Total_Mass * Wheelbase; %m

    % Calculate the Static Loads
    Fz_static_i = Static_Load(Total_Mass, Weight_d, g);
    %Fz_static

    % Calculate the initial AR Stiffness Distribution and the Roll Gradient
    [roll_gradient, AR_stiff_d_1, ha, ART_front, ART_rear] = Anti_Roll_Torque(...
        hs, a, b, hrf, hrr, Wheelbase, track_f, track_r, K_spring_f, K_spring_r...
        , K_ARB_f, K_ARB_r, MR_wheel_spring_f, MR_wheel_spring_r, MR_ARB_f...
        , MR_ARB_r, S_Mass, ay, g);
    %AR_stiff_d_1 % 1st Magic Number (1)

    % Calculate the Load Transfer Elastic distribution and total Load Transfer
    [LT_e_distribution, LT_distribution_1] = Load_Transfer(...
        S_Mass_f, S_Mass_r, US_Mass_f, US_Mass_r, AR_stiff_d_1 , huf, hur, hrf...
        , hrr, hs, ay, track_f, track_r);
    %LT_e_distribution % 1st Magic Number (2)
    %LT_distribution_1 % 2nd Magic Number 

    % Calculate the AR Stiffness Distribution considering the tire stiffness
    AR_stiff_d_2 = Third_Magic_Number(track_f, track_r, K_tire_f, K_tire_r, ART_front, ART_rear);
    %AR_stiff_d_2 % 3rd Magic Number (1)

    % Re calculate the total Load Transfer considering the tire stiffness
    [LT_e_distribution, LT_distribution_2, LT_total_f, LT_total_r] = Load_Transfer(...
        S_Mass_f, S_Mass_r, US_Mass_f, US_Mass_r, AR_stiff_d_2 , huf, hur, hrf...
        , hrr, hs, ay, track_f, track_r);
    %LT_distribution_2 % 3rd Magic Number (2)
    %LT_total_f
    %LT_total_r

    % Calculate the Normal Load on each corner
    Fz_i = Normal_Load(Fz_static_i, LT_total_f, LT_total_r, ay, g);
    %Fz % Corner Loads

    % Calculate the Aerodynamic downforce
    Fz_downforce_i = Aerodynamics(air_t, atm_p, downforce_coef, area, speed);
    
    % Calculate the Lateral Force from the tires based on the Normal load
    Fz_i = Fz_i + Fz_downforce_i; % Add the downforce to the Normal load
    Fy_i = Tire_Model(c2, c1, c0, g, Fz_i);
    %Fy % Lateral Force on each tire

    % Calculate the Max Speed on corner
    Vxmax = Max_Speed_Corner(Fy, Total_Mass, g);

    % Final calculations
    yaw_m = a * (Fy_i(1,1) + Fy_i(1,2)) - b * (Fy_i(2,1) + Fy_i(2,2));

    Fz_static(:,:,i) = Fz_static_i; % Current run execution´s Static Load
    Fz_downforce(i) = Fz_downforce_i; % Current run execution's Downforce
    Fz(:,:,i) = Fz_i; % Current run execution´s Normal Load
    Fy(:,:,i) = Fy_i; % Current run execution´s Lateral Force
    yaw_moment(i) = yaw_m; % Current run execution´s Yaw Moment
    
    % Command line information:
    fprintf ('\nFor a Weight Distribution of: %d\n',Weight_d);
    fprintf ('The Vertical Load Static is: %d\n',Fz_static(:,:,i));
    fprintf ('The Downforce is: %d\n', Fz_downforce(i));
    fprintf ('The Total Vertical Load on corner is: %d\n',Fz(:,:,i));
    fprintf ('And the Lateral Force is: %d\n',Fy(:,:,i));
    fprintf ('The Yaw Moment is: %d\n',yaw_moment(i));
    fprintf ('//---------------------------------//\n\n');
end

% Plotting Yaw Moment against Weight Distribution
set(figure,'Name','Yaw Moment','NumberTitle','off');
scatter(Weight_distribution, yaw_moment,6,'b');
xlabel('Weight Distribution [%F]');
ylabel('Yaw Moment');
grid on;


% Plotting Static Normal Load against Weight Distribution
set(figure,'Name','Static Normal Load','NumberTitle','off');
subplot(2,2,1);
scatter(Weight_distribution, Fz_static(1,1,:),6,'r');
xlabel('Weight Distribution [%F]');
ylabel('Static Normal Load FL [N]');
grid on;
subplot(2,2,2);
scatter(Weight_distribution, Fz_static(1,2,:),6,'g');
xlabel('Weight Distribution [%F]');
ylabel('Static Normal Load FR [N]');
grid on;
subplot(2,2,3);
scatter(Weight_distribution, Fz_static(2,1,:),6,'b');
xlabel('Weight Distribution [%F]');
ylabel('Static Normal Load RL [N]');
grid on;
subplot(2,2,4);
scatter(Weight_distribution, Fz_static(2,2,:),6,'y');
xlabel('Weight Distribution [%F]');
ylabel('Static Normal Load RR [N]');
grid on;


% Plotting Normal Load against Weight Distribution
set(figure,'Name','Normal Load','NumberTitle','off');
subplot(2,2,1);
scatter(Weight_distribution, Fz(1,1,:),6,'r');
xlabel('Weight Distribution [%F]');
ylabel('Normal Load FL [N]');
grid on;
subplot(2,2,2);
scatter(Weight_distribution, Fz(1,2,:),6,'g');
xlabel('Weight Distribution [%F]');
ylabel('Normal Load FR [N]');
grid on;
subplot(2,2,3);
scatter(Weight_distribution, Fz(2,1,:),6,'b');
xlabel('Weight Distribution [%F]');
ylabel('Normal Load RL [N]');
grid on;
subplot(2,2,4);
scatter(Weight_distribution, Fz(2,2,:),6,'y');
xlabel('Weight Distribution [%F]');
ylabel('Normal Load RR [N]');
grid on;



% Plotting Lateral Force against Weight Distribution
set(figure,'Name','Lateral Force','NumberTitle','off');
subplot(2,2,1);
scatter(Weight_distribution, Fy(1,1,:),6,'r');
xlabel('Weight Distribution [%F]');
ylabel('Lateral Force FL [N]');
grid on;
subplot(2,2,2);
scatter(Weight_distribution, Fy(1,2,:),6,'g');
xlabel('Weight Distribution [%F]');
ylabel('Lateral Force FR [N]');
grid on;
subplot(2,2,3);
scatter(Weight_distribution, Fy(2,1,:),6,'b');
xlabel('Weight Distribution [%F]');
ylabel('Lateral Force RL [N]');
grid on;
subplot(2,2,4);
scatter(Weight_distribution, Fy(2,2,:),6,'y');
xlabel('Weight Distribution [%F]');
ylabel('Lateral Force RR [N]');
grid on;







% %Calculate the initial AR Stiffness Distribution considering the tire stiffness
% [AR_stiff_d_2] = Roll_Stiffness(roll_gradient, AR_stiff_d_1, track_f...
%     , track_r, K_spring_f, K_spring_r, K_tire_f, K_tire_r, MR_wheel_spring_f...
%     , MR_wheel_spring_r, MR_ARB_f, MR_ARB_r, S_Mass, ha, g);
% AR_stiff_d_2 %3rd Magic Number (1)



%%%%%%%%%%%%%%
% Transcient %
%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
%INPUTS
%--------------------------------------------------------------------------
%CALCULATIONS
%--------------------------------------------------------------------------
%OUTPUTS
%--------------------------------------------------------------------------



%%%%%%%%%%%%%%%%
% Section_name %
%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
%INPUTS
%--------------------------------------------------------------------------
%CALCULATIONS
%--------------------------------------------------------------------------
%OUTPUTS
%--------------------------------------------------------------------------