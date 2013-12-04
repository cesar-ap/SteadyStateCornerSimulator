%%%%%%%%%%%%%%%%%%%%
% Anti Roll Torque %
%%%%%%%%%%%%%%%%%%%%

function [roll_gradient, AR_stiff_d_1, ha, ART_front, ART_rear] = Anti_Roll_Torque(hs, a, b...
    ,hrf, hrr, Wheelbase, track_f, track_r, K_spring_f, K_spring_r...
    , K_ARB_f, K_ARB_r, MR_wheel_spring_f, MR_wheel_spring_r, MR_ARB_f...
    , MR_ARB_r, S_Mass, ay, g)
%--------------------------------------------------------------------------
%INPUTS
    roll_angle_00 = 1 * pi / 180; % covnersion from deg to rad
    ha = hs - ((a * hrr + b * hrf) /Wheelbase); %m
%--------------------------------------------------------------------------
%CALCULATIONS
    ART_springs_f = track_f^2 * (K_spring_f / MR_wheel_spring_f^2) * tan(roll_angle_00) / 2; %Nm/deg
    ART_ARB_f = K_ARB_f * roll_angle_00 / MR_ARB_f^2; %Nm/deg
    ART_springs_r = track_r^2 * (K_spring_r / MR_wheel_spring_r^2) * tan(roll_angle_00) / 2; %Nm/deg
    ART_ARB_r = K_ARB_r * roll_angle_00 / MR_ARB_r^2; %Nm/deg
    ART_front = ART_springs_f + ART_ARB_f; %Nm/deg
    ART_rear = ART_springs_r + ART_ARB_r; %Nm/deg
%--------------------------------------------------------------------------
%OUTPUTS
    AR_stiff_d_1 = ART_front / (ART_front + ART_rear); %
    M_lat_F = S_Mass * ay * g * ha; %N
    roll_angle =  M_lat_F / (ART_front + ART_rear); %deg
    roll_gradient = roll_angle / ay; %deg/g
end
%--------------------------------------------------------------------------