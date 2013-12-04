%%%%%%%%%%%%%%%%%%
% Roll Stiffness %
%%%%%%%%%%%%%%%%%%

function [AR_stiff_d_2] = Roll_Stiffness(roll_gradient, AR_stiff_d_1...
    , track_f, track_r, K_spring_f, K_spring_r, K_tire_f, K_tire_r...
    , MR_wheel_spring_f, MR_wheel_spring_r, MR_ARB_f, MR_ARB_r, S_Mass...
    , ha, g)
%--------------------------------------------------------------------------
%INPUTS
%--------------------------------------------------------------------------
%CALCULATIONS
    roll_stiff_springs_f = track_f^2 * K_spring_f/MR_wheel_spring_f^2 * ...
        K_spring_f/MR_wheel_spring_f^2 * pi / ...
        ((K_spring_f/MR_wheel_spring_f^2 + K_spring_f/MR_wheel_spring_f^2)*180); %Nm/deg

    roll_stiff_springs_r = track_r^2 * K_spring_r/MR_wheel_spring_r^2 * ...
        K_spring_r/MR_wheel_spring_r^2 * pi / ...
        ((K_spring_r/MR_wheel_spring_r^2 + K_spring_r/MR_wheel_spring_r^2)*180); %Nm/deg
    
    roll_stiff_springs_total = roll_stiff_springs_f + roll_stiff_springs_r; %Nm/deg

    roll_stiff_tires = (K_tire_f * track_f^2/2 + K_tire_r * track_f^2/2)*pi/180; %Nm/deg

    roll_stiff_desired = S_Mass * g * ha / (roll_gradient); %Nm/deg
    roll_stiff_ARB_total_wheel = roll_stiff_desired - roll_stiff_springs_total - roll_stiff_tires; %Nm/deg
    roll_stiff_ARB_f_wheel = roll_stiff_desired * AR_stiff_d_1 - roll_stiff_springs_f; %Nm/deg
    roll_stiff_ARB_r_wheel = roll_stiff_ARB_total_wheel - roll_stiff_ARB_f_wheel; %Nm/deg
%--------------------------------------------------------------------------
%OUTPUTS
    roll_stiff_ARB_f = roll_stiff_ARB_f_wheel * MR_ARB_f^2; %Nm/deg
    roll_stiff_ARB_r = roll_stiff_ARB_r_wheel * MR_ARB_r^2; %Nm/deg
    AR_stiff_d_2 = roll_stiff_ARB_f / (roll_stiff_ARB_f + roll_stiff_ARB_r); %
end
%--------------------------------------------------------------------------