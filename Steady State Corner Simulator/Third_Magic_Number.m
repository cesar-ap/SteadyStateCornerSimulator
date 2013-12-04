%%%%%%%%%%%%%%%%%%%%%%
% Third Magic Number %
%%%%%%%%%%%%%%%%%%%%%%

function [AR_stiff_d_2]=Third_Magic_Number(track_f, track_r, K_tire_f...
    , K_tire_r, ART_front, ART_rear)
%--------------------------------------------------------------------------
%INPUTS
    roll_angle_00 = 1 * pi / 180; % covnersion from deg to rad
%--------------------------------------------------------------------------
%CALCULATIONS
    ART_tire_f = track_f^2 * tan(roll_angle_00) * K_tire_f / 2; %Nm
    ART_tire_r = track_r^2 * tan(roll_angle_00) * K_tire_r / 2; %Nm

    ART_total_f = ART_front * ART_tire_f / (ART_front + ART_tire_f); %Nm
    ART_total_r = ART_rear * ART_tire_r / (ART_rear + ART_tire_r); %Nm
    AR_stiff_f = ART_total_f; %Nm/deg
    AR_stiff_r = ART_total_r; %Nm/deg
%--------------------------------------------------------------------------
%OUTPUTS
    AR_stiff_d_2 = ART_total_f / (ART_total_f + ART_total_r); %
end
%--------------------------------------------------------------------------