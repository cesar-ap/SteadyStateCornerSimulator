%%%%%%%%%%%%%%%%%
% Load Transfer %
%%%%%%%%%%%%%%%%%

function [LT_e_distribution, LT_distribution, LT_total_f, LT_total_r] = Load_Transfer(S_Mass_f...
    , S_Mass_r, US_Mass_f, US_Mass_r, AR_stiff_d , huf, hur, hrf, hrr...
    , hs, ay ,track_f, track_r)
%--------------------------------------------------------------------------
%INPUTS
    q = AR_stiff_d; % mechanical roll stiffness distribution to the front
%--------------------------------------------------------------------------
    %CALCULATIONS
    LT_u_f = US_Mass_f * ay * huf / track_f; %N
    LT_u_r = US_Mass_r * ay * hur / track_r; %N

    LT_g_f = S_Mass_f * ay * hrf / track_f; %N
    LT_g_r = S_Mass_r * ay * hrr / track_r; %N

    haf = hs - hrf; %m
    har = hs - hrr; %m
    LT_e_f = S_Mass_f * ay * haf * q / track_f; %N
    LT_e_r = S_Mass_r * ay * har * (1 - q) / track_r; %N

    LT_total_f = LT_u_f + LT_g_f + LT_e_f; %N
    LT_total_r = LT_u_r + LT_g_r + LT_e_r; %N
%--------------------------------------------------------------------------
%OUTPUTS
    LT_distribution = LT_total_f / (LT_total_f + LT_total_r); %
    LT_e_distribution = LT_e_f / (LT_e_f + LT_e_r); %
end
%------------------------------------------------------------------------
