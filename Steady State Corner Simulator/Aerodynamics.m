%%%%%%%%%%%%%%%%
% Aerodynamics %
%%%%%%%%%%%%%%%%

function [Fz_downforce] = Aerodynamics (air_t, atm_p, downforce_coef, area, speed)
%--------------------------------------------------------------------------
%INPUTS
    air_d = 1.2255 * (15+273) / (air_t + 273) * atm_p / 1000;
    speed = speed * 0.27; % 130 Km/h converted to m/s
%--------------------------------------------------------------------------
%CALCULATIONS
    
%--------------------------------------------------------------------------
%OUTPUTS
    Fz_downforce = 0.5 * air_d * area * downforce_coef * speed^2;
end
%--------------------------------------------------------------------------
