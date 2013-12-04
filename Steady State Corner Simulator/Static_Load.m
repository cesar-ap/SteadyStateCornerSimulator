%%%%%%%%%%%%%%%
% Static load %
%%%%%%%%%%%%%%%

function [Fz_static] = Static_Load(Total_Mass, Weight_distribution, g)
%--------------------------------------------------------------------------
%INPUTS
%--------------------------------------------------------------------------
%CALCULATIONS
    Fzfl = g * (Total_Mass * Weight_distribution) / 2; %N
    Fzfr = g *(Total_Mass * Weight_distribution) / 2; %N
    Fzrr = g * Total_Mass * (1 - Weight_distribution) / 2; %N
    Fzrl = g * Total_Mass * (1 - Weight_distribution) / 2; %N
%--------------------------------------------------------------------------
%OUTPUTS
    Fz_static = [Fzfl Fzfr; Fzrl Fzrr];
end
%--------------------------------------------------------------------------