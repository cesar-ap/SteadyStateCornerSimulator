%%%%%%%%%%%%%%%
% Normal Load %
%%%%%%%%%%%%%%%

function [Fz] = Normal_Load (Fz_static, LT_total_f, LT_total_r, ay, g)
%--------------------------------------------------------------------------
%INPUTS
    Fzfl = Fz_static(1,1); %N
    Fzfr = Fz_static(1,2); %N
    Fzrl = Fz_static(2,1); %N
    Fzrr = Fz_static(2,2); %N
%--------------------------------------------------------------------------
%CALCULATIONS
    if ay > 0
        % Left turn
        Fzfl = Fzfl - LT_total_f; %N
        Fzfr = Fzfr + LT_total_f; %N
        Fzrl = Fzrl - LT_total_r; %N
        Fzrr = Fzrr + LT_total_r; %N       
    else
        % Right turn
        Fzfl = Fzfl + LT_total_f; %N
        Fzfr = Fzfr - LT_total_f; %N
        Fzrl = Fzrl + LT_total_r; %N
        Fzrr = Fzrr - LT_total_r; %N        
    end
    
%--------------------------------------------------------------------------
%OUTPUTS
    Fz = [Fzfl Fzfr; Fzrl Fzrr]; %N
end
%--------------------------------------------------------------------------