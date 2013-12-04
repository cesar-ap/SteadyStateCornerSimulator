%%%%%%%%%%%%%%
% Tire Model %
%%%%%%%%%%%%%%

function [Fy] = Tire_Response (c2, c1, c0, g, Fz)
%--------------------------------------------------------------------------
%INPUTS
    Fzfl = Fz(1,1); %N
    Fzfr = Fz(1,2); %N
    Fzrl = Fz(2,1); %N
    Fzrr = Fz(2,2); %N
%--------------------------------------------------------------------------
%CALCULATIONS
    Fyfl = c2(1) * Fzfl^2 + c1(1) * Fzfl + c0(1);
    Fyfr = c2(1) * Fzfr^2 + c1(1) * Fzfr + c0(1);
    Fyrl = c2(2) * Fzrl^2 + c1(2) * Fzrl + c0(2);
    Fyrr = c2(2) * Fzrr^2 + c1(2) * Fzrr + c0(2);
%--------------------------------------------------------------------------
%OUTPUTS
    Fy = [Fyfl Fyfr; Fyrl Fyrr];
%--------------------------------------------------------------------------
% GRAPHIC
    min_weight=0;
    max_weight=1000;
    delta_weight=max_weight-min_weight;
    gap=10;
    samples=(delta_weight/gap);

    Weight = linspace (min_weight,max_weight,samples); %Kg
    Normal_load = Weight * g; %N
    Fy_f = zeros(1,samples); %empty array to save Fy_f
    Fy_r = zeros(1,samples); %empty array to save Fy_r

    for i = 1:samples
        Fz = Normal_load(i);
        Fy_i = c2(1) * Fz^2 + c1(1) * Fz + c0(1);
        Fy_f(i) = Fy_i;
    end
    for i = 1:samples
        Fz = Normal_load(i);
        Fy_i = c2(2) * Fz^2 + c1(2) * Fz + c0(2);
        Fy_r(i) = Fy_i;
    end

    figH = figure;
    set(gca,'Color',[0.2 0.2 0.2]);
    set(figH,'Name','Tire Response','NumberTitle','off')
    plot (Normal_load, Fy_f, Normal_load, Fy_r);
    legend('Front Tire','Rear Tire');
    grid on;
    xlabel('Normal Load [N]')
    ylabel('Lateral Force [N]')
    hold on;
    plot(Fzfl,Fyfl,'-.hr');
    text(Fzfl+200,Fyfl,'FL')

    plot(Fzfr,Fyfr,'-.hg');
    text(Fzfr+200,Fyfr,'FR')
    
    plot(Fzrl,Fyrl,'-.hb');
    text(Fzrl+200,Fyrl,'RL')    
    
    plot(Fzrr,Fyrr,'-.hy');    
    text(Fzrr+200,Fyrr,'RR')    
end
%--------------------------------------------------------------------------