function [dx] = odefun(Y,Uin)

% m    = 1400;                % Mass of Car
% N_w  = 2.00;                % ?? 
% f    = 0.01;                % ??
% I_z  = 2667;                % Momemnt of Inertia
% a    = 1.35;                % Front Axle to COM
% b    = 1.45;                % Rear Axle to Com
% B_y  = 0.27;                % Empirically Fit Coefficient
% C_y  = 1.35;                % Empirically Fit Coefficient
% D_y  = 0.70;                % Empirically Fit Coefficient
% E_y  = -1.6;                % Empirically Fit Coefficient
% g    = 9.806;               % Graviational Constant
% 
% alpha_f = (Uin(2) - atan((Y(4)+a*Y(6))/Y(2)));
% alpha_r = (- atan((Y(4)-b*Y(6))/Y(2)));
% 
% psi_yf = ((1-E_y)*alpha_f + E_y/B_y*atan(B_y*alpha_f));  % S_hy = 0
% psi_yr = ((1-E_y)*alpha_r + E_y/B_y*atan(B_y*alpha_r));  % S_hy = 0
% 
% 
% F_yf = (b/(a+b)*m*g*D_y*sin(C_y*atan(B_y*psi_yf))); %S_vy = 0;
% F_yr = (a/(a+b)*m*g*D_y*sin(C_y*atan(B_y*psi_yr))); %S_vy = 0;
% 
% dx = [          Y(2)*cos(Y(5))-Y(4)*sin(Y(5));
%            1/m*(-f*m*g+N_w*Uin(1)-F_yf*sin(Uin(2)))+Y(4)*Y(6);
%                      Y(2)*sin(Y(5))+Y(4)*cos(Y(5));
%                 1/m*(F_yf*cos(Uin(2))+F_yr)-Y(2)*Y(6);
%                                     Y(6);
%                       1/I_z*(a*F_yf*cos(Uin(2))-b*F_yr)];

alpha_f = (Uin(2) - atan((Y(4)+1.35*Y(6))/Y(2)));
alpha_r = (- atan((Y(4)-1.45*Y(6))/Y(2)));

psi_yf = ((1-(-1.6))*alpha_f + (-1.6)/0.27*atan(0.27*alpha_f));  % S_hy = 0
psi_yr = ((1-(-1.6))*alpha_r + (-1.6)/0.27*atan(0.27*alpha_r));  % S_hy = 0


F_yf = (1.45/(1.35+1.45)*1400*9.806*0.7*sin(1.35*atan(0.27*psi_yf))); %S_vy = 0;
F_yr = (1.35/(1.35+1.45)*1400*9.806*0.7*sin(1.35*atan(0.27*psi_yr))); %S_vy = 0;

dx = [          Y(2)*cos(Y(5))-Y(4)*sin(Y(5));
           1/1400*(-0.01*1400*9.806+2.00*Uin(1)-F_yf*sin(Uin(2)))+Y(4)*Y(6);
                     Y(2)*sin(Y(5))+Y(4)*cos(Y(5));
                1/1400*(F_yf*cos(Uin(2))+F_yr)-Y(2)*Y(6);
                                    Y(6);
                      1/2667*(1.35*F_yf*cos(Uin(2))-1.45*F_yr)];
                  
end