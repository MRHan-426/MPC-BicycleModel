function [ip] = inputpart_hand(Y_vec,U_in)
    
%Initialize Variables.
m = 1400; N_w = 2.00; f = 0.01; I_z = 2667; a = 1.35; b = 1.45; B_y = 0.27;
C_y = 1.2; D_y = 0.70; E_y = -1.6; g = 9.806;

%Set Variables in Interest.
X = Y_vec(1); u = Y_vec(2); Y = Y_vec(3); v = Y_vec(4); psi = Y_vec(5); r = Y_vec(6);
Fx = U_in(1); delta_f = U_in(2);

%Partial Derivatives of phi_yf.
alpha_f = delta_f - atan((v+a*r)/u); alpha_r = - atan((v-b*r)/u);
dphi_yf = [0, 1 - E_y + E_y/(1+(B_y*alpha_f)^2)];

%Partial Derivatives of F_yf.
phi_yf = (1 - E_y)*alpha_f + E_y*atan(B_y*alpha_f)/B_y;
F_zf = b*m*g/(a+b); F_yf = F_zf*D_y*sin(C_y*atan(B_y*phi_yf));
dF_yf = [0, F_zf*D_y*cos(C_y*atan(B_y*phi_yf))*C_y*(1/(1+(B_y*phi_yf)^2))*B_y*dphi_yf(2)];

%Finally, Compute Partial Derivatives for Jacobian Matrix.
ip = zeros(6,2);
ip(2,:) = [N_w/m, -sin(delta_f)*dF_yf(2)/m - cos(delta_f)*F_yf/m];
ip(4,:) = [0, a*cos(delta_f)*dF_yf(2)/I_z - a*F_yf*sin(delta_f)/I_z];
ip(6,:) = [0, cos(delta_f)*dF_yf(2)/m - F_yf*sin(delta_f)/m];
end