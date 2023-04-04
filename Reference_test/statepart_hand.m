function [pd] = statepart_hand(Y_vec,U_in)

%Initialize Variables.
m = 1400; N_w = 2.00; f = 0.01; I_z = 2667; a = 1.35; b = 1.45; B_y = 0.27;
C_y = 1.2; D_y = 0.70; E_y = -1.6; g = 9.806;

%Set Variables in Interest.
X = Y_vec(1); u = Y_vec(2); Y = Y_vec(3); v = Y_vec(4); psi = Y_vec(5); r = Y_vec(6);
Fx = U_in(1); delta_f = U_in(2);

%Partial Derivatives of alpha_f and alpha_r.
temp1 = -1/(1 + ((v+a*r)/u)^2); temp2 = -1/(1 + ((v-b*r)/u)^2);
dalpha_f = [0, temp1*-(v+a*r)/u^2, 0, temp1/u, 0, temp1*a/u];
dalpha_r = [0, temp2*-(v-b*r)/u^2, 0, temp2/u, 0, temp2*-b/u];

%Partial Derivatives of phi_yf and phi_yr.
alpha_f = delta_f - atan((v+a*r)/u); alpha_r = - atan((v-b*r)/u);
temp3 = E_y/(1 + (B_y*alpha_f)^2); temp4 = E_y/(1 + (B_y*alpha_r)^2);
dphi_yf = [0, (1 - E_y + temp3)*dalpha_f(2), 0, (1 - E_y + temp3)*dalpha_f(4) ...
    0, (1 - E_y + temp3)*dalpha_f(6)];
dphi_yr = [0, (1 - E_y + temp4)*dalpha_r(2), 0, (1 - E_y + temp4)*dalpha_r(4) ...
    0, (1 - E_y + temp4)*dalpha_r(6)];

%Partial Derivatives of F_yf and F_yr.
phi_yf = (1 - E_y)*alpha_f + E_y*atan(B_y*alpha_f)/B_y;
phi_yr = (1 - E_y)*alpha_r + E_y*atan(B_y*alpha_r)/B_y;
F_zf = b*m*g/(a+b); F_yf = F_zf*D_y*sin(C_y*atan(B_y*phi_yf));
F_zr = a*m*g/(a+b); F_yr = F_zr*D_y*sin(C_y*atan(B_y*phi_yr));
temp5 = F_zf*D_y*cos(C_y*atan(B_y*phi_yf))*C_y*(1/(1+(B_y*phi_yf)^2))*B_y;
temp6 = F_zr*D_y*cos(C_y*atan(B_y*phi_yr))*C_y*(1/(1+(B_y*phi_yr)^2))*B_y;
dF_yf = [0, temp5*dphi_yf(2), 0, temp5*dphi_yf(4), 0, temp5*dphi_yf(6)];
dF_yr = [0, temp6*dphi_yr(2), 0, temp6*dphi_yr(4), 0, temp6*dphi_yr(6)];

%Finally, Compute Partial Derivatives for Jacobian Matrix.
pd = zeros(6);
temp7 = a*cos(delta_f)/I_z;
pd(1,:) = [0, cos(psi), 0, -sin(psi), -u*sin(psi)-v*cos(psi), 0];
pd(2,:) = [0, -sin(delta_f)/m*dF_yf(2), 0, -sin(delta_f)/m*dF_yf(4)+r, ...
    0, -sin(delta_f)/m*dF_yf(6)+v];
pd(3,:) = [0, sin(psi), 0, cos(psi), u*cos(psi)-v*sin(psi), 0];
pd(4,:) = [0, (cos(delta_f)*dF_yf(2)+dF_yr(2))/m - r, 0, ...
    (cos(delta_f)*dF_yf(4)+dF_yr(4))/m, 0, (cos(delta_f)*dF_yf(6)+dF_yr(6))/m - u];
pd(5,:) = [0, 0, 0, 0, 0, 1];
pd(6,:) = [0, temp7*dF_yf(2) - b*dF_yr(2)/I_z, 0, temp7*dF_yf(4) - b*dF_yr(4)/I_z, ...
    0, temp7*dF_yf(6) - b*dF_yr(6)/I_z];
end