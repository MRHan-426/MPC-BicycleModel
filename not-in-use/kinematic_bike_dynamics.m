function dzdt=kinematic_bike_dynamics(t,z,U_in,T,Nw,f,Iz,a,b,By,Cy,Dy,Ey,Shy,Svy,m,g)

if length(T)<=1 || isempty(T) || size(U_in,2)==1
    Fx=U_in(2);
    delta_f=U_in(1);
else
    Fx=interp1(T',U_in(2,:)',t,'previous');
    delta_f=interp1(T',U_in(1,:)',t,'previous');
end

% dzdt=[u*cos(z(3))-b/L*u*tan(delta)*sin(z(3));...
%       u*sin(z(3))+b/L*u*tan(delta)*cos(z(3));...
%       u/L*tan(delta)];

x=z(1);u=z(2);y=z(3);v=z(4);phi=z(5);r=z(6);

a_f=rad2deg(delta_f-atan2(v+a*r,u));
a_r=rad2deg(-atan2((v-b*r),u));

%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*Fx)^2+(F_yr^2));
F_max=0.7*m*g;

if F_total>F_max
    
    Fx=F_max/F_total*Fx;
  
    F_yr=F_max/F_total*F_yr;
end

%vehicle dynamics
% x u y v phi r
 f1 = u*cos(phi)-v*sin(phi);
 f2 = (-f*m*g+Nw*Fx-F_yf*sin(delta_f))/m+v*r;
 f3 = u*sin(phi)+v*cos(phi);
 f4 = (F_yf*cos(delta_f)+F_yr)/m-u*r;
 f5 = r;
 f6 = (F_yf*a*cos(delta_f)-F_yr*b)/Iz;

dzdt = [f1;f2;f3;f4;f5;f6];

end