function [Y,T]=forwardIntegrateControlInput(U,x0)
% function [Y] = forwardIntegrateControlInput(U,x0)
% 
% Given a set of inputs and an initial condition, returns the vehicles
% trajectory. If no initial condition is specified the default for the track
% is used.
% 
%  INPUTS:
%    U           an N-by-2 vector of inputs, where the first column is the
%                steering input in radians, and the second column is the 
%                longitudinal force in Newtons.
%    
%    x0          a 1-by-6 vector of the initial state of the vehicle.
%                If not specified, the default is used
% 
%  OUTPUTS:
%    Y           an N-by-6 vector where each column is the trajectory of the
%                state of the vehicle
%
%    T           a 1-by-N vector of time stamps for each of the rows of Y.
% 
%  Written by: Sean Vaskov
%  Created: 31 Oct 2018
%  Modified: 6 Nov 2018
%
%   Revision notes:
%   - Sid Dey (6 Dec 2019)

    %  if initial condition not given use default
    if nargin < 2
        x0 = [287,5,-176,0,2,0] ;
    end

    %generate time vector
    T=0:0.01:(size(U,1)-1)*0.01;

    if (length(T) == 1)
        % in case U is a 1x2 vector, meaning T would be a 1x1 scalar, 
        % we return the initial condition.
        Y = x0;
        
    else
        %Solve for trajectory
        options = odeset('MaxStep',0.01);
        [~,Y]=ode45(@(t,x)bike(t,x,T,U),T,x0,options);
        
        % in case U is a 2x2 vector, meaning T would be a 1x2 vector [t0 tf],
        % ode45 would provide the solution at its own integration timesteps
        % (in between t0 and tf). To avoid this, we simply take the first
        % and last value of Y (which should correspond with t0 and tf).
        
        if ( size(U,1) ~= size(Y,1) )
            % Take first and last value of Y.
            Y = [Y(1, :); Y(end, :)];
        end
        
    end
    
end

function dzdt=bike(t,x,T,U)
%constants
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=0.7;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
g=9.806;


%generate input functions
delta_f=interp1(T,U(:,1),t,'previous','extrap');
F_x=interp1(T,U(:,2),t,'previous','extrap');

%slip angle functions in degrees
a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*F_x)^2+(F_yr^2));
F_max=0.7*m*g;

if F_total>F_max
    
    F_x=F_max/F_total*F_x;
  
    F_yr=F_max/F_total*F_yr;
end

%vehicle dynamics
dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end

