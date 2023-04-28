function [g,h,dg,dh]=nonlcon(z)
x0   =   287;
u0   =   5.0;
y0   =  -176;
v0   =   0.0;
psi0 =   2.0;
% psi0 =   theta(1);
r0   =   0.0;

z0 = [x0, u0, y0, v0, psi0, r0];


    if size(z,2) > size(z,1)
        z = z' ;
    end
    
    nsteps = (size(z,1)+2)/8 ;
    dt = 0.01 ;

    zx = z(1:nsteps*6) ;
    zu = z(nsteps*6+1:end) ;

%     g = zeros(nsteps,1) ;
%     dg = zeros(nsteps,5*nsteps-2) ;
    
    g = [];
    dg = [];
    
    h = zeros(6*nsteps,1) ;
    dh = zeros(6*nsteps,8*nsteps-2);

%     h(1:6) = z(1:6,:) ;
%     dh(1:6,1:6) = eye(6) ;

    for i = 1:nsteps
        
        if i == 1
            h(1:6) = z(1:6,:) - z0' ;
            dh(1:6,1:6) = eye(6) ; 
        else
            h(6*i-5:6*i) = zx(6*i-5:6*i)-zx(6*i-11:6*i-6)-...
                               dt*odefun(zx(6*i-11:6*i-6),zu(2*i-3:2*i-2)) ;
                           
            dh(6*i-5:6*i,6*i-11:6*i) = [-eye(6)-dt*statepart_hand(zx(6*i-11:6*i-6),zu(2*i-3:2*i-2)),eye(6)] ;
            dh(6*i-5:6*i,6*nsteps+2*i-3:6*nsteps+2*i-2) = -dt*inputpart_hand(zx(6*i-11:6*i-6),zu(2*i-3:2*i-2)) ;
        end
        
%         g(i) = (0.7^2)-((z(3*i-2)-3.5)^2+(z(3*i-1)+0.5)^2) ;
%         dg(i,3*i-2:3*i-1) = -2*[z(3*i-2)-3.5, z(3*i-1)+0.5] ;

    end

    dg = dg' ;
    dh= dh' ;
end