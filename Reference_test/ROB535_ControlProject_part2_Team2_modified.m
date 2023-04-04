function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team2_modified(TestTrack, Xobs_seen, curr_state)

curr_state = curr_state';

%% parameters
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


%Interpolates all three track lines to the same set of evenly spaced points 
%to maintain consistency between them. 
%This is important for subsequent calculations and visualization operations.
track = TestTrack;
l1 = size(track.cline,2);
x_orig = linspace(1,10,l1);
x_interp = linspace(1,10,l1*20);
r1 = interp1(x_orig,track.cline(1,:),x_interp);
r2 = interp1(x_orig,track.cline(2,:),x_interp);
track.cline = [r1;r2];
r1 = interp1(x_orig,track.bl(1,:),x_interp);
r2 = interp1(x_orig,track.bl(2,:),x_interp);
track.bl = [r1;r2];
r1 = interp1(x_orig,track.br(1,:),x_interp);
r2 = interp1(x_orig,track.br(2,:),x_interp);
track.br = [r1;r2];
theta = interp1(x_orig,track.theta,x_interp);

%number of states and inputs in dynamic model
nstates=6;
ninputs=2;

%input ranges (first row is wheel angle, second row forward force)
input_range=[-0.499,   0.499;...
            -4999,4999]; 

%%
% persistent U_ref Y_ref
% 
% if isempty(U_ref)
%     load('part1_track.mat', 'U', 'Y');
%     U_ref = U';
%     Y_ref = Y';
% end

%% Initial Conditions

x0   =   287;
u0   =   5.0;
y0   =  -176;
v0   =   0.0;
psi0 =   2.0;
r0   =   0.0;

%% Generate Reference Trajectory
nsteps = size(track.bl,2);

lowbounds = min(track.bl,track.br);
highbounds = max(track.bl,track.br);

[lb,ub]=bound_cons(nsteps,theta ,lowbounds, highbounds);


options = optimoptions('fmincon','SpecifyConstraintGradient',true,...
                       'SpecifyObjectiveGradient',true) ;
                   
X0 = [repmat([x0,u0,y0,v0,psi0,r0],1,nsteps), repmat([0,0],1,nsteps-1)];

cf=@costfun;
nc=@nonlcon;

z=fmincon(cf,X0,[],[],[],[],lb',ub',nc,options);

Y_ref=reshape(z(1:6*nsteps),6,nsteps);
U_ref=reshape(z(6*nsteps+1:end),2,nsteps-1);


%%
dt=0.01;
persistent T
if isempty(T)
    len1 = size(Y_ref,2);
    T = 0:dt:dt*(len1-1); %time span
end

%% Setting up the dynamics and partials
persistent dfdx dfdu

if isempty(dfdx)
    syms x u y v phi r Fx delta_f
    % x u y v phi r
    %slip angle functions in degrees
    a_f=rad2deg(delta_f-atan2(v+a*r,u)); %(8)
    a_r=rad2deg(-atan2((v-b*r),u)); %(9)

    %Nonlinear Tire Dynamics
    phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy)); %(6)
    phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy)); %(7)

    F_zf=b/(a+b)*m*g; %(2)
    F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy; %(3)

    F_zr=a/(a+b)*m*g; %(4)
    F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy; %(5)

    %vehicle dynamics  (1)
    % x u y v phi r
    f1 = u*cos(phi)-v*sin(phi);
    f2 = (-f*m*g+Nw*Fx-F_yf*sin(delta_f))/m+v*r;
    f3 = u*sin(phi)+v*cos(phi);
    f4 = (F_yf*cos(delta_f)+F_yr)/m-u*r;
    f5 = r;
    f6 = (F_yf*a*cos(delta_f)-F_yr*b)/Iz;
     
    %state = [x u y v phi r]
    dfdx = jacobian([f1,f2,f3,f4,f5,f6],[x,u,y,v,phi,r]);
    
    % input  = [delta_f Fx]
    % delta_f = front wheel steering angle; 
    % Fx = traction force generated at each tire by the vehicle’s motor;
    dfdu = jacobian([f1,f2,f3,f4,f5,f6],[delta_f,Fx]);

    %these are the system linearized in discrete time about the reference
    %trajectory i.e. x(i+1)=A_i*x_i+B_i*u_i
    dfdx = matlabFunction(dfdx);
    dfdu = matlabFunction(dfdu);
end


A=@(i) eye(nstates) + dt*dfdx(U_ref(1,i),Y_ref(5,i),Y_ref(6,i),Y_ref(2,i),Y_ref(4,i));
B=@(i) dt*dfdu(U_ref(1,i),Y_ref(6,i),Y_ref(2,i),Y_ref(4,i));

%%
%decision variable will be z=[x_1...x_11;u_1...u_10] 
% (x refers to state vector, u refers to input vector)
npred=20;

batch = 0.5/dt+1;

%final trajectory
Y=NaN(nstates,batch);

%applied inputs
U=NaN(ninputs,batch);

%input from QP
u_mpc=NaN(ninputs,batch);

%error in states (actual - reference)
eY=NaN(nstates,batch);

%set initial condition
Y(:,1)=curr_state;

%calculate euclidean distance
euc_dis = (Y_ref(1,:)-curr_state(1,:)).^2 + (Y_ref(3,:)-curr_state(3,:)).^2;
[~, t_index] = min(euc_dis);

FLAG_terminate = 0;

%%
% tic
for j=1:batch

    i = j + t_index - 1;
    
    %shorten prediction horizon if we are at the end of trajectory
    npred_i=min([npred,length(T)-i-1]);
    
    %calculate error in states(actual - reference)
    eY(:,j)=Y(:,j)-Y_ref(:,i);

    %generate equality constraints
    [Aeq,beq]=eq_cons(i,A,B,eY(:,j),npred_i,nstates,ninputs);
    
    %generate inequality constraints
    vehicle = [Y(1,j); Y(3,j)];
    [A_con,b_con] = build_constraints(vehicle,track,Xobs_seen);
    
    %generate boundary constraints
    [Lb,Ub]=bound_cons(i,U_ref,npred_i,input_range,nstates,ninputs);
    
    %cost for states [x, longi vel, y, lat vel, heading, rot vel] 
    Q=[8, 0.4, 8, 0.4, 160, 2];
    
    %cost for inputs [steering, throttle]
    R=[150, 0];
    
    H=diag([repmat(Q,[1,npred_i+1]),repmat(R,[1,npred_i])]);
    
    func=zeros(nstates*(npred_i+1)+ninputs*npred_i,1);

    options = optimoptions('quadprog', 'Display', 'off');
    while 1
        % inequality constraints
        [Aineq,bineq]=ineq_cons(A_con,b_con, npred_i,nstates,ninputs);
        bineq = bineq - Aineq*[reshape(Y_ref(:,i:i+npred_i),1,[]), zeros(1, npred_i*ninputs)]';
        % use QP to solve the problem
        [x,~,exitflag,~,~] = quadprog(H,func,Aineq,bineq,Aeq,beq,Lb,Ub,[],options);
        if exitflag == -2
            n = size(A_con, 1);
            A_con = A_con(1:n-1,:);
            b_con = b_con(1:n-1,:);
            if n <= 2
                FLAG_terminate = 1;
                break;
            end
        else
            break;
        end
    end
    
    
    if FLAG_terminate == 1
        break;
    end
    
    if size(x) < nstates+ninputs
        break
    end
    
    %get linearized input
    u_mpc(:,j)=x(nstates*(npred_i+1)+1:nstates*(npred_i+1)+ninputs);
    
    %get input
    U(:,j)=u_mpc(:,j)+U_ref(:,i);
    
    %simulate model
    [~,y]=ode45(@(t,z)kinematic_bike_dynamics(t,z,U(:,j),0,Nw,f,Iz,a,b,By,Cy,Dy,Ey,Shy,Svy,m,g),[0 dt],Y(:,j));
    
    %store final state
    Y(:,j+1)=y(end,:)';
    disp(['Done loop ' num2str(i) ' out of ' num2str(length(T)-1)])

end
%toc;

%%
sol_2 = U';
t_index = t_index + batch;
if t_index >= length(T)
    FLAG_terminate = 1;
end

if FLAG_terminate == 1
    clear ROB535_ControlProject_part2_Team2;
end

end

%% constraints.m
function [A, b] = constraint(p1, p2)

d = p1 - p2;
A = [-d(2) d(1)];
b = d(1)*p2(2) - d(2)*p2(1);

end

%% eq_cons.m
function [Aeq,beq]=eq_cons(initial_idx,A,B,x_initial,npred,nstates,ninputs)
%build matrix for A_i*x_i+B_i*u_i-x_{i+1}=0
%in the form Aeq*z=beq
%initial_idx specifies the time index of initial condition from the reference trajectory 
%A and B are function handles above

%initial condition
x_initial=x_initial(:);

%size of decision variable and size of part holding states
zsize=(npred+1)*nstates+npred*ninputs;
xsize=(npred+1)*nstates;

Aeq=zeros(xsize,zsize);
Aeq(1:nstates,1:nstates)=eye(nstates); %initial condition 
beq=zeros(xsize,1);
beq(1:nstates)=x_initial;

state_idxs=nstates+1:nstates:xsize;
input_idxs=xsize+1:ninputs:zsize;

for i=1:npred
    %negative identity for i+1
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i):state_idxs(i)+nstates-1)=-eye(nstates);
    
    %A matrix for i
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i)-nstates:state_idxs(i)-1)=A(initial_idx+i-1);
    
    %B matrix for i
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,input_idxs(i):input_idxs(i)+ninputs-1)=B(initial_idx+i-1);
end

end

%% ineq_cons.m
function [Aineq, bineq] = ineq_cons(A,b,npred,nstates,ninputs)

ncons = size(b,1);
zsize = (npred+1)*nstates+npred*ninputs;
xsize = (npred+1)*ncons;

Aineq = zeros(xsize,zsize);
bineq = ones(xsize,1);

state_idxs=nstates+1:nstates:zsize;
con_idxs=(ncons+1):ncons:xsize;

if npred <= 1
    return
end

for i=npred-1:npred
    Aineq(con_idxs(i):con_idxs(i)+ncons-1, state_idxs(i)) = A(:,1);
    Aineq(con_idxs(i):con_idxs(i)+ncons-1, state_idxs(i)+2) = A(:,2);
    bineq(con_idxs(i):con_idxs(i)+ncons-1) = b;
end

end

%% build_constraints.m
function [A, b] = build_constraints(vehicle, track, obstacles)

% Find the closest point along the lines to the vehicle
[left_i, left_j] = nearest_points(track.bl, vehicle);
[right_i, right_j] = nearest_points(track.br, vehicle);
[center_i, ~] = nearest_points(track.cline, vehicle);

% Create constraints from the sides of the road
[A_left, b_left] = constraint(track.bl(:,left_j), track.bl(:,left_i));
[A_right, b_right] = constraint(track.br(:,right_i), track.br(:,right_j));
A = [A_left; A_right];
b = [b_left; b_right];

% Order the obstacles along the track
n_obs = size(obstacles, 2);
obstacle_order = zeros(1, n_obs);
for o = 1:n_obs
    obstacle_order(o) = nearest_points(track.cline, obstacles{o}(1, :)');
end

% Find the next obstacle in front of the vehicle
inds = 1:n_obs;
in_front = inds(obstacle_order > center_i - 2);
if any(in_front)
    [~, i] = mink(obstacle_order(in_front), 2);
    next_obs = in_front(i);
    
    for o = next_obs
        % Only if we are within a certain distance of the obstacle
        if sum((obstacles{o}(1,:)' - vehicle).^2)^0.5 < 30
            % Choose a side to pass the obstacle on
            [left_clearance, obs_left] = clearance(track.bl, obstacles{o});
            [right_clearance, obs_right] = clearance(track.br, obstacles{o});

            if left_clearance > right_clearance
                for ob_i = 1:size(obs_left, 2)
                    [A_obs, b_obs] = constraint(vehicle, obs_left(:,ob_i));
                    A = [A; A_obs];
                    b = [b; b_obs];
                end
            else
                for ob_i = 1:size(obs_right, 2)
                    [A_obs, b_obs] = constraint(obs_right(:,ob_i), vehicle);
                    A = [A; A_obs];
                    b = [b; b_obs];
                end
            end
        end
    end
end

end

%% bound_cons.m
% function [Lb,Ub]=bound_cons(initial_idx,U_ref,npred,input_range,nstates,ninputs)
% %time_idx is the index along uref the initial condition is at
% xsize=(npred+1)*nstates;
% usize=npred*ninputs;
% 
% Lb=[];
% Ub=[];
% for j=1:ninputs
% Lb=[Lb;input_range(j,1)-U_ref(j,initial_idx:initial_idx+npred-1)];
% Ub=[Ub;input_range(j,2)-U_ref(j,initial_idx:initial_idx+npred-1)];
% end
% 
% Lb=reshape(Lb,[usize,1]);
% Ub=reshape(Ub,[usize,1]);
% 
% Lb=[-Inf(xsize,1);Lb];
% Ub=[Inf(xsize,1);Ub];
% 
% end

function [lb,ub]=bound_cons(nsteps,theta ,lowbounds, highbounds)
ub = [];
lb = [];

for i = 1:nsteps

    ub = [ub,[highbounds(1,i), +inf,highbounds(2,i), +inf, theta(i)+pi/2, +inf]];

    lb = [lb,[lowbounds(1,i), -inf, lowbounds(2,i), -inf, theta(i)-pi/2, -inf]];

end

ub = [ub,repmat([2500,0.5],1,nsteps-1) ]';
lb = [lb,repmat([-5000,-0.5],1,nsteps-1) ]';

end

%% kinematic_bike_dynamics.m
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

%% nearest_points.m
function [i, j] = nearest_points(line, p)

euc_dis = (line(1,:) - p(1,:)).^2 + (line(2,:) - p(2,:)).^2;
[~, t_index] = mink(euc_dis, 2);

i = t_index(1);
j = t_index(2);

if j < i
    [i, j] = deal(j, i);
end

end

%% clearance.m
function [d, obstacle] = clearance(line, obs)

closest_inds = [
    nearest_points(line, obs(1, :)');
    nearest_points(line, obs(2, :)');
    nearest_points(line, obs(3, :)');
    nearest_points(line, obs(4, :)');
];

diffs = line(:, closest_inds) - obs';
dists_2 = diffs(1,:).^2 + diffs(2,:).^2;

[ds, is] = mink(dists_2, 2);
d = min(ds).^0.5;
obstacle = [];
for p = [-0.05, 0.01, 0.15]
    obstacle = [obstacle (1-p)*obs(is,:)' + p*line(:, closest_inds(is))];
end

end










