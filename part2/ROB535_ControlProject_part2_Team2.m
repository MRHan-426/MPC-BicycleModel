function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team2(TestTrack, Xobs_seen, curr_state)

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
x_interp = linspace(1,10,l1*20); %previously X20
r1 = interp1(x_orig,track.cline(1,:),x_interp); %
r2 = interp1(x_orig,track.cline(2,:),x_interp);
track.cline = [r1;r2];
r1 = interp1(x_orig,track.bl(1,:),x_interp);
r2 = interp1(x_orig,track.bl(2,:),x_interp);
track.bl = [r1;r2];
r1 = interp1(x_orig,track.br(1,:),x_interp);
r2 = interp1(x_orig,track.br(2,:),x_interp);
track.br = [r1;r2];


%number of states and inputs in dynamic model
nstates=6;
ninputs=2;

%input ranges (first row is wheel angle, second row forward force)
input_range=[-0.499,   0.499;...
            -4999,4999]; 

%load('project_traj_01_timestepfast_centerline.mat'); %loads Eric's fast trajectory
%load('project_traj_01_timestepfast.mat'); %loads Eric's fast trajectory
%load('project_traj_01_timestepslow.mat'); %loads Nikhil's slow trajectory
%load('project_traj_01_timestepfast_centerline2.mat'); %loads Meet's stable trajectory

%%
persistent U_ref Y_ref
% U_ref = control input
% Y_ref = vehicle state, initial condition = (14)

if isempty(U_ref)
    %load('ROB535_ControlProject_part1_Team2.mat', 'U', 'Y');
    %load('project_traj_01_timestep_centerline_ND.mat', 'U', 'Y');
    load('trajectories/RefTraj.mat', 'U', 'Y');
    U_ref = U'; %already 0.01 timestep
    Y_ref = Y'; %already 0.01 timestep
end

dt=0.01;  % time discretization
persistent T
if isempty(T)
    len1 = size(Y_ref,2);
    T = 0:dt:dt*(len1-1); %time span
end

%%
persistent dfdx dfdu

if isempty(dfdx)
    %Setting up the dynamics and partials
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

    % F_total=sqrt((Nw*Fx)^2+(F_yr^2)); %(10)
    % F_max=0.7*m*g; %(11)

    % if F_total>F_max
    %     
    %     Fx=F_max/F_total*Fx; %(12)
    %   
    %     F_yr=F_max/F_total*F_yr; %(13)
    % end

    %vehicle dynamics  (1)
    % x u y v phi r
    f1 = u*cos(phi)-v*sin(phi);
    f2 = (-f*m*g+Nw*Fx-F_yf*sin(delta_f))/m+v*r;
    f3 = u*sin(phi)+v*cos(phi);
    f4 = (F_yf*cos(delta_f)+F_yr)/m-u*r;
    f5 = r;
    f6 = (F_yf*a*cos(delta_f)-F_yr*b)/Iz;
     
    % state = [x u y v phi r]
    dfdx = [diff(f1,x)   diff(f1,u)   diff(f1,y)   diff(f1,v)   diff(f1,phi)   diff(f1,r)
            diff(f2,x)   diff(f2,u)   diff(f2,y)   diff(f2,v)   diff(f2,phi)   diff(f2,r)
            diff(f3,x)   diff(f3,u)   diff(f3,y)   diff(f3,v)   diff(f3,phi)   diff(f3,r)
            diff(f4,x)   diff(f4,u)   diff(f4,y)   diff(f4,v)   diff(f4,phi)   diff(f4,r)
            diff(f5,x)   diff(f5,u)   diff(f5,y)   diff(f5,v)   diff(f5,phi)   diff(f5,r)
            diff(f6,x)   diff(f6,u)   diff(f6,y)   diff(f6,v)   diff(f6,phi)   diff(f6,r)];
    
    % input  = [delta_f Fx]
    % delta_f = front wheel steering angle; 
    % Fx = traction force generated at each tire by the vehicle’s motor;
    dfdu = [diff(f1,delta_f) diff(f1,Fx)
            diff(f2,delta_f) diff(f2,Fx)
            diff(f3,delta_f) diff(f3,Fx)
            diff(f4,delta_f) diff(f4,Fx)
            diff(f5,delta_f) diff(f5,Fx)
            diff(f6,delta_f) diff(f6,Fx)];


    %these are the system linearized in discrete time about the reference
    %trajectory i.e. x(i+1)=A_i*x_i+B_i*u_i
    dfdx = matlabFunction(dfdx);
    dfdu = matlabFunction(dfdu);

end


A=@(i) eye(nstates) + dt*dfdx(U_ref(1,i),Y_ref(5,i),Y_ref(6,i),Y_ref(2,i),Y_ref(4,i));
B=@(i) dt*dfdu(U_ref(1,i),Y_ref(6,i),Y_ref(2,i),Y_ref(4,i));

%%
%11 timesteps for 3 states, 10 timesteps for 2 inputs
npred=20;
Ndec=(npred+1)*nstates + ninputs*npred;

%decision variable will be z=[x_1...x_11;u_1...u_10] 
% (x refers to state vector, u refers to input vector)

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

%find start point
diffs2 = (Y_ref - curr_state).^2;
sum_diffs2 = diffs2(1,:) + diffs2(3,:);
[~, t_index] = min(sum_diffs2);

FLAG_terminate = 0;

%%
%for i=t_index:min(t_index+batch-1, length(T)-1)
for local_i=1:batch
    diffs2 = (Y_ref(:,t_index:min(t_index+batch*2, size(Y_ref, 2))) - Y(:,local_i)).^2;
    sum_diffs2 = diffs2(1,:) + diffs2(3,:);
    [~, j] = min(sum_diffs2);
    %i = t_index + j - 1;
    i = local_i + t_index - 1;
    
    %shorten prediction horizon if we are at the end of trajectory
    npred_i=min([npred,length(T)-i-1]);
    
    %calculate error in states(actual - reference)
    eY(:,local_i)=Y(:,local_i)-Y_ref(:,i);

    %generate equality constraints
    [Aeq,beq]=eq_cons(i,A,B,eY(:,local_i),npred_i,nstates,ninputs);
    
    %generate inequality constraints
    car = [Y(1,local_i); Y(3,local_i)];
    [A_con,b_con] = find_constraints(car,track,Xobs_seen);
    
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
        horizon_ref = [reshape(Y_ref(:,i:i+npred_i),1,[]), zeros(1, npred_i*ninputs)];
        bineq = bineq - Aineq*horizon_ref';
        % use QP to solve the problem
        [x,fval,exitflag,output] = quadprog(H,func,Aineq,bineq,Aeq,beq,Lb,Ub,[],options);
        if exitflag == -2
            n = size(A_con, 1);
%             disp(['reducing constraints to ' num2str(n-1)]);
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
    u_mpc(:,local_i)=x(nstates*(npred_i+1)+1:nstates*(npred_i+1)+ninputs);
    
    %get input
    U(:,local_i)=u_mpc(:,local_i)+U_ref(:,i);
    
    %simulate model
    [~,ztemp]=ode45(@(t,z)kinematic_bike_dynamics(t,z,U(:,local_i),0,Nw,f,Iz,a,b,By,Cy,Dy,Ey,Shy,Svy,m,g),[0 dt],Y(:,local_i));
    
    %store final state
    Y(:,local_i+1)=ztemp(end,:)';
    disp(['Done loop ' num2str(i) ' out of ' num2str(length(T)-1)])
end

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
% Creates linear inequality in the form of A*x < b
% The constraint corresponds to all points to the left of the line traced
% FROM p1 TO p2
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

%% find_constraints.m
function [A, b] = find_constraints(car, track, obstacles)

% Find the closest point along the lines to the car
[left_i, left_j] = nearest_points(track.bl, car);
[right_i, right_j] = nearest_points(track.br, car);
[center_i, center_j] = nearest_points(track.cline, car);

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

% Find the next obstacle in front of the car
inds = 1:n_obs;
in_front = inds(obstacle_order > center_i - 2);
if any(in_front)
    [~, i] = mink(obstacle_order(in_front), 2);
    next_obs = in_front(i);
    
    for o = next_obs
        % Only if we are within a certain distance of the obstacle
        if sum((obstacles{o}(1,:)' - car).^2)^0.5 < 30
            % Choose a side to pass the obstacle on
            [left_clearance, obs_left] = clearance(track.bl, obstacles{o});
            [right_clearance, obs_right] = clearance(track.br, obstacles{o});

            if left_clearance > right_clearance
                for ob_i = 1:size(obs_left, 2)
                    [A_obs, b_obs] = constraint(car, obs_left(:,ob_i));
                    A = [A; A_obs];
                    b = [b; b_obs];
                end
            else
                for ob_i = 1:size(obs_right, 2)
                    [A_obs, b_obs] = constraint(obs_right(:,ob_i), car);
                    A = [A; A_obs];
                    b = [b; b_obs];
                end
            end
        end
    end
end

end

%% bound_cons.m
function [Lb,Ub]=bound_cons(initial_idx,U_ref,npred,input_range,nstates,ninputs)
%time_idx is the index along uref the initial condition is at
xsize=(npred+1)*nstates;
usize=npred*ninputs;

Lb=[];
Ub=[];
for j=1:ninputs
Lb=[Lb;input_range(j,1)-U_ref(j,initial_idx:initial_idx+npred-1)];
Ub=[Ub;input_range(j,2)-U_ref(j,initial_idx:initial_idx+npred-1)];
end

Lb=reshape(Lb,[usize,1]);
Ub=reshape(Ub,[usize,1]);

Lb=[-Inf(xsize,1);Lb];
Ub=[Inf(xsize,1);Ub];

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

diffs = line - p;
dists_2 = diffs(1,:).^2 + diffs(2,:).^2;
[~, inds] = mink(dists_2, 2);

i = inds(1);
j = inds(2);

if j < i
    [i, j] = deal(j, i);
end

end

%% clearance.m
% Returns the clearance between obst and line and the point o on obst
% which is closest to the line
function [d, os] = clearance(line, obst)

closest_inds = [
    nearest_points(line, obst(1, :)');
    nearest_points(line, obst(2, :)');
    nearest_points(line, obst(3, :)');
    nearest_points(line, obst(4, :)');
];

diffs = line(:, closest_inds) - obst';
dists_2 = diffs(1,:).^2 + diffs(2,:).^2;

[ds, is] = mink(dists_2, 2);
d = min(ds).^0.5;
%o = obst(i,:)';
os = [];
for p = [-0.05, 0.01, 0.15]
    os = [os (1-p)*obst(is,:)' + p*line(:, closest_inds(is))];
end

end










