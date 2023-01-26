%% ME 449 Final Project
% Kevin Nella

addpath('/Users/Kevin/repos/ME449/ModernRobotics/packages/MATLAB/mr')

%% Setting Segment timing, Controls, Initial/Final Cube Configuration for newTask

%Tf_1 = Time between segments for BEST AND OVERSHOOT
Tf1 = 2; Tf2 = 0.5; Tf3 = 0.63; Tf4 = 0.5; Tf5 = 2; Tf6 = 0.5; Tf7 = 0.63; Tf8 = 0.5;
Tf_1= [Tf1 Tf2 Tf3 Tf4 Tf5 Tf6 Tf7 Tf8];

% "Best" control constants
FF_b = true; % Indicates use of Feedforward term
Kp_b = 3*eye(6); %Kp gain matrix
Ki_b = 0*eye(6); %Ki gain matrix

% "Overshoot" control constants
FF_o = true; % Indicates use of Feedforward term
Kp_o = 5*eye(6); %Kp gain matrix
Ki_o = 17*eye(6); %Ki gain matrix

%%%%%%%%%%%%%%%%% newTask %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Tf_2 = Time between segments for NEWTASK
Tf1 = 2; Tf2 = 0.5; Tf3 = 0.63; Tf4 = 0.5; Tf5 = 6; Tf6 = 0.5; Tf7 = 0.63; Tf8 = 0.5;
Tf_2= [Tf1 Tf2 Tf3 Tf4 Tf5 Tf6 Tf7 Tf8];

% "newTask" control constants
FF_n = true; % Indicates use of Feedforward term
Kp_n = 10*eye(6); %Kp gain matrix
Ki_n = 0*eye(6); %Ki gain matrix

% initial block configuration for "newTask"
Tsc_i_2 = [1 0 0 1;
           0 1 0 -1;
           0 0 1 0.025;
           0 0 0 1];
% final block configuration for "newTask"
Tsc_f_2 = [1 0 0 1;
           0 1 0 2;
           0 0 1 0.025;
           0 0 0 1];

%% Generating Full Reference Trajectory

% Body Jacobian
B1 = [0;0;1;0;.033;0];
B2 = [0;-1;0;-0.5076;0;0];
B3 = [0;-1;0;-0.3526;0;0];
B4 = [0;-1;0;-0.2176;0;0];
B5 = [0;0;1;0;0;0];
Blist = [B1 B2 B3 B4 B5];

% initial config of frame {b} of base to {s}
Tsb_i = [1 0 0 0;
         0 1 0 0;
         0 0 1 0.0963;
         0 0 0 1];

% initial config of frame {0} to {b}
Tb0 = [1 0 0 0.1662;
         0 1 0 0;
         0 0 1 0.0026;
         0 0 0 1];

%initial config (home config) of EE {e} to {0}
M0e = [1 0 0 0.033;
         0 1 0 0;
         0 0 1 0.6546;
         0 0 0 1];

% frame {e} relative to {s} at home config
%Tse_i = Tsb_i*Tb0*M0e; %used for milestone 2

Tse_i = [0 0 1 0;...            %as instructed in Final Step
         0 1 0 0;...
        -1 0 0 0.5;...
         0 0 0 1];

% {e} relative to {c} when grasping
Tce_gr = [0 0 1 0;
          0 1 0 0;
         -1 0 0 0.01;
          0 0 0 1];
% {e} relative to {c} at standoff
Tce_so = [0 0 1 0;
          0 1 0 0;
         -1 0 0 0.1;
          0 0 0 1];


g_state = 0; %gripper state

%% Reference trajectory for "best" and "overshoot"
k = 1;
N = Tf_1./(.01/k);
TN1 = sum(N);

% initial block configuration for "Best" and "Overshoot"
Tsc_i_1 = [1 0 0 1;
           0 1 0 0;
           0 0 1 0.025;
           0 0 0 1];
% final block configuration for "Best" and "Overshoot"
Tsc_f_1 = [0 1 0 0;
          -1 0 0 -1;
           0 0 1 0.025;
           0 0 0 1];

fprintf("Creating reference trajectories...")

traj_full1 = TrajectoryGenerator(Tse_i, Tsc_i_1,Tsc_f_1, Tce_gr, Tce_so, Tf_1, N);
traj_matrix1 = zeros(TN1,13); %initialize Nx13 matrix
j=1;

for n=1:8
    if n>=3 && n< 7
        g_state = 1;
    else 
        g_state = 0;
    end

    for i=1:N(n)
        traj_matrix1(j,:) = [traj_full1{n}{i}(1,1) traj_full1{n}{i}(1,2) traj_full1{n}{i}(1,3) ...
                            traj_full1{n}{i}(2,1) traj_full1{n}{i}(2,2) traj_full1{n}{i}(2,3) ...
                            traj_full1{n}{i}(3,1) traj_full1{n}{i}(3,2) traj_full1{n}{i}(3,3) ...
                            traj_full1{n}{i}(1,4) traj_full1{n}{i}(2,4) traj_full1{n}{i}(3,4) g_state];
        j=j+1;
    end
end

%% Reference trajectory for "newTask"
k = 1;
N2 = Tf_2./(.01/k);
TN2 = sum(N2);

traj_full2 = TrajectoryGenerator(Tse_i, Tsc_i_2,Tsc_f_2, Tce_gr, Tce_so, Tf_2, N2);
traj_matrix2 = zeros(TN2,13); %initialize Nx13 matrix
j=1;

for n=1:8
    if n>=3 && n< 7
        g_state = 1;
    else 
        g_state = 0;
    end

    for i=1:N2(n)
        traj_matrix2(j,:) = [traj_full2{n}{i}(1,1) traj_full2{n}{i}(1,2) traj_full2{n}{i}(1,3) ...
                            traj_full2{n}{i}(2,1) traj_full2{n}{i}(2,2) traj_full2{n}{i}(2,3) ...
                            traj_full2{n}{i}(3,1) traj_full2{n}{i}(3,2) traj_full2{n}{i}(3,3) ...
                            traj_full2{n}{i}(1,4) traj_full2{n}{i}(2,4) traj_full2{n}{i}(3,4) g_state];
        j=j+1;
    end
end

fprintf("\nDone.")
%% Final Computation constants

r = .0475; %radius of wheel in meters
l = 0.47/2; %length between center of chassis to rear axle
w = 0.3/2; %length between center of wheel to center plane of chassis
lw = l+w;
% F is pseudoinverse of H(0) used for determining Vb
F = r/4*[-1/lw  1/lw  1/lw  -1/lw; ...
          1     1     1       1;   ...
         -1     1    -1       1];
F6 = zeros(6,4); % initialize 6 x m=4 matrix
F6(3:5,1:4) = F;
dt = 0.01;
u_max = 20; % max wheel speed


%% Control Law & Computation for "best"
%initial transformation between space and EE frames AKA initial X
X = Tse_i;
% initial configuration AKA first line
cfg =   [0.4;0;0; ... %chassis config (q = phi,x,y) (0.4m position error)
         0;0;0;0;0; ... % arm joint angles (90 degree orientation error)
         0;0;0;0]; % wheel angles

cfg_all = zeros(TN1-1,13); % initialize csv variable
Xerr_all = zeros(6,TN1);
err_i = 0; %initial error integral
j = 1; % index for cfg_all

fprintf("\nComputing actual trajectory for best Task...")

for n=1:8
    for i=1:N(n)
        if (i+1) > N(n)
            if n+1 > 8
                break;
            end
            Xd = traj_full1{n}{i};
            Xd_n = traj_full1{n+1}{1};
        else
            Xd = traj_full1{n}{i};
            Xd_n = traj_full1{n}{i+1};
        end

        [V,err_i, Xerr] = FeedbackControl(X, Xd, Xd_n, Kp_b, Ki_b, dt, err_i, FF_b); %calculate commanded twist
        Xerr_all(1:6,j) = Xerr; % Store error over time

        %calculate u and theta from V
        thetalist = cfg(4:8); % current arm joint angles
        Jb_arm = JacobianBody(Blist,thetalist); %current body jacobian of arm
        T_0e = FKinBody(M0e,Blist,thetalist); % current EE to {0} transformation
        Jb_base = Adjoint(TransInv(T_0e)*TransInv(Tb0))*F6; %body jacobian of base
        Je = [Jb_base Jb_arm]; %complete jacobian
        u_th = pinv(Je,1e-2)*V; %extracting wheel speed and joint speeds from V (commanded twist)

        %plug in u and thetadot to get next state
        cfg = NextState(cfg,u_th,dt,u_max);
        cfg_all(j,1:12)  = cfg;
        cfg_all(j,13) = traj_matrix1(j,13);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Calculate X - "current" actual end-effector configuration T_se
        % of next iteration
            % Need Tsb, Tb0 (constant), T_0e
        phi = cfg(1); x = cfg(2); y = cfg(3);
        Tsb = [cos(phi)    -sin(phi)   0   x;...
                sin(phi)     cos(phi)   0   y;...
                0            0          1   .0963;...
                0            0          0   1];
        %need forward kinematics of e to 0 (T_0e)
            %need M0e, Blist, joint values
        thetalist = cfg(4:8); %next iteration's joint angles
        T0e = FKinBody(M0e,Blist,thetalist);
        X = Tsb*Tb0*T0e;

        j = j+1;
    end
end

fprintf("\nDone.")
fprintf("\nSaving .csv files...")
writematrix(cfg_all,'best.csv')
writematrix(Xerr_all,'best_Xerr.csv')
fprintf("\nDone.")
x = linspace(1,TN1,TN1);
fprintf("\nPlotting X_error for 'best'...")
figure(1)
bp = plot(x,Xerr_all);
xlabel("Time step","FontSize",14,"FontWeight","bold");
ylabel("Twist Error","FontSize",14,"FontWeight","bold");
title("Error Convergence - 'Best'","FontSize",14,"FontWeight","bold");
legend([bp(1),bp(2),bp(3),bp(4),bp(5),bp(6)],'\omega_{x}','\omega_{y}','\omega_{z}','v_{x}','v_{y}','v_{z}','FontWeight',"bold",'FontSize',14);
fprintf("Done.")
%% Control Law & Computation for "overshoot"
%initial transformation between space and EE frames AKA initial X
X = Tse_i;
% initial configuration AKA first line
cfg =   [0.4;0;0; ... %chassis config (q = phi,x,y) (0.4m position error)
         0;0;0;0;0; ... % arm joint angles (90 degree orientation error)
         0;0;0;0]; % wheel angles

cfg_all = zeros(TN1-1,13); % initialize csv variable
err_i = 0; %initial error integral
Xerr_all = zeros(6,TN1);
j = 1; % index for cfg_all

fprintf("\nComputing actual trajectory for overshoot Task...")

for n=1:8
    for i=1:N(n)
        if (i+1) > N(n)
            if n+1 > 8
                break;
            end
            Xd = traj_full1{n}{i};
            Xd_n = traj_full1{n+1}{1};
        else
            Xd = traj_full1{n}{i};
            Xd_n = traj_full1{n}{i+1};
        end

        [V,err_i, Xerr] = FeedbackControl(X, Xd, Xd_n, Kp_o, Ki_o, dt, err_i, FF_o); %calculate commanded twist
        Xerr_all(1:6,j) = Xerr; % Store error over time

        %calculate u and theta from V
        thetalist = cfg(4:8); % current arm joint angles
        Jb_arm = JacobianBody(Blist,thetalist); %current body jacobian of arm
        T_0e = FKinBody(M0e,Blist,thetalist); % current EE to {0} transformation
        Jb_base = Adjoint(TransInv(T_0e)*TransInv(Tb0))*F6; %body jacobian of base
        Je = [Jb_base Jb_arm]; %complete jacobian
        u_th = pinv(Je,1e-2)*V; %extracting wheel speed and joint speeds from V (commanded twist)

        %plug in u and thetadot to get next state
        cfg = NextState(cfg,u_th,dt,u_max);
        cfg_all(j,1:12)  = cfg;
        cfg_all(j,13) = traj_matrix1(j,13);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Calculate X - "current" actual end-effector configuration T_se
        % of next iteration
            % Need Tsb, Tb0 (constant), T_0e
        phi = cfg(1); x = cfg(2); y = cfg(3);
        Tsb = [cos(phi)    -sin(phi)   0   x;...
                sin(phi)     cos(phi)   0   y;...
                0            0          1   .0963;...
                0            0          0   1];
        %need forward kinematics of e to 0 (T_0e)
            %need M0e, Blist, joint values
        thetalist = cfg(4:8); %next iteration's joint angles
        T0e = FKinBody(M0e,Blist,thetalist);
        X = Tsb*Tb0*T0e;

        j = j+1;
    end
end
fprintf("\nDone.")
fprintf("\nSaving .csv files...")
writematrix(cfg_all,'overshoot.csv')
writematrix(Xerr_all,'overshoot_Xerr.csv')
fprintf("\nPlotting X_error for 'overshoot' task...")
x = linspace(1,TN1,TN1);
figure(2)
op = plot(x,Xerr_all);
xlabel("Time step","FontSize",14,"FontWeight","bold");
ylabel("Twist Error","FontSize",14,"FontWeight","bold");
title("Error Convergence - 'Overshoot'","FontSize",14,"FontWeight","bold");
legend([op(1),op(2),op(3),op(4),op(5),op(6)],'\omega_{x}','\omega_{y}','\omega_{z}','v_{x}','v_{y}','v_{z}','FontWeight',"bold",'FontSize',14);
fprintf("\nDone.")
%% Control Law & Computation for "newTask"
%initial transformation between space and EE frames AKA initial X
X = Tse_i;
% initial configuration AKA first line
cfg =   [0.4;0;0; ... %chassis config (q = phi,x,y) (0.4m position error)
         0;0;0;0;0; ... % arm joint angles (90 degree orientation error)
         0;0;0;0]; % wheel angles

cfg_all = zeros(TN2-1,13); % initialize csv variable
err_i = 0; %initial error integral
Xerr_all = zeros(6,TN2);
j = 1; % index for cfg_all

fprintf("\nComputing actual trajectory for newTask...")

for n=1:8
    for i=1:N2(n)
        if (i+1) > N2(n)
            if n+1 > 8
                break;
            end
            Xd = traj_full2{n}{i};
            Xd_n = traj_full2{n+1}{1};
        else
            Xd = traj_full2{n}{i};
            Xd_n = traj_full2{n}{i+1};
        end

        [V,err_i, Xerr] = FeedbackControl(X, Xd, Xd_n, Kp_n, Ki_n, dt, err_i, FF_n); %calculate commanded twist
        Xerr_all(1:6,j) = Xerr; % Store error over time

        %calculate u and theta from V
        thetalist = cfg(4:8); % current arm joint angles
        Jb_arm = JacobianBody(Blist,thetalist); %current body jacobian of arm
        T_0e = FKinBody(M0e,Blist,thetalist); % current EE to {0} transformation
        Jb_base = Adjoint(TransInv(T_0e)*TransInv(Tb0))*F6; %body jacobian of base
        Je = [Jb_base Jb_arm]; %complete jacobian
        u_th = pinv(Je,1e-2)*V; %extracting wheel speed and joint speeds from V (commanded twist)


        %plug in u and thetadot to get next state
        cfg = NextState(cfg,u_th,dt,u_max);
        cfg_all(j,1:12)  = cfg;
        cfg_all(j,13) = traj_matrix2(j,13);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Calculate X - "current" actual end-effector configuration T_se
        % of next iteration
            % Need Tsb, Tb0 (constant), T_0e
        phi = cfg(1); x = cfg(2); y = cfg(3);
        Tsb = [cos(phi)    -sin(phi)   0   x;...
                sin(phi)     cos(phi)   0   y;...
                0            0          1   .0963;...
                0            0          0   1];
        %need forward kinematics of e to 0 (T_0e)
            %need M0e, Blist, joint values
        thetalist = cfg(4:8); %next iteration's joint angles
        T0e = FKinBody(M0e,Blist,thetalist);
        X = Tsb*Tb0*T0e;

        j = j+1;
    end
end

fprintf("\nDone.")
fprintf("\nSaving .csv files...")
writematrix(cfg_all,'newTask.csv')
writematrix(Xerr_all,'newTask_Xerr.csv')
fprintf("\nDone.")
fprintf("\nPlotting X_error for 'overshoot' task...")
x = linspace(1,TN2,TN2);
figure(3)
np = plot(x,Xerr_all);
xlabel("Time step","FontSize",14,"FontWeight","bold");
ylabel("Twist Error","FontSize",14,"FontWeight","bold");
title("Error Convergence - 'newTask'","FontSize",14,"FontWeight","bold");
legend([np(1),np(2),np(3),np(4),np(5),np(6)],'\omega_{x}','\omega_{y}','\omega_{z}','v_{x}','v_{y}','v_{z}','FontWeight',"bold",'FontSize',14);
fprintf("\nDone.")
%% Function - NextState

function cfg = NextState(cfg_c, ctrl, dt, u_max)
    qk = cfg_c(1:3); ja_i = cfg_c(4:8); wa_i = cfg_c(9:12); % extract current chassis config, arm config, wheel config
    u = ctrl(1:4); arm_s = ctrl(5:9);

    for i=1:4
        if abs(u(i)) > u_max
            if u(i) < 0
                u(i) = -1*u_max;
            elseif u(i) > 0
                u(i) = u_max;
            end
        end
    end

    % setting up F (pseudo inverse of H(0)
    r = .0475; %radius of wheel in meters
    l = 0.47/2; %length between center of chassis to rear axle
    w = 0.3/2; %length between center of wheel to center plane of chassis
    lw = l+w;
    F = r/4*[-1/lw  1/lw   1/lw   -1/lw; ...
                1     1     1       1;   ...
               -1     1    -1       1];
    % transforming wheel speed to body twist
    Vb = F*u;
    
    %dqb - change of chassis coordinates relative to body frame
    if Vb(1) == 0
        dqb = [0;Vb(2);Vb(3)];
    else
        dqb = [Vb(1);...
              (Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1); ...
              (Vb(3)*sin(Vb(1))+Vb(2)*(1-cos(Vb(1))))/Vb(1)];
    end

    %dq: dqb in fixed frame {s}
    phik = qk(1); %current chassis phi_k
    T = [1  0   0; ...
         0  cos(phik) -sin(phik); ...
         0  sin(phik)  cos(phik)];
    dq = T*dqb;

    %new chassis config
    cfg = zeros(12,1);
    cfg(1:3) = qk+dq*dt;
    %new joint angles
    ja_f = ja_i + dt*arm_s;
    cfg(4:8) = ja_f;
    % new wheel angles
    wa_f = wa_i + dt*u;
    cfg(9:12) = wa_f;

end

%% Function - TrajectoryGenerator

function [traj_full] = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_gr, Tce_so, Tf, N)
    
    %segment 1: initial config of {e} relative to {s} to standoff
    Xstart = Tse_i; Xend = Tsc_i*Tce_so;
    method = 3;
    traj_1 = ScrewTrajectory(Xstart,Xend,Tf(1),N(1),method);

    %segment 2: standoff to grasp
    Xstart = Xend; Xend = Tsc_i*Tce_gr;
    traj_2 = ScrewTrajectory(Xstart,Xend,Tf(2),N(2),method);

    %segment 3: grasp
    Xstart = Xend; Xend = Tsc_i*Tce_gr;
    traj_3 = ScrewTrajectory(Xstart,Xend,Tf(3),N(3),method);

    %segment 4: grasp to standoff
    Xstart = Xend; Xend = Tsc_i*Tce_so;
    traj_4 = ScrewTrajectory(Xstart,Xend,Tf(4),N(4),method);

    %segment 5: standoff to final standoff
    Xstart = Xend; Xend = Tsc_f*Tce_so;
    traj_5 = ScrewTrajectory(Xstart,Xend,Tf(5),N(5),method);

    %segment 6: final to grasp
    Xstart = Xend; Xend = Tsc_f*Tce_gr;
    traj_6 = ScrewTrajectory(Xstart,Xend,Tf(6),N(6),method);

    %segment 7: grasp
    Xstart = Xend; Xend = Tsc_f*Tce_gr;
    traj_7 = ScrewTrajectory(Xstart,Xend,Tf(7),N(7),method);

    %segment 7: grasp standoff
    Xstart = Xend; Xend = Tsc_f*Tce_so;
    traj_8 = ScrewTrajectory(Xstart,Xend,Tf(8),N(8),method);

    traj_full = [{traj_1} {traj_2} {traj_3} {traj_4} {traj_5} {traj_6} {traj_7} {traj_8}];

end

%% Function - FeedbackControl

function [V, err_i, Xerr] = FeedbackControl(X, Xd, Xd_n, Kp, Ki, dt, err_i, FF)
    
    % Feedforward term (FFt)
    Vd = 1/dt*MatrixLog6(TransInv(Xd)*Xd_n);
    Vd = se3ToVec(Vd);
    FFt = Adjoint(TransInv(X)*Xd)*Vd;
    if FF == false
        FFt=0;
    end

    %Proportional term (FBp)
    Xerr = MatrixLog6(TransInv(X)*Xd);
    Xerr = se3ToVec(Xerr); %error twist
    FBp = Kp*Xerr;

    %integral term (err_i)
    err_i = err_i + Xerr*dt;
    FBi = Ki*err_i;

    V = FFt + FBp + FBi;

end

