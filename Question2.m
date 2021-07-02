%% QUESTION #2
% 
%% a) Observation model

% Range = norm(x_b - x_k);
% Zrel = (x_k - x_b) + sigma_v_beacon ; for Rmin < Range < d

% ************************************************************************
%% b) GenerateObservationFromBeacons

n       = 9 ;   % No. of landmarks  
d       = 1 ;   % Max distance to generate relative observation to beacon
r_min   = 0.1 ; % Min range from beacon

% Locate beacon on 3X3 grid, distance between beacons B_dis = 3 (positioned apart) 
B_dis = 4.5 ; 

x_b = zeros(2,n);
for i = 1:sqrt(n)
    for j = 1:sqrt(n)
    x_b(:,j+(i-1)*sqrt(n))= [(i-1)*B_dis,(j-1)*B_dis]; 
    % each column of x_b is i beacon location, where i=1:9 ; x_b(:,i) = transpose(xbi,ybi)  
    end
end

% Generate Observations function (for each state, we need to check visibility to each one of the beacons) 
%[z_rel,beacon_number] = GenerateObservationfromBeacon(mu_x, x_b, d, r_min,n);

%% c) 

%initial state

mu_0    = zeros(2,1);
sigma_0 = eye(2);
sigma_w = 0.1^2*eye(2);
sigma_v = 0.1^2*eye(2);
a_i     = [0.1;0.1];
x_0     = [-0.5;-0.2];

% ************************************************************************
%% 2_C.i(1_d.i) generate trajectory

T                   = 100;
trajectory          = zeros(2,T);
trajectory(1:2,1)   = x_0;
current_pos         = x_0;

for i = 1:(T-1)
    current_pos = trajectory(1:2,i);
    trajectory(1:2,i+1)= SampleMotionModel(current_pos, sigma_w, a_i);
   % [trajectory_check(1:2,i+1),varargout] = mvg(current_pos, sigma_w,1);
end

% PLOT TRAJECTORY
figure()
hold on
plot( x_b(1,:),x_b(2,:),'r.','MarkerSize',20) %Beacons
for i=1:n
    b_i = int2str(i);
    text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','red');
end
plot(trajectory(1,:),trajectory(2,:),'k','LineWidth',1,'HandleVisibility','off');
plot(trajectory(1,:),trajectory(2,:),'b+','MarkerSize',3);

grid on
xticks(0:1:10);
yticks(0:1:12);
xlabel('X [m]')
ylabel('Y [m]')
title('Q2Ci - 2D Robot Trajectory')
legend('Beacons','Trajectory')  
hold on

% ************************************************************************
%% 2_C.i(1_d.ii)Generate observations 

observations = zeros(2,T);
observations_landmarks = NaN(3,T);
sigma_vRel = NaN(T*2,2);

for i = 1:T
    current_pos = trajectory(1:2,i);
    [observations_landmarks(1:2,i),observations_landmarks(3,i),sigma_vRel((2*i-1):2*i,:)] = GenerateObservationfromBeacon(current_pos, x_b, d, r_min,n);
end

%Plot relative Observations

% PLOT TRAJECTORY
figure()
hold on
plot( x_b(1,:),x_b(2,:),'r.','MarkerSize',20,'HandleVisibility','off') %Beacons
for i=1:n
    b_i = int2str(i);
    text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','red');
end
plot(trajectory(1,:),trajectory(2,:),'k','LineWidth',1,'HandleVisibility','off');
plot(trajectory(1,:),trajectory(2,:),'b+','MarkerSize',3);
hold on
for i = 1:T
    if isnan(observations_landmarks(1,i))==0
        xrel = observations_landmarks(1:2,i)+ x_b(:,observations_landmarks(3,i));
        plot(xrel(1),xrel(2),'go')
    end 
end
grid on
xticks(0:1:12)
yticks(0:1:12)
xlabel('X [m]')
ylabel('Y [m]')
legend('Trajectory','Relative observations')
title('Q2Ci - Landmarks Relative Observations')
hold on

% ************************************************************************
%% 2_C.i(1_d.iii). generate partial beliefs (without relative observation)

partial_beliefs_mu = zeros(2,T);
partial_beliefs_sigma = zeros(T*2, 2);

partial_beliefs_mu(1:2,1) = mu_0;
partial_beliefs_sigma(1:2,1:2)=sigma_0;

for i = 0:T-1
    old_belief_mu = partial_beliefs_mu(1:2,i+1);
    old_belief_sigma = partial_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    
    [partial_beliefs_mu(1:2,i+2),partial_beliefs_sigma((1+2*i+2):(1+2*i+3),1:2)]= propagatePartialUpdateBelief(old_belief_mu,old_belief_sigma,a_i,sigma_w);
end

% ************************************************************************
%% 2_C.i(1_d.iv). Generate full beliefs with relative observation 

full_beliefs_mu = zeros(2,T);
full_beliefs_sigma = zeros(T*2, 2);

full_beliefs_mu(1:2,1) = mu_0;
full_beliefs_sigma(1:2,1:2)=sigma_0;

for i = 0:T-1
    old_belief_mu = full_beliefs_mu(1:2,i+1);
    old_belief_sigma = full_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    current_observation = observations(1:2,i+1);
    current_rel_observation = observations_landmarks(1:3,i+1);
    current_sigma_vRel = sigma_vRel((2*i+1):2*(i+1),:);
    
    % creating binary parameter to indecate if we have landmark measurements
    if isnan(current_rel_observation(1))
        gamma = 0;
        current_x_bRel = [NaN;NaN];
    else
        gamma = 1;
    current_x_bRel = x_b(:,current_rel_observation(3)); % beacon known location
    end
    
  [full_beliefs_mu(1:2,i+2),full_beliefs_sigma((1+2*i+2):(1+2*i+3),1:2)]= propagateUpdateBeliefBeacon(old_belief_mu,old_belief_sigma,current_observation,a_i,sigma_w,sigma_v,current_rel_observation,current_x_bRel,current_sigma_vRel,gamma);
end

% PLOT Belief without observations
figure()
hold on
plot( x_b(1,:),x_b(2,:),'r.','MarkerSize',20,'HandleVisibility','off') %Beacons
for i=1:n
    b_i = int2str(i);
    text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','red')
end
plot(trajectory(1,:),trajectory(2,:),'-k','LineWidth',1,'HandleVisibility','off')
plot(trajectory(1,:),trajectory(2,:),'k+','MarkerSize',3)
plot(partial_beliefs_mu(1,:),partial_beliefs_mu(2,:),'b','LineWidth',1,'HandleVisibility','off')
plot(partial_beliefs_mu(1,:),partial_beliefs_mu(2,:),'c+','MarkerSize',3)
grid on
xticks(0:1:10)
yticks(0:1:12)
xlabel('X [m]')
ylabel('Y [m]')
title(('Q_2Ci - Belief Space 2d Trajecctory without Observations '))
hold on 

for i = 0:T-1
    belief_mu = partial_beliefs_mu(1:2,i+1);
    belief_sigma = partial_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    drawCovarianceEllipse(belief_mu,belief_sigma,'green','-');
end
legend('Trjectory','Patial Belief','Covariance elipse')
hold off

% PLOT Belief with landmarks observations
figure()
hold on
plot( x_b(1,:),x_b(2,:),'r.','MarkerSize',20,'HandleVisibility','off') %Beacons
for i=1:n
    b_i = int2str(i)
    text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','red')
end
plot(trajectory(1,:),trajectory(2,:),'-k','LineWidth',1,'HandleVisibility','off')
plot(trajectory(1,:),trajectory(2,:),'k+','MarkerSize',3)
plot(full_beliefs_mu(1,:),full_beliefs_mu(2,:),'b','LineWidth',1,'HandleVisibility','off')
plot(full_beliefs_mu(1,:),full_beliefs_mu(2,:),'c+','MarkerSize',3)
grid on
xticks(0:1:10)
yticks(0:1:12)
xlabel('X [m]')
ylabel('Y [m]')
title('Q2ci - Belief Space 2d Trajecctory with Landmark Relative Observations')
hold on 

for i = 0:T-1
    belief_mu = full_beliefs_mu(1:2,i+1);
    belief_sigma = full_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    drawCovarianceEllipse(belief_mu,belief_sigma,'green','-')
end
legend('Trjectory','Posterior Belief','Covariance elipse')
hold off


% KEEP Ci full beliefs Results:
full_beliefs_mu_Ci = full_beliefs_mu; 
full_beliefs_sigma_Ci = full_beliefs_sigma; 

clear full_beliefs_mu;

%% 2_C.ii 
% (1_d.i)Generate Trajectory 
% (Same as Ci)
%% (1_d.ii)Generate observations 

observations = zeros(2,T);
observations_landmarks = NaN(3,T);
sigma_vRel = 0.01^2*eye(2);

for i = 1:T
    current_pos = trajectory(1:2,i);
    [observations_landmarks(1:2,i),observations_landmarks(3,i)] = GenerateObservationfromBeacon_sigmaRel(current_pos, x_b, sigma_vRel, d, r_min,n);
end

% PLOT TRAJECTORY
figure()
hold on
plot( x_b(1,:),x_b(2,:),'r.','MarkerSize',20,'HandleVisibility','off') %Beacons
for i=1:n
    b_i = int2str(i);
    text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','red');
end
plot(trajectory(1,:),trajectory(2,:),'k','LineWidth',1,'HandleVisibility','off');
plot(trajectory(1,:),trajectory(2,:),'b+','MarkerSize',3);
hold on
for i = 1:T
    if isnan(observations_landmarks(1,i))==0
        xrel = observations_landmarks(1:2,i)+ x_b(:,observations_landmarks(3,i));
        plot(xrel(1),xrel(2),'go')
    end 
end
grid on
xticks(0:1:12)
yticks(0:1:12)
xlabel('X [m]')
ylabel('Y [m]')
legend('Trajectory','Relative observations')
title('Q2Cii - Landmarks Relative Observations constant Covariance')
hold on
%% (1_d.iii). generate partial beliefs (without relative observation)
% Same as Ci

%% 2_C.ii(1_d.iv). Generate full beliefs with relative observation 

full_beliefs_mu = zeros(2,T);
full_beliefs_sigma = zeros(T*2, 2);

full_beliefs_mu(1:2,1) = mu_0;
full_beliefs_sigma(1:2,1:2)=sigma_0;

for i = 0:T-1
    old_belief_mu = full_beliefs_mu(1:2,i+1);
    old_belief_sigma = full_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    current_observation = observations(1:2,i+1);
    current_rel_observation = observations_landmarks(1:3,i+1)
    current_sigma_vRel = sigma_vRel;
    
    % creating binary parameter to indecate if we have landmark measurements
    if isnan(current_rel_observation(1))
        gamma = 0;
        current_x_bRel = [NaN;NaN];
    else
        gamma = 1;
        current_x_bRel = x_b(:,current_rel_observation(3)); % beacon known location
    end
    
  [full_beliefs_mu(1:2,i+2),full_beliefs_sigma((1+2*i+2):(1+2*i+3),1:2)]= propagateUpdateBeliefBeacon(old_belief_mu,old_belief_sigma,current_observation,a_i,sigma_w,sigma_v,current_rel_observation,current_x_bRel,current_sigma_vRel,gamma);
end

% PLOT Belief with landmarks observations
figure()
hold on
plot( x_b(1,:),x_b(2,:),'r.','MarkerSize',20,'HandleVisibility','off') %Beacons
for i=1:n
    b_i = int2str(i);
    text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','red')
end
plot(trajectory(1,:),trajectory(2,:),'-k','LineWidth',1,'HandleVisibility','off')
plot(trajectory(1,:),trajectory(2,:),'k+','MarkerSize',3)
plot(full_beliefs_mu(1,:),full_beliefs_mu(2,:),'b','LineWidth',1,'HandleVisibility','off')
plot(full_beliefs_mu(1,:),full_beliefs_mu(2,:),'c+','MarkerSize',3)
grid on
xticks(0:1:10)
yticks(0:1:12)
xlabel('[X]')
ylabel('[Y]')
title('Q2cii - Belief Space 2d Trajecctory with Landmark Relative Observation')
hold on 

for i = 0:T-1
    belief_mu = full_beliefs_mu(1:2,i+1);
    belief_sigma = full_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    drawCovarianceEllipse(belief_mu,belief_sigma,'green','-')
end

legend('Trjectory','Posterior Belief','Covariance elipse')
hold off

%% 2_C.iii. Localization estimation Errors^2 for Constant sigma_vRel(Cii) and range dependent sigma_vRel (Ci)

Error_EstimationConstSigmaV = zeros(1,T);
Error_EstimationRangeDependentSigmaV = zeros(1,T);

for i=1:T
    belief_mu_ConstSigmaV = full_beliefs_mu(1:2,i);
    belief_mu_RangeDependentSigmaV = full_beliefs_mu_Ci(1:2,i);
    X_Trajectory = trajectory(:,i);
    Error_EstimationConstSigmaV(i) = immse(X_Trajectory,belief_mu_ConstSigmaV);
    Error_EstimationRangeDependentSigmaV(i) = immse(X_Trajectory,belief_mu_RangeDependentSigmaV);    

    % Error_EstimationConstSigmaV(i) = norm(X_Trajectory - belief_mu_ConstSigmaV) ;
   % Error_EstimationRangeDependentSigmaV(i) = norm(X_Trajectory - belief_mu_RangeDependentSigmaV);    
end 

figure()
hold on
plot(1:T,Error_EstimationConstSigmaV(:),'k-');
plot(1:T, Error_EstimationRangeDependentSigmaV(:),'r-');
grid on
title('Q2Ciii - Localization Estimation Mean Squared Error ');
xlabel('time[sec]');
ylabel('MSE');
legend('Constant Covariance', 'Range Dependent Covariance'); 

%% 2_C.iv. Square Root of the Estimation Covariance Matrix trace, for Constant sigma_vRel(Cii) and range dependent sigma_vRel (Ci)

SqrtTraceCovariance_ConstSigmaV = zeros(1,T);
SqrtTraceCovariance_RangeDependentSigmaV = zeros(1,T);

for i=1:T
    SqrtTraceCovariance_ConstSigmaV(i) = sqrt(trace(full_beliefs_sigma((2*i-1):2*i,1:2)));
    SqrtTraceCovariance_RangeDependentSigmaV(i) = sqrt(trace(full_beliefs_sigma_Ci((2*i-1):2*i,1:2)));
end 

figure()
hold on
plot(2:T, SqrtTraceCovariance_ConstSigmaV(2:T),'k-','Linewidth',1);
plot(2:T, SqrtTraceCovariance_RangeDependentSigmaV(2:T),'r--','Linewidth',1);
grid on
title('Q2Civ - Square Root of Estimation Covariance Trace');
xlabel('time[sec]');
ylabel('sqrt(Trace(Covariance))');
legend('Constant Covariance', 'Range Dependent Covariance'); 

%% 2_D.i. Trajectory for different action sequences
N_actions       = 10;
a               = zeros(2,N_actions);
trajectory_a    = zeros(2*N_actions,T);
current_pos     = x_0;

for j=1:N_actions
    a(1:2,j) = [0.1 , (0.1 * (j/5))]';
end
for j = 1:N_actions
    trajectory_a((2*j-1:2*j),1)   = x_0;
end

% Generate Trajectory
for j=1:N_actions
    aj = a(1:2,j);
    for i = 1:(T-1)
        current_pos = trajectory_a((2*j-1:2*j),i);
        trajectory_a((2*j-1:2*j),i+1) = SampleMotionModel(current_pos, sigma_w, aj);
    end
end

% PLOT TRAJECTORY
figure()
hold on
plot( x_b(1,:),x_b(2,:),'k.','MarkerSize',20) %Beacons
for i=1:n
    b_i = int2str(i);
    text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','black')
end
for j=1:N_actions
    plot(trajectory_a(2*j-1,:),trajectory_a(2*j,:),'LineWidth',1);
    hold on
   % plot(trajectory_a(2*j-1,:),trajectory_a(2*j,:),'+','MarkerSize',2);
end
legend ('Beacons','a1','a2','a3','a4','a5','a6','a7','a8','a9','a10');
grid on
xticks(0:1:15);
yticks(0:1:21);
xlabel('X[m]');
ylabel('Y[m]');
title('2D Robot Trajectories for different action sequence - Linear');
hold off
% 
% % PLOT TRAJECTORY
% figure()
% hold on
% plot( x_b(1,:),x_b(2,:),'r.','MarkerSize',20) %Beacons
% for i=1:n
%     b_i = int2str(i);
%     text(x_b(1,i)+0.1,x_b(2,i)+0.1,b_i,'Color','red')
% end
% 
% legend on 
% grid on
% xticks(0:1:15);
% yticks(0:1:21);
% xlabel('[X]');
% ylabel('[Y]');
% title('2D Robot Trajectory');
% hold off

% **************************************************************************
%% 2_D.ii. Information theoric cost

information_cost    = zeros(T,N_actions);
full_beliefs_mu     = zeros(2,T);
full_beliefs_sigma  = zeros(T*2,20);

full_beliefs_mu(1:2,1)      = mu_0;
full_beliefs_sigma(1:2,1:2) = sigma_0;

observations_landmarks  = NaN(3*N_actions,T);
sigma_vRel = NaN(T*2,20);

%sigma_vRel              = 0.01^2*eye(2);

% % Generate observation for Each action sequance (different landmarks are visible) 
% for i = 1:T
%     current_pos = trajectory(1:2,i);
%     [observations_landmarks(1:2,i),observations_landmarks(3,i),sigma_vRel((2*i-1):2*i,:)] = GenerateObservationfromBeacon(current_pos, x_b, d, r_min,n);
% end

for j = 1:N_actions
    for i = 1:T
        current_pos = trajectory_a((2*j-1:2*j),i);
        [observations_landmarks((2*(j-1)+j):(2*(j-1)+j+1),i),observations_landmarks(2*(j-1)+j+2,i),sigma_vRel((2*i-1):2*i,(2*j-1):2*j)] = GenerateObservationfromBeacon(current_pos, x_b, d, r_min,n);
    end
end
        
for j = 1:N_actions
    
    full_beliefs_mu(1:2,j)                  = mu_0;
    full_beliefs_sigma(1:2,(2*j-1):(2*j))   = sigma_0;
    information_cost(1,j)                   = det(sigma_0);
    a_i                                     = a(1:2,j);
        
    for i = 0:T-1
        old_belief_mu = full_beliefs_mu(1:2,i+1);
        old_belief_sigma = full_beliefs_sigma((2*i+1):(2*(i+1)),(2*j-1):2*j);
     
        current_rel_observation = observations_landmarks((2*(j-1)+j):(2*(j-1)+j+2),i+1);
        current_sigma_vRel = sigma_vRel((2*i+1):2*(i+1),(2*j-1):2*(j));
        %current_sigma_vRel = sigma_vRel;

        % creating binary parameter to indecate if we have landmark measurements
        if isnan(current_rel_observation(1))
            gamma = 0;
            current_x_bRel = [NaN;NaN];
        else
            gamma = 1;
            current_x_bRel = x_b(:,current_rel_observation(3)); % beacon known location
        end

      [full_beliefs_mu((2*j-1):(2*j),i+1),full_beliefs_sigma((1+2*i+2):(1+2*i+3),(2*j-1):(2*j))]= propagateUpdateBeliefBeacon(old_belief_mu,old_belief_sigma,current_observation,a_i,sigma_w,sigma_v,current_rel_observation,current_x_bRel,current_sigma_vRel,gamma);
      
      information_cost(i+1,j) = det(full_beliefs_sigma((2*i+1):(2*(i+1)),(2*j-1):(2*j)));
    end
end


% PLOT COST
figure()
hold on
title('Cost of trajectories');
subplot(2,5,1)
plot(2:T,information_cost(2:T,1),'k-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,1),'k+','MarkerSize',2);
grid on
legend j=1
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,2)
plot(2:T,information_cost(2:T,2),'b-','LineWidth',1);
hold on
plot(2:T,information_cost(2:T,2),'b+','MarkerSize',2);
grid on
legend j=2
% xticks(0:10:100);
% yticks(0:0.01:1)
subplot(2,5,3)
plot(2:T,information_cost(2:T,3),'r-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,3),'r+','MarkerSize',2);
grid on
legend j=3
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,4)
plot(2:T,information_cost(2:T,4),'k-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,4),'k+','MarkerSize',2);
grid on
legend j=4
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,5)
plot(2:T,information_cost(2:T,1),'g-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,1),'g+','MarkerSize',2);
grid on
legend j=5
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,6)
plot(2:T,information_cost(2:T,6),'k-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,6),'k+','MarkerSize',2);
grid on
legend j=6
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,7)
plot(2:T,information_cost(2:T,7),'b-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,7),'b+','MarkerSize',2);
grid on
legend j=7
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,8)
plot(2:T,information_cost(2:T,8),'k-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,8),'k+','MarkerSize',2);
grid on
legend j=8
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,9)
plot(2:T,information_cost(2:T,9),'r-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,9),'r+','MarkerSize',2);
grid on
legend j=9
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on
subplot(2,5,10)
plot(2:T,information_cost(2:T,10),'g-','LineWidth',0.01);
hold on
plot(2:T,information_cost(2:T,10),'g+','MarkerSize',2);
grid on
legend j=10
% xticks(0:10:100);
% yticks(0:0.01:1)
hold on

%PLOT BEST TRAJECTORY
full_cost = zeros(1,N_actions);
for j=1:N_actions
    full_cost(j)=sum(information_cost(2:T,j));
end
figure()
hold on
xlabel('Trajectory #');
ylabel('Total Cost');
title('Total Cost of trajectories');
plot(1:N_actions ,full_cost,'o');
grid on
hold off


