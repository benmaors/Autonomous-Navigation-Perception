%initial state

mu_0 = zeros(2,1);
sigma_0 = eye(2);
sigma_w = 0.1^2*eye(2);
sigma_v = 0.1^2*eye(2);
a_i = [1;0];
x_0 = [0.5;0.2];

%i. generate trajectory
T = 10;
trajectory = zeros(2,T);
trajectory(1:2,1)= x_0;
current_pos = x_0;

for i = 1:(T-1)
    current_pos = trajectory(1:2,i);
    trajectory(1:2,i+1)= SampleMotionModel(current_pos, sigma_w, a_i);
end

%ii. generate observations 

observations = zeros(2,T);

for i = 1:T
    current_pos = trajectory(1:2,i);
    observations(1:2,i)= GenerateObservation(current_pos, sigma_v);
end

%iii. generate partial beliefs (without observation)
partial_beliefs_mu = zeros(2,T);
partial_beliefs_sigma = zeros(T*2, 2);

partial_beliefs_mu(1:2,1) = mu_0;
partial_beliefs_sigma(1:2,1:2)=sigma_0;

for i = 0:T-1
    old_belief_mu = partial_beliefs_mu(1:2,i+1);
    old_belief_sigma = partial_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    
    [partial_beliefs_mu(1:2,i+2),partial_beliefs_sigma((1+2*i+2):(1+2*i+3),1:2)]= propagatePartialUpdateBelief(old_belief_mu,old_belief_sigma,a_i,sigma_w);
end

%iv. generate full beliefs with observation generated in ii.
full_beliefs_mu = zeros(2,T);
full_beliefs_sigma = zeros(T*2, 2);

full_beliefs_mu(1:2,1) = mu_0;
full_beliefs_sigma(1:2,1:2)=sigma_0;


for i = 0:T-1
    old_belief_mu = full_beliefs_mu(1:2,i+1);
    old_belief_sigma = full_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    current_observation = observations(1:2,i+1);
    
    [full_beliefs_mu(1:2,i+2),full_beliefs_sigma((1+2*i+2):(1+2*i+3),1:2)]= propagateUpdateBelief(old_belief_mu,old_belief_sigma,current_observation,a_i,sigma_w,sigma_v);
end

figure()
hold on 
plot(full_beliefs_mu(1,:),full_beliefs_mu(2,:),'k-','Linewidth',1)
plot(full_beliefs_mu(1,:),full_beliefs_mu(2,:),'k+','MarkerSize',3,'HandleVisibility','off')
plot(trajectory(1,:),trajectory(2,:),'b','LineWidth',1);
plot(trajectory(1,:),trajectory(2,:),'c+','MarkerSize',3,'HandleVisibility','off')
for i = 0:T-1
    belief_mu = full_beliefs_mu(1:2,i+1);
    belief_sigma = full_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    drawCovarianceEllipse(belief_mu,belief_sigma,'green','-')
end
grid on
xlabel('X [m]')
ylabel('Y [m]')
legend( 'Posterior Belief','Trajectory')
title('Q1di - 2D Posterior Belief with measurements')
hold on

figure()
hold on 
plot(partial_beliefs_mu(1,:),partial_beliefs_mu(2,:),'k-','Linewidth',1)
plot(partial_beliefs_mu(1,:),partial_beliefs_mu(2,:),'k+','MarkerSize',3,'HandleVisibility','off')
plot(trajectory(1,:),trajectory(2,:),'b','LineWidth',1);
plot(trajectory(1,:),trajectory(2,:),'c+','MarkerSize',3,'HandleVisibility','off')
for i = 0:T-1
    belief_mu = partial_beliefs_mu(1:2,i+1);
    belief_sigma = partial_beliefs_sigma((1+2*i):(1+2*i+1),1:2);
    drawCovarianceEllipse(belief_mu,belief_sigma,'green','-')
end
xlabel('X [m]')
ylabel('Y [m]')
legend( 'Posterior Belief','Trajectory')
title('Q1di - 2D Posterior Belief without measurements')
grid on
hold on
hold off


