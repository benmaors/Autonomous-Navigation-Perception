function [mu_x_updated, sigma_x_updated] = propagateUpdateBelief(mu_x, sigma_x, mu_z,a,sigma_w,sigma_v)
%PropagateUpdateBelief
% input: 1) the Gaussians parameters for the old belief before action a (mu_x, sigma_x) 
%        2) the action a
%        3) the observation after the action mu_z
%        4) the covariance matrix for the noises of the motion (sigma_w) and observation model (sigma_v)  
% output: the Gaussians parameters for the new belief after action a
% (mu_x_updated, sigma_x_updated)


A = zeros(6,4);

A(1:2,1:2) = sqrt(sigma_x^(-1));
A(3:4,1:2) = -sqrt(sigma_w^(-1));
A(3:4,3:4) = sqrt(sigma_w^(-1));
A(5:6,3:4) = -sqrt(sigma_v^(-1));

b = zeros(6,1);

b(1:2) = sqrt(sigma_x^(-1))*mu_x;
b(3:4) = sqrt(sigma_w^(-1))*a;
b(5:6) = -sqrt(sigma_v^(-1))*mu_z;

joint_belief_mu = (A'*A)^-1*A'*b;
joint_belief_sigma = (A'*A)^-1;

mu_x_updated = joint_belief_mu(3:4);
sigma_x_updated = joint_belief_sigma(3:4,3:4);

end

