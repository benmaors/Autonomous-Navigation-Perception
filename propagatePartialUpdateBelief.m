function [mu_x_updated, sigma_x_updated] = propagatePartialUpdateBelief(mu_x, sigma_x, a,sigma_w)
%PropagateUpdateBelief without observational model
% input: 1) the Gaussians parameters for the old belief before action a (mu_x, sigma_x) 
%        2) the action a
%        3) the covariance matrix for the noises of the motion (sigma_w)   
% output: the Gaussians parameters for the new belief after action a
% (mu_x_updated, sigma_x_updated)


A = zeros(4,4);

A(1:2,1:2) = sqrt(sigma_x^(-1));
A(3:4,1:2) = -sqrt(sigma_w^(-1));
A(3:4,3:4) = sqrt(sigma_w^(-1));

b = zeros(4,1);

b(1:2) = sqrt(sigma_x^(-1))*mu_x;
b(3:4) = sqrt(sigma_w^(-1))*a;

joint_belief_mu = (A'*A)^-1*A'*b;
joint_belief_sigma = (A'*A)^-1;

mu_x_updated = joint_belief_mu(3:4);
sigma_x_updated = joint_belief_sigma(3:4,3:4);

end

