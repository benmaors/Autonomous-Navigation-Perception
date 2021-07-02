function [X_sample] = SampleMotionModel( mu_x, sigma_w, a )
%SampleMotionModel 
% input: 1) the state expectation before the action mu_x
%        2) the action a
%        4) the covariance matrix for the noises of the motion (sigma_w)  
% output: A sample for the new state according to the motion model
% (X_sample)

X_sample= mvnrnd(mu_x + a ,sigma_w);
% z = repmat((mu_x + a)',10,1) + randn(10,2)*sigma_w;
end

