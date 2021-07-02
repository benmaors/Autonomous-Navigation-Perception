function [z_sample] = GenerateObservation( mu_x, sigma_v)
%SampleMotionModel 
% input: 1) the state expectation before the action mu_x
%        4) the covariance matrix for the noises of the observation (sigma_w)  
% output: A sample for the new observation according to the observation model
% (X_sample)

z_sample= mvnrnd(mu_x ,sigma_v);
end

