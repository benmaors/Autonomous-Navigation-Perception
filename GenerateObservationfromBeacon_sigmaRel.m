%Question2 b
function [z_rel,beacon_number] = GenerateObservationfromBeacon_sigmaRel( mu_x, x_b, sigma_vRel, d, r_min,n)
%===================================================================================================
% INPUT:
% mu_x       = mean Robot Location
% x_b        = Beacons locations (used to find the beacon in range)
% d          = max possible distacne from beacon to generate observation
% r_min      = minimal possible distance from beacon 
%===================================================================================================
% OUTPUT:
% z_eacon    = Beacon measurement
% sigma_v    = observation model covariance
% r          = range target to landmark (beacon)  
%===================================================================================================
z_rel = nan(2,1);
beacon_number = nan;

for i=1:n
    % Calculate distance from beacons (Range)
    r = norm(mu_x - x_b(:,i));
    mu_b = mu_x - x_b(:,i) ;
    if r<=d % assuming only one beacon is close to the robot 
        % Covariance is not rangae dependence 
        z_rel = mvnrnd(mu_b ,sigma_vRel);
        beacon_number=i;
        return
    end
   
end 
end 
