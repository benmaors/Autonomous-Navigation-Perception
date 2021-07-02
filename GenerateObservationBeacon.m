%Question2 b
function [z_rel,beacon_number] = GenerateObservationfromBeacon( mu_x, x_b, d, r_min,n)
%===================================================================================================
% INPUT:
% mu_x       = mean Robot Location
% x_b        = Beacons locations (used to find the beacon in range)
% d          = max possible distacne from beacon to generate observation
% r_min      = minimal possible distance from beacon 
%===================================================================================================
% OUTPUT:
% z_rel      = Beacon measurement
% beacon_number 
%===================================================================================================

z_rel = NaN;
beacon_number = 0;

for i=1:n
    % Calculate distance from beacons (Range)
    r = norm(x_b(:,i)-mu_x);
    mu_b = x_b(:,i)-mu_x ;
    if r<=d 
        % Covariance is rangae dependence 
        sigma_v = (0.01*max(r,r_min))^2*eye(2);
        z_rel = mvnrnd((mu_b-mu_x) ,sigma_v);
        beacon_number=i;
        return; % assuming only one beacon is close to the robot 
    end
end

end 
