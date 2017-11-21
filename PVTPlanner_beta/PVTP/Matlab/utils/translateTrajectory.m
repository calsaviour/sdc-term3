function [ T ] = translateTrajectory( T, p, t )
%translateTrajectory Translate a trajectory on the PT plane
%   Translate a trajectory on the PT plane

T(:,1) = T(:,1) + p;
T(:,2) = T(:,2) + t;

end

