function [ zi ] = observation_model( orientation, position, fi )
% observation model
    zi = orientation'*(fi-position);
end

