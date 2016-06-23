function [ zi ] = ObservationModel( orientation, position, fi )
% observation model
    zi = orientation'*(fi-position);
end

