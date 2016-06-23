function [ zi ] = observation_model( orientation, position, fi )

    zi=orientation'*(fi-position);


end

