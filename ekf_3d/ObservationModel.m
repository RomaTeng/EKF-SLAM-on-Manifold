function [ zi ] = ObservationModel( orientation, position, fi )

zi=orientation'*(fi-position);


end

