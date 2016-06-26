function estimation_results = Ideal_EKF_SLAM(data)

%addpath('Math_Liegroup/')

% load pre-given data: odometry and observations
if nargin < 1
    load('../data.mat');
end


% load pre-given data: odometry and observations
data_matrix = data.state;
odo_cov = data.odom_cov;   % constant variable
obs_cov = data.obse_cov;   % constant variable

odom_sigma = data.odom_sigma;
obsv_sigma = data.obsv_sigma;


%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%% In every step, all elements of Estimation_X will be changed %%%%%%%%%%%%
estimation_x.orientation = data.poses.orientation(1:3,1:3);
estimation_x.position    = data.poses.position(:,1);
estimation_x.cov         = sparse(6,6);
estimation_x.landmarks   = [];       % the landmarks observed until this step (included), 4*N format, the 4-th row is the index
%Estimation_X.IndexOfFeature=[];     % the names(indexes) of the landmarks observed until this step (included)
%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  


% Initialize
n_steps = max(data_matrix(:,4));  % step instead of pose,  hence, it does not include pose 0
estimation_results = cell(1, n_steps+1);
estimation_results{1} = estimation_x;


for i = 0:n_steps
    IndexOfCurrentStepInDataMatrix = find(data_matrix(:,4) == i); 
    m = size(IndexOfCurrentStepInDataMatrix, 1);
    if ( mod(i, 10) == 0 )
        disp(['Processing pose ', int2str(i)]);
    end
    % det(Estimation_X.cov)
    if i ~= n_steps
        OdometryFromThis2Next = data_matrix(IndexOfCurrentStepInDataMatrix(m-5):IndexOfCurrentStepInDataMatrix(m),1);
        if m > 6
            CameraMeasurementThis = [ data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 1 ),...
                                      data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 3 )];    
            [estimation_x] = Ideal_EKF_update(estimation_x, CameraMeasurementThis, obsv_sigma, data.poses.orientation(3*i+1:3*i+3,1:3),data.poses.position(1:3,i+1), data.landmarks );
        end
        
        estimation_results{i+1} = estimation_x;
        
        % propagation using odometry info
        [estimation_x] = Ideal_EKF_propagate(estimation_x, OdometryFromThis2Next, odom_sigma, data.poses.orientation(3*i+1:3*i+3,1:3),data.poses.position(1:3,i+1) );

    else
        if m > 6
            CameraMeasurementThis = [ data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 1 ) , data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 3 )];
           [estimation_x] = Ideal_EKF_update(estimation_x, CameraMeasurementThis, obsv_sigma, data.poses.orientation(3*i+1:3*i+3,1:3),data.poses.position(1:3,i+1), data.landmarks );
        end
        estimation_results{i+1} = estimation_x;
    end
end
clearvars -except estimation_results