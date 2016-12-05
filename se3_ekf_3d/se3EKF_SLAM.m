function estimation_results = se3EKF_SLAM(data, NumberOfSteps)
% R-EKF SLAM 

% load pre-given data: odometry and observations
if nargin < 1
    load('./data.mat');
end

data_matrix = data.state;

odom_sigma = data.odom_sigma;
obsv_sigma = data.obsv_sigma;

% odoCov = data.odom_cov;   % constant variable
% obsCov = data.obse_cov;   % constant variable

%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%% In every step, all elements of Estimation_X will be changed %%%%%%%%%%%%
Estimation_X.orientation = data.poses.orientation(1:3,1:3);
Estimation_X.position    = data.poses.position(:,1);
Estimation_X.cov         = sparse(6,6);
Estimation_X.landmarks   = [];       % the landmarks observed until this step (included), 4*N format, the 4-th row is the index
Estimation_X0.IndexObservedNew=[];
Estimation_X0.IndexObservedAlreadyThis=[];
%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  


% Initialize
if nargin < 2
    NumberOfSteps = max(data_matrix(:,4));  % step instead of pose,  hence, it does not include pose 0
elseif NumberOfSteps > max(data_matrix(:,4))
    NumberOfSteps = max(data_matrix(:,4));
end
estimation_results = cell(1, NumberOfSteps+1);
estimation_results{1} = Estimation_X;
row_idx = (data_matrix(:, end) <= NumberOfSteps+1);
data_matrix = data_matrix(row_idx, :);


for i = 0:NumberOfSteps
    IndexOfCurrentStepInDataMatrix = find(data_matrix(:,4) == i); 
    m = size(IndexOfCurrentStepInDataMatrix, 1);
    if ( mod(i, 50) == 0 )
        disp(['Processing pose ', int2str(i)]);
    end
    % det(Estimation_X.cov)
    if i==NumberOfSteps-1
    a=1;
    end
    
    if i ~= NumberOfSteps
        OdometryFromThis2Next = data_matrix(IndexOfCurrentStepInDataMatrix(m-5):IndexOfCurrentStepInDataMatrix(m),1);
        if m > 6
            CameraMeasurementThis = [ data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 1 ),...
                                      data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 3 )];    
           [Estimation_X] = se3EKF_update(Estimation_X, CameraMeasurementThis, obsv_sigma );
        end
        
        estimation_results{i+1} = Estimation_X;
        
%        propagation using odometry info
        [Estimation_X] = se3EKF_propagate(Estimation_X, OdometryFromThis2Next, odom_sigma );

    else
        a=2;
        if m > 6
            CameraMeasurementThis = [ data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 1 ) , ...
                                      data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 3 )];
            [Estimation_X] = se3EKF_update(Estimation_X, CameraMeasurementThis, obsv_sigma );
        end
        estimation_results{i+1} = Estimation_X;
    end
end
clearvars -except estimation_results