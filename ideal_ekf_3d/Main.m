clc
clear

addpath('Math_Liegroup/');

% load pre-given data: odometry and observations
load('data.mat');
DataMatrix = data.state;
odoCov = data.odom_cov;   % constant variable
obsCov = data.obse_cov;   % constant variable

%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%% In every step, all elements of Estimation_X will be changed %%%%%%%%%%%%
Estimation_X.orientation = data.poses.orientation(1:3,1:3);
Estimation_X.position    = data.poses.position(:,1);
Estimation_X.cov         = sparse(6,6);
Estimation_X.landmarks   = [];       % the landmarks observed until this step (included), 4*N format, the 4-th row is the index
%Estimation_X.IndexOfFeature=[];     % the names(indexes) of the landmarks observed until this step (included)
%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  


% Initialize
NumberOfSteps = max(DataMatrix(:,4));  % step instead of pose,  hence, it does not include pose 0
EstimationHistory = cell(1, NumberOfSteps+1);
EstimationHistory{1} = Estimation_X;


for i = 0:NumberOfSteps
    IndexOfCurrentStepInDataMatrix = find(DataMatrix(:,4) == i); 
    m = size(IndexOfCurrentStepInDataMatrix, 1);
    if ( mod(i, 10) == 0 )
        disp(['Processing pose ', int2str(i)]);
    end
    % det(Estimation_X.cov)
    if i ~= NumberOfSteps
        OdometryFromThis2Next = DataMatrix(IndexOfCurrentStepInDataMatrix(m-5):IndexOfCurrentStepInDataMatrix(m),1);
        if m > 6
            CameraMeasurementThis = [ DataMatrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 1 ),...
                                      DataMatrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 3 )];    
            [Estimation_X] = EKFonestepUpdate(Estimation_X, CameraMeasurementThis, obsCov, data.poses.orientation(3*i+1:3*i+3,1:3),data.poses.position(1:3,i+1), data.landmarks );
        end
        
        EstimationHistory{i+1} = Estimation_X;
        
        % propagation using odometry info
        [Estimation_X] = EKFonestepPropagate(Estimation_X, OdometryFromThis2Next, odoCov, data.poses.orientation(3*i+1:3*i+3,1:3),data.poses.position(1:3,i+1) );

    else
        if m > 6
            CameraMeasurementThis = [ DataMatrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 1 ) , DataMatrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 3 )];
           [Estimation_X] = EKFonestepUpdate(Estimation_X, CameraMeasurementThis, obsCov, data.poses.orientation(3*i+1:3*i+3,1:3),data.poses.position(1:3,i+1), data.landmarks );
        end
        EstimationHistory{i+1} = Estimation_X;
    end
end

%% plot estimated trajectory
PlotTrajectory;