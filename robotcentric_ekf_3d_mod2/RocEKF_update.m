function [Estimation_X0] = RocEKF_update(Estimation_X0, CameraMeasurementThis, Sigma_OB)




NumberOfLandmarksObInThisStep = size(CameraMeasurementThis,1)/3;

% initialise the IndexOfFeature if possible
if size(Estimation_X0.landmarks,2) > 0
    IndexOfFeature = Estimation_X0.landmarks(4,:)';
else
    IndexOfFeature = [];
end

IndexObservedAlreadyThis = [];
IndexObservedNew = [];   
for i = 1:NumberOfLandmarksObInThisStep
    % check whether the feature is observed before or not
    M = find( IndexOfFeature== CameraMeasurementThis(3*i,2) );
    if isempty(M)
        IndexObservedNew = [IndexObservedNew;CameraMeasurementThis(3*i,2) ];
    else
        IndexObservedAlreadyThis = [IndexObservedAlreadyThis;CameraMeasurementThis(3*i,2)];
    end             
end

Estimation_X0.IndexObservedNew=IndexObservedNew;
Estimation_X0.IndexObservedAlreadyThis=IndexObservedAlreadyThis;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IndexObservedNew=[78; 97; 18] indicates that the 
% robot firstly observes landmarks 78 97 18 in this step
% IndexObservedAlreadyThis=[19; 20; 53] indicates 
% that the robot observes again landmarks 19 20 53 in this step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


orientation = Estimation_X0.orientation;
position    = Estimation_X0.position;
cov         = Estimation_X0.cov;

NumberOfFeature = size( IndexOfFeature,1);
NumberOfOldFeatureInThisStep = size(IndexObservedAlreadyThis,1);
NumberOfNewFeatureInThisStep = size(IndexObservedNew,1);
 
   
% update state and covariance 
if ~isempty(IndexObservedAlreadyThis)
    Z = zeros( NumberOfOldFeatureInThisStep*3 , 1); 
    Y = zeros( NumberOfOldFeatureInThisStep*3 , 1); 
    H = zeros(3*NumberOfOldFeatureInThisStep, 6+3*NumberOfFeature);
    
    temp = repmat({eye(3)}, NumberOfOldFeatureInThisStep,1 );
    R = blkdiag(temp{:});
    
    % update old features
    for i = 1:NumberOfOldFeatureInThisStep
        ind = find(IndexOfFeature == IndexObservedAlreadyThis(i));
        fi  = Estimation_X0.landmarks(1:3,ind);
        Y(3*i-2:3*i,1) = fi;
        
        ind2 = find(CameraMeasurementThis(:,2) == IndexObservedAlreadyThis(i));
        Z(3*i-2:3*i,1 ) = CameraMeasurementThis(ind2,1);
        
        %H(3*i-2:3*i, 4:6) = orientation';
        %H(3*i-2:3*i, 6+3*ind-2:6+3*ind) = -eye(3);   
        H(3*i-2:3*i, 6+3*ind-2:6+3*ind) = -eye(3);
        R(3*i-2:3*i,3*i-2:3*i) = diag(CameraMeasurementThis(ind2,3).^2)*Sigma_OB^2;
    end    
    
    % question @RomaTeng, different computaton scheme
    z = Z-Y;
    S = H*cov*H'+R;
    K = cov*H'*inv(S);
    s = K*z;
    
    Estimation_X0 = special_add_centric2(Estimation_X0,-s);
    cov = ( eye(6+3*NumberOfFeature) -K*H )*cov;
    Estimation_X0.cov = cov;
    
    % @todo @RomaTeng, right Jacobian % No need for consistency
    % Estimation_X0.cov=JJJr(s)*cov*(JJJr(s))';
end  
     

% update state vector and covariance by considering 
% new feature into state and covariance
if ~isempty(IndexObservedNew)
    % copy previous covariance
    temp    = repmat({eye(3)}, NumberOfNewFeatureInThisStep, 1 );
    tempKK  = blkdiag(temp{:});
    Sigma   = blkdiag(Estimation_X0.cov,tempKK);
    KK      = eye(6+3*(NumberOfFeature+NumberOfNewFeatureInThisStep));
    
    % add new features
    for i = 1:NumberOfNewFeatureInThisStep
        indNewf = IndexObservedNew(i);
        Estimation_X0.landmarks(4,NumberOfFeature+i) = indNewf;
        m2 = find( CameraMeasurementThis(:,2) == indNewf );
        nf = CameraMeasurementThis( m2, 1 );

        Estimation_X0.landmarks(1:3,NumberOfFeature+i) = nf;
        newnf=CameraMeasurementThis( m2, 3 );
        tempKK(3*i-2:3*i,3*i-2:3*i)=diag(newnf.^2)*Sigma_OB^2;
    end
        Sigma   = blkdiag(Estimation_X0.cov,tempKK);
    Estimation_X0.cov = Sigma;
end

end
  
     
     



