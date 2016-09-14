% configuration 

N_LANDMARKS = 500;      % number of landmarks

ADD_NOISE = 1;          % add noise or not to odometry and observation

% percentage of noise
SIGMA_ODOM = 0.2;
SIGMA_OBSV = 0.1;

ODOM_NOISE = 2*diag([0.05^2,0.05^2,0.05^2, 0.05^2,0.05^2,0.05^2]); % odometry noise cov matrix should be 6*6

OBSV_NOISE = 2*0.05^2*eye(3);           % observation noise, assume RGB-D sensor and dx, dy, dz measurement


% sensor config FOV
MAX_DEGREE = 2*pi/3;        % maximum field of view
MAX_RANGE  = 25.0;          % maximum range of the sensor