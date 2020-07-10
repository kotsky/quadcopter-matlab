
%% define main variables
dt = 1;  %our sampling rate

u = .01; % define acceleration magnitude
HexAccel_noise_mag = .1; %process noise: the variability in how fast the Hexbug is speeding up (stdv of acceleration: meters/sec^2)
tkn_x = 1;  %measurement noise in the horizontal direction (x axis).
tkn_y = 1;  %measurement noise in the horizontal direction (y axis).
Ez = [tkn_x 0; 0 tkn_y];
Ex = [dt^4/4 0 dt^3/2 0; ...
    0 dt^4/4 0 dt^3/2; ...
    dt^3/2 0 dt^2 0; ...
    0 dt^3/2 0 dt^2].*HexAccel_noise_mag^2; % Ex convert the process noise (stdv) into covariance matrix
P = Ex; % estimate of initial Hexbug position variance (covariance matrix)

%% Define update equations in 2-D! (Coefficent matrices): A physics based model for where we expect the HEXBUG to be [state transition (state + velocity)] + [input control (acceleration)]
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; %state update matrice
B = [(dt^2/2); (dt^2/2); dt; dt];
C = [1 0 0 0; 0 1 0 0];  %this is our measurement function C, that we apply to the state estimate Q to get our expect next/new measurement


%% initize result variables
% Initialize for speed
Q_loc = []; % ACTUAL hexbug motion path
vel = []; % ACTUAL hexbug velocity
Q_loc_meas = []; % the hexbug path extracted by the tracking algo

%% initize estimation variables
Q_loc_estimate = []; %  position estimate
vel_estimate = []; % velocity estimate
P_estimate = P;
predic_state = [];
predic_var = [];