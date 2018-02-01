function [allStates, variances] = SimpleWindKF(time,groundspeeds,airspeeds)
%SimpleWindKF Filter that estimates the windspeeds
%   
% Vg=[Vgn,Vge] - horizontal groundspeed[m/s] in North,East
% Va=[Van,Vae] - horizontal airspeed[m/s] in North,East
% Vw=[Vwn,Vwe] - horizontal windspeed[m/s] in North,East
%
% State x[k] = [Vg,Vw]
% Measurement z[k] = [Vg,Va]
%

%% Initialize
estimation = [transpose(groundspeeds(1,:)); 0; 0; 0];
covariance = eye(6);
A = eye(6);

%% Run filter
i = 2;
max = size(time,1);
allStates = zeros(max,6);
variances = zeros(max,6);
allStates(1,:) = estimation;
while i<=max
    %% Compute dt
    dt = time(i) - time(i-1);
    z = [transpose(groundspeeds(i,:)); transpose(airspeeds(i,:))];
    
    %% Predict state
    prediction = A*estimation;
    %% Predict covariance
    Q = diag([1; 1; 1; 0.0001; 0.0001; 0.0001]);
    predCovariance = A*covariance*transpose(A) + dt*Q*dt;

    %% Create measurement matrix H
    H = [1 0  0  0  0 0;
         0 1  0  0  0 0;
         0 0  1  0  0 0;
         1 0  0 -1  0 0;
         0 1  0  0 -1 0;
         0 0  1  0  0 -1];
    
    R = diag([1; 1; 1; 4; 4; 4]);
    S = H * predCovariance * transpose(H) + R;
    
    %% Compute Kalman gain K
    K = (predCovariance * transpose(H))/S;
    
    %% Estimate 
    estimation = prediction + K*(z - H*prediction);
    %covariance = (eye(6) - K*H)*predCovariance;
    covariance= (eye(6) - K*H) * predCovariance * transpose(eye(6) - K*H) + K*R*transpose(K);
    
    %% Save state
    allStates(i,:) = estimation;
    variances(i,:) = diag(covariance);
    
    i=i+1;
end


end

