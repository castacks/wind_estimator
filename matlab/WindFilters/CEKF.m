function [ allStates, variances ] = CEKF(time, groundspeeds, dynPressures, quaternions)
% Wind estimator from Pulkit Goyal
%
% Vg_g = [Vg_gn, Vg_ge] - groundspeed in earth frame [m/s]
% Vw_g = [Vw_gn, Vw_ge] - wind speed in earth frame [m/s]
% Va_f = [Va_fx, Va_y] - airspeed in body fixed frame [m/s]
% eta - correction factor
% dP - differecial pressure
%
% State vector s: [Vw_g, eta, Va_f]
% Measurement vector z: [Vg_g, dP]
%
% x_p[k] - predicted state
% x[k] - estimated state
% A - Model matrix
% P_p[k] - predicted error covariance
% P[k] - estimated error covariance
% Q[k] - model uncertainty
% y[k] - innovation
% S[k] - innovation covariance
% h(x) - measurement function
% H[k] - measurement matrix
% R[k] - Measurement uncertainty
% K - Kalman gain
% 
% 
%Algorithm:
%   Predict state
%       x_p[k] = A * x[k-1]
%       P_p[k] = A * P[k-1] * A^T + Q[k]
%   Measurement
%       y[k] = z[k] - h( x_p[k] )
%       S[k] = H[k] * P_p[k] * H[k]^T + R[k]
%   Compute Kalman Gain
%       K[k] = P_p[k] * H[k]^T * S[k]^-1  
%   Estimate state
%       x[k] = x_p[k] + K * y[k]
%       P[k] = (I - K[k]*H[k]) * P_p[k] * (I - K[k]*H[k])^T + K[k]*R[k]*K[k]^T
%   
%                         
%
%

%% Initialize variables
R_g2f = getR_g2f(quaternions(1,:));
R_f2g = transpose(R_g2f);
initialAirspeed = R_g2f*transpose(groundspeeds(1,:));
estimation = [0.1; 0.1; 0.1; 1.225/2; initialAirspeed(1); initialAirspeed(2); initialAirspeed(3)];
covariance = diag([25; 25; 8; 1e-3; 12; 12; 12]);
A = eye(7);

%% Run the filter
i = 2;
max = size(time,1);
allStates = zeros(max,7);
variances = zeros(max,7);
allStates(1,:) = estimation;
while i<=max
    %% Compute dt
    dt = time(i) - time(i-1);
    z = [groundspeeds(i,1); groundspeeds(i,2); groundspeeds(i,3); dynPressures(i)];
    R_g2f = getR_g2f(quaternions(i,:));
    R_f2g = transpose(R_g2f);
    
    %% Predict state
    prediction = A*estimation;
    Q = eye(7)*1e-4;
    Q(4,4) = 1e-8;
    Q(5,5) = 1e-2;
    Q(6,6) = 0.1;
    Q(7,7) = 0.1;
    predCovariance = A*covariance*transpose(A) + dt*Q*dt;
       
    Vw_g = prediction(1:3);
    eta = prediction(4);
    Va_f = prediction(5:7);
    %% Predict measurement vector z
    %z_p[k] = [Vw_g + cos(phi)*Va_f; eta*(Va_fx)^2] = h(x_p[k])
    z_p = [Vw_g + R_f2g*Va_f;
           eta*Va_f(1)*Va_f(1)];
    
    y = z - z_p;
    %% Create measurement matrix H
    H = [1 0 0 0 R_f2g(1,:); 
         0 1 0 0 R_f2g(2,:); 
         0 0 1 0 R_f2g(3,:);
         0 0 0 Va_f(1)*Va_f(1) 2*eta*Va_f(1) 0 0];
    
    R = diag([1; 1; 1; 10]);
    S = H * predCovariance * transpose(H) + R;
    
    %% Compute Kalman gain K
    K = (predCovariance * transpose(H))/S;
    
    %% Estimate 
    estimation = prediction + K*y;
    %covariance = (eye(7) - K*H)*predCovariance;
    covariance = (eye(7) - K*H)*predCovariance*transpose(eye(7) - K*H) + K*R*transpose(K);
    
    %% Save state
    allStates(i,:) = estimation;
    variances(i,:) = diag(covariance);
    
    i=i+1;
end

end

