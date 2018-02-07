clear();
%delete(findall(0,'Type','figure'));

%% Load datalog
[CEKF_states, CEKF_var, localPosition] = CEKFWindFromPX4Log('2017-11-22/dflogs/flight7-dubins_trochoid_ws05/2017-11-22 16-19-41.mat',400,1);

localPosition = localPosition(100:end,:);
CEKF_states = CEKF_states(100:end,:);
CEKF_var = CEKF_var(100:end,:);

%================================================================================
%================================================================================
% %% Find hyperparameters l, sigma_f of Kernel Function
sigma_n = 0.1;
sigma_n_E = diag(sqrt(CEKF_var(:,1))); %sigma_n_E2 = sigma_n*eye(size(CEKF_states,1));
sigma_n_N = diag(sqrt(CEKF_var(:,2))); %sigma_n_N2 = sigma_n*eye(size(CEKF_states,1));
sigma_n_U = diag(sqrt(CEKF_var(:,3))); %sigma_n_U2 = sigma_n*eye(size(CEKF_states,1));
% Initial guess
Nstarts = 20;
l_samp = hypSample ([0.1 200], Nstarts); % for l
sf_samp = hypSample ([0.1 5], Nstarts); % for sigma_f
sn_samp = hypSample ([0.1 4], Nstarts); % for sigma_n
inits = log([l_samp sf_samp]);
options = optimoptions('fminunc', 'Algorithm', 'trust-region', 'SpecifyObjectiveGradient', true);
whichVars = [0 0 1];


paramVecE = [];
paramVecN = [];
paramVecU = [];

for i = 1:Nstarts
    fprintf('Optimizer run %d / %d\n',i,Nstarts);
    
    disp('Finding hyperparameters for East direction...');
    [thetaE, fval(1,i)] = fminunc(@(hyperparams) WFGPOptimizeParams(hyperparams, localPosition, CEKF_states(:,1), whichVars, sigma_n_E), inits(i,:), options);
    
    disp('Done. Finding hyperparameters for North direction...');   
    [thetaN, fval(2,i)] = fminunc(@(hyperparams) WFGPOptimizeParams(hyperparams, localPosition, CEKF_states(:,2), whichVars, sigma_n_N), inits(i,:), options);
    
    disp('Done. Finding hyperparameters for Vertical direction...');
    [thetaU, fval(3,i)] = fminunc(@(hyperparams) WFGPOptimizeParams(hyperparams, localPosition, CEKF_states(:,3), whichVars, sigma_n_U), inits(i,:), options);
    disp('Done.');
    paramVecE = [paramVecE; fval(1,i) inits(i,:) thetaE];
    paramVecN = [paramVecN; fval(2,i) inits(i,:) thetaN];
    paramVecU = [paramVecU; fval(3,i) inits(i,:) thetaU];
end
%% Select best parameters
paramVecE(:,2:end) = exp(paramVecE(:,2:end));
paramVecN(:,2:end) = exp(paramVecN(:,2:end));
paramVecU(:,2:end) = exp(paramVecU(:,2:end));
% d) Select best candidate
paramVecE = sortrows(paramVecE);
paramVecN = sortrows(paramVecN);
paramVecU = sortrows(paramVecU);

%% Set parameters
l = 66.0342;
sigma_f = 1.6508;

l_E = 326.139;
sigma_f_E = 2.165;

l_N = 157;
sigma_f_N = 1.70;

l_U = 770;
sigma_f_U = 0.247;

%% Results:
% For sigma_n = 0.1 : 
% E: l = 181.5; sigma_f = 1.8487; -logpyX = -197.80
% N: l = 9.696; sigma_f = 1.8009; -logpyX =  787.72
% U: l = 27.04; sigma_f = 0.2994; -logpyX = -297.34

% For sigma_n = diag(stateCovariance) : 
% E: l = 8.378; sigma_f = 1.6212; -logpyX =  560.66
% N: l = 6.756; sigma_f = 1.9346; -logpyX =  864.65
% U: l = 5.884; sigma_f = 0.2782; -logpyX = -78.853

% For sigma_n = diag(sqrt(stateCovariance)) : 
% E: l = 246.8; sigma_f = 1.7411; -logpyX = -285.18
% N: l = 13.07; sigma_f = 1.8407; -logpyX =  704.16
% U: l = 252.1; sigma_f = 0.2810; -logpyX = -208.14

% For full optimization : 
% E: l = 3021.; sigma_f = 2.8617; sigma_n = 0.1460; -logpyX = -307.08
% N: l = 79.89; sigma_f = 3.1663; sigma_n = 0.2931; -logpyX =  195.45
% U: l = 214.3; sigma_f = 0.3545; sigma_n = 0.1373; -logpyX = -342.37

%% Plot Kernel function
% r = 0:0.1:200;
% r = r';
% for i = 1:1:size(r,1)
%     k(i,1) = SquaredExponentialKernel([0,0,0], [0,0,r(i)], l, sigma_f);
% end
% figure();
% plot(r,k);
% xlabel('distance r in m');
% ylabel('Covariance k(r)');


%% Calculate marginal likelihood
% x_samples = 20;
% y_samples = 20;
% [x,y] = meshgrid(linspace(170,195,x_samples), linspace(1.7,2.1,y_samples));
% logpyX = zeros(x_samples, y_samples);
% for i = 1:x_samples
%     for j = 1:y_samples
%         [logpyX(i,j), grd] = WFGPOptimizeParams([log(x(i,j)); log(y(i,j))], localPosition, CEKF_states(:,1), [0 0 1], sigma_n_E2);
%     end
%     fprintf('%d\n', i);
% end
% 
% %% Plot marginal likelihood
% figure();
% contour(x,y,exp(-logpyX),20);
% xlabel('length scale factor l');
% ylabel('sigma_f');