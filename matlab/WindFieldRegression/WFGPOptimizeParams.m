function [fcn, grd] = WFGPOptimizeParams(hyperparams, X_train,y_train,whichVars,sigma_n2)
%WindFieldGPRegression
%   Does a Gaussian Process Regression for 3-dim input samples X_train and
%   1-dim output y_train, y_star with a squared exponential Kernel
N = size(X_train,1);
%Q = size(X_test,1);

if whichVars(1) == 0
    l = exp(hyperparams(1));
else
    l = whichVars(1);
end

if whichVars(2) == 0
    sigma_f = exp(hyperparams(2));
else
    sigma_f = whichVars(2);
end

if whichVars(3) == 0
    sigma_n = eye(N) * exp(hyperparams(3));
else
    if exist('sigma_n2')
        sigma_n = sigma_n2;
    else
        sigma_n = eye(N) * whichVars(3);
    end
end



K = zeros(N); dKdl = zeros(N); dKdf = zeros(N); dKdn = zeros(N);

% Compute covariance matrix K
for i = 1:1:N
    for j = 1:1:N
        [K(i,j), dKdl(i,j), dKdf(i,j), dKdn(i,j)] = SquaredExponentialKernelOptimize(X_train(i,:),X_train(j,:),l,sigma_f,sigma_n(i,j));
    end
end

L = chol(K,'lower');
alpha = L'\(L\y_train);
invK = inv(K);
alpha2 = invK*y_train;

logpyX = -0.5 * y_train' * alpha - sum(log(diag(L))) - N/2 * log(2*pi);
fcn = -logpyX;

grd = [];
%%Calculate derivative for gradient
if whichVars(1) == 0
    dlogp_dl = l*trace((alpha2*alpha2' - invK)*dKdl)/2; %%Calculate derivative for gradient
    grd = [grd; -dlogp_dl];
end
if whichVars(2) == 0
    dlogp_df = sigma_f*trace((alpha2*alpha2' - invK)*dKdf)/2;
    grd = [grd; -dlogp_df];
end
if whichVars(3) == 0
    dlogp_dn = sigma_n*trace((alpha2*alpha2' - invK)*dKdn)/2;
    grd = [grd; -dlogp_dn];
end




end

