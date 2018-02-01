function [fcn, grd] = WFGPOptimizeParams(hyperparams, X_train,y_train, sigma_n2)
%WindFieldGPRegression
%   Does a Gaussian Process Regression for 3-dim input samples X_train and
%   1-dim output y_train, y_star with a squared exponential Kernel
N = size(X_train,1);
%Q = size(X_test,1);

l = exp(hyperparams(1));
sigma_f = exp(hyperparams(2));

K = zeros(N); dKdl = zeros(N); dKdf = zeros(N);

% Compute covariance matrix K
for i = 1:1:N
    for j = 1:1:N
        [K(i,j), dKdl(i,j), dKdf(i,j)] = SquaredExponentialKernelOptimize(X_train(i),X_train(j),l,sigma_f);
    end
end

Ky = K + sigma_n2;

L = chol(Ky,'lower');
alpha = L'\(L\y_train);
invK = inv(Ky);
alpha2 = invK*y_train;

logpyX = -0.5 * y_train' * alpha - sum(log(diag(L))) - N/2 * log(2*pi);
%%Calculate derivative for gradient
dlogp_dl = l*trace((alpha2*alpha2' - invK)*dKdl)/2; %%Calculate derivative for gradient
dlogp_df = sigma_f*trace((alpha2*alpha2' - invK)*dKdf)/2;

fcn = -logpyX;

grd(1) = -dlogp_dl;
grd(2) = -dlogp_df;

end

