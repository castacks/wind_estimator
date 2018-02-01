function [y_star] = WindFieldGPRegression(X_train,y_train, X_test, l, sigma_f, sigma_n2)
%WindFieldGPRegression
%   Does a Gaussian Process Regression for 3-dim input samples X_train and
%   1-dim output y_train, y_star with a squared exponential Kernel
N = size(X_train,1);
Q = size(X_test,1);

K = zeros(N);

% Compute covariance matrix K
for i = 1:1:N
    for j = 1:1:N
        K(i,j) = SquaredExponentialKernel(X_train(i),X_train(j),l,sigma_f);
    end
end

L = chol(K + sigma_n2,'lower');
alpha = L'\(L\y_train);

K_star = zeros(N,Q);
y_star = zeros(Q,1);

totalIterations = N*Q;
currentIteration = 0;
processingStatus = 0;
displayStep = 1;
fprintf('|');
for i = 1:1:(100/displayStep)
    fprintf('-');
end
fprintf('|\n|');
% Compute K_star
for i = 1:1:N
    for q = 1:1:Q
        K_star(i,q) = SquaredExponentialKernel(X_train(i,:), X_test(q,:), l, sigma_f);
        
        % Show current processing status
        currentIteration = currentIteration + 1;
        if (currentIteration/totalIterations * 100) >= (processingStatus + displayStep)
            processingStatus = processingStatus + displayStep;
            %fprintf('Processing status: %2d%%\n',processingStatus);
            fprintf('=');
        end
        
    end
end

fprintf('|\n Done.\n');
% Mean of prediction
y_star = K_star' * alpha;
%y_star = K_star \ (K+sigma_n*eye(N)) * y_train;

end

