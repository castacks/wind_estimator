function [y_star] = WindFieldGPRegressionMeshgrid(X_train,y_train, x, y, z, l, sigma_f, sigma_n)
%WindFieldGPRegression
%   Does a Gaussian Process Regression for 3-dim input samples X_train and
%   1-dim output y_train, y_star with a squared exponential Kernel
N = size(X_train,1);
xs = size(x,1);
ys = size(y,2);
zs = size(z,3);

K = zeros(N);

% Compute covariance matrix K
for i = 1:1:N
    for j = 1:1:N
        K(i,j) = SquaredExponentialKernel(X_train(i),X_train(j),l,sigma_f);
    end
end

L = chol(K + sigma_n*eye(N),'lower');
alpha = L'\(L\y_train);

K_star = zeros(N,1);
y_star = zeros(xs,ys,zs);

totalIterations = xs*ys*zs;
currentIteration = 0;
processingStatus = 0;
displayStep = 1;
fprintf('|');
for i = 1:1:(100/displayStep)
    fprintf('-');
end
fprintf('|\n|');
% Compute K_star
for xi = 1:1:xs
    for yi = 1:1:ys
        for zi = 1:1:zs
            for i = 1:1:N
                K_star(i) = SquaredExponentialKernel(X_train(i,:), [x(xi,yi,zi) y(xi,yi,zi) z(xi,yi,zi)], l, sigma_f);
            end
            y_star(xi,yi,zi) = K_star' * alpha;
            % Show current processing status
            currentIteration = currentIteration + 1;
            if (currentIteration/totalIterations * 100) >= (processingStatus + displayStep)
                processingStatus = processingStatus + displayStep;
                %fprintf('Processing status: %2d%%\n',processingStatus);
                fprintf('=');
            end
        end
    end
end
fprintf('|\n Done.\n');
% Mean of prediction
%y_star = K_star' * alpha;

end

