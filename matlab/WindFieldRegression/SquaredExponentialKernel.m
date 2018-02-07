
function [result] = SquaredExponentialKernel(x1,x2,l,sigma_f)
%SquaredExponentialKernel
%   SquaredExponentialKernel function
    r = norm(x1-x2);
    result = sigma_f^2 * exp( -(abs(r))^2 / (2*l^2) );
end

