function [covar, d_l, d_sigmaf, d_sigman] = SquaredExponentialKernelOptimize(x1,x2,l,sigma_f,sigma_n)
%SquaredExponentialKernel
%   SquaredExponentialKernel function
    r = norm(x1-x2);
    covar = sigma_f^2 * exp( -(abs(r))^2 / (2*l^2) );
    d_l = covar * (l^-3) * (r)^2; % Differentiate (2.16) from Rasmussen and Williams (2006)
    d_sigmaf = 2*sigma_f * exp(-(r)^2/(2*l^2));
 
    if r == 0
        covar = covar + sigma_n^2;
        d_sigman = 2*sigma_n;
    else
        d_sigman = 0;
    end
end

