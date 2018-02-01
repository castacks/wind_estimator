function [varargout] = vec2meshgrid(v, X)
%VEC2MESHGRID   reshape column vectors to size of X component matrices
%   [varargout] = VEC2MESHGRID(v, X) reshapes the matrix of N-D column
%   vectors v to N 2D matrices, each of size(X), for use as arguments of
%   grid points to SURF or QUIVER, or as vector components to QUIVER.
%   Each of the resulting N matrices is a component of the column vectors
%   in v.
%
% input
%   v = matrix of column vectors
%     = [#dim x #vectors]
%
%   X = matrix of x coordinates of meshgrid points
%     = [M1 x M2 x ... x MN]
%
% output
%   [v1, v2, ..., vN] = vectors' component matrices with size(X)
%     = [M1 x M2 x ... x MN]
%
% See also DOMAIN2VEC, SCALAR2MESHGRID, MESHGRID2VEC, DOMAIN2MESHGRID.
%
% File:      vec2meshgrid.m
% Author:    Ioannis Filippidis, jfilippidis@gmail.com
% Date:      2012.01.14 - 
% Language:  MATLAB R2011b
% Purpose:   reshape matrix of column vectors to meshgrid component
%            matrices
% Copyright: Ioannis Filippidis, 2012-

ndim = size(v, 1);
varargout = cell(1, ndim);
for i=1:ndim
    f = v(i, :);
    varargout{1, i} = reshape(f, size(X) );
end
