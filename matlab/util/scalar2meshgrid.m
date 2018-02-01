function [f] = scalar2meshgrid(f, X)
%SCALAR2MESHGRID   reshape row vector f to size of X
%   [F] = SCALAR2MESHGRID(F, X) reshapes the row array of values of scalar
%   function f to size(X), for use as arguments to SURF.
%
% input
%   f = row array of scalar function values at grid points
%     = [1 x #(meshgrid points) ]
%
%   X = matrix of x coordinates of meshgrid points
%     = [M1 x M2 x ... x MN]
%
% output
%   f = matrix with size(X) of scalar function values at grid points
%     = [M1 x M2 x ... x MN]
%
% Note
%   VEC2MESHGRID can fully replace this function.
%
% See also DOMAIN2VEC, VEC2MESHGRID, DOMAIN2MESHGRID, MESHGRID2VEC.
%
% File:      scalar2meshgrid.m
% Author:    Ioannis Filippidis, jfilippidis@gmail.com
% Date:      2012.01.23 - 
% Language:  MATLAB R2011b
% Purpose:   reshape row array of scalar function values to meshgrid matrix
% Copyright: Ioannis Filippidis, 2012-

f = reshape(f, size(X) );
