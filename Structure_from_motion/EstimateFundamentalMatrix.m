function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2
N = size(x1,1);

A = [x1(:,1).*x2(:,1), x1(:,1).*x2(:,2), x1(:,1), x1(:,2).*x2(:,1), x1(:,2).*x2(:,2), x1(:,2), x2(:,1), x2(:,2), ones(N,1)];
[U,S,V] = svd(A);

x = V(:,end);
F = reshape(x,3,3);
[U_f,S_f,V_f] = svd(F);
S_f(end,end) = 0;
F = U_f*S_f*V_f';
F = F ./ norm(F);
o = Vec2Skew([x1(1,:) 1]')*[x1(1,:) 1]'
k = [x1(1,:) 1]*Vec2Skew([x1(1,:) 1]')'
