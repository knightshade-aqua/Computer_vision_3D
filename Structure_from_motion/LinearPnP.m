function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly
N = size(x,1);
x_c = zeros(N,3);
A = zeros(N*3,12);
Z = zeros(1,4);
K_inv = inv(K);

for i=1:N
    x_c(i,:) = K_inv*[x(i,:) 1]';
end

for i=1:N
    j = 1 + (i-1)*3;
    Xc = [X(i,:) 1];
    B = [Xc, Z, Z; Z, Xc, Z; Z, Z, Xc];
    M = Vec2Skew(x_c(i,:)');
    A(j:j+2,:) = M*B; 
end

[u,s,v] = svd(A);
P = reshape(v(:,end),4,3);
P = P';

t = P(:,end);
R = P(1:3,1:3);

[u,d,v] = svd(R);
if det(u*v') > 0
    R = u*v';
    t = t./d(1,1);
elseif det(u*v') < 0
    R = -u*v';
    t = -t./d(1,1);
end

C = -R'*t;
        
