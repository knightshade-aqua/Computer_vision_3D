function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
H_temp = H;
if H_temp(3,3)<0
     H_temp(3,3) = H_temp(3,3)*-1;
end
H(:,1) = H(:,1);
H(:,2) = H(:,2);
H(:,3) = cross(H(:,1),H(:,2));

if H(3,3)<0
    H(3,3) = H(3,3)*-1;
end

[U,S,V] = svd(H);
diag_temp = [1,0,0;0,1,0;0,0,det(U*V')];
R = U*diag_temp*V';
t = H_temp(:,3)./norm(H_temp(:,1));
% YOUR CODE HERE: Project the points using the pose
proj_points = zeros(3,size(render_points,1));
for k=1:size(render_points,1)
    proj_points(:,k) = K*(R*render_points(k,:)'+ t);
    proj_points(:,k) = proj_points(:,k)./proj_points(3,k);
end
proj_points(3,:) = [];
proj_points = proj_points';
end
