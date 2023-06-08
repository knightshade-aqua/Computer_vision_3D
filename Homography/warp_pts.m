function [ warped_pts ] = warp_pts( video_pts, logo_pts, sample_pts)
% warp_pts computes the homography that warps the points inside
% video_pts to those inside logo_pts. It then uses this
% homography to warp the points in sample_pts to points in the logo
% image
% Inputs:
%     video_pts: a 4x2 matrix of (x,y) coordinates of corners in the
%         video frame
%     logo_pts: a 4x2 matrix of (x,y) coordinates of corners in
%         the logo image
%     sample_pts: a nx2 matrix of (x,y) coordinates of points in the video
%         video that need to be warped to corresponding points in the
%         logo image
% Outputs:
%     warped_pts: a nx2 matrix of (x,y) coordinates of points obtained
%         after warping the sample_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% Complete est_homography first!
[ H ] = est_homography(video_pts, logo_pts);

% YOUR CODE HERE
n = size(sample_pts,1);
warped_pts = zeros(3,n);
sample_pts_temp = sample_pts';
sample_pts_temp = [sample_pts_temp;ones(1,n)];
for k=1:n
    warped_pts(:,k) = H*sample_pts_temp(:,k);
end
warped_pts(1,:) = warped_pts(1,:)./warped_pts(3,:);
warped_pts(2,:) = warped_pts(2,:)./warped_pts(3,:);
warped_pts(3,:) = [];
warped_pts = warped_pts';
end

