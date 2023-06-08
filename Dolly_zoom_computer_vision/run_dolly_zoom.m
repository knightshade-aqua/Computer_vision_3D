% %% load data and set parameters
% points = load('points.mat');
% 
% 
% d_ref = 4;
% f_ref = 400;
% pos = 0 : -0.1 : -9.9;
% 
% d1_ref = 4;
% d2_ref = 20;
% H1 = points.points_A(1,2) - points.points_A(2,2);
% H2 = points.points_C(1,2) - points.points_C(2,2);
% ratio = 2;
% 
% %% Dolly Zoom: keep one object's height constant 
% 
% f = compute_focal_length(d_ref, f_ref, pos);
% 
% for i = 1 : length(f)
%     if i == 1
%         fprintf('Processing frame %03d / %d...', i, length(f));
%     else
%         fprintf(repmat('\b',1,12));  
%         fprintf('%03d / %d...', i, length(f));
%         clf;
%     end
%     
%     figure(1), hold on, axis equal;
%     xlim([0,1920]), ylim([0,1080]);
%     project_objects(f(i), pos(i), points, 1);
%     pause(0.1);
% end
% fprintf('\n');
% 
% %% Dolly Zoom:  keep one object's height constant and adjust another objects height
% 
% [f, pos] = compute_f_pos(d1_ref, d2_ref, H1, H2, ratio, f_ref);
% 
% figure(2), hold on, axis equal;
% xlim([0,1920]), ylim([0,1080]);
% project_objects(f, pos, points, 2);
%A = [0,0,3;0,-1,0;2,0,0];
%[u,s,v] = svd(A);
% function [d] = distance(v)
%     sum = 0;
%     for i=1:4 
%         d = 0.58*v(i,1) - 0.59*v(i,2)-0.57;
%         a = (abs(d))/(sqrt(0.58.^2+0.59.^2));
%         sum = sum + a;
%     end
% v = [0,-0.8;1,0;2.2,0.9;2.9,2.1];
% disp(distance(v));

a = [1,1,-1,-1;1,-1,-1,1;1,1,1,1];
b = [-1.2131,-1.4413,0.3470,0.5752;
    0.0851,-0.7858,-1.6594,-0.7885;
    -1.2334,0.5525,0.3550,-1.4309];
[u,s,v] = svd(b*a');

​
  



​
  



​
  



​
  

