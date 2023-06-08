function X_t = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 
N = size(x1,1);
X_t = zeros(N,3);
m = 5;
for j=1:m
    for i=1:N
    X_t(i,:) = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:)');  
    end
end

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
J1 = Jacobian_Triangulation(C1, R1, K, X0);
J2 = Jacobian_Triangulation(C2, R2, K, X0);
J3 = Jacobian_Triangulation(C3, R3, K, X0);

J_m = [J1'; J2'; J3'];
b = [x1, x2, x3]';
img_1 = K*R1*(X0 - C1);
img_2 = K*R2*(X0 - C2);
img_3 = K*R3*(X0 - C3);

f_x = [img_1(1,1)/img_1(3,1), img_1(2,1)/img_1(3,1), img_2(1,1)/img_2(3,1), img_2(2,1)/img_2(3,1), img_3(1,1)/img_3(3,1), img_3(2,1)/img_3(3,1)]';
delta_x = inv(J_m'*J_m)*J_m'*(b - f_x);
X = X0 + delta_x;
X = X';
end

function J = Jacobian_Triangulation(C, R, K, X)
img = K*R*(X - C);
temp = K*R;
d_u = temp(1,:);
d_v = temp(2,:);
d_w = temp(3,:);
d_f = [((img(3,1).*d_u - img(1,1).*d_w)./(img(3,1).^2)); ((img(3,1).*d_v - img(2,1).*d_w)./(img(3,1).^2))];
J = d_f';
end
