function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

x1 = [x1, ones(length(x1), 1)];
x2 = [x2, ones(length(x2), 1)];

X = zeros(length(x1), 3);

P1 = K*[R1, -R1*C1];
P2 = K*[R2, -R2*C2];

for i=1:length(x2)
    X1=Vec2Skew(x1(i, :)');
    X2=Vec2Skew(x2(i, :)');
    [U, S, V] = svd([X1*P1; X2*P2]);
    X(i,:) = [V(1, 4) ./ V(4, 4), V(2, 4) ./ V(4, 4), V(3, 4) ./ V(4, 4)];
end
