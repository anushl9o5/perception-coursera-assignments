function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

a_i = @(x1i, x2i)[x1i(1)*x2i(1), x1i(1)*x2i(2), x1i(1), x1i(2)*x2i(1), x1i(2)*x2i(2), x1i(2), x1i(1)*x1i(1), x1i(2)*x1i(2), 1];

A = zeros(size(x1, 1), 9);

for i=1:length(x1)
    A(i, :) = a_i(x1(i, :), x2(i, :));
end

[U, D, V] = svd(A);

F = V(:, 9);

F = reshape(F, 3, 3);
[U, S, V] = svd(F);

S(3, 3) = 0;

F = U*S*V';

F = F / norm(F);
end




