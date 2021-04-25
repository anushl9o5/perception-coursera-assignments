function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];

%ax = @(x, xp)[-x(1), - x(2), -1, 0, 0, 0, x(1)*xp(1), x(2)*xp(1), xp(1)];
%ay = @(x, xp)[0, 0, 0, -x(1), -x(2), -1, x(1)*xp(2), x(2)*xp(2), xp(2)];

ax = [-video_pts(:, 1) -video_pts(:, 2) -1*ones(4, 1) zeros(4, 1) zeros(4, 1) zeros(4, 1) video_pts(:, 1).*logo_pts(:, 1) video_pts(:, 2).*logo_pts(:, 1) -logo_pts(:, 1)];
ay = [zeros(4, 1) zeros(4, 1) zeros(4, 1) -video_pts(:, 1) -video_pts(:, 2) -1*ones(4, 1) video_pts(:, 1).*logo_pts(:, 2) video_pts(:, 2).*logo_pts(:, 2) -logo_pts(:, 2)];

assert(length(video_pts) == 4);

A = ones(length(video_pts), 9);

for i=1:length(video_pts)
    A(2*i - 1, :) = ax(i, :); %ax(video_pts(i, :), logo_pts(i, :));
    A(2*i, :) = ay(i, :); %ay(video_pts(i, :), logo_pts(i, :));
end

[U,S,V] = svd(A);
H = reshape(V(:,9), [3, 3])';

%{
ax = @(x, xp)[-x(1), - x(2), -1, 0, 0, 0, x(1)*xp(1), x(2)*xp(1), xp(1)];
ay = @(x, xp)[0, 0, 0, -x(1), -x(2), -1, x(1)*xp(2), x(2)*xp(2), xp(2)];

% get number of points (should be 4)
len = length(video_pts);
assert(len == 4);

% initialize matrix A with zeros to prevent resizing in loop
A = zeros(len, 9);

for i=1:len
    A(i * 2 - 1, :) = ax(video_pts(i, :), logo_pts(i, :));
    A(i * 2, :) = ay(video_pts(i, :), logo_pts(i, :));
end
    
[U, S, V] = svd(A);
H = reshape(V(:, end), [3,3])';
%}

end

