In ar_cube.m
````
1. You don't need to multiply the estimated homography by K_inverse.
2. This is mainly because in the SVD step of the rotation matrix, you get rid of the singular matrix.
3. U and V.T are strictly norm 1. Since K is just a multiplying a bunch of contants to [R | t], it gets removed when we remove the singular matrix.
4. Also we explicitly make sure the R matrix is norm 1 using the det(U*V.T) in the identity matrix.
````
