function [ R_g2f ] = getR_g2f( q )
%   Returns rotation matrix from a quaternion
qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

R_g2f = [qw*qw+qx*qx-qy*qy-qz*qz 2*(qx*qy+qw*qz) 2*(qx*qz-qw*qy);
         2*(qx*qy-qw*qz) qw*qw-qx*qx+qy*qy-qz*qz 2*(qw*qx+qy*qz);
         2*(qw*qy+qx*qz) 2*(qy*qz-qw*qx) qw*qw-qx*qx-qy*qy+qz*qz];


end

