% compute the optimal alpha for 

% symbolic variables of the pose of wx
syms w11 w12 w13
syms w21 w22 w23
syms w31 w32 w33

R_o_to_wx = [w11, w12, w13;
    w21, w22, w23;
    w31, w32, w33];

syms Rot_Z(z)

Rot_Z(z) = [cos(z), -sin(z), 0;
    sin(z), cos(z), 0;
    0, 0, 1];

syms a

Rot_alpha = Rot_Z(a);

j = [0, 1, 0]';

global_j = R_o_to_wx * Rot_alpha * j;

i = [1, 0, 0]';

global_i = R_o_to_wx * Rot_alpha * i;
