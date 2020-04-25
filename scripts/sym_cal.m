syms alpha beta gama

rot_X = [1, 0, 0, 0;
    0, cos(alpha), -sin(alpha), 0;
    0, sin(alpha), cos(alpha), 0;
    0, 0, 0, 1];

rot_Y = [cos(beta), 0, sin(beta), 0,;
    0, 1, 0, 0;
    -sin(beta), 0, cos(beta), 0;
    0, 0, 0, 1];

rot_Z = [cos(gama), -sin(gama), 0, 0;
    sin(gama), cos(gama), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

rot_X_Y = rot_X * rot_Y;

rot_X_Y_Z = rot_X * rot_Y * rot_Z;