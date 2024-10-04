%% DH_Convention.m

function A = DH_Convention(theta, d, a, alpha)
    R_z_theta = [cos(theta), -sin(theta), 0, 0;
                sin(theta), cos(theta), 0, 0;
                0, 0, 1, 0;
                0, 0, 0, 1];

    T_z_d = [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, d;
            0, 0, 0, 1];

    T_x_a = [1, 0, 0, a;
            0, 1, 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];

    R_x_alpha = [1, 0, 0, 0;
                0, cos(alpha), -sin(alpha), 0;
                0, sin(alpha), cos(alpha), 0;
                0, 0, 0, 1];

    A = R_z_theta * T_z_d * T_x_a * R_x_alpha;
end