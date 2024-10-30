function floor_friction_g1_partial_dfr_dx = floor_friction_g1_partial_dfr_dx_func(in1)
%FLOOR_FRICTION_G1_PARTIAL_DFR_DX_FUNC
%    FLOOR_FRICTION_G1_PARTIAL_DFR_DX = FLOOR_FRICTION_G1_PARTIAL_DFR_DX_FUNC(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    13-Sep-2024 09:17:50

dt = in1(:,7);
fn = in1(:,5);
mu = in1(:,6);
x1s_x = in1(:,1);
x1s_y = in1(:,2);
x1s_x0 = in1(:,3);
x1s_y0 = in1(:,4);
t2 = abs(dt);
t3 = 1.0./dt;
t4 = -x1s_x0;
t5 = -x1s_y0;
t6 = 1.0./t2.^2;
t7 = t4+x1s_x;
t8 = t5+x1s_y;
t9 = abs(t7);
t10 = abs(t8);
t11 = sign(t7);
t12 = sign(t8);
t13 = t9.^2;
t14 = t10.^2;
t15 = t6.*t13;
t16 = t6.*t14;
t17 = t15+t16;
t18 = 1.0./sqrt(t17);
t19 = t18.^3;
t20 = fn.*mu.*t3.*t18;
t21 = -t20;
floor_friction_g1_partial_dfr_dx = reshape([t21+fn.*mu.*t3.*t6.*t7.*t9.*t11.*t19,fn.*mu.*t3.*t6.*t8.*t9.*t11.*t19,fn.*mu.*t3.*t6.*t7.*t10.*t12.*t19,t21+fn.*mu.*t3.*t6.*t8.*t10.*t12.*t19],[2,2]);
end
