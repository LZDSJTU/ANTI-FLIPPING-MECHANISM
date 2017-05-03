function diameter = CalculateDiameter(parameters)      %变量为12个月份的库水位高度
q(1:3)=parameters(1:3);
qd(1:3)=parameters(4:6);
qdd(1:3)=parameters(7:9);
q(4:6)=zeros(3,1);
qd(4:6)=zeros(3,1);
qdd(4:6)=zeros(3,1);
%% Section 1 Set Parameters
% Gravity
global g;
g = 9.81;
% Link Number
n = 6;
% D-H Parameter of IRB 120
d = [0.124; 0.0; 0.0; 0.302; 0; 0.072];
a = [0.0; 0.27; 0.07; 0.0; 0.0; 0.0;];
alpha = [pi/2; 0; pi/2; pi/2; -pi/2; 0];
[ IRB120, L ] = CreateRobot( d, a, alpha, n );
[ I, m, rc, r ] = DynamicParameters( d, a );

%% Section 2 Title
% Location of Manipulator on Platform
pb = [0; 0; 0];
% Base Mass
density=1.3871516;
mb = 6.21502*density;
% Base COG to it Bottom Center
rb = [-42.04; 0.08; 79.64]*1e-3;
% Platform Height and Mass
LEN=0.4;
W=0.4;
H=0.2;
M=30;
% Wheel Height
h=0.035;
% Platform COG Position
pm = [0; 0; -H/2];

%% Section 3 Mobile Manipulator Motion Parameters and Force Condition

% Base Angular Velocity, Accelaration
w0 = [0; 0; 0];
w0d = [0; 0; 0];
p0dd = [0; 0; g];
% End Effector Force Condition
f_end = [0; 0; 0];
u_end = [0; 0; 0];


%% Section 4 Read Path

    % Homogeneous Matrix Calculation, Output Rotation Matrix
    R = HomogeneousMatrix( L, q, n );
    % Newton Euler Recurrence Algorithm
    [ Fjoint1, Mjoint1 ] = DynamicRecurrence( qd, qdd, ...
        w0, w0d, p0dd, f_end, u_end, R, r, rc, m, I, n);
    % Force Reduction
    Madd = cross([0;0;0.166],Fjoint1)+cross(rb,[0;0;-mb*g]-mb*(p0dd-[0;0;g]));
    Mbase2platform = Mjoint1 + Madd;
    Fbase2platform = [0;0;-mb*g] + Fjoint1-mb*(p0dd-[0;0;g]);
    % Intersection Acquirement
    [x_intersect, y_intersect,Fc,Mc,dx] = ForceSimplify(M,H,h,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd);
    % Calculate the diameter of the intersection point
    diameter = -sqrt(x_intersect^2 + y_intersect^2);

end
