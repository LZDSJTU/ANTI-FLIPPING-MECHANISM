%% Tipover Criteria & Platform Optimization
% Multiple Functions Called
clear
clc
tic
n_experiment = 1;
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
LEN=0.3;
W=0.3;
H=0.2;
M=30;
AddtionalWeight=10;
% Wheel Height
h=0.035;
% Platform COG Position
pm = [0; 0; -H/2];

%% Section 3 Mobile Manipulator Motion Parameters and Force Condition
% Joints Angle
q_range = [165*pi/180,110*pi/180,110*pi/180,160*pi/180,120*pi/180,400*pi/180];
% Joints Angular Velocity
qd_range = [250*pi/180,250*pi/180,250*pi/180,320*pi/180,320*pi/180,420*pi/180];
% Joints Angular Accelaration
qdd_range = [100*pi/180,100*pi/180,100*pi/180,100*pi/180,100*pi/180,100*pi/180];
% Base Angular Velocity, Accelaration
w0 = [0; 0; 0];
w0d = [0; 0; 0];
p0dd = [0; 0; g];
% End Effector Force Condition
f_end = [0; 0; 0];
u_end = [0; 0; 0];


%% Section 4 Read Path
load('C:\Users\hasee\Desktop\毕设MATLAB程序\配重程序\制定轨迹\轨迹\PathPlanning_2_1.mat');
trial = size(q,1);
xsum(1)=0;
ysum(1)=0;
for i = 1:trial
    % Homogeneous Matrix Calculation, Output Rotation Matrix
    R = HomogeneousMatrix( L, q(i,:), n );
    % Newton Euler Recurrence Algorithm
    [ Fjoint1, Mjoint1 ] = DynamicRecurrence( qd(i,:), qdd(i,:), ...
        w0, w0d, p0dd, f_end, u_end, R, r, rc, m, I, n);
    % Force Reduction
    Madd = cross([0;0;0.166],Fjoint1)+cross(rb,[0;0;-mb*g]-mb*(p0dd-[0;0;g]));
    Mbase2platform = Mjoint1 + Madd;
    Fbase2platform = [0;0;-mb*g] + Fjoint1-mb*(p0dd-[0;0;g]);
    % Intersection Acquirement
    [x_intersect, y_intersect,Fc,Mc,dx] = ForceSimplify(M,H,h,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd);
    intersect(i,1:2) = [x_intersect, y_intersect];
    
    [ FcPlus, McPlus, x_intersectPlus, y_intersectPlus ] = ForceSimplifyPlus( Fc, Mc, AddtionalWeight, pm ,H, h);
    intersectPlus(i,1:2) = [x_intersectPlus, y_intersectPlus];
    
    [ FcPlusAndMoving, McPlusAndMoving, x_intersectPlusAndMoving, y_intersectPlusAndMoving ] = ForceSimplifyPlusAndMoving( Fc, Mc, AddtionalWeight, pm ,H, h, xsum(i), ysum(i));
    intersectPlusAndMoving(i,1:2) = [x_intersectPlusAndMoving, y_intersectPlusAndMoving];
    
    [ distance, x, y ] = LocationOfWeight( x_intersectPlusAndMoving, y_intersectPlusAndMoving, LEN, W, FcPlusAndMoving, AddtionalWeight );
    Location(i,1:2)=[x, y];
    xsum(i+1)=xsum(i)+Location(i,1);
    ysum(i+1)=ysum(i)+Location(i,2);
    
    [ x_intersectFinal, y_intersectFinal ] = PointAfterMovingingWeight( xsum(i+1), ysum(i+1), FcPlus, McPlus, AddtionalWeight, pm ,H, h );    
    intersectFinal(i,1:2) = [x_intersectFinal, y_intersectFinal];

end

figure(1)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
axis([-1*LEN 1*LEN -1*W 1*W]);
hold on;
plot(intersect(:,1),intersect(:,2),'*');
for i = 1:trial
    if mod(i,10)==0
    c = [' ',num2str(i)];
    text(intersect(i,1),intersect(i,2),c);
    end
end
figure(2)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
axis([-1*LEN 1*LEN -1*W 1*W]);
hold on;
plot(intersectPlus(:,1),intersectPlus(:,2),'*');
for i = 1:trial
%     if mod(i,10)==0
    c = [' ',num2str(i)];
    text(intersectPlus(i,1),intersectPlus(i,2),c);
%     end
end
figure(3)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
axis([-1*LEN 1*LEN -1*W 1*W]);
hold on;
plot(intersectPlusAndMoving(:,1),intersectPlusAndMoving(:,2),'*');
for i = 1:trial
%     if mod(i,10)==0
    c = [' ',num2str(i)];
    text(intersectPlusAndMoving(i,1),intersectPlusAndMoving(i,2),c);
%     end
end
figure(4)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
axis([-1*LEN 1*LEN -1*W 1*W]);
hold on;
plot(intersectFinal(:,1),intersectFinal(:,2),'*');
for i = 1:trial
%     if mod(i,10)==0
    c = [' ',num2str(i)];
    text(intersectFinal(i,1),intersectFinal(i,2),c);
%     end
end
figure(5)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
axis([-1*LEN 1*LEN -1*W 1*W]);
hold on;
plot(xsum,ysum,'*');

c = [' ',num2str(0)];
text(xsum(1),ysum(1),c);

for i = 2:length(xsum)

    if xsum(i)~=xsum(i-1)||ysum(i)~=ysum(i-1)
        c = [' ',num2str(i-1)];
        text(xsum(i),ysum(i),c);
    end
end

toc