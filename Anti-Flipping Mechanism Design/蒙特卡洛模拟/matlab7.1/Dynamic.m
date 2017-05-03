%% Tipover Criteria & Platform Optimization
% Multiple Functions Called
clear
clc

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
alpha=[pi/2; 0; pi/2; pi/2; -pi/2; 0];
[ IRB120, L ] = CreateRobot( d, a, alpha, n );
[ I, m, rc, r ] = DynamicParameters( d, a );

%% Section 2 Title
% Location of Manipulator on Platform
pb=[0; 0; 0];
% Base Mass
density=1.3871516;
mb=6.21502*density;
% Base COG to it Bottom Center
rb=[-42.04; 0.08; 79.64]*1e-3;
% Platform Height and Mass
LEN=0.4;
W=0.4;
H=0.2;
M=30;
AddtionalWeight=10;
% Wheel Height
h=0.035;
% Platform COG Position
pm=[0; 0; -H/2];

%% Section 3 Mobile Manipulator Motion Parameters and Force Condition
% Joints Angle
q_range=[165*pi/180,110*pi/180,110*pi/180,160*pi/180,120*pi/180,400*pi/180];
% Joints Angular Velocity
qd_range=[250*pi/180,250*pi/180,250*pi/180,320*pi/180,320*pi/180,420*pi/180];
% Joints Angular Accelaration
qdd_range=[2000*pi/180,2000*pi/180,2000*pi/180,2000*pi/180,2000*pi/180,2000*pi/180];
% Base Angular Velocity, Accelaration
w0=[0; 0; 0];
w0d=[0; 0; 0];
p0dd=[0; 0; g];
% End Effector Force Condition
f_end=[0; 0; 0];
u_end=[0; 0; 0];
%% Section 4 Monte-Carlo
nLoop=1000;

for i=1:nLoop
    % Random Paramoters Generator
    [ q, qd, qdd ,p0dd ] = GenerateRandomNumber( q_range, qd_range, qdd_range, p0dd );
    
    % Homogeneous Matrix Calculation, Output Rotation Matrix
    R  = HomogeneousMatrix( L, q, n );
    % Newton Euler Recurrence Algorithm
    [ Fjoint1, Mjoint1 ] = DynamicRecurrence( qd, qdd, w0, w0d, p0dd, f_end, u_end, R, r, rc, m, I, n);
    
    % Force Reduction
    Madd = cross([0;0;0.166],Fjoint1)+cross(rb,[0;0;-mb*g]);
    Mbase2platform = Mjoint1 + Madd;
    Fbase2platform = [0;0;-mb*g] + Fjoint1;
    % Intersection Acquirement
    [x_intersect, y_intersect, Fc, Mc] = ForceSimplify(M,H,h,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd);
    intersect(i,1:2) = [x_intersect, y_intersect];

    [ FcPlus, McPlus, x_intersectPlus, y_intersectPlus ] = ForceSimplifyPlus( Fc, Mc, AddtionalWeight, pm ,H, h);
    intersectPlus(i,1:2) = [x_intersectPlus, y_intersectPlus];

%     [ distance, x, y ] = LocationOfWeight( x_intersectPlus, y_intersectPlus, LEN, W, FcPlus, AddtionalWeight );

    [ distance, x, y ] = LocationOfWeight_circle( x_intersectPlus, y_intersectPlus, LEN, W, FcPlus, AddtionalWeight );
    Location(i,1:2)=[x, y];
    
    [ x_intersectFinal, y_intersectFinal ] = PointAfterChangingWeight( x, y, FcPlus, McPlus, AddtionalWeight, pm ,H, h );    
    intersectFinal(i,1:2) = [x_intersectFinal, y_intersectFinal];
    
    
%     diameter1 = sqrt(x_intersect^2 + y_intersect^2);
%     diameter2 = sqrt(x_intersectPlus^2 + y_intersectPlus^2);
end
    
trial=nLoop;
figure(1)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
hold on;
plot(intersect(:,1),intersect(:,2),'+');
axis equal;
axis([-1*LEN 1*LEN -1*W 1*W]);
% for i = 1:trial
%     c = [' ',num2str(i)];
%     text(intersect(i,1),intersect(i,2),c);
% end
figure(2)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
hold on;
plot(intersectPlus(:,1),intersectPlus(:,2),'*');
axis equal;
axis([-1*LEN 1*LEN -1*W 1*W]);
% for i = 1:trial
%     c = [' ',num2str(i)];
%     text(intersectPlus(i,1),intersectPlus(i,2),c);
% end
figure(3)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
hold on;
plot(intersectFinal(:,1),intersectFinal(:,2),'*');
axis equal;
axis([-1*LEN 1*LEN -1*W 1*W]);
% for i = 1:trial
%     c = [' ',num2str(i)];
%     text(intersectFinal(i,1),intersectFinal(i,2),c);
% end
figure(4)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
hold on
plot(Location(:,1),Location(:,2),'*');
axis equal;
axis([-1*LEN 1*LEN -1*W 1*W]);
% for i = 1:trial
%     c = [' ',num2str(i)];
%     text(Location(i,1),Location(i,2),c);
% end



    






%% Section 5 

%     % Random Paramoters Generator
%     [ q, qd, qdd ,p0dd ] = GenerateRandomNumber( q_range, qd_range, qdd_range, p0dd );
%     
%     % Homogeneous Matrix Calculation, Output Rotation Matrix
%     R  = HomogeneousMatrix( L, q, n );
%     % Newton Euler Recurrence Algorithm
%     [ Fjoint1, Mjoint1 ] = DynamicRecurrence( qd, qdd, w0, w0d, p0dd, f_end, u_end, R, r, rc, m, I, n);
%     
%     % Force Reduction
%     Madd = cross([0;0;0.166],Fjoint1)+cross(rb,[0;0;-mb*g]);
%     Mbase2platform = Mjoint1 + Madd;
%     Fbase2platform = [0;0;-mb*g] + Fjoint1;
%     % Intersection Acquirement
%     [x_intersect, y_intersect, Fc, Mc] = ForceSimplify(M,H,h,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd);
% 
%     [ FcPlus, x_intersectPlus, y_intersectPlus ] = ForceSimplifyPlus( Fc, Mc, AddtionalWeight, pm ,H, h);
%     
%     [ distance, x, y ] = LocationOfWeight( x_intersectPlus,
%     y_intersectPlus, LEN, W, FcPlus, AddtionalWeight );
%     
% 
%     intersect= [x_intersect, y_intersect];
%     intersectPlus = [x_intersectPlus, y_intersectPlus];
% 
%     diameter1 = sqrt(x_intersect^2 + y_intersect^2);
%     diameter2 = sqrt(x_intersectPlus^2 + y_intersectPlus^2);
% 
%     
% 
% plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
% axis([-1*LEN 1*LEN -1*W 1*W]);
% axis([-0.7*LEN 0.7*LEN -0.7*W 0.7*W]);
% axis equal;
% hold on;
% plot(intersect(1),intersect(2),'+');
% plot(intersectPlus(1),intersectPlus(2),'*');



