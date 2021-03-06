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
H = 0.2;
M = 30;
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
%% Section 4.1 Monte-Carlo
% trial = 3e3;
% for i = 1:trial
%     % Random Paramoters Generator
%     [ q, qd, qdd ,p0dd ] = GenerateRandomNumber( q_range, qd_range, qdd_range, p0dd );
%     q(4:6) = [0;0;0];
%     qd(4:6) = [0;0;0];
%     qdd(4:6) = [0;0;0];
%     % Homogeneous Matrix Calculation, Output Rotation Matrix
%     R = HomogeneousMatrix( L, q, n );
%     % Newton Euler Recurrence Algorithm
%     [ Fjoint1, Mjoint1 ] = DynamicRecurrence( qd, qdd, w0, w0d, p0dd, f_end, u_end, R, r, rc, m, I, n);
%     % Force Reduction
%     Madd = cross([0;0;0.166],Fjoint1)+cross(rb,[0;0;-mb*g]);
%     Mbase2platform = Mjoint1 + Madd;
%     Fbase2platform = [0;0;-mb*g] + Fjoint1;
%     % Intersection Acquirement
%     [x_intersect, y_intersect] = ForceSimplify(M,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd);
%     % Save Intersection Point Info
%     intersect(i,1:23) = [x_intersect, y_intersect, q', qd', qdd', p0dd'];
%     d(i,1) = sqrt(x_intersect^2 + y_intersect^2);
%     % pause(0.1)
% end

%% Section 4.2 Read Path
load('PathPlanning_3.mat');
trial = size(q,1);
for i = 1:trial
    % Homogeneous Matrix Calculation, Output Rotation Matrix
    R = HomogeneousMatrix( L, q(i,:), n );
    % Newton Euler Recurrence Algorithm
    [ Fjoint1, Mjoint1 ] = DynamicRecurrence( qd(i,:), qdd(i,:), ...
        w0, w0d, p0dd, f_end, u_end, R, r, rc, m, I, n);
    % Force Reduction
    Madd = cross([0;0;0.166],Fjoint1)+cross(rb,[0;0;-mb*g]-mb*(p0dd-[0;0;g]));
    Mbase2platform = Mjoint1 + Madd
    Fbase2platform = [0;0;-mb*g] + Fjoint1-mb*(p0dd-[0;0;g])
    % Intersection Acquirement
    [x_intersect, y_intersect,Fc,Mc,dx] = ForceSimplify(M,H,h,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd);
    % Save Intersection Point Info
    intersect(i,1:23) = [x_intersect, y_intersect, q(i,:), qd(i,:), qdd(i,:), p0dd'];
    d(i,1) = sqrt(x_intersect^2 + y_intersect^2);
%     pause
end

% Save Data
printData(intersect, n_experiment);

%% Section 5 Plot Figure
% % Probability Distribution
% figure(1)
% [f, Xi] = probabilityDistribution(d);
% plot(Xi,f)
% hold on
% threshold = confidentRadius(0.05, f, Xi);
% plot([threshold,threshold],[0,10],'r');
% title('Probability Distribution');
% % Plot Interdection Point, Save Fig
figure(2);
plot(intersect(2:trial,1),intersect(2:trial,2),'+');
hold on
for i = 1:trial
    c = [' ',num2str(i)];
    text(intersect(i,1),intersect(i,2),c);
end
title('轨迹3等效交点分布图')
xlabel('单位：m')
ylabel('单位：m')
% rectangle('Position',[-threshold,-threshold,2*threshold,2*threshold],...
%     'Curvature',[1,1],'EdgeColor','r')
% axisRange = 1.3*threshold;
% axis([-axisRange axisRange -axisRange axisRange])
axis equal
grid on
% title('Intersection Point');
% saveas(2,['experiment' num2str(n_experiment)],'fig');

% Finish
toc