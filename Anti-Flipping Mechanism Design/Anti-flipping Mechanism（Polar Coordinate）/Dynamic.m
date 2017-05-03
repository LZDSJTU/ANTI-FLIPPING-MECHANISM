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
AddtionalWeight=0;
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
load('C:\Users\hasee\Desktop\制定轨迹（极坐标调整法）\轨迹\PathPlanning_3_1.mat');
% % 规定操作空间范围，并作图
trial = size(q,1);
xsum(1)=0;
ysum(1)=0;

% q_balance(:,1)=q(:,1)-pi/2;
% q_balance(:,2)=zeros(trial,1);
% qd_balance(:,1)=qd(:,1);
% qd_balance(:,2)=zeros(trial,1);
% qdd_balance(:,1)=qdd(:,1);
% qdd_balance(:,2)=zeros(trial,1);

q_balance(:,1)=q(:,1)-pi/2;
q_balance(:,2)=0.2-0.2/(max(q(:,2))-min(q(:,2)))*(q(:,2)-min(q(:,2)));
% q_balance(:,2)=mean(q_balance(:,2))-0.25/(max(q(:,2))-min(q(:,2)))*(q(:,2)-min(q(:,2)))-0.25/(max(q(:,3))-min(q(:,3)))*(q(:,3)-min(q(:,3)));
qd_balance(:,1)=qd(:,1);
qd_balance(:,2)=-0.2/(max(q(:,2))-min(q(:,2)))*qd(:,2);
% qd_balance(:,2)=-0.5/(max(q(:,2))-min(q(:,2)))*qd(:,2);
qdd_balance(:,1)=qdd(:,1);
qdd_balance(:,2)=-0.2/(max(q(:,2))-min(q(:,2)))*qdd(:,2);
% qdd_balance(:,2)=-0.5/(max(q(:,2))-min(q(:,2)))*qdd(:,2);


Z(1)=Link('d',-0.3,'a',0,'alpha',pi/2);
Z(2)=Link('theta',0,'a',0,'alpha',0,'prismatic');
BalanceRobot=SerialLink(Z,'name','BalanceRobot');
LengthOfBalanceLink=0.2;
% 不确定项，代入循环R_balance = HomogeneousMatrix( Z, q_balance, 2 );
% 不确定项，代入循环r_balance=[0, 0 ; 0.124, 0 ; 0, q_balance(2) ];
rc_balance=[0, 0 ; 0, 0 ; LengthOfBalanceLink/2, 0];
m_balance=[5,5];
I1=[0, 0, 0; 0, 1/2*m_balance(1)*LengthOfBalanceLink^2, 0; 0, 0, 0];
I2=[0, 0, 0; 0, 1/6*m_balance(2)*0.05^2, 0; 0, 0, 0];
I_balance(1:3,1:3,1)=I1;
I_balance(1:3,1:3,2)=I2;


for i = 1:trial
    % Homogeneous Matrix Calculation, Output Rotation Matrix
    R = HomogeneousMatrix( L, q(i,:), n );

% ---------------------------------------------------------------- 
    R_balance = HomogeneousMatrix( Z, q_balance(i,:), 2 );
    r_balance=[0, 0 ; 0.124, 0 ; 0, q_balance(i,2) ];
    [ F_balance, M_balance ] = DynamicForBalance( qd_balance(i,:), qdd_balance(i,:), ...
        w0, w0d, p0dd, f_end, u_end, R_balance, r_balance, rc_balance, m_balance, I_balance, 2);
% ----------------------------------------------------------------

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

    Fjoint=Fjoint1+F_balance;
    Mjoint=Mjoint1+M_balance;
    % Force Reduction
    Madd = cross([0;0;0.166],Fjoint)+cross(rb,[0;0;-mb*g]-mb*(p0dd-[0;0;g]));
    Mbase2platform = Mjoint + Madd;
    Fbase2platform = [0;0;-mb*g] + Fjoint-mb*(p0dd-[0;0;g]);
    % Intersection Acquirement
    [x_intersectPlus, y_intersectPlus,Fc,Mc,dx] = ForceSimplify(M,H,h,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd);
    intersectPlus(i,1:2) = [x_intersectPlus, y_intersectPlus];
end

figure(1)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
axis([-1*LEN 1*LEN -1*W 1*W]);
hold on;
plot(intersect(:,1),intersect(:,2),'*');
for i = 1:trial

    c = [' ',num2str(i)];
    text(intersect(i,1),intersect(i,2),c);

end
figure(2)
plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
axis([-1*LEN 1*LEN -1*W 1*W]);
hold on;
plot(intersectPlus(:,1),intersectPlus(:,2),'*');
for i = 1:trial

    c = [' ',num2str(i)];
    text(intersectPlus(i,1),intersectPlus(i,2),c);

end
figure(3)
s=[-1.2 0.8 -0.8 0.8 -0.3 0.8];
IRB120.plot(q,'workspace',s,'fps',100);

figure(4)
s=[-1.2 0.8 -0.8 0.8 -0.3 0.8];
BalanceRobot.plot(q_balance,'workspace',s,'fps',100);
max(intersect(:,1))
min(intersect(:,1))
max(intersectPlus(:,1))
min(intersectPlus(:,1))


%% Section 5 Output the data for ADAMS 
t=linspace(0,1,length(q)+1);
t=t';
q=[zeros(1,6);q];
q1=[t,q(:,1)];
q2=[t,q(:,2)-pi/2];
q3=[t,q(:,3)];
q4=[t,q(:,4)];
q5=[t,q(:,5)];
q6=[t,q(:,6)];

save('displacement3_1_1.txt','q1','-ascii');
save('displacement3_1_2.txt','q2','-ascii');
save('displacement3_1_3.txt','q3','-ascii');
save('displacement3_1_4.txt','q4','-ascii');
save('displacement3_1_5.txt','q5','-ascii');
save('displacement3_1_6.txt','q6','-ascii');

q_balance=[zeros(1,2);q_balance];
q1_balance=[t,q_balance(:,1)];
q2_balance=[t,q_balance(:,2)];
save('displacement3_1_1_balance.txt','q1_balance','-ascii');
save('displacement3_1_2_balance.txt','q2_balance','-ascii');