clear
clc
g=9.8;
Z(1)=Link('d',0.124,'a',0,'alpha',pi/2);
Z(2)=Link('theta',0,'a',0,'alpha',0,'prismatic');
BalanceRobot=SerialLink(Z,'name','BalanceRobot');
q_balance=[0, 0];
W=[-2,2,-2,2,-2,2];
BalanceRobot.plot(q_balance,'workspace',W);

qd_balance=[0, 0];
qdd_balance=[1, 0];

LengthOfBalanceLink=0.2;
w0 = [0; 0; 0];
w0d = [0; 0; 0];
p0dd = [0; 0; g];
f_end=[0;0;0];
u_end=[0;0;0];
R_balance = HomogeneousMatrix( Z, q_balance, 2 );
r_balance=[0, 0 ; 0.124, 0 ; 0, q_balance(2) ];
rc_balance=[0, 0 ; 0, 0 ; LengthOfBalanceLink/2, 0];
m_balance=[10,10];
I1=[0, 0, 0; 0, 1/2*m_balance(1)*LengthOfBalanceLink^2, 0; 0, 0, 0];
I2=[0, 0, 0; 0, 1/6*m_balance(2)*0.05^2, 0; 0, 0, 0];
I_balance(1:3,1:3,1)=I1;
I_balance(1:3,1:3,2)=I2;

[ F_balance, M_balance ] = DynamicForBalance( qd_balance, qdd_balance, ...
        w0, w0d, p0dd, f_end, u_end, R_balance, r_balance, rc_balance, m_balance, I_balance, 2)
