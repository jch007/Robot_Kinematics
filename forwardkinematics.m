function [x,y,z,R,P,Y]=forwardkinematics(theta1,theta2,theta3,theta4,theta5,theta6)
%D-H参数
% a0=0;                 % [m]杆长
% a1=0;                 % [m]
% a2=0;                 % [m]
% a3=0;                 % [m]
% a4=0;                 % [m]
% a5=0;                 % [m]
 
a0=0;                 % [m]杆长
a1=0;                 % [m]
a2=0;                 % [m]
a3=0;                 % [m]
a4=0;                 % [m]
a5=0;                 % [m]

alpha0=0;            % [rad]连杆转角
alpha1=-pi/2;        % [rad]
alpha2=0;            % [rad]
alpha3=-pi/2;        % [rad]
alpha4=pi/2;         % [rad]
alpha5=-pi/2;        % [rad]

% [d1,d2,d3,d5,d6]=deal(0);    % [rad]连杆偏距
% d4=0.624;
[d1,d2,d3,d5]=deal(0);    % [rad]连杆偏距
d4 = 0;
d6 = 0;
% 连杆变换矩阵

% 连杆变换矩阵
T01=[cos(theta1),-sin(theta1),0,a0;
    sin(theta1)*cos(alpha0),cos(theta1)*cos(alpha0),-sin(alpha0),-sin(alpha0)*d1;
    sin(theta1)*sin(alpha0),cos(theta1)*sin(alpha0),cos(alpha0),cos(alpha0)*d1;
    0,0,0,1];

T12=[cos(theta2),-sin(theta2),0,a1;
    sin(theta2)*cos(alpha1),cos(theta2)*cos(alpha1),-sin(alpha1),-sin(alpha1)*d2;
    sin(theta2)*sin(alpha1),cos(theta2)*sin(alpha1),cos(alpha1),cos(alpha1)*d2;
    0,0,0,1]

T23=[cos(theta3),-sin(theta3),0,a2;
    sin(theta3)*cos(alpha2),cos(theta3)*cos(alpha2),-sin(alpha2),-sin(alpha2)*d3;
    sin(theta3)*sin(alpha2),cos(theta3)*sin(alpha2),cos(alpha2),cos(alpha2)*d3;
    0,0,0,1];

T34=[cos(theta4),-sin(theta4),0,a3;
    sin(theta4)*cos(alpha3),cos(theta4)*cos(alpha3),-sin(alpha3),-sin(alpha3)*d4;
    sin(theta4)*sin(alpha3),cos(theta4)*sin(alpha3),cos(alpha3),cos(alpha3)*d4;
    0,0,0,1];

T45=[cos(theta5),-sin(theta5),0,a4;
    sin(theta5)*cos(alpha4),cos(theta5)*cos(alpha4),-sin(alpha4),-sin(alpha4)*d5;
    sin(theta5)*sin(alpha4),cos(theta5)*sin(alpha4),cos(alpha4),cos(alpha4)*d5;
    0,0,0,1];

T56=[cos(theta6),-sin(theta6),0,a5;
    sin(theta6)*cos(alpha5),cos(theta6)*cos(alpha5),-sin(alpha5),-sin(alpha5)*d6;
    sin(theta6)*sin(alpha5),cos(theta6)*sin(alpha5),cos(alpha5),cos(alpha5)*d6;
    0,0,0,1];

% 连杆变换矩阵

T=T01*T12*T23*T34*T45*T56%机器人末端坐标系相对于机器人基坐标系的位置和姿态

%机器人末端姿态
rotm=T(1:3,1:3);
%机器人末端位置
x=T(1,4);
y=T(2,4);
z=T(3,4);

%齐次变换矩阵转化为欧拉角表示
R=atan2(rotm(3,2),rotm(3,3));
P=atan2(-rotm(3,1),sqrt(rotm(3,2)*rotm(3,2)+rotm(3,3)*rotm(3,3)));
Y=atan2(rotm(2,1),rotm(1,1));


