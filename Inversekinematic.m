

function Theta=ik(pos)
%D-H����
a0=0;                 % [m]�˳�
a1=184;             % [m]
a2=600;             % [m]
a3=114;             % [m]
a4=0;                 % [m]
a5=0;                 % [m]
% [d1,d2,d3,d5,d6]=deal(0);    % [rad]����ƫ��
d4=664;
%ŷ����ת��Ϊ��ת����
x=pos(1);y=pos(2);z=pos(3);theta_x=(pos(4)*pi)/180;theta_y=(pos(5)*pi)/180;theta_z=(pos(6)*pi)/180;
rotm=[cos(theta_y)*cos(theta_z),cos(theta_z)*sin(theta_x)*sin(theta_y)-cos(theta_x)*sin(theta_z),sin(theta_x)*sin(theta_z)+cos(theta_x)*cos(theta_z)*sin(theta_y);
      cos(theta_y)*sin(theta_z),cos(theta_x)*cos(theta_z)+sin(theta_x)*sin(theta_y)*sin(theta_z),cos(theta_x)*sin(theta_y)*sin(theta_z)-cos(theta_z)*sin(theta_x);
      -sin(theta_y),cos(theta_y)*sin(theta_x),cos(theta_x)*cos(theta_y)];
r11=rotm(1,1);
r12=rotm(1,2);
r13=rotm(1,3);
xc=x;
r21=rotm(2,1);
r22=rotm(2,2);
r23=rotm(2,3);
yc=y;
r31=rotm(3,1);
r32=rotm(3,2);
r33=rotm(3,3);
zc=z;
%6������˵��˶�ѧ�����8�飬����Ľ�Ϊ8��6�еľ���
Theta=zeros(8,6);

for i=1:2    
    %����theta1,����
    Theta((i-1)*4+1,1) = atan2(yc,xc);
    Theta((i-1)*4+2,1) = atan2(yc,xc);
    Theta((i-1)*4+3,1) = pi+atan2(yc,xc);
    Theta((i-1)*4+4,1) = pi+atan2(yc,xc);
    %�Ƕ�����
    Theta((i-1)*4+1,1)=AngleCorrection(Theta((i-1)*4+1,1));
    Theta((i-1)*4+2,1)=AngleCorrection(Theta((i-1)*4+2,1));
    Theta((i-1)*4+3,1)=AngleCorrection(Theta((i-1)*4+3,1));
    Theta((i-1)*4+4,1)=AngleCorrection(Theta((i-1)*4+4,1));
end

for i=1:2
    %����theta3����4����
    c1 = cos(Theta((i-1)*2+1,1));
    s1 = sin(Theta((i-1)*2+1,1));
    k(i) =  xc*c1+yc*s1-a1 ;
    w(i) = (k(i)*k(i)+zc^2 -a2^2-a3^2-d4^2)/(2*a2);
    if(a3^2+d4^2-w(i)*w(i)<0)
        %��ʱtheta3���н��������
        for n=1:4
            Theta(n,3)=0;
            Theta(n+4,3)=0;
        end
    else
        Theta((i-1)*2+1,3)=atan2(a3,d4)-atan2(w(i),sqrt(a3^2+d4^2-w(i)*w(i)));
        Theta((i-1)*2+2,3)=atan2(a3,d4)-atan2(w(i),-sqrt(a3^2+d4^2-w(i)*w(i)));
        Theta((2*i+3),3)=atan2(a3,d4)-atan2(w(i),sqrt(a3^2+d4^2-w(i)*w(i)));
        Theta((2*i+4),3)=atan2(a3,d4)-atan2(w(i),-sqrt(a3^2+d4^2-w(i)*w(i)));
        %�Ƕ�����
        Theta((i-1)*2+1,3)=AngleCorrection(Theta((i-1)*2+1,3));
        Theta((i-1)*2+2,3)=AngleCorrection(Theta((i-1)*2+2,3));
        Theta((2*i+3),3)=AngleCorrection(Theta((2*i+3),3));
        Theta((2*i+4),3)=AngleCorrection(Theta((2*i+4),3)); 
    end
end

    %����theta2,��4����(������)     
    for j=1:4
        c1 = cos(Theta(j,1));
        s1 = sin(Theta(j,1));
        c3 = cos(Theta(j,3));
        s3 = sin(Theta(j,3));
        Theta(j,2) = atan2((-a3-a2*c3)*zc-(d4-a2*s3)*(c1*xc+s1*yc-a1),(a2*s3-d4)*zc+(a3+a2*c3)*(c1*xc+s1*yc-a1))-Theta(j,3);
        Theta(j+4,2) = Theta(j,2);
         %�Ƕ�����
        Theta(j,2)=AngleCorrection(Theta(j,2));
        Theta(j+4,2)=AngleCorrection(Theta(j+4,2));
    end
    
    %����theta4��5,��8����
    for k=1:4
        c1 = cos(Theta(k,1));
        s1 = sin(Theta(k,1));
        c23= cos(Theta(k,2))*cos(Theta(k,3))-sin(Theta(k,2))*sin(Theta(k,3));
        s23= cos(Theta(k,2))*sin(Theta(k,3))+sin(Theta(k,2))*cos(Theta(k,3));
        h(k) = -r33 * c23 - ( c1 * r13 + s1*r23 ) * s23;
        Theta(k,5) = atan2( sqrt(1-h(k)*h(k)),h(k));
        Theta(k+4,5) = -Theta(k,5);
        if( sin(Theta(k,5))~=0)
            Theta(k,4) = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
            Theta(k+4,4) = Theta(k,4)+pi;
             %�Ƕ�����
            Theta(k,4)=AngleCorrection(Theta(k,4));
            Theta(k+4,4)=AngleCorrection(Theta(k+4,4));
        else
            Theta(k,4)=0;
            Theta(k+4,4)=0;
        end
    end
    
    for m=1:4
        %����theta6����8����
        c1 = cos(Theta(m,1));
        s1 = sin(Theta(m,1));
        c23= cos(Theta(m,2))*cos(Theta(m,3))-sin(Theta(m,2))*sin(Theta(m,3));
        s23= cos(Theta(m,2))*sin(Theta(m,3))+sin(Theta(m,2))*cos(Theta(m,3));
        c4 = cos(Theta(m,4)) ;
        s4 = sin(Theta(m,4)) ;
        Theta(m,6) = atan2(r11*(-s4*c1*c23+s1*c4)+r21*(-s1*s4*c23-c1*c4)+r31*s4*s23,r12*(-c1*c23*s4+s1*c4)+r22*(-s1*s4*c23-c1*c4)+r32*s4*s23);
        Theta(m+4,6)=Theta(m,6)+pi;
        %�Ƕ�����
        Theta(m,6)=AngleCorrection(Theta(m,6));
        Theta(m+4,6)=AngleCorrection(Theta(m+4,6));
    end









