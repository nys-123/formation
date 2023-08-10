clc;clear;close all;
n = 12550;
%% 期望值依靠构型计算出
gg21=[0,-1];
gg23=[-1,-1];
gg24=[1,-1];

gg31=[1,0];
gg32=[1,1];
gg35=[1,-1];

gg41=[-1,0];
gg42=[-1,1];
gg45=[-1,-1];

gg51=[0,1];
gg53=[-1,1];
gg54=[1,1];
%将计算期望单位化
gg21=gg21/norm(gg21);
gg23=gg23/norm(gg23);
gg24=gg24/norm(gg24);

gg31=gg31/norm(gg31);
gg32=gg32/norm(gg32);
gg35=gg35/norm(gg35);

gg41=gg41/norm(gg41);
gg42=gg42/norm(gg42);
gg45=gg45/norm(gg45);

gg51=gg51/norm(gg51);
gg53=gg53/norm(gg53);
gg54=gg54/norm(gg54);

%% 目标点
p1 =[0,0];
%% 2号
p2(1,:)=[2,3];
%% 3号
p3(1,:)=[-1,-3];
%% 4号
p4(1,:)=[5,-2];
%% 5号
p5(1,:)=[-3,1];
%% 参数
w = 0.1;%角速度
d = 5;%距离目标点之间的距离
for i=1:n
dt=0.01;   %仿真步长
t= 0.01*(i-1);%模拟时间
a = w*t;%角加速度
R = [cos(a),-sin(a);  %旋转矩阵
     sin(a),cos(a) ];
% R = 1;   %不旋转 
I = [1,0;
     0,1;];%单位矩阵
%% 智能体之间的方位信息

g23 = (p3- p2)/sqrt( (p2(i,1)-p3(i,1))^2 + (p2(i,2)-p3(i,2))^2 );
g24 = (p4 - p2)/sqrt( (p2(i,1)-p4(i,1))^2 + (p2(i,2)-p4(i,2))^2 );

g32 = (p2- p3)/sqrt( (p3(i,1)-p2(i,1))^2 + (p3(i,2)-p2(i,2))^2 );
g35 = (p5 - p3)/sqrt( (p3(i,1)-p5(i,1))^2 + (p3(i,2)-p5(i,2))^2 );

g42 = (p2 - p4)/sqrt( (p4(i,1)-p2(i,1))^2 + (p4(i,2)-p2(i,2))^2 );
g45 = (p5 - p4)/sqrt( (p4(i,1)-p5(i,1))^2 + (p4(i,2)-p5(i,2))^2 );


g53 = (p3 - p5)/sqrt( (p5(i,1)-p3(i,1))^2 + (p5(i,2)-p3(i,2))^2 );
g54 = (p4 - p5)/sqrt( (p5(i,1)-p4(i,1))^2 + (p5(i,2)-p4(i,2))^2 );


%% 智能体到目标点的方位
g21 = (p1 - p2)/sqrt( (p2(i,1)-p1(i,1))^2 + (p2(i,2)-p1(i,2))^2 );
g31 = (p1 - p3)/sqrt( (p3(i,1)-p1(i,1))^2 + (p3(i,2)-p1(i,2))^2 );
g41 = (p1 - p4)/sqrt( (p4(i,1)-p1(i,1))^2 + (p4(i,2)-p1(i,2))^2 );
g51 = (p1 - p5)/sqrt( (p5(i,1)-p1(i,1))^2 + (p5(i,2)-p1(i,2))^2 );
%% 智能体到中心点的距离
dis21 = sqrt( (p2(i,1)-p1(i,1))^2 + (p2(i,2)-p1(i,2))^2  );
dis31 = sqrt( (p3(i,1)-p1(i,1))^2 + (p3(i,2)-p1(i,2))^2  );
dis41 = sqrt( (p4(i,1)-p1(i,1))^2 + (p4(i,2)-p1(i,2))^2  );
dis51 = sqrt( (p5(i,1)-p1(i,1))^2 + (p5(i,2)-p1(i,2))^2  );

dis53 = sqrt( (p5(i,1)-p3(i,1))^2 + (p5(i,2)-p3(i,2))^2  );
dis23 = sqrt( (p2(i,1)-p3(i,1))^2 + (p2(i,2)-p3(i,2))^2  );
dis34 = sqrt( (p3(i,1)-p4(i,1))^2 + (p3(i,2)-p4(i,2))^2  );
dis42 = sqrt( (p4(i,1)-p2(i,1))^2 + (p4(i,2)-p2(i,2))^2  );
%% 计算投影矩阵 注意一定要是.乘的形式 不然会报错
Pg21 = I - (g21(i,:).*g21(i,:)');
Pg23 = I - (g23(i,:).*g23(i,:)');
Pg24 = I - (g24(i,:).*g24(i,:)');

Pg31 = I - (g31(i,:).*g31(i,:)');
Pg32 = I - (g32(i,:).*g32(i,:)');
Pg35 = I - (g35(i,:).*g35(i,:)');

Pg41 = I - (g41(i,:).*g41(i,:)');
Pg42 = I - (g42(i,:).*g42(i,:)');
Pg45 = I - (g45(i,:).*g45(i,:)');

Pg51 = I - (g51(i,:).*g51(i,:)');
Pg53 = I - (g53(i,:).*g53(i,:)');
Pg54 = I - (g54(i,:).*g54(i,:)');


p1(i+1,1)=p1(i,1);
p1(i+1,2)=p1(i,2);


%% 计算控制量
%参数x1 x2 x3 *R

x1=-5;
x2=-1;
x3=1;

u2 =(x1*( Pg23*R*(gg23)' + Pg24*R*(gg24)'))'+x2*(Pg21*R*(gg21)')';

p2(i+1,1)=p2(i,1)+ u2(1,1)*dt + x3*(dis21 - d)*g21(i,1)*dt ;
p2(i+1,2)=p2(i,2)+ u2(1,2)*dt + x3*(dis21 - d)*g21(i,2)*dt  ;



u3 = (x1*( Pg32*R*(gg32)' + Pg35*R*(gg35)'))'+x2*(Pg31*R*(gg31)')';

p3(i+1,1)=p3(i,1)+ u3(1,1)*dt + x3*(dis31- d)*g31(i,1)*dt;
p3(i+1,2)=p3(i,2)+ u3(1,2)*dt + x3*(dis31- d)*g31(i,2)*dt;

 
u4 =(x1*( Pg42*R*(gg42)' + Pg45*R*(gg45)'))'+x2*(Pg41*R*(gg41)')';

p4(i+1,1)=p4(i,1)+ u4(1,1)*dt + x3*(dis41- d)*g41(i,1)*dt;
p4(i+1,2)=p4(i,2)+ u4(1,2)*dt + x3*(dis41- d)*g41(i,1)*dt;



u5 =(x1*( Pg53*R*(gg53)' + Pg54*R*(gg54)'))'+x2*(Pg51*R*(gg51)')';
p5(i+1,1)=p5(i,1)+ u5(1,1)*dt + x3*(dis51- d)*g51(i,1)*dt;
p5(i+1,2)=p5(i,2)+ u5(1,2)*dt + x3*(dis51- d)*g51(i,2)*dt;

    
%% 画图
if((mod(i,1)==0&&(i<20))||(mod(i,5)==0&&(i>20)&&i<100)||(mod(i,10)==0&&(i>100)&&i<200)||(mod(i,50)==0&&(i>200)))


     plot(p2(:,1),p2(:,2),"b-");
     
     hold on;
     plot(p3(:,1),p3(:,2),"m-");
     plot(p4(:,1),p4(:,2),"g-");
     plot(p5(:,1),p5(:,2),"-k");


plot(p1(i,1),p1(i,2),'r.','markersize',30);
plot(p2(i,1),p2(i,2),'c.','markersize',30);
hold on;
plot(p3(i,1),p3(i,2),'m.','markersize',30);
plot(p4(i,1),p4(i,2),'g.','markersize',30);
plot(p5(i,1),p5(i,2),'k.','markersize',30);

plot([p1(i,1),p2(i,1)],[p1(i,2),p2(i,2)],'r-','linewidth',1.5);
plot([p1(i,1),p3(i,1)],[p1(i,2),p3(i,2)],'r-','linewidth',1.5);
plot([p1(i,1),p4(i,1)],[p1(i,2),p4(i,2)],'r-','linewidth',1.5);
plot([p1(i,1),p5(i,1)],[p1(i,2),p5(i,2)],'r-','linewidth',1.5);
plot([p3(i,1),p2(i,1)],[p3(i,2),p2(i,2)],'b-','linewidth',1.5);
plot([p4(i,1),p5(i,1)],[p4(i,2),p5(i,2)],'b-','linewidth',1.5);
plot([p4(i,1),p2(i,1)],[p4(i,2),p2(i,2)],'b-','linewidth',1.5);
plot([p5(i,1),p4(i,1)],[p5(i,2),p4(i,2)],'b-','linewidth',1.5);
plot([p5(i,1),p3(i,1)],[p5(i,2),p3(i,2)],'b-','linewidth',1.5);

grid on;

text(p1(i,1)+0.1,p1(i,2),['目标']);
text(p2(i,1)+0.1,p2(i,2),['Agent 2']);
text(p3(i,1)+0.1,p3(i,2),['Agent 3']);
text(p4(i,1)+0.1,p4(i,2),['Agent 4']);
text(p5(i,1)+0.1,p5(i,2),['Agent 5']);

xlim([-10,10]);
ylim([-10,10]);


hold off;
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);

hold off;   
end
    
    
end    