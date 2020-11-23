function [sita4]=predictdeltasita_2(point1,point2,point3)
%this function aim to calculate present sita for module2
% sita1是蓝方相对于x轴正向的方向角，sita2是红相对于x轴正向的方向角
P_local1 = [point1(1)-point2(1) point1(2)-point2(2)];
dist = sqrt(P_local1(1)^2+P_local1(2)^2);
P_local2 = [point1(1)-point3(1) point1(2)-point3(2)];
sita1 = atan(P_local2(2)/P_local2(1));
% calculate present
vred=20;
vblue=25;
factor=30; %影响因子
max=sqrt(10000^2+2000^2);
parameter=factor*exp(-dist/max);
predict_location=[point1(1)+vblue*parameter*cos(sita1) point1(2)+vblue*parameter*sin(sita1)];
[azimuth, distt] = GetAzimuth_2points(predict_location, point2);
sita4=azimuth;
end