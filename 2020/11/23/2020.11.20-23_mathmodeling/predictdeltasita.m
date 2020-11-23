function [sita5]=predictdeltasita(point1,point2)
%this function aim to calculate present sita for module2
% sita1是蓝方相对于x轴正向的方向角，sita2是红相对于x轴正向的方向角
P_local = [point1(1)-point2(1) point1(2)-point2(2)];
sita3=atan(abs(P_local(2)/P_local(1)));
% calculate present
% sita3是蓝方位置指向红方位置的夹角
vred=20;
vblue=25;
sita4=asin(vblue*sin(sita3)/vred); % 正弦定理计算两方连线与预测后方向的夹角
sita5=abs(pi-(sita3+sita4)); % sita6转到与x轴正向的夹角
end