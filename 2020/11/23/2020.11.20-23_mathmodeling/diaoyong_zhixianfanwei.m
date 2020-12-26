close all;
clear all;
clc

fig = figure;
frame = getframe(fig); % 获取frame

set(gcf,'outerposition',get(0,'screensize'));
axis equal;

FlyBox = [0 0;
    0 7000;
    10000 7000;
    10000 0;
    0 0];
pause(1)
% plot(FlyBox(1:end-1,1),FlyBox(1:end-1,2),'gp', 'MarkerSize',20,'MarkerEdgeColor','b','MarkerFaceColor',[0 1 0]);%方形点填充颜色
plot(FlyBox(1:end-1,1),FlyBox(1:end-1,2),'gp', 'MarkerSize',20,'MarkerEdgeColor','b','MarkerFaceColor',[0 1 0]);%方形点填充颜色
hold on;
plot(FlyBox(:,1),FlyBox(:,2),'b');
% plot(FlyBox(:,1),FlyBox(:,2),'b', 'MarkerSize',15);
hold on;

for i=0:7000
	r=zhixiang_xiexian(i);
	if r==1
		line([0 10000],[3500 i]);
		hold on;
	end
end

