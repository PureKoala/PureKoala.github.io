% Steps:
% 1 假设条件，设计各自的运动策略；
% 2 规划函数和子函数，做好分工；
% 3 红蓝军主体的运动策略；
% 4 红军的子飞行器的运动策略；
% 5 统计计算仿真的相关参数
% 6 仿真开始
% 7 条件判断---子函数
% 8 迭代重新执行
% unmanned aerial vehicle
% Author: ruogu7(380545156)
% date: sep, 17th.
% Latest update: sep, 17th.

close all;
clear all;
clc

%% 参数
% 仿真步长
% 模型一：蓝色无人机初始位置，AB中间，一直往右飞行；红色无人机集群始终朝着蓝色无人机飞行。
% 步长为0.1s
ver=3500;
time=7000;
step_length = 3;
speed_blue_UAV = 25;
speed_red_UAV = 20;
Besiege_distance = 100;
Flag_Track=1;
% UAV集群的半径
r_red_uav = 80;

% 先画框架
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

% 蓝色UAV的初始位置
Blue_UAV_Location = [0 ver]; % x水平,y竖直
% 红色UAV的初始位置，
Red_1_UAV_Location = [10000 5000]; % 中心（舰载机1）FY01
Red_1_1_UAV_Location = [10000+r_red_uav*cos(18*pi/180) 5000+ r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
Red_1_2_UAV_Location = [10000+r_red_uav*cos(90*pi/180) 5000+ r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
Red_1_3_UAV_Location = [10000+r_red_uav*cos(162*pi/180) 5000+ r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
Red_1_4_UAV_Location = [10000+r_red_uav*cos(234*pi/180) 5000+ r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
Red_1_5_UAV_Location = [10000+r_red_uav*cos(306*pi/180) 5000+ r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
Red_2_UAV_Location = [10000 2000]; % 中心（舰载机2）FY02
Red_2_1_UAV_Location = [10000+r_red_uav*cos(18*pi/180) 2000+ r_red_uav*sin(18*pi/180)];  % 无人机FY0201的位置
Red_2_2_UAV_Location = [10000+r_red_uav*cos(90*pi/180) 2000+ r_red_uav*sin(90*pi/180)];  % 无人机FY0202的位置
Red_2_3_UAV_Location = [10000+r_red_uav*cos(162*pi/180) 2000+ r_red_uav*sin(162*pi/180)]; % 无人机FY0203的位置
Red_2_4_UAV_Location = [10000+r_red_uav*cos(234*pi/180) 2000+ r_red_uav*sin(234*pi/180)];  % 无人机FY0204的位置
Red_2_5_UAV_Location = [10000+r_red_uav*cos(306*pi/180) 2000+ r_red_uav*sin(306*pi/180)];  % 无人机FY0205的位置

Figure_num = 0;

while Figure_num < time && Flag_Track && Blue_UAV_Location(end,1)<1.02*10000
    % 当前无人机的位置
    Blue_UAV_Location_temp = [Blue_UAV_Location(end,1) Blue_UAV_Location(end,2)];
    Red_1_UAV_Location_temp = [Red_1_UAV_Location(end,1) Red_1_UAV_Location(end,2)];
    Red_2_UAV_Location_temp = [Red_2_UAV_Location(end,1) Red_2_UAV_Location(end,2)];
    
    % 计算红色无人机的飞行方向
    [azimuth_Red_1, dist_Red_1 ] = GetAzimuth_2points(Blue_UAV_Location_temp, Red_1_UAV_Location_temp);
    [azimuth_Red_2, dist_Red_2 ] = GetAzimuth_2points(Blue_UAV_Location_temp, Red_2_UAV_Location_temp);
    
    % 更新蓝色无人机的位置
    Blue_UAV_Location_new_x = Blue_UAV_Location_temp(1) + step_length * speed_blue_UAV * cos(0);
    Blue_UAV_Location_new_y = Blue_UAV_Location_temp(2) + step_length * speed_blue_UAV * sin(0);
    Blue_UAV_Location_new = [Blue_UAV_Location_new_x Blue_UAV_Location_new_y];
    Blue_UAV_Location = [Blue_UAV_Location;Blue_UAV_Location_new];
    
    % 计算红色无人机群1的位置
    Red_1_UAV_Location_new_x = Red_1_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_1-90))*pi/180);
    Red_1_UAV_Location_new_y = Red_1_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_1-90))*pi/180);
    Red_1_UAV_Location_new = [Red_1_UAV_Location_new_x Red_1_UAV_Location_new_y];
    
    Red_1_1_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(18*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
    Red_1_2_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(90*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
    Red_1_3_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(162*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
    Red_1_4_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(234*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
    Red_1_5_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(306*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
    
    % 更新红色无人机群1的位置
    Red_1_UAV_Location = [Red_1_UAV_Location;Red_1_UAV_Location_new];
    Red_1_1_UAV_Location = [Red_1_1_UAV_Location;Red_1_1_UAV_Location_new];
    Red_1_2_UAV_Location = [Red_1_2_UAV_Location;Red_1_2_UAV_Location_new];
    Red_1_3_UAV_Location = [Red_1_3_UAV_Location;Red_1_3_UAV_Location_new];
    Red_1_4_UAV_Location = [Red_1_4_UAV_Location;Red_1_4_UAV_Location_new];
    Red_1_5_UAV_Location = [Red_1_5_UAV_Location;Red_1_5_UAV_Location_new];
    
    % 计算红色无人机群2的位置
    Red_2_UAV_Location_new_x = Red_2_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_2-90))*pi/180);
    Red_2_UAV_Location_new_y = Red_2_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_2-90))*pi/180);
    Red_2_UAV_Location_new = [Red_2_UAV_Location_new_x Red_2_UAV_Location_new_y];
    
    Red_2_1_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(18*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(18*pi/180)];  % 无人机FY0201的位置
    Red_2_2_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(90*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(90*pi/180)];  % 无人机FY0202的位置
    Red_2_3_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(162*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(162*pi/180)]; % 无人机FY0203的位置
    Red_2_4_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(234*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(234*pi/180)];  % 无人机FY0204的位置
    Red_2_5_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(306*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(306*pi/180)];  % 无人机FY0205的位置
    
    % 更新红色无人机群2的位置
    Red_2_UAV_Location = [Red_2_UAV_Location;Red_2_UAV_Location_new];
    
    Red_2_1_UAV_Location = [Red_2_1_UAV_Location;Red_2_1_UAV_Location_new];  % 无人机FY0201的位置
    Red_2_2_UAV_Location = [Red_2_2_UAV_Location;Red_2_2_UAV_Location_new];  % 无人机FY0202的位置
    Red_2_3_UAV_Location = [Red_2_3_UAV_Location;Red_2_3_UAV_Location_new];  % 无人机FY0203的位置
    Red_2_4_UAV_Location = [Red_2_4_UAV_Location;Red_2_4_UAV_Location_new];  % 无人机FY0204的位置
    Red_2_5_UAV_Location = [Red_2_5_UAV_Location;Red_2_5_UAV_Location_new];  % 无人机FY0205的位置
    
    %% 路径可视化
    % 蓝色无人机
	hold on;
    plot(Blue_UAV_Location(:,1),Blue_UAV_Location(:,2),'g','LineWidth',5);
    hold on;
    
    % 红色UAV集群1,画线
    plot(Red_1_UAV_Location(:,1),Red_1_UAV_Location(:,2),'r');
    hold on;
    
    % 红色UAV集群1的UAV，画点
    plot(Red_1_1_UAV_Location(end,1),Red_1_1_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_1_2_UAV_Location(end,1),Red_1_2_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_1_3_UAV_Location(end,1),Red_1_3_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_1_4_UAV_Location(end,1),Red_1_4_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_1_5_UAV_Location(end,1),Red_1_5_UAV_Location(end,2),'pr');
    hold on;
    
    % 红色UAV集群2,画线
    plot(Red_2_UAV_Location(:,1),Red_2_UAV_Location(:,2),'r');
    hold on;
    
    % 红色UAV集群2的UAV，画点
    plot(Red_2_1_UAV_Location(end,1),Red_2_1_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_2_2_UAV_Location(end,1),Red_2_2_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_2_3_UAV_Location(end,1),Red_2_3_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_2_4_UAV_Location(end,1),Red_2_4_UAV_Location(end,2),'pr');
    hold on;
    plot(Red_2_5_UAV_Location(end,1),Red_2_5_UAV_Location(end,2),'pr');
    hold on;
    
    
    
    Point_Attack = Blue_UAV_Location(end,:);
    Point_Array_Besiege = [Red_1_1_UAV_Location_new;
        Red_1_2_UAV_Location_new;
        Red_1_3_UAV_Location_new;
        Red_1_4_UAV_Location_new;
        Red_1_5_UAV_Location_new;
        Red_2_1_UAV_Location_new;
        Red_2_2_UAV_Location_new;
        Red_2_3_UAV_Location_new;
        Red_2_4_UAV_Location_new;
        Red_2_5_UAV_Location_new;
        ];
    
    besiege_status = check_besiege_status(Point_Attack, Point_Array_Besiege, Besiege_distance);
    
    if besiege_status
        % 结束后续的循环
        Flag_Track = 0;
    end
    
    Figure_num = Figure_num + 1;
    % 截图
    % imwrite(ImageWith_BBox, strcat('.\PeoplewithLJDetected_0728\','PeoplewithLJDetected_',num2str(Flag_Track),'.png'));
    % imwrite(ImageWith_BBox, num2str(Flag_Track),'.png'));
    % print(['Frame_' num2str(Flag_Track)], '-dpng', '-r200');
    % print(['Frame_' num2str(Figure_num)], '-dpng');
    frame = getframe(fig); 
    img = frame2im(frame); % 将frame变换成imwrite函数可以识别的格式;
    imwrite(img,strcat('.\first\Frame_',num2str(Figure_num),'.png')); 
    
end
