function judge=zhixiang_xiexian(yy)

ver=3500;
time=7000;
step_length = 3;
speed_blue_UAV = 25;
speed_red_UAV = 20;
Besiege_distance = 100;
Flag_Track=1;
% UAV集群的半径
r_red_uav = 80;


% 蓝色UAV的初始位置
Blue_UAV_Location = [0 ver]; % x水平,y竖直
directionsita=atan((yy-ver)/10000);
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
    Blue_UAV_Location_new_x = Blue_UAV_Location_temp(1) + step_length * speed_blue_UAV * cos(directionsita);
    Blue_UAV_Location_new_y = Blue_UAV_Location_temp(2) + step_length * speed_blue_UAV * sin(directionsita);
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
    
end

	judge=Flag_Track;
end