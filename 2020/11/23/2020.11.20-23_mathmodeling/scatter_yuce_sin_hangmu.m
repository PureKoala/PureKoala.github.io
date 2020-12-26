function judge=scatter_yuce_sin_hangmu(Limit,gap,factor1)

%% 参数
% 仿真步长
% 模型一：蓝色无人机初始位置，AB中间，一直往右飞行；红色无人机集群始终朝着蓝色无人机飞行。
% 步长为0.1s
ver=3500;
time=7000/4;
step_length = 4;
speed_blue_UAV = 25;
speed_red_UAV = 20;
speed_red_Ship = 16;
Besiege_distance = 100;
Flag_Track=1;
xlabel=10000;
ylabel=7000;
% UAV集群的半径
r_red_uav = 80;
ff=1;


% 蓝色UAV的初始位置
Blue_UAV_Location = [0 ver]; % x水平,y竖直
% 红色UAV的初始位置，
Red_1_UAV_Location = [10000 3500+gap/2]; % 中心（舰载机1）FY01
Red_1_Ship_Location = Red_1_UAV_Location;
Red_1_1_UAV_Location = [10000+r_red_uav*cos(18*pi/180) 5000+ r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
Red_1_2_UAV_Location = [10000+r_red_uav*cos(90*pi/180) 5000+ r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
Red_1_3_UAV_Location = [10000+r_red_uav*cos(162*pi/180) 5000+ r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
Red_1_4_UAV_Location = [10000+r_red_uav*cos(234*pi/180) 5000+ r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
Red_1_5_UAV_Location = [10000+r_red_uav*cos(306*pi/180) 5000+ r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
Red_2_UAV_Location = [10000 3500-gap/2]; % 中心（舰载机2）FY02
Red_2_Ship_Location = Red_2_UAV_Location;
Red_2_1_UAV_Location = [10000+r_red_uav*cos(18*pi/180) 2000+ r_red_uav*sin(18*pi/180)];  % 无人机FY0201的位置
Red_2_2_UAV_Location = [10000+r_red_uav*cos(90*pi/180) 2000+ r_red_uav*sin(90*pi/180)];  % 无人机FY0202的位置
Red_2_3_UAV_Location = [10000+r_red_uav*cos(162*pi/180) 2000+ r_red_uav*sin(162*pi/180)]; % 无人机FY0203的位置
Red_2_4_UAV_Location = [10000+r_red_uav*cos(234*pi/180) 2000+ r_red_uav*sin(234*pi/180)];  % 无人机FY0204的位置
Red_2_5_UAV_Location = [10000+r_red_uav*cos(306*pi/180) 2000+ r_red_uav*sin(306*pi/180)];  % 无人机FY0205的位置

Figure_num = 0;

while Figure_num < time && Flag_Track && Blue_UAV_Location(end,1)<1.02*10000
    dist1=sqrt((Blue_UAV_Location(end,2)-Red_1_UAV_Location(end,2))^2+(Blue_UAV_Location(end,1)-Red_1_UAV_Location(end,1))^2);
	dist2=sqrt((Blue_UAV_Location(end,2)-Red_2_UAV_Location(end,2))^2+(Blue_UAV_Location(end,1)-Red_2_UAV_Location(end,1))^2);
	distt=min(dist1,dist2);
	if distt > Limit
	% 当前无人机的位置
    Blue_UAV_Location_temp = [Blue_UAV_Location(end,1) Blue_UAV_Location(end,2)];
    Red_1_UAV_Location_temp = [Red_1_UAV_Location(end,1) Red_1_UAV_Location(end,2)];
    Red_2_UAV_Location_temp = [Red_2_UAV_Location(end,1) Red_2_UAV_Location(end,2)];
	Red_1_Ship_Location_temp = [Red_1_Ship_Location(end,1) Red_1_Ship_Location(end,2)];
    Red_2_Ship_Location_temp = [Red_2_Ship_Location(end,1) Red_2_Ship_Location(end,2)];
	
	len=length(Blue_UAV_Location_temp(:));
	if Figure_num==0
		Blue_UAV_Location_fore_temp=[Blue_UAV_Location(end,1)-1 Blue_UAV_Location(end,2)-1];
	else
		Blue_UAV_Location_fore_temp = [Blue_UAV_Location(Figure_num,1) Blue_UAV_Location(Figure_num,2)];
	end
	
    % 计算红色无人机的飞行方向
    azimuth_Red_1 = predictdeltasita_2(Blue_UAV_Location_temp, Red_1_UAV_Location_temp,Blue_UAV_Location_fore_temp);
    azimuth_Red_2 = predictdeltasita_2(Blue_UAV_Location_temp, Red_2_UAV_Location_temp,Blue_UAV_Location_fore_temp);
	azimuth_Red_3 = predictdeltasita_2(Blue_UAV_Location_temp, Red_1_Ship_Location_temp,Blue_UAV_Location_fore_temp);
    azimuth_Red_4 = predictdeltasita_2(Blue_UAV_Location_temp, Red_2_Ship_Location_temp,Blue_UAV_Location_fore_temp);
    
    % 更新蓝色无人机的位置
%	factor1=1500;
	factor2=1/2;
	factor3=1/5000;
	Blue_UAV_Location_new_x = Blue_UAV_Location_temp(1) + speed_blue_UAV * step_length ;
	Blue_UAV_Location_new_y = ver + factor1 * sin( Blue_UAV_Location_new_x * factor2 ) * exp( - factor3 * Blue_UAV_Location_new_x );
%	Blue_UAV_Location_new_y = ver + factor1 * sin( Blue_UAV_Location_new_x * factor2 );
	Blue_UAV_Location_new = [Blue_UAV_Location_new_x Blue_UAV_Location_new_y];
    Blue_UAV_Location = [Blue_UAV_Location;Blue_UAV_Location_new];
    
    % 计算红色无人机群1的位置
    Red_1_UAV_Location_new_x = Red_1_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_1-90))*pi/180);
    Red_1_UAV_Location_new_y = Red_1_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_1-90))*pi/180);
    Red_1_UAV_Location_new = [Red_1_UAV_Location_new_x Red_1_UAV_Location_new_y];
    
	% 计算红色航母1的位置
    Red_1_Ship_Location_new_x = Red_1_Ship_Location_temp(1) + step_length * speed_red_Ship * cos((360-(azimuth_Red_3-90))*pi/180);
    Red_1_Ship_Location_new_y = Red_1_Ship_Location_temp(2) + step_length * speed_red_Ship * sin((360-(azimuth_Red_3-90))*pi/180);
    Red_1_Ship_Location_new = [Red_1_Ship_Location_new_x Red_1_Ship_Location_new_y];
	
	Red_1_1_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(18*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
    Red_1_2_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(90*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
    Red_1_3_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(162*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
    Red_1_4_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(234*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
    Red_1_5_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(306*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
    
    % 更新红色无人机群1的位置
    Red_1_UAV_Location = [Red_1_UAV_Location;Red_1_UAV_Location_new];
	Red_1_Ship_Location = [Red_1_Ship_Location;Red_1_Ship_Location_new];
	
    Red_1_1_UAV_Location = [Red_1_1_UAV_Location;Red_1_1_UAV_Location_new];
    Red_1_2_UAV_Location = [Red_1_2_UAV_Location;Red_1_2_UAV_Location_new];
    Red_1_3_UAV_Location = [Red_1_3_UAV_Location;Red_1_3_UAV_Location_new];
    Red_1_4_UAV_Location = [Red_1_4_UAV_Location;Red_1_4_UAV_Location_new];
    Red_1_5_UAV_Location = [Red_1_5_UAV_Location;Red_1_5_UAV_Location_new];
    
    % 计算红色无人机群2的位置
    Red_2_UAV_Location_new_x = Red_2_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_2-90))*pi/180);
    Red_2_UAV_Location_new_y = Red_2_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_2-90))*pi/180);
    Red_2_UAV_Location_new = [Red_2_UAV_Location_new_x Red_2_UAV_Location_new_y];
	
	% 计算红色航母2的位置
    Red_2_Ship_Location_new_x = Red_2_Ship_Location_temp(1) + step_length * speed_red_Ship * cos((360-(azimuth_Red_4-90))*pi/180);
    Red_2_Ship_Location_new_y = Red_2_Ship_Location_temp(2) + step_length * speed_red_Ship * sin((360-(azimuth_Red_4-90))*pi/180);
    Red_2_Ship_Location_new = [Red_2_Ship_Location_new_x Red_2_Ship_Location_new_y];
    
    Red_2_1_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(18*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(18*pi/180)];  % 无人机FY0201的位置
    Red_2_2_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(90*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(90*pi/180)];  % 无人机FY0202的位置
    Red_2_3_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(162*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(162*pi/180)]; % 无人机FY0203的位置
    Red_2_4_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(234*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(234*pi/180)];  % 无人机FY0204的位置
    Red_2_5_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(306*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(306*pi/180)];  % 无人机FY0205的位置
    
    % 更新红色无人机群2的位置
    Red_2_UAV_Location = [Red_2_UAV_Location;Red_2_UAV_Location_new];
	Red_2_Ship_Location = [Red_2_Ship_Location;Red_2_Ship_Location_new];
	
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	
	else
	if ff
		Red_3_UAV_Location = [Red_1_Ship_Location(end,1) Red_1_Ship_Location(end,2)-200]; % 中心（舰载机1）FY01
		Red_3_1_UAV_Location = [Red_1_Ship_Location(end,1)+r_red_uav*cos(18*pi/180) Red_1_Ship_Location(end,2)-200+ r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
		Red_3_2_UAV_Location = [Red_1_Ship_Location(end,1)+r_red_uav*cos(90*pi/180) Red_1_Ship_Location(end,2)-200+ r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
		Red_3_3_UAV_Location = [Red_1_Ship_Location(end,1)+r_red_uav*cos(162*pi/180) Red_1_Ship_Location(end,2)-200+ r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
		Red_3_4_UAV_Location = [Red_1_Ship_Location(end,1)+r_red_uav*cos(234*pi/180) Red_1_Ship_Location(end,2)-200+ r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
		Red_3_5_UAV_Location = [Red_1_Ship_Location(end,1)+r_red_uav*cos(306*pi/180) Red_1_Ship_Location(end,2)-200+ r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
		Red_4_UAV_Location = [Red_2_Ship_Location(end,1) Red_2_Ship_Location(end,2)+200]; % 中心（舰载机2）FY02
		Red_4_1_UAV_Location = [Red_2_Ship_Location(end,1)+r_red_uav*cos(18*pi/180) Red_2_Ship_Location(end,2)+200+ r_red_uav*sin(18*pi/180)];  % 无人机FY0201的位置
		Red_4_2_UAV_Location = [Red_2_Ship_Location(end,1)+r_red_uav*cos(90*pi/180) Red_2_Ship_Location(end,2)+200+ r_red_uav*sin(90*pi/180)];  % 无人机FY0202的位置
		Red_4_3_UAV_Location = [Red_2_Ship_Location(end,1)+r_red_uav*cos(162*pi/180) Red_2_Ship_Location(end,2)+200+ r_red_uav*sin(162*pi/180)]; % 无人机FY0203的位置
		Red_4_4_UAV_Location = [Red_2_Ship_Location(end,1)+r_red_uav*cos(234*pi/180) Red_2_Ship_Location(end,2)+200+ r_red_uav*sin(234*pi/180)];  % 无人机FY0204的位置
		Red_4_5_UAV_Location = [Red_2_Ship_Location(end,1)+r_red_uav*cos(306*pi/180) Red_2_Ship_Location(end,2)+200+ r_red_uav*sin(306*pi/180)];  % 无人机FY0205的位置
		ff=0;
	end
	
	% 当前无人机的位置
    Blue_UAV_Location_temp = [Blue_UAV_Location(end,1) Blue_UAV_Location(end,2)];
    Red_1_UAV_Location_temp = [Red_1_UAV_Location(end,1) Red_1_UAV_Location(end,2)];
    Red_2_UAV_Location_temp = [Red_2_UAV_Location(end,1) Red_2_UAV_Location(end,2)];
	Red_3_UAV_Location_temp = [Red_3_UAV_Location(end,1) Red_3_UAV_Location(end,2)];
    Red_4_UAV_Location_temp = [Red_4_UAV_Location(end,1) Red_4_UAV_Location(end,2)];
	Red_1_Ship_Location_temp = [Red_1_Ship_Location(end,1) Red_1_Ship_Location(end,2)];
    Red_2_Ship_Location_temp = [Red_2_Ship_Location(end,1) Red_2_Ship_Location(end,2)];
	
	len=length(Blue_UAV_Location_temp(:));
	if Figure_num==0
		Blue_UAV_Location_fore_temp=[Blue_UAV_Location(end,1)-1 Blue_UAV_Location(end,2)-1];
	else
		Blue_UAV_Location_fore_temp = [Blue_UAV_Location(Figure_num,1) Blue_UAV_Location(Figure_num,2)];
	end
	
    % 计算红色无人机的飞行方向
    azimuth_Red_1 = predictdeltasita_2(Blue_UAV_Location_temp, Red_1_UAV_Location_temp,Blue_UAV_Location_fore_temp);
    azimuth_Red_2 = predictdeltasita_2(Blue_UAV_Location_temp, Red_2_UAV_Location_temp,Blue_UAV_Location_fore_temp);
	azimuth_Red_3 = predictdeltasita_2(Blue_UAV_Location_temp, Red_1_Ship_Location_temp,Blue_UAV_Location_fore_temp);
    azimuth_Red_4 = predictdeltasita_2(Blue_UAV_Location_temp, Red_2_Ship_Location_temp,Blue_UAV_Location_fore_temp);
	azimuth_Red_5 = predictdeltasita_2(Blue_UAV_Location_temp, Red_3_UAV_Location_temp,Blue_UAV_Location_fore_temp);
    azimuth_Red_6 = predictdeltasita_2(Blue_UAV_Location_temp, Red_4_UAV_Location_temp,Blue_UAV_Location_fore_temp);
    
    % 更新蓝色无人机的位置
	factor1=1500;
	factor2=1/2;
	factor3=1/5000;
	Blue_UAV_Location_new_x = Blue_UAV_Location_temp(1) + speed_blue_UAV * step_length ;
	Blue_UAV_Location_new_y = ver + factor1 * sin( Blue_UAV_Location_new_x * factor2 ) * exp( - factor3 * Blue_UAV_Location_new_x );
%	Blue_UAV_Location_new_y = ver + factor1 * sin( Blue_UAV_Location_new_x * factor2 );
	Blue_UAV_Location_new = [Blue_UAV_Location_new_x Blue_UAV_Location_new_y];
    Blue_UAV_Location = [Blue_UAV_Location;Blue_UAV_Location_new];
    
    % 计算红色无人机群1的位置
    Red_1_UAV_Location_new_x = Red_1_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_1-90))*pi/180);
    Red_1_UAV_Location_new_y = Red_1_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_1-90))*pi/180);
    Red_1_UAV_Location_new = [Red_1_UAV_Location_new_x Red_1_UAV_Location_new_y];
    
	% 计算红色航母1的位置
    Red_1_Ship_Location_new_x = Red_1_Ship_Location_temp(1) + step_length * speed_red_Ship * cos((360-(azimuth_Red_3-90))*pi/180);
    Red_1_Ship_Location_new_y = Red_1_Ship_Location_temp(2) + step_length * speed_red_Ship * sin((360-(azimuth_Red_3-90))*pi/180);
    Red_1_Ship_Location_new = [Red_1_Ship_Location_new_x Red_1_Ship_Location_new_y];
	
	Red_1_1_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(18*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
    Red_1_2_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(90*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
    Red_1_3_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(162*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
    Red_1_4_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(234*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
    Red_1_5_UAV_Location_new = [Red_1_UAV_Location_new_x+r_red_uav*cos(306*pi/180) Red_1_UAV_Location_new_y+r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
    
    % 更新红色无人机群1的位置
    Red_1_UAV_Location = [Red_1_UAV_Location;Red_1_UAV_Location_new];
	Red_1_Ship_Location = [Red_1_Ship_Location;Red_1_Ship_Location_new];
	
    Red_1_1_UAV_Location = [Red_1_1_UAV_Location;Red_1_1_UAV_Location_new];
    Red_1_2_UAV_Location = [Red_1_2_UAV_Location;Red_1_2_UAV_Location_new];
    Red_1_3_UAV_Location = [Red_1_3_UAV_Location;Red_1_3_UAV_Location_new];
    Red_1_4_UAV_Location = [Red_1_4_UAV_Location;Red_1_4_UAV_Location_new];
    Red_1_5_UAV_Location = [Red_1_5_UAV_Location;Red_1_5_UAV_Location_new];
    
    % 计算红色无人机群2的位置
    Red_2_UAV_Location_new_x = Red_2_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_2-90))*pi/180);
    Red_2_UAV_Location_new_y = Red_2_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_2-90))*pi/180);
    Red_2_UAV_Location_new = [Red_2_UAV_Location_new_x Red_2_UAV_Location_new_y];
	
	% 计算红色航母2的位置
    Red_2_Ship_Location_new_x = Red_2_Ship_Location_temp(1) + step_length * speed_red_Ship * cos((360-(azimuth_Red_4-90))*pi/180);
    Red_2_Ship_Location_new_y = Red_2_Ship_Location_temp(2) + step_length * speed_red_Ship * sin((360-(azimuth_Red_4-90))*pi/180);
    Red_2_Ship_Location_new = [Red_2_Ship_Location_new_x Red_2_Ship_Location_new_y];
    
    Red_2_1_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(18*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(18*pi/180)];  % 无人机FY0201的位置
    Red_2_2_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(90*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(90*pi/180)];  % 无人机FY0202的位置
    Red_2_3_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(162*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(162*pi/180)]; % 无人机FY0203的位置
    Red_2_4_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(234*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(234*pi/180)];  % 无人机FY0204的位置
    Red_2_5_UAV_Location_new = [Red_2_UAV_Location_new(1)+r_red_uav*cos(306*pi/180) Red_2_UAV_Location_new(2)+r_red_uav*sin(306*pi/180)];  % 无人机FY0205的位置
    
    % 更新红色无人机群2的位置
    Red_2_UAV_Location = [Red_2_UAV_Location;Red_2_UAV_Location_new];
	Red_2_Ship_Location = [Red_2_Ship_Location;Red_2_Ship_Location_new];
	
    Red_2_1_UAV_Location = [Red_2_1_UAV_Location;Red_2_1_UAV_Location_new];  % 无人机FY0201的位置
    Red_2_2_UAV_Location = [Red_2_2_UAV_Location;Red_2_2_UAV_Location_new];  % 无人机FY0202的位置
    Red_2_3_UAV_Location = [Red_2_3_UAV_Location;Red_2_3_UAV_Location_new];  % 无人机FY0203的位置
    Red_2_4_UAV_Location = [Red_2_4_UAV_Location;Red_2_4_UAV_Location_new];  % 无人机FY0204的位置
    Red_2_5_UAV_Location = [Red_2_5_UAV_Location;Red_2_5_UAV_Location_new];  % 无人机FY0205的位置
	
	% 计算红色无人机群3的位置
    Red_3_UAV_Location_new_x = Red_3_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_1-90))*pi/180);
    Red_3_UAV_Location_new_y = Red_3_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_1-90))*pi/180);
    Red_3_UAV_Location_new = [Red_3_UAV_Location_new_x Red_3_UAV_Location_new_y];
	
	Red_3_1_UAV_Location_new = [Red_3_UAV_Location_new_x+r_red_uav*cos(18*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
    Red_3_2_UAV_Location_new = [Red_3_UAV_Location_new_x+r_red_uav*cos(90*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
    Red_3_3_UAV_Location_new = [Red_3_UAV_Location_new_x+r_red_uav*cos(162*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
    Red_3_4_UAV_Location_new = [Red_3_UAV_Location_new_x+r_red_uav*cos(234*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
    Red_3_5_UAV_Location_new = [Red_3_UAV_Location_new_x+r_red_uav*cos(306*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
    
    % 更新红色无人机群3的位置
    Red_3_UAV_Location = [Red_3_UAV_Location;Red_3_UAV_Location_new];
	
    Red_3_1_UAV_Location = [Red_3_1_UAV_Location;Red_3_1_UAV_Location_new];
    Red_3_2_UAV_Location = [Red_3_2_UAV_Location;Red_3_2_UAV_Location_new];
    Red_3_3_UAV_Location = [Red_3_3_UAV_Location;Red_3_3_UAV_Location_new];
    Red_3_4_UAV_Location = [Red_3_4_UAV_Location;Red_3_4_UAV_Location_new];
    Red_3_5_UAV_Location = [Red_3_5_UAV_Location;Red_3_5_UAV_Location_new];
	
	%%
	% 计算红色无人机群4的位置
    Red_4_UAV_Location_new_x = Red_4_UAV_Location_temp(1) + step_length * speed_red_UAV * cos((360-(azimuth_Red_1-90))*pi/180);
    Red_4_UAV_Location_new_y = Red_4_UAV_Location_temp(2) + step_length * speed_red_UAV * sin((360-(azimuth_Red_1-90))*pi/180);
    Red_4_UAV_Location_new = [Red_4_UAV_Location_new_x Red_4_UAV_Location_new_y];
	
	Red_4_1_UAV_Location_new = [Red_4_UAV_Location_new_x+r_red_uav*cos(18*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(18*pi/180)];  % 无人机FY0101的位置
    Red_4_2_UAV_Location_new = [Red_4_UAV_Location_new_x+r_red_uav*cos(90*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(90*pi/180)];  % 无人机FY0102的位置
    Red_4_3_UAV_Location_new = [Red_4_UAV_Location_new_x+r_red_uav*cos(162*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(162*pi/180)];  % 无人机FY0103的位置
    Red_4_4_UAV_Location_new = [Red_4_UAV_Location_new_x+r_red_uav*cos(234*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(234*pi/180)];  % 无人机FY0104的位置
    Red_4_5_UAV_Location_new = [Red_4_UAV_Location_new_x+r_red_uav*cos(306*pi/180) Red_3_UAV_Location_new_y+r_red_uav*sin(306*pi/180)];  % 无人机FY0105的位置
    
    % 更新红色无人机群3的位置
    Red_4_UAV_Location = [Red_4_UAV_Location;Red_4_UAV_Location_new];
	
    Red_4_1_UAV_Location = [Red_4_1_UAV_Location;Red_4_1_UAV_Location_new];
    Red_4_2_UAV_Location = [Red_4_2_UAV_Location;Red_4_2_UAV_Location_new];
    Red_4_3_UAV_Location = [Red_4_3_UAV_Location;Red_4_3_UAV_Location_new];
    Red_4_4_UAV_Location = [Red_4_4_UAV_Location;Red_4_4_UAV_Location_new];
    Red_4_5_UAV_Location = [Red_4_5_UAV_Location;Red_4_5_UAV_Location_new];
    
    
    
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
		Red_3_1_UAV_Location_new;
        Red_3_2_UAV_Location_new;
        Red_3_3_UAV_Location_new;
        Red_3_4_UAV_Location_new;
        Red_3_5_UAV_Location_new;
        Red_4_1_UAV_Location_new;
        Red_4_2_UAV_Location_new;
        Red_4_3_UAV_Location_new;
        Red_4_4_UAV_Location_new;
        Red_4_5_UAV_Location_new;
        ];
    
    besiege_status = check_besiege_status(Point_Attack, Point_Array_Besiege, Besiege_distance);
    
    if besiege_status
        % 结束后续的循环
        Flag_Track = 0;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	end
    
    Figure_num = Figure_num + 1;
    
end
	judge=Flag_Track && Figure_num < time;
end