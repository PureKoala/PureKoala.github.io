function [azimuth, dist ] = GetAzimuth_2points_2(point1, point2)

P_local = [point1(1)-point2(1) point1(2)-point2(2)];

dist = sqrt(P_local(1)^2+P_local(2)^2);

if (P_local(1)>=0) && (P_local(2)>=0)  % 第一象限
    azimuth = 90-asin(P_local(2)/dist) * 180/pi;
    return;
end
if (P_local(1)<=0) && (P_local(2)>=0)   % 第二象限
    azimuth = asin(P_local(2)/dist)*180/pi+270;
    return;
end
if (P_local(1)<=0) && (P_local(2)<=0)   % 第三象限
    azimuth = 180 + asin(-P_local(1)/dist) * 180 / pi;
    return;
end
if (P_local(1)>=0) && (P_local(2)<=0)   % 第四象限
    azimuth = 180 - asin(P_local(1)/dist) * 180 / pi;
    return;
end