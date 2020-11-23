function besiege_status = check_besiege_status(Point_Attack, Point_Array_Besiege, Besiege_distance)

Num_Array_Besiege = size(Point_Array_Besiege,1);
distance_array = zeros(1,Num_Array_Besiege);
ii = 0;
for ii = 1:Num_Array_Besiege
    P_local = [Point_Attack(1)-Point_Array_Besiege(ii,1) Point_Attack(2)-Point_Array_Besiege(ii,2)];
    distance_array(ii) = sqrt(P_local(1)^2+P_local(2)^2);
end

% 从distance_array中，挑选两个最小的数
[distance_array_sorted,i]=sort(distance_array);
if (distance_array_sorted(1) < Besiege_distance) && (distance_array_sorted(2) < Besiege_distance)
    besiege_status = 1;
else
    besiege_status = 0;
end

end