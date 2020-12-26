close all;
clear all;
clc


result1=[];
result2=[];
result3=[];
for i=500:100:2100
	for j=1:0.05:2
		for k=2000:100:5000
			r=scatterplot_yuceshuaijian_sin(i,j,k);
			if r==1
				result1=[result1 i];
				result2=[result2 j];
				result3=[result3 k];
			end
		end
	end
end
plot3(result1,result2,result3,'r*');
xlabel('Factor_1');
ylabel('Factor_2');
zlabel('Factor_3');
hold on;
plot3(2100,1.95,2000,'g*');
hold on;