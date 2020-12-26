close all;
clear all;
clc

result1=[];
result2=[];
result3=[];
for i=5000:10:5500
	for j=50:10:300
		for k=1:0.05:2
			r=scatterplot_yuce_shangsheng(i,j,k);
			if r==1
				result1=[result1 i];
				result2=[result2 j];
				result3=[result3 k];
			end
		end
	end
end
plot3(result1,result2,result3,'r*');
xlabel('Height');
ylabel('A');
zlabel('w');
hold on;