close all;
clear all;
clc

result1=[];
result2=[];
result3=[];
for i=1000:100:5000
	for j=2500:100:5000
		for k=100:50:2000
			r=scatter_yuce_sin_hangmu(i,j,k);
			if r==1
				result1=[result1 i];
				result2=[result2 j];
				result3=[result3 k];
			end
		end
	end
	i
end
plot3(result1,result2,result3,'r*');
xlabel('Height');
ylabel('A');
zlabel('w');
hold on;