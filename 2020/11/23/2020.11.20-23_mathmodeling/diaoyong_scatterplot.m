clear all;
clc

result=[];
for A=50:250
	for w=50:250
		j=scatterplot(A,w);
		if j==1
			result=[result;[A,w]];
		end
	end
end
len=length(result)
axis([0 300 0 500])
for i=1:len
	if result(i,1)>50 && result(i,2)>50
	scatter(result(i,1),result(i,2));
	hold on;
	end
end