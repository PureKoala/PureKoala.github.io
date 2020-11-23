function a=judge(X,Y)
xr=X(1);
yr=X(2);
xb=Y(1);
yb=Y(1);
d=sqrt((yb-yr)^2+(xb-xr)^2);
if d<=0.3
	a=1;
else
	a=0;
end