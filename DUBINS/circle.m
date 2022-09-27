function circle(x,y,r,C)

%figure(1)
 %Draw circle
xCenter = x;
yCenter = y;
raio=r;
theta = 0 : 0.01 : 2*pi;
xA = raio * cos(theta) + xCenter;
yA = raio * sin(theta) + yCenter;


plot(xA, yA,'color',C,'LineWidth',1);


end