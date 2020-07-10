h = animatedline;
axis([0 4*pi -1 1])
x = linspace(0,4*pi,10000);

for k = 1:length(x)
    y = sin(x(k));
    addpoints(h,x(k),y);
   
end
