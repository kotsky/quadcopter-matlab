function [] = symbol_H_draw( x, y, Color)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%Color = 'b';
Width = 1;


R = 1;
x1 = -R+x:0.01:R+x;
x0 = -R:0.01:R;
y1 = sqrt(R^2 - x0.^2);
y1_ = -y1;
y1 = y1 + y;
y1_ = y1_ + y;
plot(x1,y1, 'color', Color, 'LineWidth', Width)
hold on
plot(x1,y1_, 'color', Color,'LineWidth', Width)


line([-R/1.8 + x -R/1.8 + x],[-R/1.8 + y R/1.8 + y],'color',Color,'LineWidth', Width)
line([R/1.8 + x R/1.8 + x],[-R/1.8 + y R/1.8 + y],'color',Color,'LineWidth',Width)
line([-R/1.8 + x R/1.8 + x],[y y],'color',Color,'LineWidth', Width)

end

