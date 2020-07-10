function [] = symbol_S_draw( x, y )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Color = 'b';
Width = 2;

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


ys1 = -R/3:0.01:R/3;
ys2 = 0.5*(R/3):0.01:R/3;

xs1 = sqrt((R/3)^2 - ys1.^2);
xs1_ = -xs1;
ys1_ = -ys1+R/3;
xs2 = sqrt((R/3)^2 - ys2.^2);
ys2 = ys2 + (R/3);
n_xs2 = length(xs2);
n_ys2 = length(ys2);

n_xs1_ = length(xs1_);
n_ys1_ = length(ys1_);

plot(xs1_ + x, ys1_ + y, 'color', Color, 'LineWidth', Width)
plot(xs2 + x, ys2 + y, 'color', Color, 'LineWidth', Width)
line([xs1_(1) + x xs2(n_xs2) + x], [ys1_(1) + y ys2(n_ys2) + y], 'color','b','LineWidth', Width)

plot(-xs1_ + x, -ys1_ + y, 'color', Color, 'LineWidth', Width)
plot(-xs2 + x, -ys2 + y, 'color', Color, 'LineWidth', Width)
line([-xs1_(1) + x -xs2(n_xs2) + x], [-ys1_(1) + y -ys2(n_ys2) + y], 'color',Color,'LineWidth', Width)

line([xs1_(n_xs1_) + x -xs1_(n_xs1_) + x], [ys1_(n_ys1_) + y -ys1_(n_ys1_) + y], 'color',Color,'LineWidth', Width)

end