 
R = 1;
Width_target = 2;

circle_H = rectangle('Position',[-R -R 2*R 2*R],'Curvature',[1 1], 'LineWidth', Width_target, 'EdgeColor', 'b')

H1 = line([0 0],[-0.8*R 0.8*R],'LineWidth', Width_target, 'Color', 'b')
H2 = line([-0.8*R 0.8*R],[0 0],'LineWidth',Width_target, 'Color', 'b')

%target draw
                    draw_Xc_tr = Xc_tr;
                    draw_Yc_tr = -Yc_tr;
                    set(circle_H, 'Position',[-R+draw_Xc_tr -R+draw_Yc_tr 2*R 2*R])
                    set(H1, 'XData', [draw_Xc_tr draw_Xc_tr], 'YData', [-0.8*R+draw_Yc_tr 0.8*R+draw_Yc_tr] )
                    set(H2, 'XData', [-0.8*R+draw_Xc_tr 0.8*R+draw_Xc_tr], 'YData', [draw_Yc_tr draw_Yc_tr] )
