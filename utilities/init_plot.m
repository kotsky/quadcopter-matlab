
% This function initializes the plots

function init_plot(FLY_MODE)


switch FLY_MODE
    case 1
        figure('units','normalized','position',[.1 .1 .8 .8],'name','Quadrotor AUS','numbertitle','off','color','w');
        axes('units','normalized','position',[.2 .1 .6 .8]);
        axis equal
        axis([-35 35 -25 25 0 35])
    case 2
        figure('units','normalized','position',[.1 .1 .8 .8],'name','Quadrotor AUS','numbertitle','off','color','w');
        axes('units','normalized','position',[.2 .1 .6 .8]);
        %axis equal
        %axis([-20 20 -20 20 0 22])
        axis auto
    case 3
        axes('units','normalized','position',[.2 .1 .6 .8]);
        axis equal
        %axis([-2 2 -2 2 0 2])
        axis auto
end

% E1 = uicontrol('units','normalized','position',[.11 .85 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% % E2 = uicontrol('units','normalized','position',[.11 .75 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% % E3 = uicontrol('units','normalized','position',[.11 .65 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% E4 = uicontrol('units','normalized','position',[.11 .55 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% E5 = uicontrol('units','normalized','position',[.11 .45 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% E6 = uicontrol('units','normalized','position',[.11 .35 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');

% uicontrol('units','normalized','position',[.02 .83 .05 .07],'style','text','fontsize',13,'string','Altitude','backgroundcolor','w');
% % uicontrol('units','normalized','position',[.02 .73 .05 .07],'style','text','fontsize',13,'string','Roll','backgroundcolor','w');
% % uicontrol('units','normalized','position',[.02 .63 .05 .07],'style','text','fontsize',13,'string','Pitch','backgroundcolor','w');
% uicontrol('units','normalized','position',[.02 .53 .05 .07],'style','text','fontsize',13,'string','Yaw','backgroundcolor','w');
% uicontrol('units','normalized','position',[.02 .43 .05 .07],'style','text','fontsize',13,'string','X','backgroundcolor','w');
% uicontrol('units','normalized','position',[.02 .33 .05 .07],'style','text','fontsize',13,'string','Y','backgroundcolor','w');

% uicontrol('units','normalized','position',[.11 .25 .1 .07],'style','pushbutton','fontsize',13,'string','Go','callback',@Go1);

% Motors speed
% uicontrol('units','normalized','position',[.85 .83 .05 .07],'style','text','fontsize',13,'string','Front M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .73 .05 .07],'style','text','fontsize',13,'string','Right M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .63 .05 .07],'style','text','fontsize',13,'string','Rear M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .53 .05 .07],'style','text','fontsize',13,'string','Left M','backgroundcolor',[.5 .7 1]);

% O1 = uicontrol('units','normalized','position',[.91 .86 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
% O2 = uicontrol('units','normalized','position',[.91 .76 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
% O3 = uicontrol('units','normalized','position',[.91 .66 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
% O4 = uicontrol('units','normalized','position',[.91 .56 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');

% disturbances
% uicontrol('units','normalized','position',[.13+.77 .35 .08 .07],'style','pushbutton','fontsize',13,'string','Z','callback',@d1);
% uicontrol('units','normalized','position',[.02+.77 .35 .08 .07],'style','pushbutton','fontsize',13,'string','Yaw','callback',@d2);
% uicontrol('units','normalized','position',[.13+.77 .25 .08 .07],'style','pushbutton','fontsize',13,'string','Pitch','callback',@d3);
% uicontrol('units','normalized','position',[.02+.77 .25 .08 .07],'style','pushbutton','fontsize',13,'string','Roll','callback',@d4);

% pop1 = uicontrol('units','normalized','position',[.02 .15 .19 .07],'style','popupmenu','fontsize',13,'string',{'3D view';'Camera view'},'callback',@view1,'value',1);

% axis([-5 5 -5 5 0 5])

        

%axis auto

%rectangle('Position',[-100 -100 200 200], 'FaceColor', [0 .55 .1])

%axis equal
view(30,30)
grid on
hold on

xlabel('x, [m]')
ylabel('y, [m]')
zlabel('z, [m]')

%---------- Camera --------------------%


camproj perspective         %creating 3D view
camva('manual')             %view of camera

hlight = camlight('headlight'); %light 

lighting gouraud

set(gcf,'Renderer','OpenGL')

line([-1 1],[0 0],[0 0])
line([0 0],[-1 1],[0 0],'color','r')
hold on


end