import cv2
import numpy as np
import os
import RPi.GPIO as gpio
import time

###################################################################################################

def adjust_gamma(image, gamma):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
	for i in np.arange(0, 256)]).astype("uint8")
 
    return cv2.LUT(image, table)



gpio.setmode(gpio.BCM)

pin_x_l = 12
pin_x_r = 16
pin_y_up = 20
pin_y_d = 21

freq_x = 50
freq_y = 50

gpio.setup(pin_x_l, gpio.OUT)
gpio.setup(pin_x_r, gpio.OUT)

pwmObject_x_l = gpio.PWM(pin_x_l, freq_x)
pwmObject_x_r = gpio.PWM(pin_x_r, freq_x)

pwmObject_x_l.start(0) 
pwmObject_x_r.start(0) 

gpio.setup(pin_y_up, gpio.OUT)
gpio.setup(pin_y_d, gpio.OUT)

pwmObject_y_up = gpio.PWM(pin_y_up, freq_y)
pwmObject_y_d = gpio.PWM(pin_y_d, freq_y)

pwmObject_y_up.start(0)
pwmObject_y_d.start(0)

###################################################################################################

def moved_x(AK_x,Turn_x_l,Turn_x_r):
	
	if AK_x == 1:
		DC_x_l = int(Turn_x_l)
		DC_x_r = int(0)
		pwmObject_x_r.ChangeDutyCycle(DC_x_r)
		pwmObject_x_l.ChangeDutyCycle(DC_x_l)
		
	else:   
		DC_x_r = int(Turn_x_r)
                DC_x_l = int(0)
		pwmObject_x_l.ChangeDutyCycle(DC_x_l)
		pwmObject_x_r.ChangeDutyCycle(DC_x_r)

	
def moved_y(AK_y,Turn_y_up,Turn_y_d):

        if AK_y == 1:
		DC_y_d = int(Turn_y_d)
                DC_y_up = int(0)
                pwmObject_y_up.ChangeDutyCycle(DC_y_up)
                pwmObject_y_d.ChangeDutyCycle(DC_y_d)

        else:
		DC_y_up = int(Turn_y_up)
                DC_y_d = int(0)
                pwmObject_y_d.ChangeDutyCycle(DC_y_d)
                pwmObject_y_up.ChangeDutyCycle(DC_y_up)



###################################################################################################
def main():
    DELAY = 0.001
    diap_x = int(20)
    diap_y = int(10)
    AK_x = 1
    AK_y = 1

    Angle_x = 1
    Koeff_x = 1

    Angle_y = 1
    Koeff_y = 1

###################################################################################################

    capWebcam = cv2.VideoCapture(0)

    print("default resolution = " + str(capWebcam.get(cv2.CAP_PROP_FRAME_WIDTH)) + "x" + str(capWebcam.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    capWebcam.set(cv2.CAP_PROP_FRAME_WIDTH, 320.0)
    capWebcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240.0)

    intXFrameCenter = int(float(capWebcam.get(cv2.CAP_PROP_FRAME_WIDTH)) / 2.0)
    intYFrameCenter = int(120)

    Position_x = 0
    Position_y = 0

    if capWebcam.isOpened() == False:
        print("error: capWebcam not accessed successfully\n\n")
        os.system("pause")
        return
    # end if

###################################################################################################

    while cv2.waitKey(1) != 27 and capWebcam.isOpened():
        blnFrameReadSuccessfully, imgOriginal = capWebcam.read()

        if not blnFrameReadSuccessfully or imgOriginal is None:
            print("error: frame not read from webcam\n")
            os.system("pause")
            break
        # end if

        gray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 7, 12, 12)
        med = np.median(gray)

        if med < 11:
            gamma = 2
        elif med < 31:
            gamma = 1.6
        elif med < 51:
            gamma = 1.4
        elif med < 130:
            gamma = 2
        else:
            gamma = 1.3

        imgGamma = adjust_gamma(imgOriginal, gamma)
        imgHSV = cv2.cvtColor(imgGamma, cv2.COLOR_BGR2HSV)

        imgThreshLow = cv2.inRange(imgHSV, np.array([0, 170, 120]), np.array([20, 240, 255]))
        imgThreshHigh = cv2.inRange(imgHSV, np.array([20, 70, 170]), np.array([40, 170, 255]))
        imgThresh = cv2.add(imgThreshLow, imgThreshHigh)

        st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15), (7, 7))
        st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5))
        imgThresh = cv2.morphologyEx(imgThresh, cv2.MORPH_CLOSE, st1)
        imgThresh = cv2.morphologyEx(imgThresh, cv2.MORPH_OPEN, st2)
        
        sigma = 33
        low = int(max(0, (1.0 - sigma/100) * med))
        up = int(min(255, (1.0 + sigma/100) * med))

        edged = cv2.Canny(imgThresh, low, up)


       
        img, contours,hierarchy = cv2.findContours(edged.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE )
        try:
            cnt = contours[0]
###################################################################################################

        except Exception:
	    Turn_x_l = int(0)
	    Turn_x_r = int(0)
            if Angle_x == 1:
		if Position_x >= 140:
                    Koeff_x = -1
		    

            else:
                if Position_x <= -140:
                    Koeff_x = -1


            Turn_y_up = int(0)
            Turn_y_d = int(0)
            if Angle_y == 1:
                if Position_y >= 140:
                    Koeff_y = -1


            else:
                if Position_y <= -140:
                    Koeff_y = -1
        
###################################################################################################
	else:
            Koeff = 1
            a,b,w,h = cv2.boundingRect(cnt)
            cv2.rectangle(imgOriginal,(a,b),(a+w,b+h),(0,255,0),2)
            x = a+w/2
            y = b+h/2
            print("ball position x = " + str(x) + ", y = " + str(y))

###################################################################################################

            if x > intXFrameCenter:
                Angle_x = 1
                Turn_x_r = int(0)
                if x < intXFrameCenter + diap_x:
                    Turn_x_l = int((x-160)*0.2)

                else:
                    Turn_x_l = int(0.3 * (x-160))


            else:
                Angle_x = -1
                Turn_x_l = int(0)
                if intXFrameCenter - diap_x < x:
                    Turn_x_r = int(-(x-160)*0.2)

                else:
                    Turn_x_r = int(0.3 * (-(x-160)))

###################################################################################################

            if y > intYFrameCenter:
                Angle_y = 1
                Turn_y_up = int(0)
                if y < intYFrameCenter + diap_y:
                    Turn_y_d = int((y-120)*0.25)

                else:
                    Turn_y_d = int((y-120)*0.35)


            else:
                Angle_y = -1
                Turn_y_d = int(0)
                if intYFrameCenter - diap_y < y:
                    Turn_y_up = int(-(y-120)*0.25)

                else:
                    Turn_y_up = int(0.35*(-(y-120)))


###################################################################################################

	AK_x = Angle_x*Koeff_x
	AK_y = Angle_y*Koeff_y

	moved_x(AK_x,Turn_x_l,Turn_x_r)
	moved_y(AK_y,Turn_y_up,Turn_y_d)

###################################################################################################
        
#        cv2.namedWindow("imgOriginal", cv2.WINDOW_AUTOSIZE)
 #       cv2.namedWindow("imgThresh", cv2.WINDOW_AUTOSIZE)
  #      cv2.imshow("imgOriginal", imgOriginal)
   #     cv2.imshow("imgThresh", imgThresh)
        # end while

    cv2.destroyAllWindows()
    return

###################################################################################################
if __name__ == "__main__":
    main()


	
	
	0	
	4,36836088412015	
	8,39513069622820	
	11,8445744743349	
	14,6224014978397	
	16,7425610040527	
	18,2718445024260	
	19,2865770782421	
	19,8488249665799
	
	25	0 0
	24,3963823901878 0	4,36836088412015	
	22,6909184956658 0	8,39513069622820
	20,1442476095044 0	11,8445744743349
	17,0562202013287 0	14,6224014978397
	13,6751743033666 0	16,7425610040527
	10,1657896335550 0	18,2718445024260
	6,61767810076558 0	19,2865770782421
	3,06801733771060 0	19,8488249665799
	
waypoints = [
    0 0 20
    3.06801733771060 0 19.8488249665799
    6.61767810076558 0 19.2865770782421
    10.1657896335550 0 18.2718445024260
    13.6751743033666 0 16.7425610040527
    17.0562202013287 0 14.6224014978397
    20.1442476095044 0 11.8445744743349
    22.6909184956658 0 8.39513069622820
    24.3963823901878 0 4.3683608841201
    25.0001231321231 0 0];
	
	
	[0 0 20
    5 0 19.9
    10 0 19.6
    15 0 19
    20 0 17
    22.5 0 10
    25 0 4
    25 0 0];
	
	
	waypoints = [
    0 0 20
    3.06801733771060 0 19.8488249665799
    6.61767810076558 0 19.2865770782421
    10.1657896335550 0 18.2718445024260
    13.6751743033666 0 16.7425610040527
    17.0562202013287 0 14.6224014978397
    20.1442476095044 0 11.8445744743349
    22.6909184956658 0 8.39513069622820
    24.3963823901878 0 4.3683608841201
    25.0001231321231 0 0];
	
	
	% This function initializes the plots

function init_plot


figure('units','normalized','position',[.1 .1 .8 .8],'name','Quadrotor AUS','numbertitle','off','color','w');
axes('units','normalized','position',[.2 .1 .6 .8]);
%axis auto
axis equal

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
%axis([-5 5 -5 5 0 10]);
axis auto
view(30,30)
grid on
hold on

xlabel('x')

%---------- Camera --------------------%


camproj perspective         %creating 3D view
camva('manual')             %view of camera

hlight = camlight('headlight'); %light 

lighting gouraud

set(gcf,'Renderer','OpenGL')

line([-1 1],[0 0],[0 0])
line([0 0],[-1 1],[0 0],'color','r')
hold on
%img = imread('Untitled.png');
%imshow(img)


end
	
	
	
	
	Quad.Z_KP = 10/1.7;    % KP value in altitude control
Quad.Z_KI = 0.3;    % KI value in altitude control
Quad.Z_KD = -10/1.980;  % KD value in altitude control
Quad.Z_KI_lim = .25;
	
	
	
	% Wil Selby
% Washington, DC
% May 30, 2015

% This script defines and initializes the variables for the quadrotor simulator.                                  %

global Quad;

%% Initialize Variables

% Simulation Parameters
Quad.init = 0;     % used in initilization 
Quad.Ts = .01;     % Sampling time (100 Hz)
Quad.sim_time = 10; % Simulation time (seconds)
Quad.counter = 1;                      % the counter that holds the time value

% Plotting Variables
Quad.t_plot = [0:Quad.Ts:Quad.sim_time-Quad.Ts];       % the time values
Quad.Xtemp = 0;     % Temp variables used rotating and plotting quadrotor
Quad.Ytemp = 0;     % Temp variables used rotating and plotting quadrotor
Quad.Ztemp = 0;     % Temp variables used rotating and plotting quadrotor

% Environmental Parametes
Quad.g = 9.81;     % Gravity (m/s^2)

% Quadrotor Physical Parameters
Quad.m = 1.4;      % Quadrotor mass (kg)
Quad.l = .56;     % Distance from the center of mass to the each motor (m)
Quad.t = .02;   %Thickness of the quadrotor's arms for drawing purposes (m)
Quad.rot_rad = .1;   %Radius of the propellor (m)
Quad.Kd = 1.3858e-6;    % Drag torque coeffecient (kg-m^2)

Quad.Kdx = 0.16481;    % Translational drag force coeffecient (kg/s)
Quad.Kdy = 0.31892;    % Translational drag force coeffecient (kg/s)
Quad.Kdz = 1.1E-6;    % Translational drag force coeffecient (kg/s)

Quad.Jx = .05;     % Moment of inertia about X axis (kg-m^2)
Quad.Jy = .05;     % Moment of inertia about Y axis (kg-m^2)
Quad.Jz = .24;    % Moment of inertia about Z axis (kg-m^2)

% Quadrotor Sensor Paramaters
Quad.GPS_freq = (1/Quad.Ts)/1;  
Quad.X_error = 0;  %+/- m
Quad.Y_error = 0;  %+/- m
Quad.Z_error = 0;  %+/- m

Quad.x_acc_bias = 0.16594;  % m/s^2
Quad.x_acc_sd = 0.0093907;
Quad.y_acc_bias = 0.31691;  % m/s^2
Quad.y_acc_sd = 0.011045;
Quad.z_acc_bias = -8.6759;  % m/s^2
Quad.z_acc_sd = 0.016189;

Quad.x_gyro_bias = 0.00053417;  % rad/s
Quad.x_gyro_sd = 0.00066675;
Quad.y_gyro_bias = -0.0011035;  % rad/s
Quad.y_gyro_sd = 0.00053642;
Quad.z_gyro_bias = 0.00020838;  % rad/s
Quad.z_gyro_sd = 0.0004403;

Quad.ground_truth = 1;  % Use perfect sensor measurements
Quad.sensor_unfiltered = 0; % Use sensor errors, no filter
Quad.sensor_kf = 0;     % Use sensor error, Kalman Filter

% Motor Parameters
Quad.KT = 1.3328e-5;    % Thrust force coeffecient (kg-m)
Quad.Jp = 0.044;     % Moment of Intertia of the rotor (kg-m^2)
Quad.max_motor_speed = 925; % motors upper limit (rad/s)
Quad.min_motor_speed = 0; %-1*((400)^2); % motors lower limit (can't spin in reverse)

Quad.Obar = 0;     % sum of motor speeds (O1-O2+O3-O4, N-m) 
Quad.O1 = 0;       % Front motor speed (raidans/s)
Quad.O2 = 0;       % Right motor speed (raidans/s)
Quad.O3 = 0;       % Rear motor speed (raidans/s)
Quad.O4 = 0;       % Left motor speed (raidans/s)

%Translational Positions
Quad.X = 0;        % Initial position in X direction GF (m)
Quad.Y = 0;        % Initial position in Y direction GF (m)
Quad.Z = 10;        % Initial position in Z direction GF (m)
Quad.X_BF = 0;     % Initial position in X direction BF (m)
Quad.Y_BF = 0;     % Initial position in Y direction BF (m)
Quad.Z_BF = 0;     % Initial position in the Z direction BF (m)

%Translational Velocities
Quad.X_dot = 0;    % Initial velocity in X direction GF (m/s)
Quad.Y_dot = 0;    % Initial velocity in Y direction GF (m/s)
Quad.Z_dot = 0;    % Initial velocity in Z direction GF (m/s)
Quad.X_dot_BF = 0;    % Initial velocity in X direction BF (m/s)
Quad.Y_dot_BF = 0;    % Initial velocity in Y direction BF (m/s)
Quad.Z_dot_BF = 0;    % Initial velocity in Y direction BF (m/s)

%Angular Positions
Quad.phi = 0;      % Initial phi value (rotation about X GF, roll,  radians)
Quad.theta = 0;    % Initial theta value (rotation about Y GF, pitch, radians)
Quad.psi = 0;      % Initial psi value (rotation about Z GF, yaw, radians)

%Angular Velocities
Quad.p = 0;        % Initial p value (angular rate rotation about X BF, radians/s)
Quad.q = 0;        % Initial q value (angular rate rotation about Y BF, radians/s)
Quad.r = 0;        % Initial r value (angular rate rotation about Z BF, radians/s)

% Desired variables
Quad.X_des_GF = 10;         % desired value of X in Global frame
Quad.Y_des_GF = 10;         % desired value of Y in Global frame
Quad.Z_des_GF = 10;         % desired value of Z in Global frame
Quad.X_des = 0;            % desired value of X in Body frame
Quad.Y_des = 0;            % desired value of Y in Body frame
Quad.Z_des = 0;            % desired value of Z in Body frame

Quad.phi_des = 0;          % desired value of phi (radians)
Quad.theta_des = 0;        % desired value of theta (radians)
Quad.psi_des = 0;          % desired value of psi (radians)

% Measured variables
Quad.X_meas = 0;
Quad.Y_meas = 0;
Quad.Z_meas = 0;
Quad.phi_meas = 0;
Quad.theta_meas = 0;
Quad.psi_meas = 0;

% Disturbance Variables
Quad.Z_dis = 0;            % Disturbance in Z direction
Quad.X_dis = 0;            % Disturbance in X direction
Quad.Y_dis = 0;            % Ddisturbance in Y direction
Quad.phi_dis = 0;            % Disturbance in Yaw direction
Quad.theta_dis = 0;            % Disturbance in Pitch direction
Quad.psi_dis = 0;            % Disturbance in Roll direction

% Control Inputs
Quad.U1 = 0;       % Total thrust (N)
Quad.U2 = 0;       % Torque about X axis BF (N-m)
Quad.U3 = 0;       % Torque about Y axis BF (N-m)
Quad.U4 = 0;       % Torque about Z axis BF (N-m)

% Control Limits (update values)
Quad.U1_max = 43.5;   % Quad.KT*4*Quad.max_motor_speed^2
Quad.U1_min = 0;      % 
Quad.U2_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U2_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U3_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U3_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U4_max = 2.25; % Quad.Kd*2*Quad.max_motor_speed^2
Quad.U4_min = -2.25;% Quad.Kd*2*Quad.max_motor_speed^2

% PID parameters
Quad.X_KP = .5;          % KP value in X position control
Quad.X_KI = .25;            % KI value in X position control
Quad.X_KD = -.7;         % KD value in X position control
Quad.X_KI_lim = .25;         % Error to start calculating integral term

Quad.Y_KP = .5;          % KP value in Y position control
Quad.Y_KI = .25;            % KI value in Y position control
Quad.Y_KD = -.7;         % KD value in Y position control
Quad.Y_KI_lim = .25;         % Error to start calculating integral term

Quad.Z_KP = 5;    % KP value in altitude control
Quad.Z_KI = 1;    % KI value in altitude control
Quad.Z_KD = -5;  % KD value in altitude control
Quad.Z_KI_lim = .25;         % Error to start calculating integral term

Quad.phi_KP = 2;      % KP value in roll control 2
Quad.phi_KI = 1;       % KI value in roll control   1        
Quad.phi_KD = -.0;     % KD value in roll control  -.5
Quad.phi_max = pi/4;   % Maximum roll angle commanded
Quad.phi_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

Quad.theta_KP = 2;    % KP value in pitch control 2
Quad.theta_KI = 1;     % KI value in pitch control 1
Quad.theta_KD = -.0;   % KD value in pitch control -.5
Quad.theta_max = pi/4; % Maximum pitch angle commanded
Quad.theta_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

Quad.psi_KP = 2;     % KP value in yaw control
Quad.psi_KI = 0.75;     % KI value in yaw control .75
Quad.psi_KD = -0.0;     % KD value in yaw control -.5
Quad.psi_KI_lim = 8*(2*pi/360);  % Error to start calculating integral 

Quad.p_KP = 3;    % KP value in pitch control 2
Quad.p_KI = 0.7;     % KI value in pitch control
Quad.p_KD = -.01;   % KD value in pitch control -.5
Quad.p_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.p_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 

Quad.q_KP = 3;    % KP value in pitch control
Quad.q_KI = 0.7;     % KI value in pitch control
Quad.q_KD = -.01;   % KD value in pitch control -.5
Quad.q_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.q_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 

Quad.r_KP = 3;    % KP value in pitch control
Quad.r_KI = 0.7;     % KI value in pitch control
Quad.r_KD = -.01;   % KD value in pitch control
Quad.r_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.r_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 

	
	% Wil Selby
% Washington, DC
% May 30, 2015

% This script defines and initializes the variables for the quadrotor simulator.                                  %

global Quad;

%% Initialize Variables

% Simulation Parameters
Quad.init = 0;     % used in initilization 
Quad.Ts = .01;     % Sampling time (100 Hz)
Quad.sim_time = 10; % Simulation time (seconds)
Quad.counter = 1;                      % the counter that holds the time value

% Plotting Variables
Quad.t_plot = [0:Quad.Ts:Quad.sim_time-Quad.Ts];       % the time values
Quad.Xtemp = 0;     % Temp variables used rotating and plotting quadrotor
Quad.Ytemp = 0;     % Temp variables used rotating and plotting quadrotor
Quad.Ztemp = 0;     % Temp variables used rotating and plotting quadrotor

% Environmental Parametes
Quad.g = 9.81;     % Gravity (m/s^2)

% Quadrotor Physical Parameters
Quad.m = 1.4;      % Quadrotor mass (kg)
Quad.l = .56;     % Distance from the center of mass to the each motor (m)
Quad.t = .02;   %Thickness of the quadrotor's arms for drawing purposes (m)
Quad.rot_rad = .1;   %Radius of the propellor (m)
Quad.Kd = 1.3858e-6;    % Drag torque coeffecient (kg-m^2)

Quad.Kdx = 0.16481;    % Translational drag force coeffecient (kg/s)
Quad.Kdy = 0.31892;    % Translational drag force coeffecient (kg/s)
Quad.Kdz = 1.1E-6;    % Translational drag force coeffecient (kg/s)

Quad.Jx = .05;     % Moment of inertia about X axis (kg-m^2)
Quad.Jy = .05;     % Moment of inertia about Y axis (kg-m^2)
Quad.Jz = .24;    % Moment of inertia about Z axis (kg-m^2)

% Quadrotor Sensor Paramaters
Quad.GPS_freq = (1/Quad.Ts)/1;  
Quad.X_error = .01;  %+/- m
Quad.Y_error = .01;  %+/- m
Quad.Z_error = .02;  %+/- m

Quad.x_acc_bias = 0.16594;  % m/s^2
Quad.x_acc_sd = 0.0093907;
Quad.y_acc_bias = 0.31691;  % m/s^2
Quad.y_acc_sd = 0.011045;
Quad.z_acc_bias = -8.6759;  % m/s^2
Quad.z_acc_sd = 0.016189;

Quad.x_gyro_bias = 0.00053417;  % rad/s
Quad.x_gyro_sd = 0.00066675;
Quad.y_gyro_bias = -0.0011035;  % rad/s
Quad.y_gyro_sd = 0.00053642;
Quad.z_gyro_bias = 0.00020838;  % rad/s
Quad.z_gyro_sd = 0.0004403;

Quad.ground_truth = 1;  % Use perfect sensor measurements
Quad.sensor_unfiltered = 0; % Use sensor errors, no filter
Quad.sensor_kf = 0;     % Use sensor error, Kalman Filter

% Motor Parameters
Quad.KT = 1.3328e-5;    % Thrust force coeffecient (kg-m)
Quad.Jp = 0.044;     % Moment of Intertia of the rotor (kg-m^2)
Quad.max_motor_speed = 925; % motors upper limit (rad/s)
Quad.min_motor_speed = 0; %-1*((400)^2); % motors lower limit (can't spin in reverse)

Quad.Obar = 0;     % sum of motor speeds (O1-O2+O3-O4, N-m) 
Quad.O1 = 0;       % Front motor speed (raidans/s)
Quad.O2 = 0;       % Right motor speed (raidans/s)
Quad.O3 = 0;       % Rear motor speed (raidans/s)
Quad.O4 = 0;       % Left motor speed (raidans/s)

%Translational Positions
Quad.X = 0;        % Initial position in X direction GF (m)
Quad.Y = 0;        % Initial position in Y direction GF (m)
Quad.Z = 1;        % Initial position in Z direction GF (m)
Quad.X_BF = 0;     % Initial position in X direction BF (m)
Quad.Y_BF = 0;     % Initial position in Y direction BF (m)
Quad.Z_BF = 0;     % Initial position in the Z direction BF (m)

%Translational Velocities
Quad.X_dot = 0;    % Initial velocity in X direction GF (m/s)
Quad.Y_dot = 0;    % Initial velocity in Y direction GF (m/s)
Quad.Z_dot = 0;    % Initial velocity in Z direction GF (m/s)
Quad.X_dot_BF = 0;    % Initial velocity in X direction BF (m/s)
Quad.Y_dot_BF = 0;    % Initial velocity in Y direction BF (m/s)
Quad.Z_dot_BF = 0;    % Initial velocity in Y direction BF (m/s)

%Angular Positions
Quad.phi = 0;      % Initial phi value (rotation about X GF, roll,  radians)
Quad.theta = 0;    % Initial theta value (rotation about Y GF, pitch, radians)
Quad.psi = 0;      % Initial psi value (rotation about Z GF, yaw, radians)

%Angular Velocities
Quad.p = 0;        % Initial p value (angular rate rotation about X BF, radians/s)
Quad.q = 0;        % Initial q value (angular rate rotation about Y BF, radians/s)
Quad.r = 0;        % Initial r value (angular rate rotation about Z BF, radians/s)

% Desired variables
Quad.X_des_GF = 1;         % desired value of X in Global frame
Quad.Y_des_GF = 1;         % desired value of Y in Global frame
Quad.Z_des_GF = 1;         % desired value of Z in Global frame
Quad.X_des = 0;            % desired value of X in Body frame
Quad.Y_des = 0;            % desired value of Y in Body frame
Quad.Z_des = 0;            % desired value of Z in Body frame

Quad.phi_des = 0;          % desired value of phi (radians)
Quad.theta_des = 0;        % desired value of theta (radians)
Quad.psi_des = 0;          % desired value of psi (radians)

% Measured variables
Quad.X_meas = 0;
Quad.Y_meas = 0;
Quad.Z_meas = 0;
Quad.phi_meas = 0;
Quad.theta_meas = 0;
Quad.psi_meas = 0;

% Disturbance Variables
Quad.Z_dis = 0;            % Disturbance in Z direction
Quad.X_dis = 0;            % Disturbance in X direction
Quad.Y_dis = 0;            % Ddisturbance in Y direction
Quad.phi_dis = 0;            % Disturbance in Yaw direction
Quad.theta_dis = 0;            % Disturbance in Pitch direction
Quad.psi_dis = 0;            % Disturbance in Roll direction

% Control Inputs
Quad.U1 = 0;       % Total thrust (N)
Quad.U2 = 0;       % Torque about X axis BF (N-m)
Quad.U3 = 0;       % Torque about Y axis BF (N-m)
Quad.U4 = 0;       % Torque about Z axis BF (N-m)

% Control Limits (update values)
Quad.U1_max = 43.5;   % Quad.KT*4*Quad.max_motor_speed^2
Quad.U1_min = 0;      % 
Quad.U2_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U2_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U3_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U3_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U4_max = 2.25; % Quad.Kd*2*Quad.max_motor_speed^2
Quad.U4_min = -2.25;% Quad.Kd*2*Quad.max_motor_speed^2

% PID parameters
Quad.X_KP = .95;          % KP value in X position control
Quad.X_KI = .25;            % KI value in X position control
Quad.X_KD = -.7;         % KD value in X position control
Quad.X_KI_lim = .25;         % Error to start calculating integral term

Quad.Y_KP = 0.95;          % KP value in Y position control
Quad.Y_KI = .3;            % KI value in Y position control
Quad.Y_KD = -.7;         % KD value in Y position control
Quad.Y_KI_lim = .25;         % Error to start calculating integral term

Quad.Z_KP = 5;    % KP value in altitude control
Quad.Z_KI = 1;    % KI value in altitude control
Quad.Z_KD = -5;  % KD value in altitude control
Quad.Z_KI_lim = .25;         % Error to start calculating integral term

Quad.phi_KP = 8;      % KP value in roll control 2
Quad.phi_KI = 1;       % KI value in roll control   1        
Quad.phi_KD = -0.5;     % KD value in roll control  -.5
Quad.phi_max = pi/4;   % Maximum roll angle commanded
Quad.phi_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

Quad.theta_KP = 8;    % KP value in pitch control 2
Quad.theta_KI = 1;     % KI value in pitch control 1
Quad.theta_KD = -0.5;   % KD value in pitch control -.5
Quad.theta_max = pi/4; % Maximum pitch angle commanded
Quad.theta_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

Quad.psi_KP = 6;     % KP value in yaw control
Quad.psi_KI = .75;     % KI value in yaw control .75
Quad.psi_KD = -.5;     % KD value in yaw control -.5
Quad.psi_KI_lim = 8*(2*pi/360);  % Error to start calculating integral 

Quad.p_KP = 3;    % KP value in pitch control 2
Quad.p_KI = 0.2;     % KI value in pitch control
Quad.p_KD = -0.05;   % KD value in pitch control -.5
Quad.p_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.p_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 

Quad.q_KP = 3;    % KP value in pitch control
Quad.q_KI = 0.2;     % KI value in pitch control
Quad.q_KD = -0.05;   % KD value in pitch control -.5
Quad.q_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.q_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 

Quad.r_KP = 3;    % KP value in pitch control
Quad.r_KI = 0.2;     % KI value in pitch control
Quad.r_KD = -0.05;   % KD value in pitch control
Quad.r_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.r_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 
	
	
	         Host ID: DISK_SERIAL_NUM=be57ad4a         Release: R2017b         Windows User Name: kotvytsk