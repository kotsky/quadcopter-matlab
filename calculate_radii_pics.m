function [ R_min_pic, R_max_pic ] = calculate_radii_pics( altitude, cam_resolution,K_opt, real_Radii )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
altitude = (altitude); %convert to real 

Radii_pic = K_opt*cam_resolution*real_Radii/altitude ;

R_max_pic = Radii_pic + 10; %+ 5 * 1/altitude
R_max_pic = round(R_max_pic);
R_min_pic = Radii_pic - 10; %- 5 * 1/altitude
R_min_pic = round(R_min_pic);


if R_min_pic <= 10
    R_min_pic = 10;
    R_max_pic = 25;
end
if R_max_pic >= 100
    R_max_pic = 100;
    R_min_pic = 100 - 20;
end

end


