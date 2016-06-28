%% ----------------------------------------------------
% control signals calculation
% input - error (8x1)
% output - velocity and turn angle
%% ----------------------------------------------------
function [velocity,turnangle] = velocitycontrols(error,Lx)
Kx=0.2 ;
Ky=0.2 ;
eta=0.01 ;
flag=1 ;
threshold_x=5 ;
threshold_y=5 ;

vel_camera_frame=Lx*error ;
error_x_robot=vel_camera_frame(2) ;
error_y_robot=vel_camera_frame(1) ;
error_angular_robot=vel_camera_frame(3) ;
velocity=Kx*error_x_robot ;

k_theta=Ky/(abs(error_y_robot)+eta) ;

if(abs(error_x_robot)> threshold_x  &&  abs(error_x_robot)> threshold_x) 
   flag=0; 
    
end

turnangle=(ky*error_y_robot)+ (flag*k_theta*error_angular_robot) ;



end