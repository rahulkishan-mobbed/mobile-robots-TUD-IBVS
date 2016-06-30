
%% ----------------------------------------------------
% control signals calculation
% input - error (8x1)
% output - velocity and turn angle
%% ----------------------------------------------------
function [velocity,turnangle,error_robot] = velocitycontrols(error,Lx)
Kx=0.2 ;
Ky=-0.02 ;
eta=0.01 ;
flag=1 ;
threshold_x=0.2 ;
threshold_y=0.2 ;

error_camera_frame=Lx*error ;
error_x_robot=error_camera_frame(1)/1000 ;
error_y_robot=error_camera_frame(2)/1000 ;
error_angular_robot=error_camera_frame(3) ;

velocity=Kx*error_x_robot ;

k_theta=Ky/(abs(error_y_robot)+eta) ;

if(abs(error_x_robot)> threshold_x  &&  abs(error_y_robot)> threshold_y) 
   flag=0; 
    
end

turnangle=(Ky*error_y_robot)+ (flag*k_theta*error_angular_robot) ;
turnangle=0 ;
error_robot=[error_x_robot;error_y_robot;error_angular_robot] ;

end