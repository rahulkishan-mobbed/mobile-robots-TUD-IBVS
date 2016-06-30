%% Interaction matrix
% Constant
% Calculating the L-matrix
rosshutdown ;
clear all ;
close all ;
clc ;

rosinit('129.217.130.116') ;
Z = 1*10^3;           % constant distance in mm, 1px is 2.8 micro metre
f = 575.8157*2.8*10^(-3);       %focal length
cu = 314.5;
cv = 235.5;

X = [361.97;...
    362.84;... 
    507.4;...
    506.26];
Y = [155.89;...
    236.23;...
    153.71;...
    235.4];

x = ((X-cu)/f)*2.8*10^(-3);    % converting to mm and normalize
y = ((Y-cv)/f)*2.8*10^(-3);



Matrix_z = (-1/Z)*ones(4,1);
Matrix_12 = x./Z;
Matrix_13 = -1.*(1+x.^2);
Matrix_22 = -y./Z;
Matrix_23 = -x.*y;
Lx = [Matrix_z Matrix_12 Matrix_13;...
    zeros(4,1) Matrix_22 Matrix_23];

Lx_inv = pinv(Lx);

hold on;

%% Get image from Kinect 

handles.colorImgSub = exampleHelperTurtleBotEnableColorCamera;
handles.cliffSub = rossubscriber('/mobile_base/events/cliff', 'BufferSize', 5);
handles.bumpSub = rossubscriber('/mobile_base/sensors/bumper_pointcloud', 'BufferSize', 5);
handles.soundPub = rospublisher('/mobile_base/commands/sound', 'kobuki_msgs/Sound');

handles.camInfo=rossubscriber('/camera/rgb/image_color','sensor_msgs/CameraInfo') ;
timerHandles.pub = rospublisher('/mobile_base/commands/velocity'); % Set up publisher
timerHandles.pubmsg = rosmessage(rostype.geometry_msgs_Twist);
error=[5;5;5] ;
data_ref=imread('reference_image_2.png') ;
[im_feature_ref,co_ord_ref]=image_segmentation(data_ref);
prev_centers=0 ;
loop_counter=1 ;hold on;
prev=0;
while(abs(error(1))>0.001  )
latestImg = readImage(handles.colorImgSub.LatestMessage);
    data=latestImg ;
%% get co-ordinates of the features plus the image with extracted features
% function call image_segmentation
[im_feature,co_ord]=image_segmentation(data);
if(length(co_ord) ~= 4)
co_ord=prev;

end
prev=co_ord ;

figure(2) 
clf;
hold on;
imshow(im_feature);
hold on;

plot(X,Y,'or');hold on;

plot(co_ord(:,1), co_ord(:,2), 'xg');

plot([X(1), co_ord(1,1)], [Y(1), co_ord(1,2)]);
plot([X(2), co_ord(2,1)], [Y(2), co_ord(2,2)]);
plot([X(3), co_ord(3,1)], [Y(3), co_ord(3,2)]);
plot([X(4), co_ord(4,1)], [Y(4), co_ord(4,2)]);

hold on ;
%% error computation
u_actual = co_ord(:,1);
U(loop_counter,:)=u_actual ;
v_actual = co_ord(:,2);
V(loop_counter,:)=v_actual ;
u_act_mm = ((u_actual-cu)/f)*2.8*10^(-3);    % converting to mm and normalize
v_act_mm = ((v_actual-cv)/f)*2.8*10^(-3);
u_error = -u_act_mm +x;
v_error = -v_act_mm +y;
error_vec = cat(1,u_error,v_error);

%% Robot movement control
[vel,omega,error]=velocitycontrols(error_vec,Lx_inv) 
err_x(loop_counter)=error(1) ;
vel_save(loop_counter)=vel ;

 
   timerHandles.pubmsg.Angular.Z = omega;
   timerHandles.pubmsg.Linear.X = vel;
   send(timerHandles.pub,timerHandles.pubmsg);
% optimise
% function 
loop_counter=loop_counter+1 ;
end



