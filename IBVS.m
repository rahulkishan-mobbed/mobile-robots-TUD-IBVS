%% Interaction matrix
% Constant
% Calculating the L-matrix

Z = 1*10^3;           % constant distance, 1px is 2.8 micro metre
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

x = ((X-cu)/f)*2.8*10^(-3);    % converting to mm
y = ((Y-cv)/f)*2.8*10^(-3);



Matrix_z = (-1/Z)*ones(4,1);
Matrix_12 = x./Z;
Matrix_13 = -1.*(1+x.^2);
Matrix_22 = -y./Z;
Matrix_23 = -x.*y;
Lx = [Matrix_z Matrix_12 Matrix_13;...
    zeros(4,1) Matrix_22 Matrix_23];
Lx_inv = pinv(Lx);


%% Get image from Kinect 
handles.colorImgSub = exampleHelperTurtleBotEnableColorCamera;
handles.cliffSub = rossubscriber('/mobile_base/events/cliff', 'BufferSize', 5);
handles.bumpSub = rossubscriber('/mobile_base/sensors/bumper_pointcloud', 'BufferSize', 5);
handles.soundPub = rospublisher('/mobile_base/commands/sound', 'kobuki_msgs/Sound');
handles.velPub = rospublisher('/mobile_base/commands/velocity');
handles.camInfo=rossubscriber('/camera/rgb/image_color','sensor_msgs/CameraInfo') ;

while(1)
latestImg = readImage(handles.colorImgSub.LatestMessage);
    data=latestImg ;
%% get co-ordinates of the features plus the image with extracted features
% function call image_segmentation
[im_feature,co_ord]=image_segmentation(data);
%% error computation
u_actual = co_ord(:,1);
v_actual = co_ord(:,2);
u_error = u_actual - x;
v_error = v_actual - y;
error_vec = cat(1,u_actual,v_actual);

%% Robot movement control
% optimise
% function 
[vel,omega]=velocitycontrols(error_vec,Lx_inv);
end
