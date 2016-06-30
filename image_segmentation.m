%%------------------------------------------------
%% Convert it to binary colors 
% Blob analysis
% Get the control point co-ordinates
% returns image with features extracted and the co-ordinates of feature
% mid-points
%------------------------------------------------

function [im_feature,co_ord]=image_segmentation (im_data)



   
    
    data = im_data;
 diff_im=data ;
     diff_im = rgb2hsv(data);
%        imshow(diff_im) ;
    %      diff_im = medfilt2(data);
    idxSat = (diff_im(:,:,2) > 0.6);
    idxVal = (diff_im(:,:,3) > 0.3 & diff_im(:,:,3) < 0.8);
    diff_im = (idxVal & idxSat);
%     diff_im = im2bw(diff_im,0.7003);
    diff_im = imcomplement(diff_im);
    se = strel('disk',3);
    diff_im=imerode(diff_im,se) ;
    diff_im=imdilate(diff_im,se) ;
    se = strel('disk',6);
    diff_im=imdilate(diff_im,se) ;
    diff_im=imerode(diff_im,se) ;
%             imshow(data) ;
    diff_im=imcomplement(diff_im) ;
    imshow(diff_im)
    
    stats = regionprops('table',diff_im,'Centroid',...
        'MajorAxisLength','MinorAxisLength')
    centers = stats.Centroid;
    imshow(diff_im) ;
    diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    
    radii = diameters/2;
    


    hold on 
    
    viscircles(centers,radii,'LineStyle','--');
    
  hold off ;
  co_ord = centers;
  im_feature = diff_im;
  
end