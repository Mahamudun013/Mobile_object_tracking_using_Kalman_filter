%% This code for process the images for object tracking .

clear all;  % free up system memory
close all;  % removed specified figure

% creat the frames address as a directory
directory = 'D:\Matlab-2013a\bin\ALL_TRACKING\Person_trk\person (26-Jul-17 6-18-32 PM)';

% by using this command enter the directory
cd(directory);

% for get the list of frames
frame_list =  dir('*jpg');

%%Make average of background images

N = 30;  % num of frames to use for averaged background

%define image stack for averaging
image_avg = zeros(480,640,N); 

for i = 1:N   
    
    image_temp = imread(frame_list(i).name); % for reading given image
    image_avg(:,:,i) =image_temp(:,:,1); % use first dimension of the image

end

background_image = (mean(image_avg,3)); % take the average image
subplot(121);
imagesc(background_image)

subplot(122);
imagesc(image_avg(:,:,1))
colormap(gray)

%%gaussian filter initialization

hsize = 20;
sigma = 10;
gaussian_filter = fspecial('gaussian',hsize, sigma); %create 2D Gaussian filter

subplot(121);
imagesc(gaussian_filter)
subplot(122);
mesh(gaussian_filter)
colormap(gray)


%for making the coordinate locations more visible.

SE = strel('diamond', 7);

% initialize the variable that will store the object locations(x,y)

Track_data = zeros(length(frame_list),2);

%% iteratively find the object

for i = 1:length(frame_list) 
   
    image_temp = double(imread(frame_list(i).name)); % load the image and convert it into double for computation. 
    image = image_temp(:,:,1); % reduce image in first dimension
    
    subplot(221);
    imagesc(image);
    title('Original');
  
    %subtract background from the image
    subtract_image = (image - background_image);
    subplot(222);
    imagesc(subtract_image);
    title('Background subtracted');
    
    %gaussian filtering for image
    gaussian_image = filter2(gaussian_filter,subtract_image,'same');     
    subplot(223);imagesc(gaussian_image);
    title('Gaussian filter smoothed');
    
    %threshold the image based on histogram
    subplot(224);
    hist(gaussian_image(:)); % create histogram
    threshold_image = (gaussian_image < -60);
    
    subplot(224);
    imagesc(threshold_image);
    title('Thresholded');
   
    
    % Tracking object and finding center of the object
  
    [x,y] = find (threshold_image); % find the nonzero value of threshold image 

    if ~isempty(x)
        
        Track_data(i,:) = ceil([mean(x) mean(y)]+1); % ceiling to avoid zero indices
   
    else
        
        Track_data(i,:) =1;
        
    end
 
  
    % make binary image with single coordinate of object

    object_image = zeros(size(threshold_image));  % crate array with zeros
    object_image(Track_data(i,1),Track_data(i,2)) = 1;  % make the object centroid is 1
    

    object_image = imdilate(object_image, SE);  % to make more visible centroid 
    subplot(224);
    imagesc(threshold_image + object_image);  % show object with centroid
    title('thresholded and point extracted');
    

    pause(0.0001)
end


%save the coordinates value
save('Track_data_store.mat', 'Track_data');


     
