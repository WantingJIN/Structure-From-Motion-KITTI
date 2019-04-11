close all; clear; clc;

%% Initialization path
addpath(genpath('./devkit'))
addpath(genpath('./data'))
base_dir = 'data/2011_09_26/2011_09_26_drive_0046_sync';
calib_dir = 'data/2011_09_26';
img_dir = 'data/2011_09_26/2011_09_26_drive_0046_sync/image_00/data/';

%% Read images

img_path_list = dir(img_dir);
img_path_list(1:2)=[];
img_num = length(img_path_list);
images=cell(1,img_num);
for j = 1:img_num  
    image_name = img_path_list(j).name;
    image = imread(strcat(img_dir,image_name)); 
    images{j}=image;
end
images = images(1:23);


%% Load camera parameters
calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
P_rect = calib.P_rect{1}; %extract P matrix after rectification
K = P_rect(:,1:3)'; % P=K[I|0]
cameraParams = cameraParameters('IntrinsicMatrix',K); %The image is already undistorted

%% Detect first image corner points

I = images{3};
[cor1,cnt1]=Harris_point_detection(I,'Prewitt',3);
[x,y]=find(cor1==1);
prevPoints=[y,x];
%show the corner points

imshow(I)
hold on
plot(y,x,'go')

%% Create a View Set Containing the First View
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
 vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
     eye(3, 'like', prevPoints), 'Location', ...
    zeros(1, 3, 'like', prevPoints));
%% Add the Rest of the Views
for i = 2:numel(images)
    i
    I_new = images{i};
    % Detect the corner point of the current point
    [cor2,cnt2]=Harris_point_detection(I_new,'Prewitt',3);
    [x,y]=find(cor2==1);
    currPoints=[y,x];    

     % Select matched points.

    [match1,match2,indexPairs] = match_correlation(I,I_new,cor1,cor2,cnt1,cnt2);
     matchedPoints1 = prevPoints(indexPairs(:, 1),:);
     matchedPoints2 = currPoints(indexPairs(:, 2),:);
%     draw_match(I,I_new,matchedPoints1,matchedPoints2)
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    [xyzPoints,reprojectionErrors] = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine the 3-D world points and camera poses. Bundle adjustment
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

%     prevFeatures = currFeatures;
      prevPoints   = currPoints;  
    cor1=cor2;
    cnt1=cnt2;
    I=I_new;
end
disp(newline)
disp(' Completed adding view sets')
%% Display Camera Poses
% Display the refined camera poses and 3-D world points.
% Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D points.
% goodIdx = (reprojectionErrors < 5);
% xyzPointsShow = xyzPoints(goodIdx, :);
xyzPointsShow = xyzPoints;

% Display the 3-D points.
pcshow(xyzPointsShow, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-25, loc1(1)+10]);
ylim([loc1(2)-5, loc1(2)+1]);
zlim([loc1(3)-1, loc1(3)+100]);
camorbit(0, -30);

title('Refined Camera Poses');
%% Compute Dense Reconstruction
% Go through the images again. This time detect a dense set of corners,
% and track them across all views using |vision.PointTracker|.

% Read and undistort the first image
% I = undistortImage(images{1}, cameraParams); 

I = images{1};
% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I, 'MinQuality', 0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);

% Store the dense points in the view set.
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

% Track the points across all views.
for i = 2:numel(images)
    % Read and undistort the current image.
%     I = undistortImage(images{i}, cameraParams); 
    i
    I = images{i};
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);
disp('Finished computing dense reconstruction')
%% Display Dense Reconstruction
% Display the camera poses and the dense point cloud.

% Display the refined camera poses.
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-25, loc1(1)+20]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+100]);
camorbit(0, -30);

title('Dense Reconstruction');
