%% Clear
clear, clc;
f1 = figure(1);

%% Set images, load config, and get four corners - stereo
cb_img_paths = {'test_images/left03.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration board config file
cb_config = util.load_cb_config('board_stereo.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(1,2,1,'parent',f1));

% Get four points in image coordinates per calibration board image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cb_config);

        % Board 1 
        four_points_is{1} = [240 140;
                             187 257;
                             540 232;
                             499 374];
                           
        % Refine
        for i = 1:length(four_points_is)
            four_points_is{i} = alg.refine_points(four_points_is{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_is{i},cb_config), ...
                                                  cb_config);    %#ok<SAGROW>
        end   
end

%% Set images, load config, and get four corners - zhang
cb_img_paths = {'test_images/Image5.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration board config file
cb_config = util.load_cb_config('board_zhang.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(1,2,1,'parent',f1));

% Get four points in image coordinates per calibration board image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cb_config);

        % Board 1 
        four_points_is{1} = [89 193;
                             403 428;
                             223 42;
                             433 217];
                           
        % Refine
        for i = 1:length(four_points_is)
            four_points_is{i} = alg.refine_points(four_points_is{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_is{i},cb_config), ...
                                                  cb_config);    %#ok<SAGROW>
        end   
end

%% Refine points

% Get homographies for four points -----------------------------------%
[board_points_w, four_points_w] = alg.cb_points(cb_config);

homographies_four_points = {};
for i = 1:length(cb_imgs)
    homographies_four_points{i} = alg.homography(four_points_w, ...
                                                 four_points_is{i}, ...
                                                 cb_config);  %#ok<SAGROW>
end

% Refine points ------------------------------------------------------%
% Apply homography to points, then refine them
board_points_is = {};
for i = 1:length(cb_imgs)
    board_points_is{i} = alg.apply_homography(homographies_four_points{i}, ...
                                              board_points_w);  %#ok<SAGROW>
end

% Refine points
for i = 1:length(cb_imgs)    
    board_points_is{i} = alg.refine_points(board_points_is{i}, ...
                                           cb_imgs(i), ...
                                           homographies_four_points{i}, ...
                                           cb_config);  %#ok<SAGROW>
end

debug.plot_cb_refine_points(board_points_is{i}, ...
                            cb_imgs(i), ...
                            alg.homography(board_points_w,board_points_is{i},cb_config), ...
                            cb_config);

                        
%% test

p = [10 15];
WF = 0.25;
type = 1;

switch type
    case 1
        p1_dir = [-1 -1];
        p2_dir = [-1  1];   
    case 2
        p1_dir = [-1 -1];
        p2_dir = [ 1 -1];   
    case 3
        p1_dir = [-1  1];
        p2_dir = [ 1  1];  
    case 4 
        p1_dir = [ 1 -1];
        p2_dir = [ 1  1];  
end

% DEBUGGING ---%
% Get p1 and p2
p1 = [p(1)+p1_dir(1)*WF p(2)+p1_dir(2)*WF];
p2 = [p(1)+p2_dir(1)*WF p(2)+p2_dir(2)*WF];
        
p1_prime = alg.apply_homography(h,p1);
p2_prime = alg.apply_homography(h,p2);

x1_prime= p1_prime(1);
x2_prime= p2_prime(1);

y1_prime= p1_prime(2);
y2_prime= p2_prime(2);

% Set distance
D = sqrt((x2_prime-x1_prime)^2 + (y2_prime-y1_prime)^2);
% DEBUGGING ---%

% Equations boil down to 4th order polynomial
a = p1_dir(1)*h(1,1)+p1_dir(2)*h(1,2);
b = p2_dir(1)*h(1,1)+p2_dir(2)*h(1,2);
c = p1_dir(1)*h(2,1)+p1_dir(2)*h(2,2);
d = p2_dir(1)*h(2,1)+p2_dir(2)*h(2,2);
e = p1_dir(1)*h(3,1)+p1_dir(2)*h(3,2);
f = p2_dir(1)*h(3,1)+p2_dir(2)*h(3,2);
j = h(1,1)*p(1)+h(1,2)*p(2)+h(1,3);
k = h(2,1)*p(1)+h(2,2)*p(2)+h(2,3);
l = h(3,1)*p(1)+h(3,2)*p(2)+h(3,3);
r = roots([D^2*f^2*e^2-(a*f-e*b)^2-(c*f-e*d)^2 ...
           2*D^2*f*e*(l*f+l*e)-2*(a*f-e*b)*(f*j+l*a-e*j-l*b)-2*(c*f-e*d)*(f*k+l*c-e*k-l*d) ...
           2*D^2*l^2*f*e+D^2*(l*f+l*e)^2-(f*j+l*a-e*j-l*b)^2-(f*k+l*c-e*k-l*d)^2 ...
           2*D^2*l^2*(l*f+l*e) ...
           D^2*l^4]);
       
% Get smallest, real, and positive root to get window_factor
WF = min(r(arrayfun(@(x)isreal(x(1)),r) & r > 0));



