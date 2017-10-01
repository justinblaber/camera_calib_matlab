%% Clear
clear, clc;

%% Set images
cb_img_paths = {'images/Image1.tif', ...
                'images/Image2.tif', ...
                'images/Image3.tif', ...
                'images/Image4.tif', ...
                'images/Image5.tif', ...
                'images/Image6.tif', ...
                'images/Image7.tif', ...
                'images/Image8.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
%% Load calibration config file
calib_config = util.load_calib_config('configs/single.conf');

%% Get four points in pixel coordinates per calibration board image
four_points_ps{1} = [168.7272  179.7808
                     133.9287  377.5377
                     413.8812  194.8252
                     472.3155  386.7228];

four_points_ps{2} = [172.3350  108.0685
                     138.0881  379.0046
                     444.4066  129.1560
                     481.5573  389.7946];

four_points_ps{3} = [199.7681  98.1806
                     117.8841  354.4820
                     472.0018  142.9956
                     460.8990  413.7783]; 

four_points_ps{4} = [250.2379  114.2989
                     145.3761  361.4282
                     534.0855  89.3777
                     509.4070  393.5712]; 

four_points_ps{5} = [82.6358   195.8193
                     403.7691  430.2562
                     223.6562  44.6192
                     432.8029  218.4991]; 

four_points_ps{6} = [91.4560   129.4162
                     117.4309  398.5697
                     535.8790  172.1372
                     423.0276  413.6498]; 

four_points_ps{7} = [181.9291  128.9180
                     165.0031  453.1418
                     481.8723  96.0359
                     406.0600  351.5853]; 

four_points_ps{8} = [72.2896   102.1230
                     88.7938   426.2946
                     371.3808  66.0314
                     325.0443  322.2007]; 

%% Perform single calibration
calib = alg.single_calib_four_points(cb_imgs, ...
                                     four_points_ps, ...
                                     calib_config);

%% Save calibration
util.write_single_calib(calib,'calibrations/single1.txt');
                    
%% Read calibration
clear;

calib = util.read_single_calib('calibrations/single1.txt');

%% Debug with gui
f = figure(1);
debug.gui_single_calib(calib, f);
