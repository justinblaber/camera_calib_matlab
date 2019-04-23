# camera_calib
This is a camera calibration toolbox. It's partly based on [Bouguet's toolbox](http://www.vision.caltech.edu/bouguetj/calib_doc/) and Zhang's camera calibration paper, but with more functionality:

* Setup is based on an input configuration file which allows for easy tweaking and modification of algorithms and allows for greater reproducibility. If you save the images, configuration file, and script, the calibration will be repeatable.
* Includes fiducial marker recognition which makes the calibration fully automatic.
* The distortion function is input as a symbolic function (via configuration file) and is therefore very easily modifiable. Two distortion functions are provided already ("heikkila97" and "wang08"); this toolbox uses symbolic differentiation to compute the updated jacobians/hessians/gradients automatically.
* Supports multi-camera calibration.
* Implements both "distortion refinement" and "frontal refinement" techniques.
* Supports multiple calibration board targets (checkers, circles, etc...) and correctly accounts for "center of ellipse" vs "projected center of circle" for circular targets depending on the type of calibration (i.e. "frontal refinement" or "distortion refinement").
* Supports custom calibration board geometries by overriding an abstract calibration board geometry class.
* Supports (optional) covariance optimization (i.e. generalized least squares) based on uncertanties computed during target localization.
* Supports calibration board going partially "out of frame" which improves robustness and allows for bigger calibration targets to be used.
* Code is organized and documented and also utilizes oriented principles for code reuse.

Some of the theory is discussed [here](http://justinblaber.org/camera-calibration-theory/).

# Installation instructions:
1) Clone the repo:
```
git clone https://github.com/justinblaber/camera_calib.git
```

# Example:
1. First, download images/config/script from [here](http://justinblaber.org/downloads/github/camera_calib/dot_vision_stereo.zip).
2. Unzip, navigate to the folder in matlab, and check the configuration script:
```
% Calibration board target
target              = checker
target_optimization = edges

% Calibration board geometry
height_cb           = 50
width_cb            = 50
num_targets_height  = 16
num_targets_width   = 16
target_spacing      = 2.032
height_fp           = 42.672
width_fp            = 42.672
obj_cb_geom         = class.cb_geom.csgrid_cfp

% Calibration optimization
calib_optimization  = distortion_refinement
```
The targets are checkers, and the target optimization is a cool "edges" refinement algorithm from Mallon07. The calibration board geometry class used is "csgrid_cfp" which is a centered target grid with centered "four point" fiducial markers.

3. Next, open the `dot_vision_stereo.m` script in matlab, which should include:
```
%% Set environment
addpath('~/camera_calib');

%% Read calibration config
calib_config = intf.load_calib_config('dot_vision.conf');

%% Stereo calibration

img_cb_paths.L = {
'16276941_2018-07-12_01:34:06_883766_1_L.png', ...
'16276941_2018-07-12_01:34:19_464618_2_L.png', ...
'16276941_2018-07-12_01:34:37_254931_3_L.png', ...
'16276941_2018-07-12_01:34:54_380623_4_L.png', ...
'16276941_2018-07-12_01:35:03_580397_5_L.png', ...
'16276941_2018-07-12_01:35:23_088945_6_L.png', ...
'16276941_2018-07-12_01:35:42_846897_7_L.png', ...
'16276941_2018-07-12_01:35:52_379587_8_L.png', ...
'16276941_2018-07-12_01:35:59_916877_9_L.png', ...
'16276941_2018-07-12_01:36:10_280953_10_L.png'
};

img_cb_paths.R = {
'16276942_2018-07-12_01:34:06_885104_1_R.png', ...
'16276942_2018-07-12_01:34:19_465629_2_R.png', ...
'16276942_2018-07-12_01:34:37_255926_3_R.png', ...
'16276942_2018-07-12_01:34:54_382094_4_R.png', ...
'16276942_2018-07-12_01:35:03_581977_5_R.png', ...
'16276942_2018-07-12_01:35:23_090054_6_R.png', ...
'16276942_2018-07-12_01:35:42_848085_7_R.png', ...
'16276942_2018-07-12_01:35:52_380698_8_R.png', ...
'16276942_2018-07-12_01:35:59_918005_9_R.png', ...
'16276942_2018-07-12_01:36:10_282139_10_R.png'
};

% Validate all calibration board images
imgs_cbs = intf.validate_stereo_path_imgs(img_cb_paths);

% Get four points
[p_fpss, debug_fp] = intf.fp_detect(imgs_cbs, calib_config);

intf.gui_fp_detect(p_fpss, imgs_cbs, debug_fp, calib_config);

% Perform calibration
calib = intf.calib_fp(imgs_cbs, p_fpss, calib_config);

intf.gui_calib(calib);
```
Make sure to modify `addpath('~/camera_calib');` if your installation path isnt in your home directory. Go ahead and run this script.

The first figure to appear should be:
![four point detector](https://i.imgur.com/tAv8Cxc.png)

This gui is useful for debugging the detection of the four fiducial markers. You can toggle through the images by pressing the left and right arrow keys.

The next figure should be:
![calibration](https://i.imgur.com/VM8yBbW.png)

This gui is useful for debugging the calibration. The first thing to do is to double check the calibration board geometry to make sure it's correct. Next, check the residuals to make sure they are reasonably small. I would also rotate the extrinsics to confirm the relative pose of the cameras makes sense. Lastly, you can toggle "w" and zoom into the calibration point with the largest residual.

