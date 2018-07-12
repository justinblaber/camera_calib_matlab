# camera_calib
This is camera calibration toolbox based off the theory discussed [here](http://justinblaber.org/camera-calibration-theory/). Why did I write this? Simply because I wanted to know how camera calibration works. This toolbox is based off [Bouguet's toolbox](http://www.vision.caltech.edu/bouguetj/calib_doc/) and Zhang's camera calibration paper.

# Installation instructions:
1) Clone the repo:
```
git clone https://github.com/justinblaber/camera_calib.git
```

# Examples:
Some examples are already included in the toolbox in the `test` directory.

## single1.m
This example should yeild the following:
<p align="center">
  <img src="https://i.imgur.com/yTAqcJS.png">
</p>

## stereo1.m
This example should yeild the following:
<p align="center">
  <img src="https://i.imgur.com/rKG7QSd.png">
</p>

## dot_vision.m
This example does something a little extra. It detects four fiducial markers automagically:
<p align="center">
  <img src="https://i.imgur.com/YfvECWY.png">
</p>

Then it uses these four points to calibrate the cameras automatically:
<p align="center">
  <img src="https://i.imgur.com/oSkPnRS.png">
</p>
