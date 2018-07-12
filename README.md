# camera_calib
This is a camera calibration toolbox based off the theory discussed [here](http://justinblaber.org/camera-calibration-theory/), which is mostly based on [Bouguet's toolbox](http://www.vision.caltech.edu/bouguetj/calib_doc/) and Zhang's camera calibration paper. A more in depth example is given [here](http://justinblaber.org/camera-calibration-application/).

Why did I write this toolbox? Simply because I wanted to know how camera calibration works. 

# Installation instructions:
1) Clone the repo:
```
git clone https://github.com/justinblaber/camera_calib.git
```

# Examples:
Some examples are already included in the toolbox in the `tests` directory:

## single1.m
This example should yeild the following:
<p align="center">
  <img src="https://i.imgur.com/J86Myd5.png">
</p>

## stereo1.m
This example should yeild the following:
<p align="center">
  <img src="https://i.imgur.com/h5DgxT0.png">
</p>

## dot_vision.m
This example does something a little extra; it detects four fiducial markers automagically:
<p align="center">
  <img src="https://i.imgur.com/jGVCtwT.png">
</p>

Then, it uses these four points to calibrate the cameras automatically:
<p align="center">
  <img src="https://i.imgur.com/vz1wRZa.png">
</p>
