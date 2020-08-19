# KilobotArena
## Augmented Reality for Kilobots (ARK)
### Installation

A GUI for running experiments using Kilobot Smart Arena with four tracking cameras

Ubuntu is the preferred OS for ARK

KilobotArena is a Qt program, and therefore uses the Qt build tools to generate the make file. There are two ways to do this - in both cases you need a recent version of Qt [v.5.6+](www.qt.io) installed - on Ubuntu the version in aptitude should do (`sudo apt-get install qtcreator` should install everything from Qt that you need).

You also need a CUDA supporting version of OpenCV 3 - you'll need to compile this yourself - here's a [guide](https://gist.github.com/filitchp/5645d5eebfefe374218fa2cbf89189aa) that should work. 

Now either use the QtCreator gui you installed in the previous step (click the hammer button to build - it will prompt you to choose), or run `qmake` in the ARK directory to generate the Makefile. There are lots of online guides to help if you have trouble [e.g.](http://doc.qt.io/qtcreator/creator-building-targets.html)

QtCreator will run the Makefile for you - from the command line you'll have to do it yourself. If you get build errors you may need to change the path to OpenCV in the .pro file (the syntax is quite simple).

You also need to install the calibration program to generate calibrated camera maps: [KilobotArenaCalibration](https://github.com/DiODeProject/KilobotArenaCalibration).

### User permission
In order to operate the Kilobot's OHC, the user needs to be part of the dialout group. Therefore, add the user to the group `dialout` with command

```
sudo usermod -a -G dialout <user-name>
```

### Citation

If you use or adapt ARK in order to generate experimental results, please cite the following paper in any resulting publications:

* Reina A., Cope A.J., Nikolaidis E., Marshall J.A.R., Sabo C. (2017) ARK: Augmented reality for Kilobots. *IEEE Robotics and Automation Letters* **2, 1755-1761**.

### See Also

* ARK makes use of a redesigned overhead controller: [ARK_OHC](https://github.com/DiODeProject/ARK_OHC)
* [Kilobot Wiki](http://diode.group.shef.ac.uk/kilobots/index.php/Kilobots)
