This is a port of [ULgRobotics/trs](https://github.com/ULgRobotics/trs) for python3. The project uses [robopy](https://pypi.org/project/robopy/), a python port of Peter Corke's toolbox. An adaptation of the V-Rep python bindings (BSD License 2.0) are included in the project, so you do not need to add them. 

### Usage

To run the YouBot demonstration script:
- Open house.ttt in vrep
- Run simRemoteApi.start(19998) in the vrep LUA console
- Run demo_youbot.py

### Project structure

The structure was heavily modified from ULgRobotics/trs:
- Files related to the environment were moved from the `youbot` folder to a new `world` folder.
- The `matlab` folder was renamed and merged to the `youbot` folder, which is now a python package that interfaces the YouBot.
- The `focused` folder was removed and its contents were put to the root.
- Many boilerplate or no longer relevant files were removed:
  - `youbot/binding_test.m` and `youbot/binding_test.ttt`, these tests are included in `demos/youbot_demo.y`. 
  - `youbot/cfg-to-matlab.py` and `youbot/instructions.mat`, as we can simply use the `youbot/instructions.cfg` file in python directly.
  - `youbot/youbot.m` was renamed and moved to: `demo_youbot.py`.
  - `matlab/vrchk.m` was ported in python and moved to `vrep/vrchk.py` (it is now called everytime a VRep function returns a status value and it consumes the value; i.e. you do not need to worry about checking the return status in your code and the VRep API functions will not return it).
  - Additionally, matlab files that are done being ported in python will be deleted.

```
PyTRS
+-- vrep            V-Reps bindings as a package (see demos/youbot_demo for usage examples)
|   +-- ...
+-- vrep_tests      V-Reps test samples (not very useful, not yet ported)
|   +-- ...
+-- world           Environment files (doesn't contain code)
|   +-- ...
+-- youbot          Partial python port (unported matlab code remains)
|   +-- ...
+-- demo_youbot.py  Demonstration script that interacts with the bot
+-- demo...         Other demonstration scripts
+-- m2py.py         Script to convert matlab code to python
```

### Demos
Aside from `demo_youbot.py`, the demonstration scripts do something very specific. In order of increasing complexity: 

  * `demo_youbot_photo.m`: takes a 2D picture (standard RGB sensor)
  * `demo_youbot_3dpointcloud.m`: takes a 3D point cloud (Hokuyo sensor)
  * `demo_youbot_frames.m`: takes a 3D point cloud (Hokuyo sensor) and move it to the global reference frame (not the one of the robot)
  * `demo_youbot_moving.m`: moves the robot around
  * `demo_youbot_arm.m`: moves the robot's arm

The following scripts also show some tricks about plotting data; they are not linked to the previous scripts: 

  * `demo_plot_matrix.py`: plots a matrix (such as a map)
  * `demo_plot_multiple.py`: deals with multiple windows