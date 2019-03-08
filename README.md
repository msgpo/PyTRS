This is a port of [ULgRobotics/trs](https://github.com/ULgRobotics/trs) for python3. The project uses [robopy](https://pypi.org/project/robopy/), a python port of Peter Corke's toolbox. An adaptation of the V-Rep python bindings (BSD License 2.0) are included in the project, so you do not need to add them. 

### Usage

To run the YouBot demonstration script:
- Open house.ttt in vrep
- Run simRemoteApi.start(19998) in the vrep LUA console
- Run youbot_demo.py

### Project structure

The structure was heavily modified from ULgRobotics/trs:
- Files related to the environment were moved from the `youbot` folder to a new `world` folder.
- The `matlab` folder was renamed and merged to the `youbot` folder, which is now a python package that interfaces the YouBot.
- The `focused` folder was renamed and put to the root as `demos`.
- Many boilerplate or no longer relevant files were removed:
  - `youbot/binding_test.m` and `youbot/binding_test.ttt`, these tests are included in `demos/youbot_demo.y`. 
  - `youbot/cfg-to-matlab.py` and `youbot/instructions.mat`, as we can simply use the `youbot/instructions.cfg` file in python directly.
  - `youbot/youbot.m` was renamed and moved to: `demos/youbot_demo.y`.
  - `matlab/vrchk.m` was ported in python and moved to `vrep/vrchk.py` (it is now called everytime a VRep function returns a status value and it consumes the value; i.e. you do not need to worry about checking the return status in your code).
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
+-- demos           Scripts that demonstrate interaction with YouBot
|   +-- youbot_demo.py  Demonstration code that interacts with the bot
|   +-- ...
+-- m2py.py         Script to convert matlab code to python
```