## File Structure

- URDF & SDF files: `$bullet\data`

- Docs: `$bullet\docs`

- Demo projects: `$bullet\build3\vs2010`

- Import filters: `$bullet\examples`

- Executables (built demo projects): `$bullet\bin`

- Pybullet DLL: `~\AppData\Local\Programs\Python\Python36-32\DLLs\pybullet.pyd`

## Visual Studio

[Hello World (on GUI)](http://bulletphysics.org/mediawiki-1.5.8/index.php/Creating_a_project_from_scratch#Download_and_install_CMake)

[Picking Objects](http://bulletphysics.org/mediawiki-1.5.8/index.php/Picking)

Scene Render: `PhysicsServerCommandProcessor::renderScene()`      PhysicsServerCommandProcessor.cpp     Ln 3852
Initialization: `PhysicsServerCommandProcessor::createDefaultRobotAssets()`      PhysicsServerCommandProcessor.cpp     Ln 4145

Current Behaviors: right controller button shoots small spheres, right controller trigger controls grip, left controller links with the robot, and menu button resets the scene.

TODO: Object tracking coordinates maybe in `stepSimulation()`

## Pybullet

### Installation: 
[Using Pybullet](http://bulletphysics.org/mediawiki-1.5.8/index.php/Using_pybullet)

There are 3 modes of pybullet physics servers, GUI, Shared_Memory, and UDP server. Shared_Memory is the one we are using for SteamVR. To connect with pybullet, under `$bullet\examples\pybullet`, import pybullet and type `p.disconnect()` to reset all previous connections. Run App_SharedMemoryPhysics_VR demo under `$bullet\bin` (recommended),  and call `p.connect(p.SHARED_MEMORY)` to connect. If VR executable is not running, there will be a connection failure. Use pybullet API to manipulate.

You can connect to the VR app using pybullet to add robots from URDF or SDF files, joint actuator/motor control etc.

### Setup:
Currently modifying `App_SharedMemoryPhysics_VR` project under `0_Bullet3Solution` to setup our own environment. Shared memory is defined. Controller buttons behavior: `pr2_gripper (bodyID = 0)`? Only need to define ground plane and gripper behavior in cpp files. The rest can be added through pybullet, as demonstrated in Desktop python script.

This readme file is composed by two parts. First part is instructions on using the built VR environment, and the second part is documentation on three relevant scripts: `model.py`, `VRcontrol.py`, and `engine.py`.

## Demo instructions

Usage:

`python engine.py -s <simulator> -m <mode> -v <video> -d <delay> -t <task>`

There are currently four types of simulator executable from engine.py. kukaSimulator has two kaka arms with 6DOF VR controller mapping, which aims to simulate the motion control of human arms. The graspSimulator is deprecated. The demoSimulator is used for grasping task data collection, and the simSimulator is a single kaka arm without gripper; instead it’s equipped with a magnetic end effector, which is able to attach objects from contact.

To use the simulators, first initialize by calling their constructors as in the script. The second input argument is the task input, represented by a list of object and pose tuples. To record, enter the name of file you want to save as in the record method. In bullet3/bin, execute the AppsSharedMemoryVR.exe, then call engine.py in command line. Press ctrl+C to stop recording. The record file will can be replayed using replay function. 

General operations: trigger for closing grippers, touch pad for engaging arms.

Notes:

- For demoSimulator, the AppsSharedMemoryVR.exe VR interface must be closed and reopened for every different record. Other simulators will automatically reset the scene with ctrl+C.
- For kukaSimulator, two controllers need to be both turned on before running engine.py. Hold the circle button in the center to engage the arms, release for disengage. 
- For all other simulators, it is suggested to just keep one controller turned on, in order to avoid the interference from the idle controller to the arm.

### Code instructions

The file `model.py` has the abstract classes `BulletPhysicsVR` and `KukaArmVR` for all VR simulators. Class hierarchy: 
BulletPhysicsVR -> KukaArmVR 
KukaArmVR -> KukaSingleArmVR, KukaDoubleArmVR
BulletPhysicsVR -> PR2GripperVR, DemoVR

BulletPhysicsVR callable methods:

`replay(file, delay)`

`record(file, video)`

`set_camera_view(focusX, focusY, focusZ, roll, pitch, yaw, focal_len)`

Features on demoSimulator: change the boundary from red to green after placing the item inside; add constraint to the items inside boundary; add explicit labels on the items and boundary lines.

### TODO
Fix Sawyer IK, try new PR2 simulator, fix grasp replay.

