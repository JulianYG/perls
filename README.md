## perls

### System Design
![Block Diagram](structure.jpg)

Read more about API docs in this [Wiki](https://github.com/JulianYG/perls/wiki).

## Bullet Instructions

### File Structure

- URDF & SDF files: `$bullet\data`

- Docs: `$bullet\docs`

- Demo projects: `$bullet\build3\vs2010`

- Import filters: `$bullet\examples`

- Executables (built demo projects): `$bullet\bin`

- Pybullet DLL (Windows): `~$Python\DLLs\pybullet.pyd`

### Visual Studio

[Hello World (on GUI)](http://bulletphysics.org/mediawiki-1.5.8/index.php/Creating_a_project_from_scratch#Download_and_install_CMake)

[Picking Objects](http://bulletphysics.org/mediawiki-1.5.8/index.php/Picking)

Scene Render: `PhysicsServerCommandProcessor::renderScene()`      PhysicsServerCommandProcessor.cpp     Ln 3852
Initialization: `PhysicsServerCommandProcessor::createDefaultRobotAssets()`      PhysicsServerCommandProcessor.cpp     Ln 4145

Current Behaviors: right controller button shoots small spheres, right controller trigger controls grip, left controller links with the robot, and menu button resets the scene.

TODO: Object tracking coordinates maybe in `stepSimulation()`

### Pybullet

- Installation: 
[Using Pybullet](http://bulletphysics.org/mediawiki-1.5.8/index.php/Using_pybullet)

There are 3 modes of pybullet physics servers, GUI, Shared_Memory, and UDP server. Shared_Memory is the one we are using for SteamVR. To connect with pybullet, under `$bullet\examples\pybullet`, import pybullet and type `p.disconnect()` to reset all previous connections. Run App_SharedMemoryPhysics_VR demo under `$bullet\bin` (recommended),  and call `p.connect(p.SHARED_MEMORY)` to connect. If VR executable is not running, there will be a connection failure. Use pybullet API to manipulate.

You can connect to the VR app using pybullet to add robots from URDF or SDF files, joint actuator/motor control etc.

- Setup:
Currently modifying `App_SharedMemoryPhysics_VR` project under `0_Bullet3Solution` to setup our own environment. Shared memory is defined. Controller buttons behavior: `pr2_gripper (bodyID = 0)`? Only need to define ground plane and gripper behavior in cpp files. The rest can be added through pybullet, as demonstrated in Desktop python script.

This readme file is composed by two parts. First part is instructions on using the built VR environment, and the second part is documentation on three relevant scripts: `model.py`, `VRcontrol.py`, and `bulletphysicsengine.py`.

### Bullet-VR Demo instructions

Usage:

For local demo, under perls/src/bullet_ `python run.py -c <configuration>`

There are currently two types of simulator executable from `ccr_engine.py`, a keyboard simulator, interfacing with keyboard,  and a VR simulator, interfacing with SteamVR. We plan to add joystick or smartphone interfaced simulators.

To use the simulators, first initialize by calling their constructors as in the script. The second input argument is the task input, represented by a list of object and pose tuples. To record, enter the name of file you want to save as in the record method. In bullet3/bin, execute the AppsSharedMemoryVR.exe, then call run.py in command line. Press ctrl+C to stop recording. The record file will can be replayed using replay function.

The simulators are executed upon models. Current models include pr2 gripper, and robot. Robot is an abstract class that are implemented as multiple concrete classes, such as kuka, sawyer, and baxter. The models are also used for the openAI gym implementation of pybullet, cascading the learning layer with physical layer.

General operations: trigger for closing grippers, touch pad for engaging arms.

Notes:

- For VR Simulator, controllers cannot be turned on after running `run.py`. Hold the circle button in the center to engage the arms, release for disengage. 
- For all other simulators, it is suggested to just keep one controller turned on, in order to avoid the interference from the idle controller to the arm.

Simulator callable methods:

`render_simulator(agent, interface, task, filename, record=True, vr=False)`

