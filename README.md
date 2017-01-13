## File Structure

URDF & SDF files: `$bullet\data`

Docs: `$bullet\docs`

Demo projects: `$bullet\build3\vs2010`

Import filters: `$bullet\examples`

Executables (built demo projects): `$bullet\bin`

Pybullet DLL: `~\AppData\Local\Programs\Python\Python36-32\DLLs\pybullet.pyd`

## Visual Studio

[Hello World (on GUI)](http://bulletphysics.org/mediawiki-1.5.8/index.php/Creating_a_project_from_scratch#Download_and_install_CMake)

[Picking Objects](http://bulletphysics.org/mediawiki-1.5.8/index.php/Picking)

Scene Render: `PhysicsServerCommandProcessor::renderScene()`      PhysicsServerCommandProcessor.cpp     Ln 3852
Initialization: `PhysicsServerCommandProcessor::createDefaultRobotAssets()`      PhysicsServerCommandProcessor.cpp     Ln 4145

Current Behaviors: right controller button shoots small spheres, right controller trigger controls grip, left controller links with the robot, and menu button resets the scene.

TODO: Object tracking coordinates maybe in `stepSimulation()`

## Pybullet

###Installation: 
[Using Pybullet](http://bulletphysics.org/mediawiki-1.5.8/index.php/Using_pybullet)

There are 3 modes of pybullet physics servers, GUI, Shared_Memory, and UDP server. Shared_Memory is the one we are using for SteamVR. To connect with pybullet, under `$bullet\examples\pybullet`, import pybullet and type `p.disconnect()` to reset all previous connections. Run App_SharedMemoryPhysics_VR demo under `$bullet\bin` (recommended),  and call `p.connect(p.SHARED_MEMORY)` to connect. If VR executable is not running, there will be a connection failure. Use pybullet API to manipulate.

You can connect to the VR app using pybullet to add robots from URDF or SDF files, joint actuator/motor control etc.

###Setup:
Currently modifying `App_SharedMemoryPhysics_VR` project under `0_Bullet3Solution` to setup our own environment. Shared memory is defined. Controller buttons behavior: `pr2_gripper (bodyID = 0)`? Only need to define ground plane and gripper behavior in cpp files. The rest can be added through pybullet, as demonstrated in Desktop python script.
