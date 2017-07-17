# hacd-collision
### Authors: Viraj Mehta and Yurong You

Collision Detection via Convex Decomposition
Computes and generates the approximate convex decomposition meshes of an arbitrary 3D object mesh (in `.obj` format), along with a `.urdf` file that can be loaded directly into pybullet.

## Usage: 
```
python decompose.py --input object_to_decompose.obj --putputdir output_folder_name [--bodyname name] [--scale scale] [--rgba rgba] [--mass mass] 
```

Output: fill the folder specified with meshes for a convex decomposition of the object and a URDF file specifying the geometric properties etc of the object

## Dependencies:

* Ubuntu 12.04+
* [V-HACD](https://github.com/kmammou/v-hacd) (a library for convex decomposition).
We already compiled a binary called testVHACD that decompose is pointed towards, but you can install your own with cmake if you want.

* [meshconv](http://www.patrickmin.com/meshconv/) (a library for converting .wrl to .obj).
This is not open source, but binaries for all OSes are here.

## TODOs:
* [ ] figure out how to use `meshlabserver` to compute the mass center and the inertia matrix of an object
 
  Currently we have not found out how to use the CLI to compute the mass center and the inertia matrix of an object. So in the generated `.urdf` file, these values are set as default currently. You can do it manually as follows:

   * install [meshlab](https://www.meshlab.net) 
   * open meshlab, load the `.obj` file into it
   * adjust the scale of the object if you want by `filters -> Normals, Curvatures and Orientation -> Transform: Scale, Normalize`
   * compute the mass center and the inertia matrix by `filters -> Quality Measure and Computations -> Compute Geometric Measures`, then you can obtain infos on the right. (It does not work for objects which are not watertight)
   * fill the info into the `.urdf` file
