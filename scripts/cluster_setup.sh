### NOTE: I had to make a softlink from .pip to /dev/null/0 to stop AFS from complaining ###
### NOTE: I had to symlink .keras to a place on /cvgl2/u/amandlek ###

# TODO: replace with your own path
cd /cvgl2/u/amandlek
mkdir imitation
mkdir installed_libraries
cd installed_libraries

# get anaconda
wget https://repo.continuum.io/archive/Anaconda2-4.3.0-Linux-x86_64.sh

# TODO: instructions for running this script
# make sure you provide path: /cvgl2/u/amandlek/installed_libraries/anaconda2
bash Anaconda2-4.3.0-Linux-x86_64.sh

# TODO: some basic bashrc settings that are needed
nano ~/.bashrc.user
export PATH=/cvgl2/u/amandlek/installed_libraries/anaconda2/bin:$PATH
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export PYTHONPATH="${PYTHONPATH}:/cvgl2/u/amandlek/installed_libraries/bullet3/build_cmake/examples/pybullet"

# create conda env
conda update conda
conda create -n infogail pip

source activate infogail
export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.12.1-cp27-none-linux_x86_64.whl
pip install --upgrade $TF_BINARY_URL
pip install keras==1.2.2
pip install gym
conda install -c menpo opencv
cd ../imitation
git clone https://github.com/YunzhuLi/InfoGAIL.git
# TODO: include our own InfoGAIL "robot" files here...
pip install -U pip
pip install IPython
pip install h5py
pip install matplotlib
pip install -U openvr
pip install redis

### DO openvr fix here

### Important: open /cvgl2/u/amandlek/installed_libraries/anaconda2/envs/infogail/lib/python2.7/site-packages/openvr/__init__.py
#   Delete lines 37-38, replace with the following:
# # Load library
# if platform.system() == 'Windows':
#     # Add current directory to PATH, so we can load the DLL from right here.
#     os.environ['PATH'] += os.pathsep + os.path.dirname(__file__)
# else:
#     _openvr_lib_name = os.path.join(os.path.dirname(__file__), _openvr_lib_name)

cd ../installed_libraries
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
./build_cmake_pybullet_double.sh

cd ..
git clone https://github.com/JulianYG/perls.git
cd perls
git checkout deploy
cd ..

# EDIT commands with your username
ln -s /cvgl2/u/amandlek/installed_libraries/bullet3/data/ /cvgl2/u/amandlek/installed_libraries/perls/data
ln -s /cvgl2/u/amandlek/installed_libraries/perls /cvgl2/u/amandlek/installed_libraries/anaconda2/envs/infogail/lib/python2.7/site-packages/perls

