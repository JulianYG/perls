from cx_Freeze import setup, Executable

base = None

executables = [Executable("demo.py", base=base)]

paths = ['../perls/bullet_', '../perls', '../perls/bullet_/simulation', 
	'../perls/comm', '../perls/bullet_/simulation/utils',
	'c:/users/vive/appdata/local/programs/python/python36-32/lib/site-packages', ]

packages = ["pybullet", "os", "glob", "redis", "gym_", "numpy", "matplotlib", "csv"]
options = {
    'build_exe': {
        'packages':packages,
        'path': paths
    },

}

setup(
    name = "<any name>",
    options = options,
    version = "0.0.1",
    description = '<any description>',
    executables = executables
)