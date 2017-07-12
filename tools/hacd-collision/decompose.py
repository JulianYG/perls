import subprocess
import os
import argparse

command = "./bins/testVHACD --input %s --resolution %d --depth 20 "\
    "--concavity %f --planeDownsampling 4 --convexhullDownsampling 4 "\
    "--alpha 0.05 --beta 0.05 --gamma 0.00125 --pca 0 --mode 0 "\
    "--maxNumVerticesPerCH 64 --minVolumePerCH 0.0 "\
    "--output %s --log %s"

TEMP_OUTPUT_MODEL = 'output.wrl'
MAX_GROUPS = 50
MIN_GROUPS = 1


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert an obj 3D Description file "\
                    "into a urdf file with collision convex meshes')

    parser.add_argument('--rgba', dest='rgba',
                        help='The color of the object',
                        default='1 1 1 1', type=str)

    parser.add_argument('--scale', dest='scale',
                        help='The scale of the object',
                        default=1.0, type=float)

    parser.add_argument('--input', dest='input',
                        help='The filepath of the input model',
                        default=None, type=str, required=True)

    parser.add_argument('--outputdir', dest='outputdir',
                        help='The dirpath of the output urdf file'
                             'and the collision mesh file',
                        default=None, type=str, required=True)

    parser.add_argument('--mass', dest='mass',
                        help='The mass of the object',
                        default=0.5, type=float)

    parser.add_argument('--bodyname', dest='bodyname',
                        help='The name of the object',
                        default='cvgl', type=str)

    args = parser.parse_args()

    return args


def run_decomposition(input_model, resolution=100000, concavity=0.0025,
                      output_model=TEMP_OUTPUT_MODEL, log_dir="log.txt"):
    os.system(command % (input_model, resolution, concavity,
                         output_model, log_dir))


def count_output_groups(output_wrl):
    group_count = 0
    with open(output_wrl, 'r') as f:
        for line in f:
            if line.startswith('Group'):
                group_count += 1
    return group_count


def main():
    args = parse_args()
    args.outputdir = os.path.abspath(args.outputdir)
    if not os.path.isdir(args.outputdir):
        os.mkdir(args.outputdir)
    tempfolder = args.outputdir + "/temp"
    if not os.path.isdir(tempfolder):
        os.mkdir(tempfolder)

    # use VHACD to generate the collision mesh
    # Sometimes when we perform HACD, we have too many or
    # too few convex sub-meshes
    # This code adjusts the maximum concavity parameter to keep them in bounds
    resolution = 10000
    concavity = 0.0025
    run_decomposition(args.input, resolution=resolution, concavity=concavity,
                      output_model=tempfolder + "/" + TEMP_OUTPUT_MODEL,
                      log_dir=tempfolder + "/log.txt")
    num_groups = count_output_groups(tempfolder + "/" + TEMP_OUTPUT_MODEL)
    while num_groups > MAX_GROUPS or num_groups < MIN_GROUPS:
        if num_groups > MAX_GROUPS:
            concavity *= 2
        else:
            concavity /= 2
        run_decomposition(args.input, resolution=resolution,
                          concavity=concavity,
                          output_model=tempfolder + "/" + TEMP_OUTPUT_MODEL,
                          log_dir=tempfolder + "/log.txt")
        num_groups = count_output_groups(tempfolder + "/" + TEMP_OUTPUT_MODEL)

    with open(tempfolder + "/" + TEMP_OUTPUT_MODEL, 'r') as f:
        data = f.read()
        data = data.splitlines()
        count = 0
        i = 0
        while i < len(data):
            with open(tempfolder + "/" + str(count) + ".wrl", "w") \
                    as currentfile:
                currentfile.write(data[i])
                i = i + 1
                while (i < len(data)) and (data[i][:5] != "#VRML"):
                    currentfile.write(data[i] + "\n")
                    i = i + 1
            count = count + 1
    
    if not os.path.isdir(args.outputdir + "/meshes"):
        os.mkdir(args.outputdir + "/meshes")
        
    for i in xrange(count):
        os.system("./bins/meshconv -c obj -tri -o {output} "
                  "{temp}/{i}.wrl".format(
                      output=args.outputdir +
                      "/meshes/" + str(i),
                      temp=tempfolder, i=i))

    # prepare for generating the urdf file
    with open("templates/template.urdf", "r") as f:
        urdf_template = f.read()

    with open("templates/collision_template", "r") as f:
        collision_template = f.read()

    collision = ""
    for i in xrange(count):
        collision = collision + collision_template.format(
            file=args.outputdir + "/meshes/" + str(i) + ".obj",
            scale=args.scale)

    with open(args.outputdir + "/" + args.bodyname + ".urdf", "w") as f:
        f.write(urdf_template.format(body_name=args.bodyname,
                                     mass=args.mass, ixx=1, ixy=0,
                                     ixz=0, iyy=1, iyz=0, izz=1,
                                     cx=0, cy=0, cz=0,
                                     obj_file=args.input,
                                     scale=args.scale,
                                     rgba=args.rgba,
                                     collision_mesh=collision))

    os.system("rm -rf {}".format(tempfolder))


if __name__ == '__main__':
    main()
