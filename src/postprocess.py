import pybullet as p
from sim_.simulation.utils.io import parse_log as plog
import numpy as np
# from IPython import embed


class Postprocess(object):
    def __init__(self, objects_fname="sim_/log/trajectory/body_info.txt"):

        # get mapping between object ids and object names
        self.object_map = {}
        objects_fptr = open(objects_fname, "r")
        lines = objects_fptr.readlines()
        objects_fptr.close()
        for line in lines:
            obj_id, obj_name = line.strip().split(",")
            self.object_map[int(obj_id)] = obj_name

        self.object_ids = [int(x) for x in self.object_map.keys()]
        self.object_names = self.object_map.values()

        # list to map column index to name
        self.col_names = [u'stepCount', u'timeStamp', u'objectId', u'posX', u'posY', u'posZ', 
                          u'oriX', u'oriY', u'oriZ', u'oriW', u'velX', u'velY', u'velZ', 
                          u'omegaX', u'omegaY', u'omegaZ', u'qNum', 
                          u'q0', u'q1', u'q2', u'q3', u'q4', u'q5', u'q6', u'q7', u'q8', u'q9', u'q10', u'q11', 
                          u'u0', u'u1', u'u2', u'u3', u'u4', u'u5', u'u6', u'u7', u'u8', u'u9', u'u10', u'u11']

        # dictionary to map column name to index
        self.col_names_dict = {}
        for i, name in enumerate(self.col_names):
            self.col_names_dict[name] = i

    def parse_log(self, f_name, output_file, verbose=True, objects=None, cols=None):
        """
        
        This function parses a log saved by PyBullet, but it filters the
        saved objects on the provided keys and the collected data on cols.
        Both keys and cols should be lists of strings.

        Example usage below:
    
        Parse a log for data concerning the robot arm and gripper.

            log = parse_log("data.bin", "out.txt", objects=["sawyer", "gripper"])

        Same as above, but filter out only the joint positions.

            log = parse_log("data.bin", "out.txt", objects=["sawyer", "gripper"], 
                            cols=['q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8', 'q9', 'q10', 'q11'])

        """

        log = np.array(plog(f_name, verbose=verbose))

        col_inds = sorted(self.col_names_dict.values())
        if cols is not None:
            # make sure desired columns are valid
            assert(set(cols) <= set(self.col_names))

            # get column inds to filter out
            col_inds = map(self.col_names_dict.get, cols)

        # get object ids to filter out
        if objects is None:
            filter_object_ids = self.object_ids
        else:
            filter_object_ids = []
            for obj in objects:
                for obj_id in self.object_map:
                    if obj in self.object_map[obj_id]:
                        filter_object_ids.append(obj_id)

        ind = self.col_names_dict['objectId']
        def filter_row(row):
            if row[ind] in filter_object_ids:
                return row[col_inds]
            else:
                return None
        filtered = map(filter_row, log)
        return [x for x in filtered if x is not None]



if __name__ == "__main__":

    f_name = "sim_/log/trajectory/keyboard_sawyer_hanoi.bin"
    pp = Postprocess()
    log1 = pp.parse_log(f_name, None, verbose=False)
    log2 = pp.parse_log(f_name, None, verbose=False, objects=["sawyer", "gripper"])
    log3 = pp.parse_log(f_name, None, verbose=False, objects=["sawyer", "gripper"], 
                                cols=['q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6'])

    print(log1[4])
    print(log2[0])
    print(log3[0])
    # embed()









