import os
import numpy as np
from scipy.spatial.transform import Rotation as R


data_path = '/home/jzian/WorkSpace/Blockmap_gt/src/Block_Map_from_Groundtruth/data/ulhk_1.txt'
out_path = '/home/jzian/WorkSpace/Blockmap_gt/src/Block_Map_from_Groundtruth/data/ulhk_1_process2.txt'
if __name__ == "__main__":
    first_data = []
    out = [0, 0, 0, 0, 0, 0, 0, 0]
    with open(data_path, 'r') as gt:
        with open(out_path, 'w') as gt_process:
            # datas = gt.readlines()
            for (num,line) in enumerate(gt):
                data = line.rstrip('\n').split()
                if num == 0:
                    first_data = data
                results = list(map(float, data))
                first_data_ = list(map(float, first_data))
                for i, val in enumerate(results):
                    if i == 0:
                        out[i] = data[i]
                        continue
                    out[i] = val - first_data_[i]
                # print(data[0])
                # print("first: ", first_data)
                # data_out = results - first_data_
                gt_process.write(str(out[0]) + " " + str(out[1]) + " " + str(out[2]) + " " + str(0) + " " + str(out[4]) + " " + str(out[5]) + " " + str(out[6]) + " " + str(out[7]) + '\n')

        gt_process.close()
    gt.close()