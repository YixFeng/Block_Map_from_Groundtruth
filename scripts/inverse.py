import os
import numpy as np
from scipy.spatial.transform import Rotation as R


data_path = '/home/jzian/WorkSpace/Blockmap_gt/src/Block_Map_from_Groundtruth/data/nclt/gt_2012-04-29.txt'
out_path = '/home/jzian/WorkSpace/Blockmap_gt/src/Block_Map_from_Groundtruth/data/nclt/gt_2012-04-29_out3.txt'
if __name__ == "__main__":
    extrinsic_T_gt_L = np.array([ 0.002, 0.004, 0.957])
    # extrinsic_R_gt_L = np.array([ [-0.0122693,  0.9998251, -0.0141188],
    #                             [0.9999205, 0.0123089, 0.0027239],
    #                             [0.0028972,  -0.0140843,  -0.9998966 ]])

    extrinsic_R_gt_L = np.array( [ [-0.0122693,  -0.9998251, 0.0141188],
                                [-0.9999205, 0.0123089, 0.0027239],
                                [-0.0028972,  -0.0140843,  -0.9998966] ])

    # extrinsic_T_gt: [ 0.002, 0.004, 0.957]
    # extrinsic_R_gt: [ -0.0122693, -0.9998261,  0.0140477,
    #                 -0.9999205, 0.0122273,  -0.0030695,
    #                 0.0028972,  -0.0140843,  -0.9998966 ]

#00.807  0.166  90.703
#[ -0.0122693, -0.9998261,  0.0140477;
#    0.9999205, -0.0122273,  0.0030695;
#   -0.0028972,  0.0140843,  0.9998966 ]

#0.807  0.166  -90.703
# [ -0.0122693,  0.9998251, -0.0141188;
#   -0.9999205, -0.0123089, -0.0027239;
#   -0.0028972,  0.0140843,  0.9998966 ]
print(extrinsic_R_gt_L.transpose())
print(np.dot( -extrinsic_R_gt_L.transpose() , extrinsic_T_gt_L))  
''''
    first_data = []
    out = [0, 0, 0, 0, 0, 0, 0, 0]
    with open(data_path, 'r') as gt:
        with open(out_path, 'w') as gt_process:
            # datas = gt.readlines()
            for (num,line) in enumerate(gt):
                data = line.rstrip('\n').split()

                results = list(map(float, data))
                first_data_ = list(map(float, first_data))
                time = results[0]
                t = np.array([results[1],results[2],results[3]])
                q = [results[4], results[5], results[6], results[7]]
                
                rotation = R.from_quat(q)
                rotation_mat = rotation.as_matrix()
                
                # final_mat = extrinsic_R_gt.transpose() * rotation_mat
                # t_final = np.dot( extrinsic_R_gt.transpose() , t-extrinsic_T_gt)  
                final_mat = rotation_mat * extrinsic_R_gt_L 
                t_final = np.dot(rotation_mat, extrinsic_T_gt_L) + t
                
                # 四元数为xyzw
                q2 = R.from_matrix(final_mat)
                q_out = q2.as_quat()
                gt_process.write(str(time) + " " + str(t_final[0]) + " " + str(t_final[1]) + " " + str(t_final[2]) + " " + str(q_out[0]) + " " + str(q_out[1]) + " " + str(q_out[2]) + " " + str(q_out[3]) + '\n')

        gt_process.close()
    gt.close()
    '''