import os
import numpy as np
from scipy.spatial.transform import Rotation as R


data_path = '/home/jzian/WorkSpace/Blockmap_gt/src/Block_Map_from_Groundtruth/data/nclt/gt_2012-04-29.txt'
out_path = '/home/jzian/WorkSpace/Blockmap_gt/src/Block_Map_from_Groundtruth/data/nclt/gt_2012-04-29_final_lidar.txt'
if __name__ == "__main__":
    extrinsic_T_gt_L = np.array([ 0.002, 0.004, 0.957])
    # extrinsic_R_gt_L = np.array([ [-0.0122693,  0.9998251, -0.0141188],
    #                             [0.9999205, 0.0123089, 0.0027239],
    #                             [0.0028972,  -0.0140843,  -0.9998966 ]])

    extrinsic_R_I_L = np.array( [ [-0.0122693,  0.9998251, -0.0141188],
                                [-0.9999205, -0.0123089, -0.0027239],
                                [-0.0028972,  0.0140843,  0.9998966] ])
 
    # I_T_L = np.array( [ [-0.0122693,  0.9998251, -0.0141188, 0.112],
    #                     [-0.9999205, -0.0123089, -0.0027239,0.176],
    #                     [-0.0028972,  0.0140843,  0.9998966,-0.247],
    #                     [0,0,0,1] ])

    I_T_L = np.array( [ [-0.0122693,  0.9998251, -0.0141188, 0.002],
                        [-0.9999205, -0.0123089, -0.0027239, -0.004],
                        [-0.0028972,  0.0140843,  0.9998966, -0.957],
                        [0,0,0,1] ])
    # I_T_L = np.array( [ [-0.0122693,  0.9998251, -0.0141188, 0],
    #                     [-0.9999205, -0.0123089, -0.0027239, 0],
    #                     [-0.0028972,  0.0140843,  0.9998966, 0],
    #                     [0,0,0,1] ])
    gt_T_I = np.array( [[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1] ])

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
                t = np.array([[results[1],results[2],results[3]]])
                q = [results[4], results[5], results[6], results[7]]
                
                rotation = R.from_quat(q)
                rotation_mat = rotation.as_matrix()
                a=np.array([[0,0,0,1]])
                W_T_gt = np.concatenate((np.concatenate((rotation_mat, t.T),axis=1),a),axis=0)

                # W_T_L = W_T_gt * gt_T_I * I_T_L
                # W_T_L =  W_T_gt * gt_T_I * I_T_L
                # W_T_L = np.dot(W_T_gt, np.dot(I_T_L, gt_T_I))
                W_T_L = np.dot(I_T_L, np.dot(gt_T_I,W_T_gt))
                # final_mat = extrinsic_R_gt.transpose() * rotation_mat
                # t_final = np.dot( extrinsic_R_gt.transpose() , t-extrinsic_T_gt)  

                #final
                # final_mat = rotation_mat * extrinsic_R_gt_L 
                # t_final = np.dot(rotation_mat, extrinsic_T_gt_L) + t

                #final2
                # final_mat = rotation_mat * extrinsic_R_gt_L.transpose() 
                # t_final = np.dot(-np.dot(rotation_mat, extrinsic_R_gt_L.transpose()), extrinsic_T_gt_L) + t  

                #final3
                # final_mat = extrinsic_R_gt_L.transpose() * rotation_mat
                # t_final = np.dot( extrinsic_R_gt_L.transpose() , t-extrinsic_T_gt_L)  
                              
                # 四元数为xyzw
                
                q_out = R.from_matrix(W_T_L[:3,:3]).as_quat()
                t_final = W_T_L[:3,3:]

                gt_process.write(str(time) + " " + str(t_final[0][0]) + " " + str(t_final[1][0]) + " " + str(t_final[2][0]) + " " + str(q_out[0]) + " " + str(q_out[1]) + " " + str(q_out[2]) + " " + str(q_out[3]) + '\n')

        gt_process.close()
    gt.close()
