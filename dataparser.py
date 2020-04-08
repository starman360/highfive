import os
import json
import numpy as np

dirname = "Data2"
directory = "/home/anmol/robot_ws/src/highfive/" + dirname

lstdir = os.listdir(directory)
lstdir.sort()

fps = 30

with open(dirname + "_keyframes.txt", 'a') as f:
    f.write("t, keyframeid, x1, y1, c1, x2, y2, c2, x3, y3, c3, x4, y4, c4\n")

for filename in lstdir:
    if filename.endswith(".json"): 
        print(os.path.join(directory, filename))
        with open(os.path.join(directory, filename)) as json_file:
            kp = json.load(json_file)
            try:
                pose = np.array(kp['people'][-1]["pose_keypoints_2d"]).reshape(-1,3)[1:5,:]
            except IndexError:
                pose = np.zeros((4,3))
        with open(dirname + "_keyframes.txt", 'a') as f:
            pose = pose.reshape(1,-1)
            keyid = int(filename.split('_')[1])
            t = keyid/fps
            s = str(t) + ", " + str(keyid) + ", " + np.array2string(pose,separator=',', suppress_small=True, max_line_width=np.inf)[2:-2]
            f.write(s)
            f.write("\n")
        # print(pose)
        continue
    else:
        continue
