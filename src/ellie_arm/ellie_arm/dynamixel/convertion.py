
# import os 
# import glob
# import pathlib
# import json

# path = pathlib.Path("src/ellie/ellie_body/actions")
# files = [e for e in path.iterdir() if e.is_file()]
# for i in files:
    
#     with open(i, 'r+') as f:
#         data = json.load(f)
#         positions = data['positions']
#         for name, value in positions.items():
#             for name2, value2 in value.items():
#                 if name2 in  ["r_shoulder_y", "r_shoulder_x",
#                       "r_arm_z", "l_arm_z","l_elbow_y","l_shoulder_x","bust_x","bust_y"]:
#                       value2[0] = -value2[0]
#                       value2[1] = -value2[1]
#         f.seek(0)
#         json.dump(data,f,indent= 4)
#         f.truncate()