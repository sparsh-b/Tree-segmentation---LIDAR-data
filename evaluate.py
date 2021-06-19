import glob
import os
import numpy as np

req_classes = [1300,1302, 1303, 1304, 1305]#IDs for classes foliage, small_trunk, large_trunk, thin_branch, thick_branch
for var in ['2_ac','2_ad','2_ae','2_ag','2_ah','2_ai','2_aj','2_ak','2_al','2_ao','3_aj','3_ak','3_al','3_am','3_an','3_ao','3_ap']:
    print("\n**************")
    print(var)
    conf = open('confidence_files/oakland_part'+var+'_conf.txt', 'r')
    pred_wrl = open('wrl_files/AlgoOutput/tree_'+var+'_final.wrl', 'r')

    flag = 0
    wrl_lines = pred_wrl.readlines()
    pred = []
    for line in wrl_lines:
    #    print(line)
    line = line.strip()
    if line == "point [":
        flag = 1
        continue
    if flag == 0:
        continue
    if line == "0 0 0 ]":
        break
    #    print(line)
    if line != "0.000000 0.000000 0.000000,":
        pred.append(str(np.around(float(line.split(' ')[0]),2))+', '+str(np.around(float(line.split(' ')[1]),2))+', '+str(np.around(float(line.split(' ')[2].split(',')[0]),2)))

    conf_lines = conf.readlines()
    curr_line = 0
    count = 0
    correct = 0
    wrong = 0
    for conf_line in conf_lines:
    count += 1
    conf_line = conf_line.strip()
    if conf_line.split(' ')[0][0] == '#':
        continue
    curr_line += 1
    class_name = int(conf_line.split(' ')[-2])
    if class_name in req_classes:
        if (conf_line.split(' ')[0]+', '+conf_line.split(' ')[1]+', '+conf_line.split(' ')[2]) in pred:
            correct += 1
        else:
            wrong += 1
    else:
        if (conf_line.split(' ')[0]+', '+conf_line.split(' ')[1]+', '+conf_line.split(' ')[2]) not in pred:
            correct += 1
        else:
            wrong += 1

    accuracy = np.round(100*float(correct)/(correct+wrong), 2)
    print("Accuracy: ", accuracy)
