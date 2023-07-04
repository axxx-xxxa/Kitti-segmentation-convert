import numpy as np 


def KITTI_del_outsize_ring(data, label):
    '''
    第一步: 剪去安装角度相差过大的线数(保留2~-16度的线)
    '''
    R_list = []
    for i in data:
        R_list.append(np.sqrt(i[0]**2+i[1]**2+i[2]**2))
    vert_angle = np.arcsin(data[:,2]/R_list)/np.pi*180

    # 第一步： 将高度角度小与16度的部分剪去
    for i in range(len(vert_angle)):
        if vert_angle[i]<-16:
            break

    data = data[:i]
    label = label[:i]
    return data, label

def KITTI_add_ring(data, label):

    '''
    第二步: 计算所有的ring

    input:
    data : 需要添加线数的数据

    in:
    candidate_point : 所有候选点
    candidate_clean : 需要清洗的候选点

    bin_64 : 存放最后增加 ring 纬度的所有数据
    output:

    '''

    hori_angle = np.arctan2(data[:,0],data[:,1])/np.pi*180    

    candidate_point = []
    candidate_clean = []

    # 选择所有转折点
    for i in range(len(hori_angle)-1):
        if hori_angle[i]>90-0.2 and hori_angle[i+1]<90+0.2:
            candidate_point.append(i+2)

    # 筛掉噪声
    for m in range(len(candidate_point)-1):
        if candidate_point[m+1]-candidate_point[m] <10:
            candidate_clean.append(m+1)

    candidate_point = [i for num,i in enumerate(candidate_point) if num not in candidate_clean]

    bin_64 = data[0:1]
    bin_64 = np.concatenate((bin_64, np.array([[1]])), axis=1)

    label_64 = label[0:1]
    label_64 = np.reshape(label_64, (1,1))
    label_64 = np.concatenate((label_64, np.array([[1]])),axis = 1)

    for n in range(len(candidate_point)-1):
        tmp_mim_part = data[candidate_point[n]:candidate_point[n+1]]
        r = np.array([[n]]*(candidate_point[n+1]-candidate_point[n]))

        tmp_add_r = np.concatenate((tmp_mim_part,r),axis = 1)
        bin_64 = np.concatenate((bin_64, tmp_add_r), axis=0)

        tmp_mim_part_l = label[candidate_point[n]:candidate_point[n+1]]
        tmp_mim_part_l = np.reshape(tmp_mim_part_l, (tmp_mim_part_l.shape[0], 1))
        tmp_add_r_l = np.concatenate((tmp_mim_part_l,r),axis = 1)

        label_64 = np.concatenate((label_64, tmp_add_r_l))

    bin_64 = bin_64[1:]

    return bin_64, label_64

def KITTI_divid_2_to_KITTI(data, label):
    '''
    可拓展:
    现在根据当前雷达为均匀排列,故再次也只需要均匀排列即可
    '''
    data_d = []
    label_d = []
    for index, i in enumerate(data):
        if i[4]%2 == 0:
            label_d.append(label[index][0:-1].tolist())
            data_d.append(i[0:-1].tolist())
    data_d = np.array(data_d, dtype=np.float32)
    label_d = np.array(label_d, dtype=np.int32)
    return data_d, label_d

# 测试
label = np.fromfile("../04/labels/000122.label", dtype=np.int32)
data0 = np.fromfile("../04/velodyne/000122.bin", dtype=np.float32)
data0.tofile("test64.bin")
# data.tofile("test64.bin")
data = data0.reshape(-1,4)
data_del, label_del = KITTI_del_outsize_ring(data, label)
data_add_ring, label_add_ring = KITTI_add_ring(data_del, label_del)
data_divid_2, label_divid_2 = KITTI_divid_2_to_KITTI(data_add_ring, label_add_ring)


data_divid_2.tofile("test32.bin")
print(data.shape)
print(label.shape)

print(data_del.shape)
print(data_divid_2.shape)

