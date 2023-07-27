import numpy as np
import matplotlib.pyplot as plt

def read_calib(file_path):
    f = open(file_path, 'r')
    lines = f.readlines()
    T = [[float(x) for x in lines[i].split('\n')[0].split()] \
         for i in range(3)]
    T = np.array(T)
    # print(f"T = {T}")

    K = [[float(x) for x in lines[i].split('\n')[0].split()] \
         for i in range(4,7)]
    K = np.array(K)
    # print(f"K = {K}")

    b = [float(lines[i].split('\n')[0]) for i in range(8,11)] 
    b = np.array(b)

    # print(f"b = {b}")
    f.close()
    return T, K, b


acc_calib_file = f'bin/test_imu_acc.calib'
gyro_calib_file = f'bin/test_imu_gyro.calib'

Ta, Ka, ba = read_calib(acc_calib_file)
Tg, Kg, bg = read_calib(gyro_calib_file)

acc_data_file = f'bin/test_data/xsens_acc.mat'
gyro_data_file = f'bin/test_data/xsens_gyro.mat'
imu_data_file = f'bin/test_data/imu_data.txt'

f_acc = open(acc_data_file, 'rb')
f_gyro = open(gyro_data_file, 'rb')
f_imu = open(imu_data_file, 'w')

lines_acc = f_acc.readlines()
lines_gyro = f_gyro.readlines()
data_num = min(len(lines_acc), len(lines_gyro))

all_acc_data = []
all_gyro_data = []
all_ts = []

for i in range(data_num):
    data_acc = lines_acc[i].decode('utf-8').split(f'\n')[0].split()
    data_gyro = lines_gyro[i].decode('utf-8').split(f'\n')[0].split()

    ts = float(data_acc[0])
    raw_acc = np.array([float(x) for x in data_acc[1:]])
    raw_gyro = np.array([float(x) for x in data_gyro[1:]])
    raw_mag = np.array([0.0, 0.0, 0.0])

    processed_acc = Ta @ Ka @ (raw_acc - ba)
    processed_gyro = Tg @ Kg @ (raw_gyro - bg)
    processed_mag = raw_mag

    # all_acc_data.append(raw_acc.tolist())
    # all_gyro_data.append(raw_gyro.tolist())

    all_acc_data.append(processed_acc.tolist())
    all_gyro_data.append(processed_gyro.tolist())

    all_ts.append(ts)

    f_imu.write("{:.5f}".format(ts))
    f_imu.write(' ')

    for x in processed_acc:
        f_imu.write("{:.4f}".format(x))
        f_imu.write(' ')
    for x in processed_gyro:
        f_imu.write("{:.4f}".format(x))
        f_imu.write(' ')
    for x in processed_mag:
        f_imu.write("{:.4f}".format(x))
        f_imu.write(' ')
    f_imu.write(f'\n')
f_acc.close()
f_imu.close()

fig = plt.figure()

plt.subplot(2,1,1)
# plt.plot([1,2,3],[2,4,8])
for i in range(3):
    ys = [x[i] for x in all_acc_data]
    plt.plot(all_ts,ys)
plt.title('accelerometer')
plt.xlabel('Time')

plt.subplot(2,1,2)
# plt.plot([1,2,3],[2,4,8])
for i in range(3):
    ys = [x[i] for x in all_gyro_data]
    plt.plot(all_ts,ys)
plt.title('gyroscope')
plt.xlabel('Time')

plt.show()
# all_gyro_data