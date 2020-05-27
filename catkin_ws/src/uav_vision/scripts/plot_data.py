#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time


def plot_data(stored_array, methods_to_plot, variables_to_plot):

    t_id = 0            # Time index
    g_id = 1            # Ground truth index
    e_id = g_id + 6    # Ellipse index
    a_id = e_id + 6     # Arrow index
    c_id = a_id + 6     # Corner index
    d_id = c_id + 6     # Dead reckoning index

    time_stamps         = stored_array[:, t_id]
    data_ground_truth   = stored_array[:, g_id:e_id]
    data_ellipse        = stored_array[:, e_id:a_id]
    data_arrow          = stored_array[:, a_id:c_id]
    data_corners        = stored_array[:, c_id:d_id]
    data_dead_reckoning = stored_array[:, d_id:]

    print time_stamps.shape
    print data_ground_truth.shape
    print data_ellipse.shape
    print data_arrow.shape
    print data_corners.shape
    print data_dead_reckoning.shape


    if 0 in methods_to_plot:
        title = "Ground truth"

        fig, ax = plt.subplots()

        for variable in variables_to_plot:
            ax.plot(time_stamps, data_ground_truth[:,variable])
        ax.set_title(title)
        # ax.legend(loc='upper left')
        ax.set_ylabel('meters')
        # ax.set_xlim(xmin=yrs[0], xmax=yrs[-1])

        # fig.tight_layout()
        plt.show()


# data_x = data[:,0]
# data_y = data[:,1]
# data_z = data[:,2]
# data_yaw = data[:,5]

# # print time_stamps.shape
# # print data.shape


# # rng = np.arange(50)
# # rnd = np.random.randint(0, 10, size=(3, rng.size))
# # yrs = 1950 + rng

# fig, ax = plt.subplots(figsize=(5, 3))

# ax.plot(time_stamps, data_z)
# ax.set_title('Combined debt growth over time')
# # ax.legend(loc='upper left')
# ax.set_ylabel('meters')
# # ax.set_xlim(xmin=yrs[0], xmax=yrs[-1])


# # fig.tight_layout()
# plt.show()

if __name__ == '__main__':

    # Settings
    test_number = 2

    # 0: ground truth, 1: ellipse, 2: arrow, 3: corners, 4: dead reckoning
    methods_to_plot = [0]

    # 0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw
    variables_to_plot = [2]


    # Load the data
    folder = './catkin_ws/src/uav_vision/data_storage/'
    filename = 'test_'+str(test_number)+'.npy'
    path = folder + filename
    stored_array = np.load(path, allow_pickle=True)


    plot_data(stored_array, methods_to_plot, variables_to_plot)

