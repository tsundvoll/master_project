#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time


def plot_data(stored_array, methods_to_plot, variables_to_plot):
    t_id = 0            # Time index
    g_id = 1            # Ground truth index
    e_id = g_id + 6     # Ellipse index
    a_id = e_id + 6     # Arrow index
    c_id = a_id + 6     # Corner index
    d_id = c_id + 6     # Dead reckoning index

    # Remove zeroes
    stored_array[stored_array == 0] = None

    time_stamps         = stored_array[:, t_id]
    # data_ground_truth   = stored_array[:, g_id:g_id+6]
    # data_ellipse        = stored_array[:, e_id:e_id+6]
    # data_arrow          = stored_array[:, a_id:a_id+6]
    # data_corners        = stored_array[:, c_id:c_id+6]
    # data_dead_reckoning = stored_array[:, d_id:d_id+6]

    titles_variables = [
        "x", "y", "z", "None", "None", "yaw"
    ]
    titles_methods = [
        "Ground truth",
        "Ellipse",
        "Arrow",
        "Corners",
        "Dead reckogning"
    ]
    indices_methods = [g_id, e_id, a_id, c_id, d_id]

    colors_methods = [
        "g",         # green: "Ground truth"
        "b",         # "Ellipse"
        "r",         # "Arrow"
        "y",         # "Corners"
        "k"         # "Dead reckogning"
    ]



    for variable in variables_to_plot:
        title = titles_variables[variable]
        
        
        fig, ax = plt.subplots(figsize=(10,8))

        for method in methods_to_plot:
            legend_text = titles_methods[method]
            line_color = colors_methods[method]
            index = indices_methods[method]

            data = stored_array[:, index:index+6][:,variable]

            # line, = ax.plot(time_stamps, data)
            # line, = ax.scatter(time_stamps, data)
            line = ax.scatter(time_stamps, data, s=1)
            
            # line.set_drawstyle('steps')
            # line.set_solid_joinstyle('bevel')

            line.set_color(line_color)
            line.set_label(legend_text)
            ax.legend()

            ax.set_title(title)
            # ax.legend(loc='upper left')
            ax.set_xlabel('time [s]')
            ax.set_ylabel('distance [m]')
            # ax.set_xlim(xmin=yrs[0], xmax=yrs[-1])

            # fig.tight_layout()
        plt.draw()
        # plt.waitforbuttonpress(0)
        # plt.close(fig)
        plt.show()


# fig, ax = plt.subplots(figsize=(5, 3))
# ax.plot(time_stamps, data_z)
# ax.set_title('Combined debt growth over time')
# ax.legend(loc='upper left')
# ax.set_ylabel('meters')
# ax.set_xlim(xmin=yrs[0], xmax=yrs[-1])
# fig.tight_layout()
# plt.show()

if __name__ == '__main__':

    # Settings
    test_number = 7

    # 0: ground truth, 1: ellipse, 2: arrow, 3: corners, 4: dead reckoning
    methods_to_plot = [0, 1, 2, 3, 4]

    # 0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw
    variables_to_plot = [2]


    # Load the data
    folder = './catkin_ws/src/uav_vision/data_storage/'
    filename = 'test_'+str(test_number)+'.npy'
    path = folder + filename
    stored_array = np.load(path, allow_pickle=True)


    plot_data(stored_array, methods_to_plot, variables_to_plot)

