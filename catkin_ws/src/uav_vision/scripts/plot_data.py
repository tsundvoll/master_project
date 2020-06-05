#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import sys

def plot_data(stored_array, methods_to_plot, variables_to_plot, plot_error=False, plot_z_to_the_right=False, z_right_color='g'):
    t_id = 0            # Time index
    g_id = 1            # Ground truth index
    e_id = g_id + 6     # Ellipse index
    a_id = e_id + 6     # Arrow index
    c_id = a_id + 6     # Corner index
    d_id = c_id + 6     # Dead reckoning index

    error_e_id = d_id + 6     # Ellipse error index
    error_a_id = error_e_id + 6     # Arrow error index
    error_c_id = error_a_id + 6     # Corner error index
    error_d_id = error_c_id + 6     # Dead reckoning error index

    time_stamps = stored_array[:, t_id]

    titles_variables = [
        "x-Position", "y-Position", "z-Position", "None", "None", "yaw-Rotation"
    ]
    titles_error_variables = [
        "x-Position Error", "y-Position Error", "z-Position Error",
        "none", "none", "yaw-Rotation Error"
    ]

    lables_variables = [
        "x-Position [m]", "y-Position [m]", "z-Position [m]", "none", "none", "yaw-Rotation [deg]",
    ]
    lables_error_variables = [
        "x-Position Error [m]", "y-Position Error [m]", "z-Position Error [m]", "none", "none", "yaw-Rotation Error [deg]",
    ]
    titles_methods = [
        "Ground truth",
        "Ellipse",
        "Arrow",
        "Corners",
        "Dead reckogning",
        "Ellipse error",
        "Arrow error",
        "Corners error",
        "Dead reckogning error",
    ]
    indices_methods = [g_id, e_id, a_id, c_id, d_id,
        error_e_id, error_a_id, error_c_id, error_d_id
    ]
    colors_methods = [
        "g",        # green:    "Ground truth"
        "b",        # blue:     "Ellipse"
        "r",        # red:      "Arrow"
        "y",        # yellow:   "Corners"
        "k",        # black:    "Dead reckogning"
        "b",        # blue:     "Ellipse error"
        "r",        # red:      "Arrow error"
        "y",        # yellow:   "Corners error"
        "k"         # black:    "Dead reckogning error"
    ]
    y_ticks_error_pos = np.arange(-0.10, 0.11, 0.025)
    y_ticks_error_rot = np.arange(-10, 11, 2)
    y_ticks_error = np.array([
        [y_ticks_error_pos]*3, [y_ticks_error_rot]*3
    ])

    for variable in variables_to_plot:

        if plot_error:
            title = titles_error_variables[variable]
            y_label = lables_error_variables[variable]
        else:
            title = titles_variables[variable]
            y_label = lables_variables[variable]

        fig, ax = plt.subplots(figsize=(10,8))
        # fig, ax = plt.subplots(figsize=(20,15))

        if plot_error:
            ax.axhline(y=0, color='grey', linestyle='--') # Plot the zero-line


        for method in methods_to_plot:
            legend_text = titles_methods[method]
            line_color = colors_methods[method]
            index = indices_methods[method]

            data = stored_array[:, index:index+6][:,variable]
            time_stamps_local = time_stamps.copy()
            time_stamps_local[np.isnan(data)] = np.nan
      
            line, = ax.plot(time_stamps_local, data)
            line.set_color(line_color)
            line.set_label(legend_text)

            ax.set_title(title)
            ax.set_xlabel('Time [s]')
            ax.set_ylabel(y_label)
            ax.legend(loc='upper left', facecolor='white', framealpha=1)

            # if plot_error:
            #     ax.set_yticks(y_ticks_error[variable])

            # ax.xaxis.grid()
            # ax.yaxis.grid()
            # ax.grid()
        
        if plot_z_to_the_right:
            # Plot the z ground truth
            gt_method = 0
            z_variable = 2

            index = indices_methods[gt_method]
            data = stored_array[:, index:index+6][:, z_variable]
            time_stamps_local = time_stamps.copy()
            time_stamps_local[np.isnan(data)] = np.nan

            ax2 = ax.twinx()
            line, = ax2.plot(time_stamps_local, data)
            line.set_color(z_right_color)
            line.set_label("Ground truth z-Position")

            ax2.legend(loc='upper right', facecolor='white', framealpha=1)

            ax2.set_ylabel('z-Position [m]', color=z_right_color)
            ax2.set_yticks(np.arange(7))
            ax2.tick_params(axis='y', labelcolor=z_right_color)
            ax2.grid(None)

        plt.xlim(time_stamps[0], time_stamps[-1])
        plt.grid()

        fig.tight_layout()
        
        folder = './plots/'
        plt.savefig(folder+title+'.svg')


        fig.draw
        plt.waitforbuttonpress(0)
        plt.close()

        # plt.show()

def plot_data_manually(stored_array):
    # Variables
    V_X = 0
    V_Y = 1
    V_Z = 2
    V_YAW = 5

    # 0: time
    # 1: ground_truth

    # 2: est_ellipse
    # 3: est_arrow
    # 4: est_corners
    # 5: est_dead_reckoning

    # 6: est_error_ellipse
    # 7: est_error_arrow
    # 8: est_error_corners
    # 9: est_error_dead_reckoning
    index = 0
    time_stamps = stored_array[:, index]
    index += 1
    ground_truth = stored_array[:, index:index+6]

    index += 6
    est_ellipse = stored_array[:, index:index+6]
    index += 6
    est_arrow = stored_array[:, index:index+6]
    index += 6
    est_corners = stored_array[:, index:index+6]
    index += 6
    est_dead_reckoning = stored_array[:, index:index+6]

    index += 6
    est_error_ellipse = stored_array[:, index:index+6]
    index += 6
    est_error_arrow = stored_array[:, index:index+6]
    index += 6
    est_error_corners = stored_array[:, index:index+6]
    index += 6
    est_error_dead_reckoning = stored_array[:, index:index+6]

    # List if indices
    # 1	    ground_truth
    # 7	    est_ellipse
    # 13	est_arrow
    # 19	est_corners
    # 25	est_dead_reckoning

    # 31	est_error_ellipse
    # 37	est_error_arrow
    # 43	est_error_corners
    # 49	est_error_dead_reckoning

    index_values = [1, 7, 13, 19]
    color_values = ['green', 'blue', 'red', 'yellow']
    legend_values = ['ground truth', 'ellipse', 'arrow', 'corners']

    index_errors = [31, 37, 43]
    color_errors = ['blue', 'red', 'yellow']


    fig = plt.figure(figsize=(15,20))

    for i in range(1,9):
        ax = plt.subplot(4,2,i)

        # Plot the ground truth value for z
        right_ax = ax.twinx()

        legend_text = 'ground truth z-position'
        z_right_color = 'lightgrey'

        data = ground_truth[:,V_Z]
        time_stamps_local = time_stamps.copy()
        time_stamps_local[np.isnan(data)] = np.nan

        line, = right_ax.plot(time_stamps_local, data)
        line.set_color(z_right_color)
        # line.set_label(legend_text)


        right_ax.set_ylabel('z-position [m]', color='grey')
        right_ax.tick_params(axis='y', labelcolor='grey')
        # right_ax.legend(loc='upper right', facecolor='white', framealpha=1)

        if i == 1:
            variable = V_X
            y_label = "x-position [m]"
        elif i == 2:
            variable = V_X
            y_label = "x-position error [m]"
        elif i == 3:
            variable = V_Y
            y_label = "y-position [m]"
        elif i == 4:
            variable = V_Y
            y_label = "y-position error[m]"

        elif i == 5:
            variable = V_Z
            y_label = "z-position [m]"
        elif i == 6:
            variable = V_Z
            y_label = "z-position error[m]"
                    
        elif i == 7:
            variable = V_YAW
            y_label = "yaw-rotation [deg]"
        elif i == 8:
            variable = V_YAW
            y_label = "yaw-rotation error [deg]"

        else:
            print "ERROR"
            return 0

        # Label x-axis
        if i==7 or i==8:
            ax.set_xlabel('Time [s]')

        # Plot the estimate values
        if i % 2 == 1:
            for j in range(4):
                index = index_values[j]
                color = color_values[j]
                legend_text = legend_values[j]
            

                data = stored_array[:, index:index+6][:,variable]
            
                time_stamps_local = time_stamps.copy()
                time_stamps_local[np.isnan(data)] = np.nan
        
                line, = ax.plot(time_stamps_local, data)
                line.set_color(color)
                line.set_label(legend_text if i==1 else "_nolegend_")
            
            ax.set_ylabel(y_label)


            # Plot legend on top
            if i==1:
                ax.legend(bbox_to_anchor=(1.1, 1.1), loc='center', ncol=4, facecolor='white', framealpha=1)
            


        # Plot the estimate errors
        if i % 2 == 0:
            ax.axhline(y=0, color='grey', linestyle='--') # Plot the zero-line

            for j in range(3):
                index = index_errors[j]
                color = color_errors[j]            

                data = stored_array[:, index:index+6][:,variable]
            
                time_stamps_local = time_stamps.copy()
                time_stamps_local[np.isnan(data)] = np.nan
        
                line, = ax.plot(time_stamps_local, data)
                line.set_color(color)
            
            ax.set_ylabel(y_label)


        # plt.legend(bbox_to_anchor=(1.05, 1.0), loc='lower left', facecolor='white', framealpha=1)


    folder = './plots/'
    title = 'Plot_1'
    plt.savefig(folder+title+'.svg')

    # fig.draw
    # plt.waitforbuttonpress(0)
    # plt.close()

    # plt.show()
    
    


if __name__ == '__main__':

    if len(sys.argv) == 2:
        test_number = sys.argv[1]
    else:
        test_number = 1
    print "Plotting test number " + str(test_number)

    # Load the data
    folder = './catkin_ws/src/uav_vision/data_storage/'
    filename = 'test_'+str(test_number)+'.npy'
    path = folder + filename
    stored_array = np.load(path, allow_pickle=True)
    
    # Plot the data
    plot_data_manually(stored_array)




    # # Settings
    # plot_error = False
    # plot_z_to_the_right = True

    # # 0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw
    # # variables_to_plot = [0, 1, 2, 5]
    # variables_to_plot = [0, 1, 2, 5]


    # 0: ground truth, 1: ellipse, 2: arrow, 3: corners, 4: dead reckoning
    # 5: ellipse_error, 6: arrow_error, 7: corners_error, 8: dead reckoning_error
    # if plot_error:
    #     methods_to_plot = [5, 6, 7]
    #     plot_z_to_the_right = True
    #     z_right_color = 'grey'
    # else:
    #     methods_to_plot = [0, 1, 2, 3]
    #     plot_z_to_the_right = True
    #     z_right_color = 'grey'
    
    # Data
    # 0: time

    # 1: ground truth
    
    # 2: est_ellipse
    # 3: est_arrow
    # 4: est_corners
    # 5: est_dead_reckoning

    # 6: est_error_ellipse
    # 7: est_error_arrow
    # 8: est_error_corners
    # 9: est_error_dead_reckoning
    
    
    # plot_error = False
    # methods_to_plot = [0, 1, 2, 3]
    # plot_z_to_the_right = True
    # z_right_color = 'grey'
    # plot_data(stored_array, methods_to_plot, variables_to_plot, plot_error, plot_z_to_the_right, z_right_color)
    
    # plot_error = True
    # methods_to_plot = [5, 6, 7]
    # plot_z_to_the_right = True
    # z_right_color = 'grey'
    # plot_data(stored_array, methods_to_plot, variables_to_plot, plot_error, plot_z_to_the_right, z_right_color)


