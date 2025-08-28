import math
import numpy as np
import matplotlib.pyplot as plt


def get_q3_pl(q1_joint_angle_degrees):
    ###this function should accept as an input the angle of Q1 
    ###and return Q3 pos path length and Q3 neg path length in that order (Q3_Pos, Q3_Neg)

    q3_pos = 0 #mm this should be in mm
    q3_neg = 0 #mm this should be in mm

    """
    math stuff here
    """

    return q3_pos, q3_neg


def get_q4_pl(q2_joint_angle_degrees):
    ###this function should accept as an input the angle of Q1 
    ###and return Q4 pos path length and Q4 neg path length in that order (Q3_Pos, Q3_Neg)

    q4_pos = 0 #mm this should be in mm
    q4_neg = 0 #mm this should be in mm

    """
    math stuff here
    """

    return q4_pos, q4_neg

def sanity_check():
    # Plot for Q4
    q2_angles = np.linspace(0, 180, 181)
    q4_pos_list = []
    q4_neg_list = []
    for angle in q2_angles:
        q4_pos, q4_neg = get_q4_pl(angle)
        q4_pos_list.append(q4_pos)
        q4_neg_list.append(q4_neg)
    plt.figure()
    plt.plot(q2_angles, q4_pos_list, label='Q4 Pos')
    plt.plot(q2_angles, q4_neg_list, label='Q4 Neg')
    plt.xlabel('Q2 Joint Angle (degrees)')
    plt.ylabel('Path Length (mm)')
    plt.title('Q4 Path Lengths vs Q2 Joint Angle')
    plt.legend()
    plt.grid(True)
    plt.savefig('q4_path_lengths.png')
    plt.close()

    # Plot for Q3
    q1_angles = np.linspace(0, 180, 181)
    q3_pos_list = []
    q3_neg_list = []
    for angle in q1_angles:
        q3_pos, q3_neg = get_q3_pl(angle)
        q3_pos_list.append(q3_pos)
        q3_neg_list.append(q3_neg)
    plt.figure()
    plt.plot(q1_angles, q3_pos_list, label='Q3 Pos')
    plt.plot(q1_angles, q3_neg_list, label='Q3 Neg')
    plt.xlabel('Q1 Joint Angle (degrees)')
    plt.ylabel('Path Length (mm)')
    plt.title('Q3 Path Lengths vs Q1 Joint Angle')
    plt.legend()
    plt.grid(True)
    plt.savefig('q3_path_lengths.png')
    plt.close()