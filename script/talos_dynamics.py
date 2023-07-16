#! /usr/bin/env python3

# Copyright (C) 2022 Statistical Machine Learning and Motor Control Group (SLMC)
# Authors: Joao Moura (maintainer)
# email: joao.moura@ed.ac.uk

# This file is part of iiwa_pushing package.

# iiwa_pushing is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# iiwa_pushing is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
import sys
import numpy as np
np.set_printoptions(suppress=True)
import rospy
import optas

class TALOS(object):
    """docstring for talos dynamics properties."""

    def __init__(self, name):
        # initialization message
        self._name = name
#        rospy.loginfo("%s: Initializing class", self._name)
        ## --------------------------------------
        # donkey_base frame
        self._link_base = rospy.get_param('~link_base', 'link_base')
        # end-effector frame
        self._link_ee_right_arm = rospy.get_param('~link_ee_right_arm', 'link_ee_right_arm')
        self._link_ee_left_arm = rospy.get_param('~link_ee_left_arm', 'link_ee_left_arm')
        self._link_ee_right_leg = rospy.get_param('~link_ee_right_leg', 'link_ee_right_leg')
        self._link_ee_left_leg = rospy.get_param('~link_ee_left_leg', 'link_ee_left_leg')
        self._link_head = rospy.get_param('~link_head', 'link_head')
        self.link_num = 31
#        'base_joint_1' 1, 'base_joint_2', 'base_joint_3', 'base_joint_4', 'base_joint_5', 'base_joint_6',
#        'torso_1_joint' 2, 'torso_2_joint'3, 'head_1_joint'4, 'head_2_joint'5,
#        'arm_left_1_joint'6, 'arm_left_2_joint'7, 'arm_left_3_joint'8, 'arm_left_4_joint'9, 'arm_left_5_joint'10, 'arm_left_6_joint'11, 'arm_left_7_joint'12,
#        'arm_right_1_joint'13, 'arm_right_2_joint'14, 'arm_right_3_joint'15, 'arm_right_4_joint'16, 'arm_right_5_joint'17, 'arm_right_6_joint'18, 'arm_right_7_joint'19,
#        'leg_left_1_joint'20, 'leg_left_2_joint'21, 'leg_left_3_joint'22, 'leg_left_4_joint'23, 'leg_left_5_joint'24, 'leg_left_6_joint'25,
#        'leg_right_1_joint'26, 'leg_right_2_joint'27, 'leg_right_3_joint'28, 'leg_right_4_joint'29, 'leg_right_5_joint'30, 'leg_right_6_joint'31],
        self.link_parent = [0, 0,
                            1, 2, 3, 4,
                            1, 6, 7, 8, 9, 10, 11,
                            1, 13, 14, 15, 16, 17,
                            1, 20, 21, 22, 23, 24,
                            1, 26, 27, 28, 29, 30] # the first 0 represents that the parent of the world is still 0

#        self.m = [0, 15.36284,
#                       2.294658, 16.97403, 0.73746, 1.443954,
#                       2.714567, 2.425086, 2.208741, 0.877346, 1.877923, 0.40931, 0.308441,
#                       2.714567, 2.425086, 2.208741, 0.877346, 1.877923, 0.40931, 0.308441,
#                       1.845591, 1.490952, 6.239871, 3.759951, 1.29096,  1.597773,
#                       1.845591, 1.490952, 6.239871, 3.759951, 1.29096,  1.597773]

        self.m = []; self.m.append(0);
        self.m.append(15.36284); # link 1 m:  base link
        self.m.append(2.294658); # link 2 m:  torso 1 link
        self.m.append(16.97403); # link 3 m:  torso 2 link
        self.m.append(0.73746);  # link 4 m:  head 1 link
        self.m.append(1.443954); # link 5 m:  head 2 link
        self.m.append(2.714567); # link 6 m:  arm 1 left link
        self.m.append(2.425086); # link 7 m:  arm 2 left link
        self.m.append(2.208741); # link 8 m:  arm 3 left link
        self.m.append(0.877346); # link 9 m:  arm 4 left link
        self.m.append(1.877923); # link 10 m: arm 5 left link
        self.m.append(0.40931);  # link 11 m: arm 6 left link
        self.m.append(0.308441); # link 12 m: arm 7 left link
        self.m.append(2.714567); # link 13 m: arm 1 right link
        self.m.append(2.425086); # link 14 m: arm 2 right link
        self.m.append(2.208741); # link 15 m: arm 3 right link
        self.m.append(0.877346); # link 16 m: arm 4 right link
        self.m.append(1.877923); # link 17 m: arm 5 right link
        self.m.append(0.40931);  # link 18 m: arm 6 right link
        self.m.append(0.308441); # link 19 m: arm 7 right link
        self.m.append(1.845591); # link 20 m: leg 1 left link
        self.m.append(1.490952); # link 21 m: leg 2 left link
        self.m.append(6.239871); # link 22 m: leg 3 left link
        self.m.append(3.759951); # link 23 m: leg 4 left link
        self.m.append(1.29096);  # link 24 m: leg 5 left link
        self.m.append(1.597773); # link 25 m: leg 6 left link
        self.m.append(1.845591); # link 26 m: leg 1 right link
        self.m.append(1.490952); # link 27 m: leg 2 right link
        self.m.append(6.239871); # link 28 m: leg 3 right link
        self.m.append(3.759951); # link 29 m: leg 4 right link
        self.m.append(1.29096);  # link 30 m: leg 5 right link
        self.m.append(1.597773); # link 31 m: leg 6 right link

        self.pm_local = []; self.pm_local.append(np.zeros(3));
        self.pm_local.append(np.array([-0.05709419, 0.00153054, -0.0762521]));   # link 1 pm_local:  base link
        self.pm_local.append(np.array([0.00078223, 3.528e-05, -0.01782457]));    # link 2 pm_local:  torso 1 link
        self.pm_local.append(np.array([-0.0463563, -0.00099023, 0.1452805]));    # link 3 pm_local:  torso 2 link
        self.pm_local.append(np.array([-0.00157211, -0.00157919, 0.02175767]));  # link 4 pm_local:  head 1 link
        self.pm_local.append(np.array([0.01002657, 5.218e-05, 0.14136068]));     # link 5 pm_local:  head 2 link
        self.pm_local.append(np.array([-0.0002762, 0.10060223, 0.04437419]));    # link 6 pm_local:  arm 1 left link
        self.pm_local.append(np.array([0.01438831, 0.00092938, -0.08684268]));   # link 7 pm_local:  arm 2 left link
        self.pm_local.append(np.array([0.0136084, 0.01241619, -0.2499004]));     # link 8 pm_local:  arm 3 left link
        self.pm_local.append(np.array([-0.00742138, -0.0213895, -0.03312656]));  # link 9 pm_local:  arm 4 left link
        self.pm_local.append(np.array([-6e-05, 0.003262, 0.079625]));            # link 10 pm_local: arm 5 left link
        self.pm_local.append(np.array([2.1e-05, -0.001965, -0.000591]));         # link 11 pm_local: arm 6 left link
        self.pm_local.append(np.array([0.007525, 0.001378, -0.02463]));          # link 12 pm_local: arm 7 left link
        self.pm_local.append(np.array([-0.0002762, -0.10060223, 0.04437419]));   # link 13 pm_local: arm 1 right link
        self.pm_local.append(np.array([0.01438831, -0.00092938, -0.08684268]));  # link 14 pm_local: arm 2 right link
        self.pm_local.append(np.array([0.0136084, -0.01241619, -0.2499004]));    # link 15 pm_local: arm 3 right link
        self.pm_local.append(np.array([-0.00742138, 0.0213895, -0.03312656]));   # link 16 pm_local: arm 4 right link
        self.pm_local.append(np.array([-6e-05, -0.003262, 0.079625]));           # link 17 pm_local: arm 5 right link
        self.pm_local.append(np.array([2.1e-05, 0.001965, -0.000591]));          # link 18 pm_local: arm 6 right link
        self.pm_local.append(np.array([0.007525, -0.001378, -0.02463]));         # link 19 pm_local: arm 7 right link
        self.pm_local.append(np.array([0.02247868, 0.00106736, 0.03130665]));    # link 20 pm_local: leg 1 left link
        self.pm_local.append(np.array([-0.00704703, 0.02592659, 0.00273385]));   # link 21 pm_local: leg 2 left link
        self.pm_local.append(np.array([0.0058523, 0.0636967, -0.18339564]));     # link 22 pm_local: leg 3 left link
        self.pm_local.append(np.array([0.01317717, 0.02917508, -0.11594602]));   # link 23 pm_local: leg 4 left link
        self.pm_local.append(np.array([-0.01400838, 0.04180064, 0.03820186]));   # link 24 pm_local: leg 5 left link
        self.pm_local.append(np.array([-0.02034668, -0.00051514, -0.05987428])); # link 25 pm_local: leg 6 left link
        self.pm_local.append(np.array([0.02247868, -0.00106736, 0.03130665]));   # link 26 pm_local: leg 1 right link
        self.pm_local.append(np.array([-0.00704703, -0.02592659, 0.00273385]));  # link 27 pm_local: leg 2 right link
        self.pm_local.append(np.array([0.0058523, -0.0636967, -0.18339564]));    # link 28 pm_local: leg 3 right link
        self.pm_local.append(np.array([0.01317717, -0.02917508, -0.11594602]));  # link 29 pm_local: leg 4 right link
        self.pm_local.append(np.array([-0.01400838, -0.04180064, 0.03820186]));  # link 30 pm_local: leg 5 right link
        self.pm_local.append(np.array([-0.02034668, 0.00051514, -0.05987428]));  # link 31 pm_local: leg 6 right link

        self.I_bar = []; self.I_bar.append(np.zeros((3, 3)));
        self.I_bar.append(self.I_bar_gen(0.20105075811, 0.00023244734, 0.0040167728, 0.08411496729, -0.00087206649, 0.2318908414));     # link 1 I_bar:  base link
        self.I_bar.append(self.I_bar_gen(0.00638508087, -7.107e-08, -3.065592e-05, 0.00410256102, -1.46946e-06, 0.00622968815));        # link 2 I_bar:  torso 1 link
        self.I_bar.append(self.I_bar_gen(0.44372633826, 0.00069132133, -0.01218206353, 0.2998576068, -0.00019623338, 0.32201554742));   # link 3 I_bar:  torso 2 link
        self.I_bar.append(self.I_bar_gen(0.00224878584, 4.69375e-06, 8.55557e-05, 0.00111158492, -4.132536e-05, 0.00205225921));        # link 4 I_bar:  head 1 link
        self.I_bar.append(self.I_bar_gen(0.01084624339, 1.050889e-05, 0.00041594252, 0.0109569176, 2.367831e-05, 0.00571698895));       # link 5 I_bar:  head 2 link
        self.I_bar.append(self.I_bar_gen(0.01237818683, -3.625571e-05, 7.14472e-05, 0.004191372, -0.00023639064, 0.01358161109));       # link 6 I_bar:  arm 1 left link
        self.I_bar.append(self.I_bar_gen(0.01297822101, 1.208791e-05, -0.00320370433, 0.01380870278, -0.00012770059, 0.00478856621));   # link 7 I_bar:  arm 2 left link
        self.I_bar.append(self.I_bar_gen(0.00718831493, -0.00011563551, 0.00075969733, 0.00693528503, 0.00042134743, 0.00388359007));   # link 8 I_bar:  arm 3 left link
        self.I_bar.append(self.I_bar_gen(0.00251207716, 0.00010070062, -0.00032788214, 0.00275869324, 0.00040022227, 0.00120735959));   # link 9 I_bar:  arm 4 left link
        self.I_bar.append(self.I_bar_gen(0.00349507283, 1.265489e-05, 1.038286e-05, 0.00436830072, -9.736042e-05, 0.0022826337));       # link 10 I_bar: arm 5 left link
        self.I_bar.append(self.I_bar_gen(0.00010700023, -8.899e-08, -4.392e-08, 0.00014101316, 4.1702e-07, 0.00015398658));             # link 11 I_bar: arm 6 left link
        self.I_bar.append(self.I_bar_gen(0.00030894317, -1.58687e-06, 1.73418e-06, 0.00021886181, -1.221167e-05, 0.00017519492));       # link 12 I_bar: arm 7 left link
        self.I_bar.append(self.I_bar_gen(0.01237818683, 3.625571e-05, 7.14472e-05, 0.004191372, 0.00023639064, 0.01358161109));         # link 13 I_bar: arm 1 right link
        self.I_bar.append(self.I_bar_gen(0.01297822101, -1.208791e-05, -0.00320370433, 0.01380870278, 0.00012770059, 0.00478856621));   # link 14 I_bar: arm 2 right link
        self.I_bar.append(self.I_bar_gen(0.00718831493, 0.00011563551, 0.00075969733, 0.00693528503, -0.00042134743, 0.00388359007));   # link 15 I_bar: arm 3 right link
        self.I_bar.append(self.I_bar_gen(0.00251207716, -0.00010070062, -0.00032788214, 0.00275869324, -0.00040022227, 0.00120735959)); # link 16 I_bar: arm 4 right link
        self.I_bar.append(self.I_bar_gen(0.00349507283, -1.265489e-05, 1.038286e-05, 0.00436830072, 9.736042e-05, 0.0022826337));       # link 17 I_bar: arm 5 right link
        self.I_bar.append(self.I_bar_gen(0.00010700023, 8.899e-08, -4.392e-08, 0.00014101316, -4.1702e-07, 0.00015398658));             # link 18 I_bar: arm 6 right link
        self.I_bar.append(self.I_bar_gen(0.00030894317, 1.58687e-06, 1.73418e-06, 0.00021886181, 1.221167e-05, 0.00017519492));         # link 19 I_bar: arm 7 right link
        self.I_bar.append(self.I_bar_gen(0.00541533521, -0.00015428714, 0.00131612622, 0.00848624936, -1.906103e-05, 0.00474512541));   # link 20 I_bar: leg 1 left link
        self.I_bar.append(self.I_bar_gen(0.00497152259, -0.0001311358, -4.86648e-06, 0.00190550993, 4.585381e-05, 0.00474289754));      # link 21 I_bar: leg 2 left link
        self.I_bar.append(self.I_bar_gen(0.15337403915, 0.00068913396, 0.00168034366, 0.13648139346, -0.02187635445, 0.03001594885));   # link 22 I_bar: leg 3 left link
        self.I_bar.append(self.I_bar_gen(0.04287677484, 0.00043705397, 0.00078292368, 0.03598906971, 0.00037717694, 0.01364791564));    # link 23 I_bar: leg 4 left link
        self.I_bar.append(self.I_bar_gen(0.01143430819, 0.00096978273, 0.00039976581, 0.00888974312, -0.00277808667, 0.00506373875));   # link 24 I_bar: leg 5 left link
        self.I_bar.append(self.I_bar_gen(0.00439027267, -6.743674e-05, 0.00113816363, 0.00743063029, 3.943143e-05, 0.00550906699));     # link 25 I_bar: leg 6 left link
        self.I_bar.append(self.I_bar_gen(0.00541533521, 0.00015428714, 0.00131612622, 0.00848624936, 1.906103e-05, 0.00474512541));     # link 26 I_bar: leg 1 right link
        self.I_bar.append(self.I_bar_gen(0.00497152259, 0.0001311358, -4.86648e-06, 0.00190550993, -4.585381e-05, 0.00474289754));      # link 27 I_bar: leg 2 right link
        self.I_bar.append(self.I_bar_gen(0.15337403915, -0.00068913396, 0.00168034366, 0.13648139346, 0.02187635445, 0.03001594885));   # link 28 I_bar: leg 3 right link
        self.I_bar.append(self.I_bar_gen(0.04287677484, -0.00043705397, 0.00078292368, 0.03598906971, -0.00037717694, 0.01364791564));  # link 29 I_bar: leg 4 right link
        self.I_bar.append(self.I_bar_gen(0.01143430819, -0.00096978273, 0.00039976581, 0.00888974312, 0.00277808667, 0.00506373875));   # link 30 I_bar: leg 5 right link
        self.I_bar.append(self.I_bar_gen(0.00439027267, 6.743674e-05, 0.00113816363, 0.00743063029, -3.943143e-05, 0.00550906699));     # link 31 I_bar: leg 6 right link

        self.m_G = 0
        for i in range(1, self.link_num+1):
            self.m_G += self.m[i]

        self.m_larm = 0; self.m_rarm = 0; self.m_lleg = 0; self.m_rleg = 0; 
        for i in range(1, self.link_num+1):
                if(i>=6 and i<=12): # left arm bodies
                    self.m_larm += self.m[i]
                if(i>=13 and i<=19): # right arm bodies
                    self.m_rarm += self.m[i]
                if(i>=20 and i<=25): # left leg bodies
                    self.m_lleg += self.m[i]
                if(i>=26 and i<=31): # left leg bodies
                    self.m_rleg += self.m[i]

        self.I = []; self.I.append(np.zeros((6,6)))
        for i in range(1, self.link_num+1):
            self.I.append( self.I_spatial_generate(self.m[i], self.pm_local[i], self.I_bar[i]) )

        self.link = []; self.link.append(rospy.get_param('~link_1', 'link_1')) # we add link_1 two times
        for i in range(1, self.link_num+1):
            self.link.append(rospy.get_param('~link_'+str(i), 'link_' + str(i)))


    def skew(self, vec):
        A = np.asarray([ [0, -vec[2], vec[1]], [vec[2], 0, -vec[0]], [-vec[1], vec[0], 0]])
        return A

    def I_spatial_generate(self, m, pm_local, I_bar):
        I = np.zeros((6,6))
        I[0:3, 0:3] = I_bar + m * self.skew(pm_local) @ self.skew(pm_local).T
        I[0:3, 3:6] = m * self.skew(pm_local)
        I[3:6, 0:3] = m * self.skew(pm_local).T
        I[3:6, 3:6] = m * np.identity(3)
        return I

    def I_bar_gen(self, i_xx, i_xy, i_xz, i_yy, i_yz, i_zz):
        I_bar = np.asarray([ [i_xx, i_xy, i_xz],
                             [i_xy, i_yy, i_yz],
                             [i_xz, i_yz, i_zz] ])
        return I_bar
