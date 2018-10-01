# PEG BOARD 2
#
# This script sets the positions of PSM2, PSM3 for use for Peg Board 2.
# Only joint angles are used.

import dvrk
import numpy as np

p2 = dvrk.psm('PSM2')
p3 = dvrk.psm('PSM3')

p2.move_joint(np.array([-0.62631364, -0.36844386,  0.18214041,  4.52531867,  0.12707194, 0.06929524]))
p3.move_joint(np.array([0.62046753, -0.05186587,  0.20644066, -0.98847839, -0.1194203, -0.15655996]))