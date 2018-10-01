# PEG BOARD 1
#
# This script sets the positions of PSM2, PSM3R for use for Peg Board 1.
# Only joint angles are used.

import dvrk
import numpy as np

p2 = dvrk.psm('PSM2')
p3 = dvrk.psm('PSM3')

p2.move_joint(np.array([-0.15371328, -0.43780173,  0.14124376,  0.93584622,  0.08170863, -0.03877546]))
p3.move_joint(np.array([0.30278519, -0.3603041 ,  0.1902163 , -1.90419438, -0.0217252, -0.11525754]))