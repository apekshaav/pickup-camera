# PEG BOARD 2
#
# This script sets the positions of PSM2, PSM3, MTML, MTMR for use for Peg Board 2.
# Only joint angles are used.

import dvrk
import numpy as np

p2 = dvrk.psm('PSM2')
p3 = dvrk.psm('PSM3')

ml = dvrk.mtm('MTML')
mr = dvrk.mtm('MTMR')

p2.move_joint(np.array([]))
p3.move_joint(np.array([]))
ml.move_joint(np.array([]))
mr.move_joint(np.array([]))