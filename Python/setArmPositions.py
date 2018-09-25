# This script sets the positions of (PSM1), PSM2, PSM3 for use for the pickup camera study.
# Only joint angles are used.

import dvrk
import numpy as np

# p1 = dvrk.psm('PSM1')
p2 = dvrk.psm('PSM2')
p3 = dvrk.psm('PSM3')

p2.move_joint(np.array([-0.57006125, -0.36600348,  0.13136603,  3.62281315,  0.2317355, -0.06130089]))
p3.move_joint(np.array([0.59405578, -0.24744408,  0.17206953, -0.88363342, -0.00874474, -0.11612247]))