# This script sets the positions of (PSM1), PSM2, PSM3 for use for the pickup camera study.
# Only joint angles are used.

import dvrk
import numpy as np

# p1 = dvrk.psm('PSM1')
p2 = dvrk.psm('PSM2')
p3 = dvrk.psm('PSM3')

p2.move_joint(np.array([-0.58946075, -0.36493774,  0.12518143,  1.98052152, -0.01707956, 0.11390202]))
p3.move_joint(np.array([0.63817575, -0.26843447,  0.17182972, -0.88342373, -0.01448347, -0.14256252]))