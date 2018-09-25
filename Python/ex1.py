import numpy as np 

x = y = z = np.array([0.0, 5.0, 1.0])
count = 1
filename = "positions" + str(count)
np.savez(filename, x=x, y=y, z=z)
print "displaying"
path = "~/catkin_ws_1_6/src/pickup_camera/Python/" + filename + ".npz"
data = np.load(path)
data.files
data['x']
data.close()
