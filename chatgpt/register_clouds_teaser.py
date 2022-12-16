# write a python program to register two point clouds using teaser++
# the program should use two copies of the bunny set as input point clouds and output the transformation matrix
# that aligns the two point clouds

import teaserspp

# load the two bunny point clouds
pc1 = teaserspp.PointCloud('bunny.pcd')
pc2 = teaserspp.PointCloud('bunny.pcd')

# create the teaser++ registration object
reg = teaserspp.Registration()

# set the input point clouds for registration
reg.setInputSource(pc1)
reg.setInputTarget(pc2)

# register the two point clouds
reg.align()

# get the transformation matrix
transform_matrix = reg.getFinalTransformation()

# print the transformation matrix
print(transform_matrix)