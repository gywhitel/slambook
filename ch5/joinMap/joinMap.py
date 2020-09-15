import cv2
import pcl
import pcl.pcl_visualization
import numpy as np
import quaternion

# camera intrinsics
cx = 325.5
cy = 253.5
fx = 518.0
fy = 519.0
depthScale = 1000.0

colorImgs, depthImgs = [], []
pose = []

# read pose.txt
pose = []
with open('pose.txt', 'r') as f:
    for line in f.readlines():
        line = line.replace('\n', '')  # remove returns
        line = line.split(' ')  # split into 7 items
        vector = []
        for num in line:
            vector.append(float(num))
        vector = np.array(vector)
    # compute Rotation matrix based on quaternion
        quater = quaternion.Quaternion(vector[6], vector[3], vector[4], vector[5])
        R = quater.transformToRotationMatrix()
    # translation matrix
        position = np.array([vector[0], vector[1], vector[2]])  #x, y, z
        trans = quaternion.translation(position)
    # pose matrix
        # T =  trans * R  # wrong multiplication, should use dot
        
        T = np.dot(trans, R)
        pose.append(T)

# read color and depth images
view = []
for i in range(1,6):
    colorImg = cv2.imread(f"color/{i}.png")
    # cv2.imshow(f"Image{i}", colorImg)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    depthImg = cv2.imread(f"depth/{i}.pgm", cv2.IMREAD_UNCHANGED)

    height, width, channel  = colorImg.shape
    for v in range(height):
        for u in range(width):
        # get RBG value
            b = colorImg.item(v, u, 0)
            g = colorImg.item(v, u, 1)
            r = colorImg.item(v, u, 2)
            # pack RGB into PointXYZRGB structure, refer to PCL
            # http://pointclouds.org/documentation/structpcl_1_1_point_x_y_z_r_g_b.html#details
            # rgb = int(str(r)+str(g)+str(b))
            rgb = r << 16 | g << 8 | b

            depth = depthImg[v,u]
            if depth == 0:
                continue
            z = depth / depthScale
            x = (u - cx) * z/fx
            y = (v - cy) * z/fy
            # using homogeneous coordinate
            point = np.array([x, y, z, 1])
            point_world = np.dot(pose[i-1], point)
            # x,y,z,rgb
            # scene = np.insert(point_world, 3, rgb)
            scene = np.array([point_world[0], point_world[1], point_world[2], rgb], dtype=np.float32)
            view.append(scene)

colorCloud = pcl.PointCloud_PointXYZRGB()
colorCloud.from_list(view)
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(colorCloud, b"cloud")
v = True
while v:
    v = not(visual.WasStopped())