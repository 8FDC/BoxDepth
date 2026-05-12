from PIL import Image
from pyrsistent import b
from DepthPro.src.depth_pro import depth_pro
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np



image_path = "sample/box_image.jpg"
K = [
    [612.052241109163, 0.769465996625406, 640.559565043889],
    [0, 610.346303928357, 347.65333415667],
    [0, 0, 1]
    ]
box_length = 900
box_width = 610
camera_height = 1060



"""Step1"""
# Load model and preprocessing transform
model, transform = depth_pro.create_model_and_transforms()
model.eval()

# Load and preprocess an image.
image, _, f_px = depth_pro.load_rgb(image_path)
image = transform(image)

# Run inference.
prediction = model.infer(image, f_px=f_px)
depth_prediction = prediction["depth"]
plt.imsave("sample/depth_prediction.png", depth_prediction, cmap="jet_r")


"""Step2"""
fx = K[0][0]
fy = K[1][1]
cx = K[0][2]
cy = K[1][2]

h, w = image.shape[:2]
depth = o3d.geometry.Image(depth_prediction.astype(np.float32))
intrinsics = o3d.camera.PinholeCameraIntrinsic()
intrinsics.set_intrinsics(w, h, fx, fy, cx, cy)

# Create point cloud from depth image and intrinsics
pcd = o3d.geometry.PointCloud.create_from_depth_image(depth, intrinsics)



"""Step3"""
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5, max_nn=20))
normals = np.asarray(pcd.normals)
normals = normals.reshape(h, w, 3)

# Classify normals into 5 categories based on their orientation
left = np.zeros((h, w)).astype(bool)
right = np.zeros((h, w)).astype(bool)
front = np.zeros((h, w)).astype(bool)
back = np.zeros((h, w)).astype(bool)
bottom = np.zeros((h, w)).astype(bool)
# Calculate the cosine similarity between the normal vector and the 5 directions
for i in range(h):
    for j in range(w):
        n = normals[i, j]
        if np.linalg.norm(n) == 0:
            continue
        n = n / np.linalg.norm(n)
        cos_left = np.dot(n, [-1, 0, 0])
        cos_right = np.dot(n, [1, 0, 0])
        cos_front = np.dot(n, [0, -1, 0])
        cos_back = np.dot(n, [0, 1, 0])
        cos_bottom = np.dot(n, [0, 0, -1])
        max_cos = max(cos_left, cos_right, cos_front, cos_back, cos_bottom)
        
        if max_cos == cos_left: left[i, j] = True
        elif max_cos == cos_right: right[i, j] = True
        elif max_cos == cos_front: front[i, j] = True
        elif max_cos == cos_back: back[i, j] = True
        else: bottom[i, j] = True



"""Step4"""
R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
T = [[0], [0], [0]]

def compute_depth(a, b, d, u, v): return abs((d-b)/(a[0]*u + a[1]*v + a[2]))

K_inv= np.linalg.inv(K)
R_inv = np.linalg.inv(R)
A = R_inv @ K_inv
B = -R_inv @ T

depth = np.zeros((h, w))

for i in range(h):
    for j in range(w):
        compute = False
        if left[i][j] == True or right[i][j] == True:
            compute = True
            a = [A[0,0], A[0,1], A[0,2]]
            b = B[0]
            d = box_length/2
        elif front[i][j] ==True or back[i][j] == True:
            compute = True
            a = [A[1,0], A[1,1], A[1,2]]
            b = B[1]
            d = box_width/2
        u = j
        v = i
        if compute:
            depth[i][j] = compute_depth(a, b, d, u, v)
depth[bottom==True] = camera_height

depth[depth==0] = np.nan
depth[depth>camera_height] = np.nan

plt.imsave("sample/depth_computed.png", depth, cmap="jet_r")
np.save("sample/depth_computed.npy", depth)