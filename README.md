# BoxDepth

A monocular metric depth estimation approach for box interior images leveraging depth estimation priors.

## Introduction

Existing monocular depth estimation models, when not fine-tuned or trained on a specific scenario dataset, usually exhibit scale bias to varying degrees. This causes large discrepancies between estimated depth and true depth, making them difficult to use directly in visual measurement systems. This method leverages the relative depth output (with scale bias) from a depth estimation model, combined with **camera intrinsics** and **physical box dimensions**, to estimate the depth of each inner wall region of the box in the camera coordinate system. The related method has been granted an invention patent by the China National Intellectual Property Administration ([CN119648771A](https://patents.google.com/patent/CN119648771A/en?oq=CN119648771A)).

<table align="center" style="margin: 0 auto;">
  <tr>
    <td align="center">
      <img src="./sample/box_image.jpg" alt="box image" height="200" />
    </td>
    <td align="center">
      <img src="./sample/depth_computed.png" alt="height" height="200" />
    </td>
  </tr>
  <tr>
    <td align="center">
      Input image
    </td>
    <td align="center">
      Computed depth result
    </td>
  </tr>
</table>

### Data to prepare in advance

* **Camera intrinsics** (obtainable via calibration)
* **Box dimensions** (length, width, height)
* **Distance from the camera image plane to the box bottom**
* **Box image** (the camera optical center should be as aligned as possible with the geometric center of the box bottom)

### Step 1: Use a depth estimation model to predict relative depth

The depth prediction should be approximately linear with the true depth:

<p align="center">
  <img src="./assets/depth_prediction_scatter.png" alt="box image" height="400" />
</p>

You can use Depth Pro or DepthAnything v2 (metric depth version). They are intended to predict metric depth, but their results still have unavoidable scale shifts relative to the true depth.

### Step 2: Generate a point cloud from the predicted depth map

The back-projection formula from a depth map to a point cloud is:

$$
\begin{bmatrix} x_c \\\\ y_c \\\\ z_c \end{bmatrix} = z_c K^{-1} \begin{bmatrix} u \\\\ v \\\\ 1 \end{bmatrix}
$$

Here, $(u,v)$ are pixel coordinates, $z_c$ is the depth value at that pixel, and $K$ is the camera intrinsics, defined as:

$$
K = \begin{bmatrix} f_x & \alpha & u_0 \\\\ 0 & f_y & v_0 \\\\ 0 & 0 & 1 \end{bmatrix}
$$

You can use Open3D's `o3d.geometry.PointCloud.create_from_depth_image(depth, intrinsics)` to perform the conversion.

### Step 3: Compute per-pixel normals from the point cloud and segment planes using normals

Ideally, the normals of the five sides of the box (top-down view)—front, back, left, right, and bottom—should be $(0,-1,0)$, $(0,1,0)$, $(-1,0,0)$, $(1,0,0)$, and $(0,0,1)$, respectively. Determine the plane assignment of each pixel by computing the **cosine similarity** between its normal and these five reference normals:

$$
\text{CosineSimilarity} = \cos\langle \mathbf{n}_{\text{pixel}}, \mathbf{n}_{s} \rangle = \frac{\mathbf{n}_{\text{pixel}} \cdot \mathbf{n}_{s}}{\|\mathbf{n}_{\text{pixel}}\| \|\mathbf{n}_{s}\|}
$$

### Step 4: Map to true depth

According to the camera imaging model, the relationship between points in the image coordinate system and the world coordinate system is:

$$
z_c \begin{bmatrix} u \\\\ v \\\\ 1 \end{bmatrix} = \begin{bmatrix} K & 0 \end{bmatrix} \begin{bmatrix} R & T \\\\ 0 & 1 \end{bmatrix} \begin{bmatrix} x_w \\\\ y_w \\\\ z_w \\\\ 1 \end{bmatrix}
$$

This yields:

$$
\begin{bmatrix} x_w \\\\ y_w \\\\ z_w \end{bmatrix} = R^{-1} \left( K^{-1} \begin{bmatrix} u \\\\ v \\\\ 1 \end{bmatrix} z_c - T \right)
$$

For simplicity, this method assumes the world and camera coordinate systems coincide. Let $R^{-1}K^{-1} = A(a_{ij})$ and $-R^{-1}T = B(b_i)$. The ray equation through a pixel is:

$$
\begin{cases} x_w = (a_{11}u + a_{12}v + a_{13})z_c + b_1 \\\\ y_w = (a_{21}u + a_{22}v + a_{23})z_c + b_2 \\\\ z_w = (a_{31}u + a_{32}v + a_{33})z_c + b_3 \end{cases}
$$

Given the inner box length (length) and width (width), the four inner wall plane equations are:

$$
\begin{aligned} P_{\text{left}}: & \quad y_w = \frac{\text{length}}{2} \\\\ P_{\text{right}}: & \quad y_w = -\frac{\text{length}}{2} \\\\ P_{\text{front}}: & \quad x_w = \frac{\text{width}}{2} \\\\ P_{\text{back}}: & \quad x_w = -\frac{\text{width}}{2} \end{aligned}
$$

Solving the plane equations together with the ray equation, the resulting $z_c$ is the **true depth** for that pixel:

$$
\begin{aligned} \text{depth}_{\text{left}} &= \frac{\text{length}/2 - b_2}{a_{21}u + a_{22}v + a_{23}} \\\\ \text{depth}_{\text{right}} &= \frac{-\text{length}/2 - b_2}{a_{21}u + a_{22}v + a_{23}} \\\\ \text{depth}_{\text{front}} &= \frac{\text{width}/2 - b_1}{a_{11}u + a_{12}v + a_{13}} \\\\ \text{depth}_{\text{back}} &= \frac{-\text{width}/2 - b_1}{a_{11}u + a_{12}v + a_{13}} \end{aligned}
$$

Since the distance from the camera to the box bottom (height) is known, the depth for the bottom region is:

$$
\text{depth}_{\text{bottom}} = \text{height}
$$

## Acknowledgements

Some code in this project comes from Depth Pro.

```bibtex
@inproceedings{Bochkovskii2024:arxiv,
  author    = {Aleksei Bochkovskii and Ama\"{e}l Delaunoy and Hugo Germain and Marcel Santos and
               Yichao Zhou and Stephan R. Richter and Vladlen Koltun},
  title     = {Depth Pro: Sharp Monocular Metric Depth in Less Than a Second},
  booktitle  = {International Conference on Learning Representations},
  year       = {2025},
  url        = {[https://arxiv.org/abs/2410.02073](https://arxiv.org/abs/2410.02073)},
}