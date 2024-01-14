import argparse
import math

import numpy as np
import open3d as o3d


BACKGROUND_DEPTH_THRESHOLD = 2.1336  # In metres # 12ft (3.6576 m) - 15ft (4.572 m)
BACKGROUND_WIDTH_THRESHOLD = 0.9144  # In metres # 3ft
GROUND_THRESHOLD = 0.7500
DOWNSAMPLE_VOXEL_SIZE = 0.003

subtracted_pcd = o3d.geometry.PointCloud()
downsampled_pcd = o3d.geometry.PointCloud()
pcd_load = o3d.io.read_point_cloud("/workspace/LidarPCD/pcd-calibration-apr25-in-5ftwd-3.4166ftht-12fl/pcd-calibration-apr25-in-5ftwd-3.4166ftht-12fl-1.pcd")
o3d.visualization.draw_geometries([pcd_load])
pcd_array = np.asarray(pcd_load.points)
print("Pcd after subtracting background", pcd_array.shape)
# Checking the x and the y axis for depth and width threshold and storing only those pcd
subtracted_pcd_array = pcd_array[pcd_array[:, 0] < BACKGROUND_DEPTH_THRESHOLD]
subtracted_pcd_array = subtracted_pcd_array[
    subtracted_pcd_array[:, 1] < BACKGROUND_WIDTH_THRESHOLD
]
subtracted_pcd_array = subtracted_pcd_array[
    subtracted_pcd_array[:, 1] > -BACKGROUND_WIDTH_THRESHOLD
]
subtracted_pcd_array = subtracted_pcd_array[
            subtracted_pcd_array[:, 2] > -GROUND_THRESHOLD
        ]

subtracted_pcd.points = o3d.utility.Vector3dVector(subtracted_pcd_array)
o3d.visualization.draw_geometries([subtracted_pcd])
downsampled_pcd = subtracted_pcd.voxel_down_sample(voxel_size=DOWNSAMPLE_VOXEL_SIZE)
downsampled_array = np.asarray(downsampled_pcd.points)
print("Downsampled num of pcd", downsampled_array.shape)
# Apply Statistical Outlier 
# _, ind = downsampled_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
# downsampled_pcd = downsampled_pcd.select_by_index(ind)
downsampled_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
        )

print("")
# print(
#     "1) Please pick at least three correspondences using [shift + left click]"
# )
# print("   Press [shift + right click] to undo point picking")
# print("2) After picking points, press 'Q' to close the window")
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(downsampled_pcd)
vis.run()  # user picks points
vis.destroy_window()
selected_points = vis.get_picked_points()



o3d.visualization.draw_geometries([downsampled_pcd])


plane_model, inliers = downsampled_pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
inlier_cloud = downsampled_pcd.select_by_index(inliers)
o3d.visualization.draw_geometries([inlier_cloud])

oboxes_calib = inlier_cloud.detect_planar_patches(
    normal_variance_threshold_deg=70,
    coplanarity_deg=75,
    outlier_ratio=0.40,
    min_plane_edge_length=0.75,
    min_num_points=50,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30),
)

geometries = []
for obox in oboxes_calib:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(
        obox, scale=[1, 1, 0.0001]
    )
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
geometries.append(inlier_cloud)

o3d.visualization.draw_geometries(geometries)
