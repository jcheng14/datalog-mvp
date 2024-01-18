import open3d as o3d

# Load the PCD file
pcd_file = "freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_quick_start/test_whole_cloud_20hz.pcd"
pcd = o3d.io.read_point_cloud(pcd_file)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])