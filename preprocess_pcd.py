import numpy as np
import open3d as o3d


class PreprocessPCD:
    """Pre-process the pcd by background subtraction and downsampling"""

    def __init__(self, data) -> None:
        """Construct a PCD Pre-processor class

        Args:
            data (pcd-parameters): Parameters for pcd pre-processing
        """
        self.subtracted_pcd = o3d.geometry.PointCloud()
        self.downsampled_pcd = o3d.geometry.PointCloud()
        self.stanchion_base_pcd = o3d.geometry.PointCloud()
        self.stanchion_pole_pcd = o3d.geometry.PointCloud()
        self.log_pcd = o3d.geometry.PointCloud()
        self.data = data

    def subtract_background(self, pcd_file_path):
        """Subtract the back ground from the point cloud

        Args:
            pcd_file_path (os.file_path): Path for the Pcd file to be pre-processed
        """
        pcd_load = o3d.io.read_point_cloud(pcd_file_path)
        #  Add code for the rotation
        # o3d.visualization.draw_geometries([pcd_load])
        pcd_array = np.array(pcd_load.points)
        print("Pcd before background", pcd_array.shape)
        # Checking the x and the y axis for depth and the height threshold and storing only those pcd
        # The lidar is rotated along the x axis with a negative 90 - making the y of the lidar axis align with the
        # assumed height direction
        subtracted_pcd_array = pcd_array[
            pcd_array[:, 0] < self.data["BACKGROUND_DEPTH_THRESHOLD"]
        ]
        subtracted_pcd_array = subtracted_pcd_array[
            subtracted_pcd_array[:, 1] > self.data["BACKGROUND_HEIGHT_THRESHOLD"]
        ]
        print("Pcd after background subtraction", subtracted_pcd_array.shape)
        self.subtracted_pcd.points = o3d.utility.Vector3dVector(subtracted_pcd_array)
        # o3d.visualization.draw_geometries([self.subtracted_pcd])

    def downsample_filter_pcd(self):
        """Downsample the point cloud and remove outliers for further pcd processing"""
        self.downsampled_pcd = self.subtracted_pcd.voxel_down_sample(
            voxel_size=self.data["DOWNSAMPLE_VOXEL_SIZE"]
        )
        downsampled_array = np.asarray(self.downsampled_pcd.points)
        print("Downsampled num of pcd", downsampled_array.shape)
        # Apply Statistical Outlier
        _, ind = self.downsampled_pcd.remove_statistical_outlier(
            nb_neighbors=self.data["OUTLIER_NUM_NEIGHBORS"],
            std_ratio=self.data["OUTLIER_STD_RATIO"],
        )
        self.downsampled_pcd = self.downsampled_pcd.select_by_index(ind)
        self.downsampled_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.data["KD_RADIUS"], max_nn=self.data["KD_NEIGHBORS"]
            )
        )
        # o3d.visualization.draw_geometries([self.downsampled_pcd])

        return self.downsampled_pcd
