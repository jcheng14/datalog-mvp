import argparse

import numpy as np
import open3d as o3d
from colors import get_color
from dataclasses import dataclass


PLANE_PARAMETER_DICT = {
    "stanchion_base_plane_params": data["StanchionBasePlaneSegmentationParams"],
    "stanchion_pole_plane_params": data["StanchionPolePlaneSegmentationParams"],
    "log_plane_params": data["LogPlaneSegmentationParams"],
}


class SegmentPCDPlanes:
    """Class for Segmenting the planes from a cropped pcd"""

    def __init__(self):
        """Construct a PCD plane segmentation class"""

    def find_valid_planes(self, oriented_bboxes):
        """Find the angle of planes that are detected w.r.t x-z plane

        Args:
            oriented_bboxes (o3d.oriented_bboxes): Oriented bounding boxes from the
        """
        valid_planes = []
        # Directional cosines for valid plane segments
        i = 1
        for obox in oriented_bboxes:
            normal_vector = np.array(obox.extent - obox.center)
            vector_magnitude = np.linalg.norm(normal_vector, ord=2)
            angle_x = np.arccos(normal_vector[0] / vector_magnitude) * (180 / np.pi)
            angle_y = np.arccos(normal_vector[1] / vector_magnitude) * (180 / np.pi)
            angle_z = np.arccos(normal_vector[2] / vector_magnitude) * (180 / np.pi)
            print("Iteration", i)
            i += 1
            if (
                angle_z > data["VALID_PLANE_Z_MIN"]
                and angle_z < data["VALID_PLANE_Z_MAX"]
            ) and (
                angle_x < data["VALID_PLANE_X_MIN"]
                or angle_x > data["VALID_PLANE_X_MAX"]
            ):
                valid_planes.append(obox)
        return valid_planes

    def crop_pcd_roi(self, view=False):
        """Crop the stanchion pole and base ROI from the point cloud

        Args:
            view (bool, optional): Boolean to view the cropped point cloud. Defaults to True.

        Returns:
            pcd, pcd: returns the cropped point cloud data of the stanchion poles and the stanchion base
        """
        inlier_array = np.asarray(self.downsampled_pcd.points)
        stanchion_poles = inlier_array[
            inlier_array[:, 2] > data["CROPPED_UPPER_THRESHOLD"]
        ]
        self.stanchion_pole_pcd.points = o3d.utility.Vector3dVector(stanchion_poles)
        self.stanchion_pole_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=data["KD_RADIUS"], max_nn=data["KD_NEIGHBORS"]
            )
        )

        stanchion_base = inlier_array[
            inlier_array[:, 2] < -data["CROPPED_LOWER_THRESHOLD"]
        ]
        self.stanchion_base_pcd.points = o3d.utility.Vector3dVector(stanchion_base)
        self.stanchion_base_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=data["STANCHION_BASE_KD_RADIUS"], max_nn=data["KD_NEIGHBORS"]
            )
        )

        logs_array = inlier_array[inlier_array[:, 2] < data["CROPPED_UPPER_THRESHOLD"]]
        logs_array = logs_array[logs_array[:, 2] > -data["CROPPED_LOWER_THRESHOLD"]]
        self.log_pcd.points = o3d.utility.Vector3dVector(logs_array)
        self.log_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=data["KD_RADIUS"], max_nn=data["KD_NEIGHBORS"]
            )
        )

        if view:
            o3d.visualization.draw_geometries([self.stanchion_pole_pcd])
            o3d.visualization.draw_geometries([self.stanchion_base_pcd])
            o3d.visualization.draw_geometries([self.log_pcd])

        return self.stanchion_pole_pcd, self.stanchion_base_pcd

    def segment_planes(
        self,
        point_cloud,
        normal_variance,
        coplanar_degree,
        outlier_ratio,
        plane_edge_length,
        min_points,
        knn_num,
        view=True,
    ):
        """Segment planes in a cropped point cloud for log parameter estimation

        Args:
            params_name (str): String of the name corresponding to PCD ROI params
            normal_variance (int): Variance in the normal angles of the points in a plane
            coplanar_degree (int): Allowed distribution of the sampled point distance in-terms of the angle
            outlier_ratio (float): ratio of outliers to inliers in the plane
            plane_edge_length (float): minimum plane edge length in metres
            min_points (int): minimum num of points for a plane - decides octree depth
            knn_num (int): nearest neighbors for each pcd point for plane segmentation
            view (bool, optional): Boolean for viewing segmented planes. Defaults to True.

        Returns:
            oboxes(o3d.OrientedBoundingBox): oriented bounding boxes with centres and normals of the plane
        """

        oboxes = point_cloud.detect_planar_patches(
            normal_variance_threshold_deg=normal_variance,
            coplanarity_deg=coplanar_degree,
            outlier_ratio=outlier_ratio,
            min_plane_edge_length=plane_edge_length,
            min_num_points=min_points,
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn_num),
        )

        print("Detected {} num of patches".format(len(oboxes)))

        if view:
            geometries = []
            for obox in oboxes:
                mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(
                    obox, scale=[1, 1, 0.0001]
                )
                mesh.paint_uniform_color(obox.color)
                geometries.append(mesh)
            geometries.append(point_cloud)

            o3d.visualization.draw_geometries(geometries)

        return oboxes

    def view_valid_planes(self, valid_plane_obboxes):
        """Helper function to visualize the valid planar patches for the log-ends

        Args:
            valid_plane_obboxes (Open3d.OrientedBoundingBox):
        """
        geometries = []
        for plane_obbox in valid_plane_obboxes:
            mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(
                plane_obbox, scale=[1, 1, 0.0001]
            )
            mesh.paint_uniform_color(plane_obbox.color)
            geometries.append(plane_obbox)
            geometries.append(mesh)
        geometries.append(self.log_pcd)
        o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=np.array([0.0, 0.0, 0.0])
        )
        o3d.visualization.draw_geometries(geometries)

    def estimate_logend_diameter(self, valid_plane_obboxes):
        """Estimates the Log End Diameter from the segmented planar patches

        Args:
            valid_plane_bboxes (_type_): _description_
        """
        geometries = []
        count = 0
        for plane_obbox in valid_plane_obboxes:
            boundary_points = np.asarray(plane_obbox.get_box_points())
            bounding_polygon = boundary_points.astype("float64")
            # Visualization after cropping with the Axis Aligned Bounding Boxes
            # Create a SelectionPolygonVolume
            vol = o3d.visualization.SelectionPolygonVolume()
            # Specifying the axis to orient the selected polygon to
            vol.orthogonal_axis = "X"
            vol.axis_max = np.max(bounding_polygon[:, 0])
            vol.axis_min = np.min(bounding_polygon[:, 0])
            # Set all the Y values to 0 (they aren't needed since we specified what they
            # should be using just vol.axis_max and vol.axis_min).
            bounding_polygon[:, 0] = 0
            # Convert the np.array to a Vector3dVector
            vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
            cropped_pcd = vol.crop_point_cloud(self.log_pcd)
            color = get_color(count)
            cropped_pcd.paint_uniform_color(np.array(color))
            # Select the max z-coorinate
            cropped_array = np.asarray(cropped_pcd.points)
            # Max and min z coordinates
            points_zmax = cropped_array[np.argmax(cropped_array[:, 2]), :]
            points_zmin = cropped_array[np.argmin(cropped_array[:, 2]), :]
            # Max and min y coordinates
            points_ymax = cropped_array[np.argmax(cropped_array[:, 1]), :]
            points_ymin = cropped_array[np.argmin(cropped_array[:, 1]), :]
            # Rough Empirical Diametrical Calculations
            dia_z = np.linalg.norm(points_zmax - points_zmin)
            dia_y = np.linalg.norm(points_ymax - points_ymin)
            print(" Diameters: {}, {} ".format(dia_z, dia_y))
            count += 1
            geometries.append(cropped_pcd)
        o3d.visualization.draw_geometries(geometries)

    def run_processor(self, args):
        """Runs the Lidar Data Processing

        Args:
            args (Optional Args): Arg for the file path for pcd
        """
        self.subtract_background(args)
        self.downsample_filter_pcd()
        # self.crop_pcd_roi()
        self.downsampled_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=data["KD_RADIUS"], max_nn=data["KD_NEIGHBORS"]
            )
        )

        # Segment planes for the stanchion Base
        for key in PLANE_PARAMETER_DICT:
            if key == "log_plane_params":
                oriented_bboxes = self.segment_planes(
                    self.downsampled_pcd,
                    PLANE_PARAMETER_DICT[key].normal_variance_threshold_deg,
                    PLANE_PARAMETER_DICT[key].coplanarity_deg,
                    PLANE_PARAMETER_DICT[key].outlier_ratio,
                    PLANE_PARAMETER_DICT[key].min_plane_edge_length,
                    PLANE_PARAMETER_DICT[key].min_num_points,
                    PLANE_PARAMETER_DICT[key].knn_num,
                )
            # valid_plane_obboxes = self.find_valid_planes(oriented_bboxes)
            # print("Valid plane bounding boxes", valid_plane_obboxes)
            # print("Detected {} patches".format(len(valid_plane_obboxes)))
            # if key == "log_plane_params":
            #     self.estimate_logend_diameter(valid_plane_obboxes)
