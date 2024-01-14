import numpy as np
import open3d as o3d


class PCDRegistration:
    """Class to stitch pcd"""

    def __init__(self, param_data) -> None:
        """_summary_"""
        self.pose_graph = o3d.pipelines.registration.PoseGraph()
        self.param_data = param_data

    def pairwise_registration(self, source, target):
        """_summary_

        Args:
            source (_type_): _description_
            target (_type_): _description_
            coarse_correspondence_dist (_type_): _description_
            fine_correspondence_dist (_type_): _description_

        Returns:
            _type_: _description_
        """
        print("Apply point-to-plane ICP")
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source,
            target,
            self.param_data["max_correspondence_distance_coarse"],
            np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        icp_fine = o3d.pipelines.registration.registration_icp(
            source,
            target,
            self.param_data["max_correspondence_distance_fine"],
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        transformation_icp = icp_fine.transformation
        information_icp = (
            o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                source,
                target,
                self.param_data["max_correspondence_distance_fine"],
                icp_fine.transformation,
            )
        )
        return transformation_icp, information_icp

    def full_registration(self, pcds):
        """_summary_

        Args:
            pcds (_type_): _description_

        Returns:
            _type_: _description_
        """
        odometry = np.identity(4)
        self.pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)
        for source_id in range(n_pcds):
            for target_id in range(source_id + 1, n_pcds):
                transformation_icp, information_icp = self.pairwise_registration(
                    pcds[source_id],
                    pcds[target_id],
                )
                print("Build o3d.pipelines.registration.PoseGraph")
                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    self.pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            np.linalg.inv(odometry)
                        )
                    )
                    self.pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(
                            source_id,
                            target_id,
                            transformation_icp,
                            information_icp,
                            uncertain=False,
                        )
                    )
                else:  # loop closure case
                    self.pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(
                            source_id,
                            target_id,
                            transformation_icp,
                            information_icp,
                            uncertain=True,
                        )
                    )

    def run_pcd_registration(self, downsampled_pcds):
        """_summary_

        Args:
            downsampled_pcds (_type_): _description_
            param_data (_type_): _description_
        """
        with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug
        ) as cm:
            self.full_registration(downsampled_pcds)
        print("Optimizing PoseGraph ...")
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=self.param_data[
                "max_correspondence_distance_fine"
            ],
            edge_prune_threshold=0.25,
            reference_node=0,
        )
        with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug
        ) as cm:
            o3d.pipelines.registration.global_optimization(
                self.pose_graph,
                o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                option,
            )
        for point_id in range(len(downsampled_pcds)):
            print(self.pose_graph.nodes[point_id].pose)
            downsampled_pcds[point_id].transform(self.pose_graph.nodes[point_id].pose)
            o3d.visualization.draw_geometries(downsampled_pcds)
