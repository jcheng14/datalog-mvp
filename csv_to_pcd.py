# Ref: https://github.com/PARPedraza/VelodyneVeloviewFiles-CSV-to-PCD-and-Viewing/blob/master/FilesPCD.py#L94

import argparse 
import csv 
import os 
import pprint

import numpy as np 
import open3d as o3d 
import pandas as pd 
from pypcd import pypcd 

class ConvertCSVtoPCD(object):
    """Converter class for t
    """
    def __init__(self):
        """Constructor for ConvertCSVtoPCD Object
        """
        self.dir=os.getcwd()
        self.exCSV=".csv"
        self.exPCD=".pcd"

    def readCSV(self, file):
        """Opens and Reads CSV file with pcd data

        Args:
            file (filename): Filename for the CSV file

        Returns:
            pcd_array (Numpy Array): pcd data as numpy array
        """
        pcd_data = pd.read_csv(file)
        pcd_array = np.array(pcd_data)
        return pcd_array

    def readPCD(self,file):
        """Reads the pcd data, prints the pcd metadata from the .pcd file

        Args:
            file (filename): Filename for the CSV file

        Returns:
            cloud (pypcd cloud object): pypcd cloud with pcd metadata
        """
        cloud = pypcd.PointCloud.from_path(file)
        pprint.pprint(cloud.get_metadata())
        print(cloud.pc_data)
        return cloud

    def writePCD(self,file, pcd_array,pcd_dest_path):
        """Writes the pcd data to .pcd file from a .csv file

        Args:
            file (filename): Filename for the .pcd file to store the pcd data
            pcd_array (Numpy Array): Numpy Array with pcd data from the .csv file
        """
        # Write the Correct CSV Columns 
        X = pcd_array[:, 8]
        Y = pcd_array[:, 9]
        Z = pcd_array[:, 10]
        RGB = pcd_array[:, 11]
        # Convert the RGB 0 - 255 int to 0 - 1 float value
        RGB_float = np.float32(RGB/255)
        pcd_format_data = np.float32(np.column_stack((X, Y, Z, RGB_float)))
        point_cloud = pypcd.make_xyz_rgb_point_cloud(pcd_format_data)
        pprint.pprint(point_cloud.get_metadata())
        # Store the cloud uncompressed
        fileName = pcd_dest_path +"/" + file[:-4] + self.exPCD
        point_cloud.save_pcd(fileName)

    def visualizePCD(self,file):
        """Launches the Open3D Viewer and opens the pcd in the viewer

        Args:
            file (filename): Filename for the .pcd file
        """
        cloud = o3d.io.read_point_cloud(file)
        # The following code achieves the same effect as:
        # o3d.visualization.draw_geometries([cloud])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(cloud)
        vis.run()
        vis.destroy_window()


    def runConverter(self, args):
        """Runs the Converter functions that reads converts and views the pcd

        Args:
            args (Optional Args): Optional function arguments for visualizing pcd
        """
        for file in os.listdir(args.csv_folder_path):
            if file.endswith(self.exCSV):
                csv_file_path = args.csv_folder_path + "/" + file
                pcd_dest_path = args.pcd_dest_path
                pcd_array = self.readCSV(csv_file_path)
                self.writePCD(file, pcd_array, pcd_dest_path)

    def view_converted_pcd(self, args):
        """Views the Converter PCD that reads converts and views the pcd

        Args:
            args (Optional Args): Optional function arguments for visualizing pcd
        """
        pcd_file_path = args.pcd_file_path
        if pcd_file_path.endswith(self.exPCD): 
            self.readPCD(pcd_file_path)
            self.visualizePCD(pcd_file_path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Converts csv files to .pcd files")
    parser.add_argument("-v", "--view_pcd", type=bool, help="Boolean to visualizes the point cloud from written pcd file")
    parser.add_argument("-pcd_fp", "--pcd_file_path", type=str, help="pcd file to visualize the point cloud")
    parser.add_argument("-pcd_dp", "--pcd_dest_path", type=str, help="pcd destination folder path to save the converted cloud")
    parser.add_argument("-csv_fp", "--csv_folder_path", type=str, help="csv file folder path to convert to pcd ")
    parser.add_argument("-c", "--convert_csv", type=bool, help="Boolean to write the point cloud from a csv file")
    args = parser.parse_args()
    csv_pcd = ConvertCSVtoPCD()
    if args.view_pcd and args.pcd_file_path:
        csv_pcd.view_converted_pcd(args)
    if args.convert_csv and args.csv_folder_path and args.pcd_dest_path: 
        csv_pcd.runConverter(args)
