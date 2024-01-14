import argparse
import os
import yaml

from preprocess_pcd import PreprocessPCD
from pcd_registration import PCDRegistration


def main(args):
    """Run the pcd processing and combining workflow

    Args:
        args (input arguments): Args for the pcd processing workflow
    """
    with open(args.pcd_params_file) as f:
        param_data = yaml.load(f, Loader=yaml.FullLoader)
    pcd_file_paths = []

    for pcd_filename in os.listdir(args.pcd_source_dir_path):
        pcd_file_path = os.path.join(args.pcd_source_dir_path, pcd_filename)
        if os.path.isfile(pcd_file_path):
            pcd_file_paths.append(pcd_file_path)

    pcd_file_paths.sort(key=lambda path: int(''.join(filter(str.isdigit, path))))
    print(pcd_file_paths)
    preprocess_pcd = PreprocessPCD(param_data)
    pcd_registration = PCDRegistration(param_data)

    downsampled_pcds = []
    for pcd_file_path in pcd_file_paths:
        print("PCD File", pcd_file_path)
        preprocess_pcd.subtract_background(pcd_file_path)
        downsampled_pcd = preprocess_pcd.downsample_filter_pcd()
        downsampled_pcds.append(downsampled_pcd)

    pcd_registration.run_pcd_registration(downsampled_pcds)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Processes and stitches the PCD obtained from LiDAR"
    )
    parser.add_argument(
        "-pcd_sp",
        "--pcd_source_dir_path",
        type=str,
        help="source directory for the pcd files to process",
    )
    parser.add_argument(
        "-params",
        "--pcd_params_file",
        type=str,
        help="parameters yaml file path",
    )
    parser.add_argument(
        "-pcd_dp",
        "--pcd_dest_dir_path",
        type=str,
        help="pcd destination folder path to save the converted cloud",
    )
    args = parser.parse_args()
    main(args)
