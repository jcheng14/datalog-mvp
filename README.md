# datalog-mvp #

### Setting up camera drivers 

* Download the camera SDK and the Python Wrapper from the [FLIR Website](https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-download/?pn=Spinnaker+SDK&vn=Spinnaker_SDK)
* Extract the SDK Debian package and the Python directory with the .whl file
* Copy the Package and Python whl directory to the project directory for installing everything through docker using the commands below

### Container build and setup ###
* Make sure that Docker is installed on the PC you are trying to run the container with
* Clone the repository and enter the repo and build the container using 

```
$ git clone https://bitbucket.org/freshconsulting/datalog-mvp/src/develop/
$ cd datalog-mvp 
$ make build 
```
After building the docker image, launch the container using the image and start developing
```
$ make dev
```
### Getting and viewing the PCD Data from Livox LiDAR ###

* Use the Livox SDK to generate the .lvx file 

```
$ cd /Livox-SDK
$ cd build/sample/lidar_lvx_file
```
* Run the `lidar_lvx_sample` executable in that directory using: 

```
$ ./lidar_lvx_sample
```
* The `.lvx` file(s) get generated with the pcd data from the Livox LiDAR and are stored in the  same `build` directory

* Now, run the livox viewer through the following commands in the terminal

```
$ cd /Livox_Viewer_For_Linux_Ubuntu16.04_x64_0.10.0
$ ./livox_viewer.sh
```
* Once inside the Livox Viewer: 
    * Click on the Tools-> File Converter -> Select Lvx to CSV on the top-left scroll button 
    * Soure file : Navigate to the `.lvx` source file in the `Livox-SDK/build/sample/lidar_lvx_file`and select it
    * Target file : Navigate to the folder to where you want to save the corresponding `.csv` file
    * Store the csv files in the `/workspace/LidarCSV/` folder 

* In the Workspace folder, run the `csv_to_pcd.py` file to generate binary compressed `.pcd` files with

```
$ cd /workspace 
$ python3 csv_to_pcd.py --v 
```

#### LiDAR for Field Testing

* For the field testing - Livox HAP uses the [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) for direct pcd retrieval. 

### Getting and viewing the Image Data from FLIR Cameras

* FLIR also offers Spinviewer which can be used to view the image streams from the cameras and set up some image properties like 
    * White Balance 
    * Exposure Setting 
    * Gain Balance 

* Image Capture from the FLIR Cameras is done using the Spinnaker API. Also, some of the above mentioned image properties can 
also be set using the API functionality

* In the Workspace folder, run the `acquire_img.py` file to capture the images from the FLIR Cameras

```
$ cd /workspace 
$ python3 acquire_img.py 
```
