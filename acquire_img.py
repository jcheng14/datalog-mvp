import argparse 
import os
import PySpin
import socket
import sys

from datetime import date
from datetime import datetime
from PIL import Image

NUM_IMAGES = 1 # Number of Continuous image to capture
CAMERA_FRAME_TIME = 2000 # Milli-Seconds
PATH_CAMERA_RAW_DATA = os.getcwd() + "/CameraRawData/Camera-raw-first-fieldtest"
PATH_CAMERA_CALIBRATION_DATA = os.getcwd() + "/CameraRawData/CalibrationData"

def acquire_images(cam, nodemap, nodemap_tldevice, args):
    """
    This function acquires and saves 10 images from a device.

    Args: 
        cam(CameraPtr): Camera to acquire images from.
        nodemap(INodeMap): Device nodemap.
        nodemap_tldevice(INodeMap): Transport layer device nodemap. 
    Returns:
        bool : True if successful, False otherwise.
    """

    print('*** IMAGE ACQUISITION ***\n')
    try:
        result = True
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        if not PySpin.IsReadable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
            return False
        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        if not PySpin.IsReadable(node_acquisition_mode_continuous):
            print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
            return False
        # Retrieve integer value from entry node
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
        # Set integer value from entry node as new value of enumeration node
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
        print('Acquisition mode set to continuous...')

        node_pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
        if not PySpin.IsReadable(node_pixel_format) or not PySpin.IsWritable(node_pixel_format):
            print('Unable to set pixel format node (enum retrieval). Aborting...')
            return False
        # Retrieve entry node from enumeration node
        node_pixel_format_BGR8 = node_pixel_format.GetEntryByName('BGR8')
        if not PySpin.IsReadable(node_pixel_format_BGR8):
            print('Unable to set pixel format node (entry retrieval). Aborting...')
            return False
        # Retrieve integer value from entry node
        pixel_format_BGR8 = node_pixel_format_BGR8.GetValue()
        # Set integer value from entry node as new value of enumeration node
        node_pixel_format.SetIntValue(pixel_format_BGR8)
        print('Pixel format set to BGR8 - (24bit 3-channel)')

        cam.BeginAcquisition()

        print('Acquiring images...')
        #  Retrieve device serial number for filename
        #  The device serial number is retrieved in order to keep cameras from
        #  overwriting one another. Grabbing image IDs could also accomplish
        #  this.
        device_serial_number = ''
        device_ip_address = 0
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        node_device_camera_ip = PySpin.CIntegerPtr(nodemap_tldevice.GetNode('GevDeviceIPAddress'))
        if PySpin.IsReadable(node_device_serial_number):
            device_serial_number = node_device_serial_number.GetValue()
            print('Device serial number retrieved as %s...' % device_serial_number)
        if PySpin.IsReadable(node_device_camera_ip):
            device_ip_address = node_device_camera_ip.GetValue()
            print("Device IP %d..." % device_ip_address)

        timeout = 0
        if cam.ExposureTime.GetAccessMode() == PySpin.RW or cam.ExposureTime.GetAccessMode() == PySpin.RO:
            # The exposure time is retrieved in µs so it needs to be converted to ms to keep consistency with the unit being used in GetNextImage
            timeout = (int)(cam.ExposureTime.GetValue() / 1000 + 1000)
        else:
            print ('Unable to get exposure time. Aborting...')
            return False


        given_ip_address = int(socket.inet_aton(args.ip_address).hex(), 16)
        print("Given IP %d" % given_ip_address)
        if device_ip_address == given_ip_address: 
            print("Camera IP matches with given IP")
        # Close program
            print('Press enter to close the program..')

            for i in range(NUM_IMAGES):
                try:
                    image_result = cam.GetNextImage(CAMERA_FRAME_TIME)
                    #  Ensure image completion
                    if image_result.IsIncomplete():
                        print('Image incomplete with image status %d ...' % image_result.GetImageStatus())
                    else:
                        #  Images have quite a bit of available metadata including
                        #  height, width, CRC, image status, and offset values
                        width = image_result.GetWidth()
                        height = image_result.GetHeight()
                        print('Grabbed Image %d, width = %d, height = %d' % (i, width, height))
                        if args.calibration_images and args.focal_length: 
                            filename = PATH_CAMERA_CALIBRATION_DATA + '/img-%s-%s-%d.jpg' % (str(date.today()), \
                                                            str(datetime.now().strftime("%H:%M:%S")), args.focal_length)
                        else:  # if serial number is empty
                            filename = PATH_CAMERA_RAW_DATA + '/img-%d-%s-%s-%d.jpg' % ( args.load_type, str(args.load_end), str(args.ip_address[-3:]),args.stop)
                        image_data = image_result.GetNDArray()
                        # print("IMage data array shape", image_data.shape)
                        # print("IMage data from camera", image_data)
                        # Converting BGR to RGB Images
                        acquired_image = Image.fromarray(image_data[...,::-1])
                        acquired_image.save(filename)
                        print('Image saved at %s' % filename)
                        #  Release image
                        #  Images retrieved directly from the camera (i.e. non-converted
                        #  images) need to be released in order to keep from filling the
                        #  buffer.
                        image_result.Release()
                        print('')

                except PySpin.SpinnakerException as ex:
                    print('Error: %s' % ex)
                    return False
        else:
            print("Camera IP does not match, ending camera Acquisition")
        #  End acquisition
        #  Ending acquisition appropriately helps ensure that devices clean up.
        cam.EndAcquisition()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def run_single_camera(cam, args):
    """
    This function acts as the body of the example; please see NodeMapInfo example
    for more in-depth comments on setting up cameras.

    :param cam: Camera to run on.
    :type cam: CameraPtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Retrieve TL device nodemap and print device information
        nodemap_tldevice = cam.GetTLDeviceNodeMap()
        
        # Optional to have a device info node 
        # result &= print_device_info(nodemap_tldevice)

        # Initialize camera
        cam.Init()

        # Retrieve GenICam nodemap
        nodemap = cam.GetNodeMap()

        # Acquire images
        result &= acquire_images(cam, nodemap, nodemap_tldevice, args)

        # Deinitialize camera
        cam.DeInit()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result


def main(args):
    """
    Example entry point; please see Enumeration example for more in-depth
    comments on preparing and cleaning up the system.

    Returns: 
        bool: True if successful, False otherwise.
    """

    # Since this application saves images in the current folder
    # we must ensure that we have permission to write to this folder.
    # If we do not have permission, fail right away.
    try:
        test_file = open('test.txt', 'w+')
    except IOError:
        print('Unable to write to current directory. Please check permissions.')
        input('Press Enter to exit...')
        return False

    test_file.close()
    os.remove(test_file.name)

    result = True
    # Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()
    # Get current library version
    version = system.GetLibraryVersion()
    print('Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))
    # Retrieve list of cameras from the system
    cam_list = system.GetCameras()
    num_cameras = cam_list.GetSize()
    print('Number of cameras detected: %d' % num_cameras)

    # Finish if there are no cameras
    if num_cameras == 0:
        # Clear camera list before releasing system
        cam_list.Clear()
        # Release system instance
        system.ReleaseInstance()
        print('Not enough cameras!')
        input('Done! Press Enter to exit...')
        return False

    # Run example on each camera
    for i, cam in enumerate(cam_list):
        print('Running example for camera %d...' % i)
        result &= run_single_camera(cam, args)
        print('Camera %d example complete... \n' % i)

    # Release reference to camera
    # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
    # cleaned up when going out of scope.
    # The usage of del is preferred to assigning the variable to None.
    del cam

    # Clear camera list before releasing system
    cam_list.Clear()

    # Release system instance
    system.ReleaseInstance()
    input('Done! Press Enter to exit...')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Converts csv files to .pcd files")
    parser.add_argument("-s", "--stop", type=int, help="distance of camera from log ends")
    # parser.add_argument("-ver", "--vertical", type=int, help="height of the camera from the ground")
    parser.add_argument("-cal", "--calibration_images", type=bool, help="Images for Camera Lens Calibration")
    parser.add_argument("-f", "--focal_length", type=int, help="Focal-length of the lens used")
    parser.add_argument("-l", "--load_type",type=int, help="load type of the load" )
    parser.add_argument("-le", "--load_end", type=str, help="starting alphabet of the load")
    parser.add_argument("-ip", "--ip_address", type=str, help="IP-address of the desired camera")
    args = parser.parse_args()
    if main(args):
        sys.exit(0)
    else:
        sys.exit(1)