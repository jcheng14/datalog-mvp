#include <time.h>
#include <cmath>
#include <cstring>
#include "pcd_file.h"
#include "livox_lidar_def.h"
#include <pcl/io/pcd_io.h>

// N = 96 for High and Low Precision Datatypes for the raw x,y,z points
// as stated : 
//https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP(English)#23-Data-Types
#define RAW_POINT_NUM 96


PcdFileHandle::PcdFileHandle(){}

// bool PcdFileHandle::InitPcdFile() {
//     time_t curtime = time(nullptr);
//     char filename[30] = { 0 };
//     tm* local_time = localtime(&curtime);
//     strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx", local_time);
//     pcd_file_.open(filename);

//     if (!pcd_file_.is_open()) {
//     return false;
//   }
//   return true;
// }


void PcdFileHandle::LidarPointsHandle(LivoxLidarEthernetPacket *data, LivoxHAPBasePackDetail &packet){
// HAP(Tx does not have a sphericalrawpoint)
//  printf("data_num: %d, data_type: %d, length: %d, frame_counter: %d\n", \
    data->dot_num, data->data_type, data->length, data->frame_cnt);
  packet.version = data->version;
  packet.length = data->length;
  packet.time_interval = data->time_interval;
  packet.dot_num = data->dot_num;
  packet.udp_cnt = data->udp_cnt;
  packet.frame_cnt = data->frame_cnt;
  packet.data_type = data->data_type;
  packet.time_type = data->time_type;
  memcpy(packet.timestamp, data->timestamp, 12 * sizeof(uint8_t));
  memcpy(packet.timestamp, data->timestamp, 8 * sizeof(uint8_t));
  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    packet.pack_size = sizeof(LivoxHAPBasePackDetail) - sizeof(packet.raw_point) - \
          sizeof(packet.pack_size) + RAW_POINT_NUM*sizeof(LivoxLidarCartesianHighRawPoint);
    memcpy(packet.raw_point,(void *)data->data, RAW_POINT_NUM*sizeof(LivoxLidarCartesianHighRawPoint));
  }
  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    packet.pack_size = sizeof(LivoxHAPBasePackDetail) - sizeof(packet.raw_point) - \
          sizeof(packet.pack_size) + RAW_POINT_NUM*sizeof(LivoxLidarCartesianLowRawPoint);
    memcpy(packet.raw_point,(void *)data->data, RAW_POINT_NUM*sizeof(LivoxLidarCartesianLowRawPoint));
  }
}


void PcdFileHandle::savePackettoPcdFile(const std::string& filename, std::list<LivoxHAPBasePackDetail> &point_packet_list_temp){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  
  for (const auto& data_packet : point_packet_list_temp) {
    // Write code to parse each block of 96 pcd points 
    // replace place holder with the parsed data block 
    for(uint32_t j = 0; j < kDefaultNumPoints ; j++){
      pcl::PointXYZ point;
      point.x = data_packet.raw_point[j].x;
      point.y = data_packet.raw_point[j].y;
      point.z = data_packet.raw_point[j].z;
      cloud.push_back(point);
    }
  }

  pcl::io::savePCDFileASCII (filename, cloud);
  std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

}


void PcdFileHandle::ClosePcdFile() {
  if (pcd_file_.is_open())
    pcd_file_.close();
}
