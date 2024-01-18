#include "livox_lidar_def.h"
#include <memory>
#include <fstream>
#include <list>
#include <vector>
#include <mutex>
#include <condition_variable>

// TODO: Tune this later 
#define kDefaultFrameDurationTime 1000
#define kDefaultNumPoints 96
#define kMaxpointsize 1380 

// Need to update the data filed to raw_point with a max size
typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;      /**< unit: 0.1 us */
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;
  uint8_t timestamp[8];
  LivoxLidarCartesianHighRawPoint raw_point[kDefaultNumPoints]; 
  uint8_t pack_size;          
} LivoxHAPBasePackDetail;


class PcdFileHandle {
    public:
        PcdFileHandle();
        // bool InitPcdFile();
        void savePackettoPcdFile(const std::string& filename, std::list<LivoxHAPBasePackDetail> &point_packet_list_temp);
        void ClosePcdFile();

        // Might not need this - need to go through this once more.
        void LidarPointsHandle(LivoxLidarEthernetPacket *data, LivoxHAPBasePackDetail &packet);
    private:
        std::ofstream pcd_file_;
};
