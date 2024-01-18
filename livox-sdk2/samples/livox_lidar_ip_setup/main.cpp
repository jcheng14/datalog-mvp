#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <condition_variable>
#include <mutex>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <set>

LivoxLidarIpInfo lidar_ip_info;


void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
      status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (status != kLivoxLidarStatusSuccess) {
    printf("Setting IP failed :%d\n", status);
    SetLivoxLidarIp(handle,  &lidar_ip_info, SetIpInfoCallback, nullptr);
    return;
  }

    if (response->ret_code != 0) {
    printf("Setting IP failed, the response ret_Code:%d.\n", response->ret_code);
    SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    printf("Setting IP failed, the response is nullptr.\n");
    SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
    return;
  }

 
  LivoxLidarRequestReboot(handle, RebootCallback, nullptr);

  printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

}


void QueryInternalInfoCallback(livox_status status, uint32_t handle,
                               LivoxLidarDiagInternalInfoResponse *response, void *client_data)
{
    if (status != kLivoxLidarStatusSuccess)
    {
        printf("Query lidar internal info failed.\n");
        QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
        return;
    }

    if (response == nullptr)
    {
        return;
    }

    uint8_t lidar_ipaddr[4]{0};

    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i)
    {
        LivoxLidarKeyValueParam *kv = (LivoxLidarKeyValueParam *)&response->data[off];
        if (kv->key == kKeyLidarIPCfg)
        {
            printf("Lidar IP Address: %u.%u.%u.%u\n", kv->value[0], kv->value[1], kv->value[2], kv->value[3]);
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }
  
}


void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    return;
  }
  printf("LidarInfoChangeCallback Lidar handle: %u\n SN: %s\n Lidar IP Address : %s\n", handle, info->sn, info->lidar_ip);

  strcpy(lidar_ip_info.ip_addr, "192.168.1.101"); 
  strcpy(lidar_ip_info.net_mask, "255.255.255.0");
  strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
  
  SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr); 
  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr); 

}




int main(int argc, const char *argv[]) {
    if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }


  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

#ifdef WIN32
  Sleep(3000);
#else
  sleep(5);
#endif

  LivoxLidarSdkUninit();
  printf("Livox IP Setup Done!\n");
  return 0;
}
