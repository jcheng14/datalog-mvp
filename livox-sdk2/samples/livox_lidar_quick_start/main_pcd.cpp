//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "pcd_file.h"

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>

PcdFileHandle pcd_file_handler;
std::list<LivoxHAPBasePackDetail> point_packet_list;
std::mutex mtx;
std::condition_variable point_pack_condition;
#define FRAME_RATE 20

using namespace std::chrono;

int pcd_file_save_time = 10;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)
{
    if (data == nullptr)
    {
        return;
    }
    // TODO(RD): Figure out storing the handle to HAPBasePackDetail for
    // handling multiple lidars based on the handle
    std::unique_lock<std::mutex> lock(mtx);
    LivoxHAPBasePackDetail packet;
    pcd_file_handler.LidarPointsHandle(data, packet);
    point_packet_list.push_back(packet);
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)
{
    if (data == nullptr)
    {
        return;
    }
    // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
    //        handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

void WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data)
{
    if (response == nullptr)
    {
        return;
    }
    printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
           status, handle, response->ret_code, response->error_key);
}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse *response, void *client_data)
{
    if (response == nullptr)
    {
        return;
    }
    printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
           status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data)
{
    if (response == nullptr)
    {
        return;
    }
    printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
           status, handle, response->ret_code, response->error_key);

    if (response->ret_code == 0 && response->error_key == 0)
    {
        LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
    }
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

    uint8_t host_point_ipaddr[4]{0};
    uint16_t host_point_port = 0;
    uint16_t lidar_point_port = 0;

    uint8_t host_imu_ipaddr[4]{0};
    uint16_t host_imu_data_port = 0;
    uint16_t lidar_imu_data_port = 0;

    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i)
    {
        LivoxLidarKeyValueParam *kv = (LivoxLidarKeyValueParam *)&response->data[off];
        if (kv->key == kKeyLidarPointDataHostIPCfg)
        {
            memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
        }
        else if (kv->key == kKeyLidarImuHostIPCfg)
        {
            memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }

    printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
           host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

    printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
           host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo *info, void *client_data)
{
    if (info == nullptr)
    {
        printf("lidar info change callback failed, the info is nullptr.\n");
        return;
    }
    printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

int main(int argc, const char *argv[])
{
    if (argc != 2)
    {
        printf("Params Invalid, must input config path.\n");
        return -1;
    }
    const std::string path = argv[1];

    if (!LivoxLidarSdkInit(path.c_str()))
    {
        printf("Livox Init Failed\n");
        LivoxLidarSdkUninit();
        return -1;
    }
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    steady_clock::time_point last_time = steady_clock::now();
    for (int i = 0; i < 1; ++i)
    {
        std::list<LivoxHAPBasePackDetail> point_packet_list_temp;
        {
            std::unique_lock<std::mutex> lock(mtx);
            point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
            printf("Time waiting: %lld", milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
            last_time = steady_clock::now();
            point_packet_list_temp.swap(point_packet_list);
        }
        if (point_packet_list_temp.empty())
        {
            printf("Point cloud packet is empty.\n");
            break;
        }

        printf("Finish save %d frame to pcd file.\n", i);
        pcd_file_handler.savePackettoPcdFile("test_whole_cloud_20hz.pcd", point_packet_list_temp);
    }

    pcd_file_handler.ClosePcdFile();

#ifdef WIN32
    Sleep(3000);
#else
    sleep(300);
#endif
    LivoxLidarSdkUninit();
    printf("Livox Quick Start Demo End!\n");
    return 0;
}
