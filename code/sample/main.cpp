// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <k4a/k4a.h>
#include <math.h>

#include <fstream>
#include <iostream>
#include <k4abt.hpp>
#include <sstream>
#include <string>

static void create_xy_table(const k4a_calibration_t *calibration,
                            k4a_image_t xy_table) {
  k4a_float2_t *table_data =
      (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

  int width = calibration->depth_camera_calibration.resolution_width;
  int height = calibration->depth_camera_calibration.resolution_height;

  k4a_float2_t p;
  k4a_float3_t ray;
  int valid;

  for (int y = 0, idx = 0; y < height; y++) {
    p.xy.y = (float)y;
    for (int x = 0; x < width; x++, idx++) {
      p.xy.x = (float)x;

      k4a_calibration_2d_to_3d(calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH,
                               K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

      if (valid) {
        table_data[idx].xy.x = ray.xyz.x;
        table_data[idx].xy.y = ray.xyz.y;
      } else {
        table_data[idx].xy.x = nanf("");
        table_data[idx].xy.y = nanf("");
      }
    }
  }
}

static void generate_point_cloud(const k4a_image_t depth_image,
                                 const k4a_image_t xy_table,
                                 k4a_image_t point_cloud, int *point_count) {
  int width = k4a_image_get_width_pixels(depth_image);
  int height = k4a_image_get_height_pixels(depth_image);

  uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
  k4a_float2_t *xy_table_data =
      (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
  k4a_float3_t *point_cloud_data =
      (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

  *point_count = 0;
  for (int i = 0; i < width * height; i++) {
    if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) &&
        !isnan(xy_table_data[i].xy.y)) {
      point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
      point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
      point_cloud_data[i].xyz.z = (float)depth_data[i];
      (*point_count)++;
    } else {
      point_cloud_data[i].xyz.x = nanf("");
      point_cloud_data[i].xyz.y = nanf("");
      point_cloud_data[i].xyz.z = nanf("");
    }
  }
}

static void write_point_cloud(const k4a_image_t point_cloud, int point_count) {
  int width = k4a_image_get_width_pixels(point_cloud);
  int height = k4a_image_get_height_pixels(point_cloud);
  k4a_float3_t *point_cloud_data =
      (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

  std::ofstream ofs("image.data");  // text mode first
  ofs << "x,y,z" << std::endl;

  ofs.close();
  std::stringstream ss;
  for (int i = 0; i < width * height; i++) {
    if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) ||
        isnan(point_cloud_data[i].xyz.z)) {
      continue;
    }

    ss << (int)point_cloud_data[i].xyz.x << ","
       << (int)point_cloud_data[i].xyz.y << ","
       << (int)point_cloud_data[i].xyz.z << std::endl;
  }

  std::ofstream ofs_text("image.data", std::ios::out | std::ios::app);
  ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

static void print_body_information(k4abt_body_t body) {
  printf("body detected\n");
  std::ofstream ofs("joint.data");  // text mode first
  ofs << "joint_id,x,y,z,confidence" << std::endl;
  ofs.close();
  std::stringstream ss;
  for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
    k4a_float3_t position = body.skeleton.joints[i].position;
    k4abt_joint_confidence_level_t confidence_level =
        body.skeleton.joints[i].confidence_level;
    ss << i << "," << (int)position.v[0] << "," << (int)position.v[1] << ","
       << (int)position.v[2] << "," << (int)confidence_level << std::endl;
  }

  std::ofstream ofs_text("joint.data", std::ios::out | std::ios::app);
  ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
  ofs.close();
}

int main(int argc, char **argv) {
  int returnCode = 1;
  k4a_device_t device = NULL;
  const int32_t TIMEOUT_IN_MS = 1000;
  k4a_capture_t capture = NULL;
  std::string file_name;
  uint32_t device_count = 0;
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  k4a_image_t depth_image = NULL;
  k4a_image_t xy_table = NULL;
  k4a_image_t point_cloud = NULL;
  int point_count = 0;

  file_name = "data";

  device_count = k4a_device_get_installed_count();

  if (device_count == 0) {
    printf("No K4A devices found\n");
    return 0;
  }
  while (true) {
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device)) {
      printf("Failed to open device\n");
      break;
    }

    // camera config for tracking
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;

    // tracker config CPU
    k4abt_tracker_configuration_t tracker_config = {
        K4ABT_SENSOR_ORIENTATION_DEFAULT};
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;

    // calibration
    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode,
                                   config.color_resolution, &calibration)) {
      printf("Failed to get calibration\n");
      break;
    }

    // create images for point cloud
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration.depth_camera_calibration.resolution_width,
                     calibration.depth_camera_calibration.resolution_height,
                     calibration.depth_camera_calibration.resolution_width *
                         (int)sizeof(k4a_float2_t),
                     &xy_table);

    create_xy_table(&calibration, xy_table);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration.depth_camera_calibration.resolution_width,
                     calibration.depth_camera_calibration.resolution_height,
                     calibration.depth_camera_calibration.resolution_width *
                         (int)sizeof(k4a_float3_t),
                     &point_cloud);

    // start camera
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config)) {
      printf("Failed to start cameras\n");
      break;
    }

    // Get a capture handle
    k4a_wait_result_t capture_result =
        k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS);
    if (capture_result != K4A_WAIT_RESULT_SUCCEEDED) {
      printf("capture error");
      break;
    }

    // Get a depth image
    depth_image = k4a_capture_get_depth_image(capture);

    // tracker
    k4abt::tracker tracker =
        k4abt::tracker::create(calibration, tracker_config);
    if (!tracker.enqueue_capture(capture)) {
      std::cout << "Error! tracking failed" << std::endl;
      break;
    }

    // gets body frame from tracker
    k4abt::frame body_frame = tracker.pop_result();

    tracker.shutdown();

    if (body_frame == nullptr) {
      std::cout << "Error! Pop body frame result time out!" << std::endl;
      break;
    }

    // prints body data
    k4abt_body_t body = body_frame.get_body(0);
    print_body_information(body);

    if (depth_image == 0) {
      printf("Failed to get depth image from capture\n");
      returnCode = 1;
      break;
    }
    // prints point cloud
    generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);
    write_point_cloud(point_cloud, point_count);

    // release objects
    k4a_image_release(depth_image);
    k4a_image_release(xy_table);
    k4a_image_release(point_cloud);

    returnCode = 0;
    break;
  }
  if (device != NULL) {
    k4a_device_close(device);
  }

  return returnCode;
}