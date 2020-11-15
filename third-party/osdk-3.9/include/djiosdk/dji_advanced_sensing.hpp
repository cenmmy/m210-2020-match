/*! @file dji_advanced_sensing.hpp
 *  @version 3.4.0
 *  @date Dec 2017
 *
 *
 *  @Copyright (c) 2016-17 DJI.
 *  NOTE THAT this file is part of the advanced sensing
 *  closed-source library. For licensing information,
 *  please visit https://developer.dji.com/policies/eula/
 * */

#ifndef ONBOARDSDK_DJI_ADVANCED_SENSING_H
#define ONBOARDSDK_DJI_ADVANCED_SENSING_H

#include "dji_type.hpp"
#include <cstring>
#include "dji_advanced_sensing_protocol.hpp"
#include "dji_version.hpp"

#include "dji_camera_stream.hpp"

namespace DJI {
namespace OSDK {

// Forward Declaration
class Vehicle;

#pragma pack(1)
typedef struct DataConfig
{
  uint32_t  data_flag : 1;
  uint32_t  data_freq : 4;
  uint32_t  data_reserved : 27;
} DataConfig;

typedef struct ImgConfig
{
  uint32_t img_flag : 1;
  uint32_t img_freq : 4;
  uint32_t img_resolution : 4;
  uint32_t img_reserved : 23;
} ImgConfig;

typedef struct VGAConfig
{
  ImgConfig     image_selection[2][CAMERA_PAIR_NUM];
  DataConfig    reserved_0;
  DataConfig    reserved_1;
  DataConfig    reserved_2;
  DataConfig    reserved_3;
  DataConfig    reserved_4;
} VGAConfig;

typedef struct AdvancedSensingConfig
{
  bool        is_stereo_img_subscribed;
  uint32_t    image_selected[CAMERA_PAIR_NUM][IMAGE_TYPE_NUM];
  bool        is_vga_img_subscribed;
  VGAConfig   vga_subscription;
} AdvancedSensingConfig;
#pragma pack()

/*!
 * @class AdvancedSensing class contains APIs to access stereo camera, FPV camera
 * and the main gimbaled camera on M210 and M210RTK
 */
class AdvancedSensing {
public:
#pragma pack(1)
  /*!
   * @brief This struct provides an interface for user to determine
   * what images to subscribe to
   */
  typedef struct ImageSelection {
    uint16_t front_left       : 1;
    uint16_t front_right      : 1;
    uint16_t down_front       : 1;
    uint16_t down_back        : 1;
    uint16_t back_left        : 1;
    uint16_t back_right       : 1;
    uint16_t reserved         : 10;
  } ImageSelection;
#pragma pack()
  /*!
   * @brief This struct is used to pair img data with camera
   */
  enum ReceivedImgDesc
  {
    RECV_FRONT_LEFT   = 10,
    RECV_FRONT_RIGHT  = 11,
    RECV_DOWN_BACK    = 0,
    RECV_DOWN_FRONT   = 1,
    RECV_FRONT_DEPTH  = 15,
  };

public:
  AdvancedSensing(Vehicle* vehiclePtr);

  ~AdvancedSensing();
  /*! @brief subscribe to QVGA (240x320) stereo images at 20 fps
   *
   *  @param images to subscribe
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void subscribeStereoImages(const ImageSelection *select, VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief subscribe to VGA (480x640) front stereo images at 10 or 20 fps
   *
   *  @param frequency of images using enum from AdvancedSensingProtocol::FREQ
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void subscribeFrontStereoVGA(const uint8_t freq, VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief subscribe to QVGA (240x320) stereo depth map at 10 fps
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void subscribeFrontStereoDisparity(VehicleCallBack callback = 0, UserData userData = 0);
  /*!
   * @brief unsubscribe to QVGA (240x320) stereo depth map or images
   */
  void unsubscribeStereoImages();
  /*!
   * @brief unsubscribe to VGA (480x640) stereo images
   */
  void unsubscribeVGAImages();
  /*! @brief
   *
   *  A default callback function for QVGA stereo images
   *
   *  @param vehicle pointer
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void stereoCallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData);
  /*! @brief
   *
   *  A default callback function for VGA stereo images
   *
   *  @param vehicle pointer
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void VGACallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData);
  /*! @brief
   *
   *  Start the FPV Camera Stream
   *
   *  @param cb callback function that is called in a callback thread when a new
   *            image is received and decoded
   *  @param cbParam a void pointer that users can manipulate inside the callback
   *  @return true if successfully started, false otherwise
   */
  bool startFPVCameraStream(CameraImageCallback cb = NULL, void * cbParam = NULL);
  /*! @brief
   *
   *  Start the Main Camera Stream
   *
   *  @param cb callback function that is called in a callback thread when a new
   *            image is received and decoded
   *  @param cbParam a void pointer that users can manipulate inside the callback
   *  @return true if successfully started, false otherwise
   */
  bool startMainCameraStream(CameraImageCallback cb = NULL, void * cbParam = NULL);
  /*! @brief
   *
   *  Stop the FPV Camera Stream
   */

  void setAcmDevicePath(const char *acm_path);

  void stopFPVCameraStream();
  /*! @brief
   *
   *  Stop the Main Camera Stream
   */
  void stopMainCameraStream();
  /*! @brief Check if a new image from the FPV camera is received
   *
   *  @return true if a new image frame is ready, false otherwise
   */
  bool newFPVCameraImageIsReady();
  /*! @brief Check if a new image from the main camera is received
   *
   *  @return true if a new image frame is ready, false otherwise
   */
  bool newMainCameraImageReady();
  /*! @brief Get a copy of the new image from the FPV camera
   *
   *  @param A copy of the new available image will be put here.
   *         It is safe for user to manipulate this image.
   *  @note If a new image is not ready upon calling this function,
   *        it will wait for 30ms till timeout.
   *
   *  @return true if a new image frame is ready, false if timeout
   */
  bool getFPVCameraImage(CameraRGBImage& copyOfImage);
  /*! @brief Get a copy of the new image from the main camera
   *
   *  @param A copy of the new available image will be put here.
   *         It is safe for user to manipulate this image.
   *  @note If a new image is not ready upon calling this function,
   *        it will wait for 30ms till timeout.
   *
   *  @return true if a new image frame is ready, false if timeout
   */
  bool getMainCameraImage(CameraRGBImage& copyOfImage);


private:
void sendCommonCmd(uint8_t *data, uint8_t data_len, uint8_t cmd_id);

private:
AdvancedSensingProtocol* advancedSensingProtocol;
Vehicle* vehicle_ptr;
DJICameraStream* mainCam_ptr;
DJICameraStream* fpvCam_ptr;
const char* acm_dev;

public:
AdvancedSensingProtocol* getAdvancedSensingProtocol();

public:
VehicleCallBackHandler stereoHandler;
VehicleCallBackHandler vgaHandler;
};

} // OSDK
} // DJI

#endif //ONBOARDSDK_DJI_ADVANCED_SENSING_H
