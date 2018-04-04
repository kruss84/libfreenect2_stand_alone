/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file libfreenect2.hpp Header file of the Freenect2 library. */

#ifndef LIBFREENECT2_HPP_
#define LIBFREENECT2_HPP_

#include <libfreenect2/config.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/packet_pipeline.h>
#include <string>

namespace libfreenect2
{

/** @defgroup device Initialization and Device Control
 * Find, open, and control Kinect v2 devices. */
///@{

/** Device control. */
class LIBFREENECT2_API Freenect2Device
{
public:
  static const unsigned int VendorId = 0x045E;
  static const unsigned int ProductId = 0x02D8;
  static const unsigned int ProductIdPreview = 0x02C4;

  /** Color camera calibration parameters.
   * Kinect v2 includes factory preset values for these parameters. They are used in Registration.
   */
  struct ColorCameraParams
  {
    /** @name Intrinsic parameters */
    ///@{
    float fx; ///< Focal length x (pixel)
    float fy; ///< Focal length y (pixel)
    float cx; ///< Principal point x (pixel)
    float cy; ///< Principal point y (pixel)
    ///@}

    /** @name Extrinsic parameters
     * These parameters are used in [a formula](https://github.com/OpenKinect/libfreenect2/issues/41#issuecomment-72022111) to map coordinates in the
     * depth camera to the color camera.
     *
     * They cannot be used for matrix transformation.
     */
    ///@{
    float shift_d, shift_m;

    float mx_x3y0; // xxx
    float mx_x0y3; // yyy
    float mx_x2y1; // xxy
    float mx_x1y2; // yyx
    float mx_x2y0; // xx
    float mx_x0y2; // yy
    float mx_x1y1; // xy
    float mx_x1y0; // x
    float mx_x0y1; // y
    float mx_x0y0; // 1

    float my_x3y0; // xxx
    float my_x0y3; // yyy
    float my_x2y1; // xxy
    float my_x1y2; // yyx
    float my_x2y0; // xx
    float my_x0y2; // yy
    float my_x1y1; // xy
    float my_x1y0; // x
    float my_x0y1; // y
    float my_x0y0; // 1
    ///@}
  };

  /** IR camera intrinsic calibration parameters.
   * Kinect v2 includes factory preset values for these parameters. They are used in depth image decoding, and Registration.
   */
  struct IrCameraParams
  {
    float fx; ///< Focal length x (pixel)
    float fy; ///< Focal length y (pixel)
    float cx; ///< Principal point x (pixel)
    float cy; ///< Principal point y (pixel)
    float k1; ///< Radial distortion coefficient, 1st-order
    float k2; ///< Radial distortion coefficient, 2nd-order
    float k3; ///< Radial distortion coefficient, 3rd-order
    float p1; ///< Tangential distortion coefficient
    float p2; ///< Tangential distortion coefficient
  };

  /** Configuration of depth processing. */
  struct Config
  {
    float MinDepth;             ///< Clip at this minimum distance (meter).
    float MaxDepth;             ///< Clip at this maximum distance (meter).

    bool EnableBilateralFilter; ///< Remove some "flying pixels".
    bool EnableEdgeAwareFilter; ///< Remove pixels on edges because ToF cameras produce noisy edges.

    /** Default is 0.5, 4.5, true, true */
    Config();
  };

  virtual ~Freenect2Device();

  virtual std::string getSerialNumber() = 0;
  virtual std::string getFirmwareVersion() = 0;

  /** Get current color parameters.
   * @copydetails ColorCameraParams
   */
  virtual ColorCameraParams getColorCameraParams() = 0;

  /** Get current depth parameters.
   * @copydetails IrCameraParams
   */
  virtual IrCameraParams getIrCameraParams() = 0;

  /** Replace factory preset color camera parameters.
   * We do not have a clear understanding of the meaning of the parameters right now.
   * You probably want to leave it as it is.
   */
  virtual void setColorCameraParams(const ColorCameraParams &params) = 0;

  /** Replace factory preset depth camera parameters.
   * This decides accuracy in depth images. You are recommended to provide calibrated values.
   */
  virtual void setIrCameraParams(const IrCameraParams &params) = 0;

  /** Configure depth processing. */
  virtual void setConfiguration(const Config &config) = 0;

  /** Provide your listener to receive color frames. */
  virtual void setColorFrameListener(FrameListener* rgb_frame_listener) = 0;

  /** Provide your listener to receive IR and depth frames. */
  virtual void setIrAndDepthFrameListener(FrameListener* ir_frame_listener) = 0;

  /** Start data processing.
   * All above configuration must only be called before start() or after stop().
   *
   * FrameListener will receive frames when the device is running.
   *
   * @return Undefined. To be defined in 0.2.
   */
  virtual bool start() = 0;

  /** Stop data processing.
   *
   * @return Undefined. To be defined in 0.2.
   */
  virtual bool stop() = 0;

  /** Shut down the device.
   *
   * @return Undefined. To be defined in 0.2.
   */
  virtual bool close() = 0;
};

class Freenect2Impl;

/**
 * Library context to find and open devices.
 *
 * You will first find existing devices by calling enumerateDevices().
 *
 * Then you can openDevice() and control the devices with returned Freenect2Device object.
 *
 * You may open devices with custom PacketPipeline.
 * After passing a PacketPipeline object to libfreenect2 do not use or free the object,
 * libfreenect2 will take care. If openDevice() fails the PacketPipeline object will get
 * deleted. A new PacketPipeline object has to be created each time a device is opened.
 */
class LIBFREENECT2_API Freenect2
{
public:
  /**
   * @param usb_context If the libusb context is provided,
   * Freenect2 will use it instead of creating one.
   */
  Freenect2(void *usb_context = 0);
  virtual ~Freenect2();

  /** Must be called before doing anything else.
   * @return Number of devices, 0 if none
   */
  int enumerateDevices();

  /**
   * @param idx Device index
   * @return Device serial number, or empty if the index is invalid.
   */
  std::string getDeviceSerialNumber(int idx);

  /**
   * @return Device serial number, or empty if no device exists.
   */
  std::string getDefaultDeviceSerialNumber();

  /** Open device by index with default pipeline.
   * @param idx Index number. Index numbers are not determinstic during enumeration.
   * @return New device object, or NULL on failure
   */
  Freenect2Device *openDevice(int idx);

  /** Open device by index.
   * @param idx Index number. Index numbers are not determinstic during enumeration.
   * @param factory New PacketPipeline instance. This is always automatically freed.
   * @return New device object, or NULL on failure
   */
  Freenect2Device *openDevice(int idx, const PacketPipeline *factory);

  /** Open device by serial number with default pipeline.
   * @param serial Serial number
   * @return New device object, or NULL on failure
   */
  Freenect2Device *openDevice(const std::string &serial);

  /** Open device by serial number.
   * @param serial Serial number
   * @param factory New PacketPipeline instance. This is always automatically freed.
   * @return New device object, or NULL on failure
   */
  Freenect2Device *openDevice(const std::string &serial, const PacketPipeline *factory);

  /** Open the first device with default pipeline.
   * @return New device object, or NULL on failure
   */
  Freenect2Device *openDefaultDevice();

  /** Open the first device.
   * @param factory New PacketPipeline instance. This is always automatically freed.
   * @return New device object, or NULL on failure
   */
  Freenect2Device *openDefaultDevice(const PacketPipeline *factory);
private:
  Freenect2Impl *impl_;
};

///@}
} /* namespace libfreenect2 */
#endif /* LIBFREENECT2_HPP_ */
