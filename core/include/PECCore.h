/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2019 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __PE_CCore_H__
#define __PE_CCore_H__
#include <string>
// #include "PECCalibrationSummary.h"
// #include "PECNormalisation.h"

/**
 * class PECCore core functionality of Position Engine
 */
class PECCore
{

friend class PECCoreTest;

public:
   /**
    * Constructor
    */
   PECCore();
   /**
    * Destructor
    */
   ~PECCore();
   /**
    * Initialises and starts position engine
    * @return   true if it started with no error
    *
    * @param[in] cfg   string to configuration information
    */
   bool Start(const std::string& cfg);
   /**
    * Stops position engine
    * @return   string of configuration information
    */
   const std::string& Stop();
   /**
    * Cleans all internal values
    */
   void Clean();
   /**
    * Calculates position based on given coordinates, headings, speeds and sensors
    */
   void Calculate();
   /**
    * Sends new coordinates
    *
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] latitude    latitude in degrees (0..+/-90)
    * @param[in] longitude   longitude in degrees (0..+/-180) 
    * @param[in] accuracy    expectation area of position with radius around given coordinates in meters
    */
   void SendCoordinates( const double& timestamp, const double& latitude, const double& longitude, const double& accuracy);
   /**
    * Sends new heading - direction of traveling
    *
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] heading     heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
    * @param[in] accuracy    estimated deviation of heading in degrees (+/-180)
    */
   void SendHeading( const double& timestamp, const double& heading, const double& accuracy);
   /**
    * Sends new speed - velocity of the object
    *
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] speed       velocity of the object in meter per seconds [m/s]
    * @param[in] accuracy    estimated deviation of speed in meter per seconds 
    */
   void SendSpeed( const double& timestamp, const double& speed, const double& accuracy);
   /**
    * Sends new gyroscope - angular velocity of the object
    *
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] gyro        raw gyroscope sensors data dimention does not matter
    */
   void SendGyro( const double& timestamp, const double& gyro);
   /**
    * Sends new odometer - ticks count of the wheel
    *
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] odo         raw odometer sensors data dimention does not matter
    */
   void SendOdo( const double& timestamp, const double& odo);
   /**
    * Receives calculated position - coordinates, heading and speed
    * @return   true if position was calculated with no error
    *
    * @param[out] timestamp              timestamp of given sensors data in seconds
    * @param[out] latitude               latitude in degrees (0..+/-90)
    * @param[out] longitude              longitude in degrees (0..+/-180) 
    * @param[out] coordinatesAccuracy    expectation area of position with radius around given coordinates in meters
    * @param[out] heading                heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
    * @param[out] headingAccuracy        estimated deviation of heading in degrees (+/-180)
    * @param[out] speed                  velocity of the object in meter per seconds [m/s]
    * @param[out] speedAccuracy          estimated deviation of speed in meter per seconds 
    */
   bool ReceivePosition( double& timestamp, double& latitude, double& longitude, double& coordinatesAccuracy, double& heading, double& headingAccuracy, double& speed, double& speedAccuracy);
   /**
    * Receives whole distance of traveling
    * @return   true if distance was calculated with no error
    *
    * @param[out] distance   travel distance of vehicle in meters
    * @param[out] accuracy   estimated deviation of distance in meters
    */
   bool ReceiveDistance( double& distance, double& accuracy);
   /**
    * Receives gyro calibration status  ---  real_value = (raw_value - bias) x scale
    * @return true if calibration was triggered
    *
    * @param[out] bias       shift of the raw value according to real value
    * @param[out] scale      scale value for converting raw into real value
    * @param[out] reliable   percentage indicator of calibration status in (0%..100%)
    * @param[out] accuracy   estimated deviation of sensor in real_value dimention
    */
   bool ReceiveGyroStatus( double& bias, double& scale, double& reliable, double& accuracy);
   /**
    * Receives odometer calibration status  ---  real_value = (raw_value - bias) x scale
    * @return true if calibration was triggered
    *
    * @param[out] bias       shift of the raw value according to real value
    * @param[out] scale      scale value for converting raw into real value
    * @param[out] reliable   percentage indicator of calibration status in (0%..100%)
    * @param[out] accuracy   estimated deviation of sensor in real_value dimention
    */
   bool ReceiveOdoStatus( double& bias, double& scale, double& reliable, double& accuracy);
private:
   /**
    * Current configuration string
    */
   std::string m_Cfg_Str;
//    /**
//     * Last odometer timestamp
//     */
//    PE::TTimestamp m_Odo_Ts;
//    /**
//     * Odometer calibration service
//     */
//    PE::CCalibrationSummary m_Odo_Calib;
//    /**
//     * Odometer normalisation service for bias
//     */
//    PE::CNormalisation m_Odo_Bias;
//    /**
//     * Odometer normalisation service for scale
//     */
//    PE::CNormalisation m_Odo_Scale;
//    /**
//     * Last speed timestamp
//     */
//    PE::TTimestamp m_Speed_Ts;
//    /**
//     * Speed normalisation service for accuracy
//     */
//    PE::CNormalisation m_Speed_Acc;
//    /**
//     * Cleans internal values for last step of odometer processing
//     */
//    void CleanOdoStep();

};

#endif //__PE_CCore_H__
