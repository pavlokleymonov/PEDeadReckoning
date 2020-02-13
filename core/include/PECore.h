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
#ifndef __PE_Core_H__
#define __PE_Core_H__



#ifdef __cplusplus
   class PECCore;
   extern "C" {
#else
   struct PECCore;
   typedef struct PECCore PECCore;
#endif
   /**
    * Initialises and starts position engine
    * @return   pointer to the position engine instance if it started with no error
    *           in case any errors return NULL
    *
    * @param[in] cfg   simple c-type string to configuration information
    */
   PECCore* PEStart(const char* cfg);
   /**
    * Stops position engine
    * @return   pointer to configuration information c-type string if position engine stopped with no error
    *           in case any errors return NULL
    *
    * @param[in] core   pointer to the position engine instance
    */
   const char* PEStop(PECCore* core);
   /**
    * Cleans all internal values
    * @return   true if clearing of the position engine was successful
    *
    * @param[in] core   pointer to the position engine instance
    */
   bool PEClean(PECCore* core);
   /**
    * Calculates position based on given coordinates, headings, speeds and sensors
    * @return   true if position calculates with no error
    *
    * @param[in] core   pointer to the position engine instance
    */
   bool PECalculate(PECCore* core);
   /**
    * Sends new coordinates
    * @return   true if coordinates were sent with no error
    *
    * @param[in] core        pointer to the position engine instance
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] latitude    latitude in degrees (0..+/-90)
    * @param[in] longitude   longitude in degrees (0..+/-180) 
    * @param[in] accuracy    expectation area of position with radius around given coordinates in meters
    */
   bool PESendCoordinates(PECCore* core, const double& timestamp, const double& latitude, const double& longitude, const double& accuracy);
   /**
    * Sends new heading - direction of traveling
    * @return   true if heading was sent with no error
    *
    * @param[in] core        pointer to the position engine instance
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] heading     heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
    * @param[in] accuracy    estimated deviation of heading in degrees (+/-180)
    */
   bool PESendHeading(PECCore* core, const double& timestamp, const double& heading, const double& accuracy);
   /**
    * Sends new speed - velocity of the object
    * @return   true if speed was sent with no error
    *
    * @param[in] core        pointer to the position engine instance
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] speed       velocity of the object in meter per seconds [m/s]
    * @param[in] accuracy    estimated deviation of speed in meter per seconds 
    */
   bool PESendSpeed(PECCore* core, const double& timestamp, const double& speed, const double& accuracy);
   /**
    * Sends new gyroscope - angular velocity of the object
    * @return   true if gyroscope was sent with no error
    *
    * @param[in] core        pointer to the position engine instance
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] gyro        raw gyroscope sensors data dimention does not matter
    */
   bool PESendGyro(PECCore* core, const double& timestamp, const double& gyro);
   /**
    * Sends new odometer - ticks count of the wheel
    * @return   true if odometer was sent with no error
    *
    * @param[in] core        pointer to the position engine instance
    * @param[in] timestamp   timestamp of given sensors data in seconds
    * @param[in] odo         raw odometer sensors data dimention does not matter
    */
   bool PESendOdo(PECCore* core, const double& timestamp, const double& odo);
   /**
    * Receives calculated position - coordinates, heading and speed
    * @return   true if position was calculated with no error
    *
    * @param[in] core                    pointer to the position engine instance
    * @param[out] timestamp              timestamp of given sensors data in seconds
    * @param[out] latitude               latitude in degrees (0..+/-90)
    * @param[out] longitude              longitude in degrees (0..+/-180) 
    * @param[out] coordinatesAccuracy    expectation area of position with radius around given coordinates in meters
    * @param[out] heading                heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
    * @param[out] headingAccuracy        estimated deviation of heading in degrees (+/-180)
    * @param[out] speed                  velocity of the object in meter per seconds [m/s]
    * @param[out] speedAccuracy          estimated deviation of speed in meter per seconds 
    */
   bool PEReceivePosition(PECCore* core, double& timestamp, double& latitude, double& longitude, double& coordinatesAccuracy, double& heading, double& headingAccuracy, double& speed, double& speedAccuracy);
   /**
    * Receives whole distance of traveling
    * @return   true if distance was calculated with no error
    *
    * @param[in] core        pointer to the position engine instance
    * @param[out] distance   travel distance of vehicle in meters
    * @param[out] accuracy   estimated deviation of distance in meters
    */
   bool PEReceiveDistance(PECCore* core, double& distance, double& accuracy);
   /**
    * Receives gyro calibration status  ---  real_value = (raw_value - base) x scale
    * @return true if calibration was triggered
    *
    * @param[in] core        pointer to the position engine instance
    * @param[out] base       shift of the raw value according to real value
    * @param[out] scale      scale value for converting raw into real value
    * @param[out] reliable   percentage indicator of calibration status in (0%..100%)
    */
   bool PEReceiveGyroStatus(PECCore* core, double& base, double& scale, double& reliable);
   /**
    * Receives odometer calibration status  ---  real_value = (raw_value - base) x scale
    * @return true if calibration was triggered
    *
    * @param[in] core        pointer to the position engine instance
    * @param[out] base       shift of the raw value according to real value
    * @param[out] scale      scale value for converting raw into real value
    * @param[out] reliable   percentage indicator of calibration status in (0%..100%)
    */
   bool PEReceiveOdoStatus(PECCore* core, double& base, double& scale, double& reliable);

#ifdef __cplusplus
   }
#endif

#endif //__PE_Core_H__
