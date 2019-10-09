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


namespace PE
{
   /**
    * Initialises and starts position engine
    * @return   true if position engine started with no error
    */
   bool Start();
   /**
    * Stops position engine
    * @return   true if position engine stopped with no error
    */
   bool Stop();
   /**
    * Cleans all internal values
    */
   void Clean();
   /**
    * Calculates position based on given coordinates, headings, speeds and sensors
    * @return   true if position calculates with no error
    */
   bool Calculate();
   /**
    * Sends new coordinates
    * @return   true if coordinates were sent with no error
    *
    * @param[in] timestamp   todo
    * @param[in] latutude    todo
    * @param[in] longitude   todo
    * @param[in] accuracy    todo
    */
   bool SendCoordinates(const TTimestamp& timestamp, const TValue& latutude, const TValue& longitude, const TAccuracy& accuracy);
   /**
    * Sends new heading - direction of traveling
    * @return   true if heading was sent with no error
    *
    * @param[in] timestamp   todo
    * @param[in] heading     todo
    * @param[in] accuracy    todo
    */
   bool SendHeading(const TTimestamp& timestamp, const TValue& heading, const TAccuracy& accuracy);
   /**
    * Sends new speed - velocity of the object
    * @return   true if speed was sent with no error
    *
    * @param[in] timestamp   todo
    * @param[in] speed       todo
    * @param[in] accuracy    todo
    */
   bool SendSpeed(const TTimestamp& timestamp, const TValue& speed, const TAccuracy& accuracy);
   /**
    * Sends new gyroscope - angular velocity of the object
    * @return   true if gyroscope was sent with no error
    *
    * @param[in] timestamp   todo
    * @param[in] gyro        todo
    * @param[in] accuracy    todo
    */
   bool SendGyro(const TTimestamp& timestamp, const TValue& gyro, const TAccuracy& accuracy);
   /**
    * Sends new odometer - ticks count of the wheel
    * @return   true if odometer was sent with no error
    *
    * @param[in] timestamp   todo
    * @param[in] odo         todo
    * @param[in] accuracy    todo
    */
   bool SendOdo(const TTimestamp& timestamp, const TValue& odo, const TAccuracy& accuracy);
   /**
    * Receives calculated position - coordinates, heading and speed
    * @return   true if position was calculated with no error
    *
    * @param[out] timestamp              todo
    * @param[out] latutude               todo
    * @param[out] longitude              todo
    * @param[out] coordinatesAccuracy    todo
    * @param[out] heading                todo
    * @param[out] headingAccuracy        todo
    * @param[out] speed                  todo
    * @param[out] speedAccuracy          todo
    */
   bool ReceivePosition(TTimestamp& timestamp, TValue& latutude, TValue& longitude, TAccuracy& coordinatesAccuracy, TValue& heading, TAccuracy& headingAccuracy, TValue& speed, TAccuracy& speedAccuracy);
   /**
    * Receives whole distance of treveling
    * @return   true if distance was calculated with no error
    *
    * @param[out] distance   todo
    * @param[out] accuracy   todo
    */
   bool ReceiveDistance(TValue& distance, TAccuracy& accuracy);
   /**
    * Receives gyro calibration status
    * @return true if calibration was triggered
    *
    * @param[out] base       todo
    * @param[out] scale      todo
    * @param[out] reliable   todo
    */
   bool ReceiveGyroStatus(TValue& base, TValue& scale, TValue& reliable);
   /**
    * Receives odometer calibration status
    * @return true if calibration was triggered
    *
    * @param[out] base       todo
    * @param[out] scale      todo
    * @param[out] reliable   todo
    */
   bool ReceiveOdoStatus(TValue& base, TValue& scale, TValue& reliable);

} //namespace PE
#endif //__PE_Core_H__
