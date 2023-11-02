/** @file advanced_sensing_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of flight control.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

#include<dji_osdk_ros/SetJoystickMode.h>
#include<dji_osdk_ros/JoystickAction.h>

#include<math.h>
#include<dji_osdk_ros/waypointV2_node.h>
#include<dji_osdk_ros/vehicle_wrapper.h>

//CODE
using namespace dji_osdk_ros;

ros::ServiceClient task_control_client;
ros::ServiceClient set_joystick_mode_client;
ros::ServiceClient joystick_action_client;

DJI::OSDK::float32_t x = 0.0;   /*!< LOCAL X COORDINATE (initialized as 0.0)*/
DJI::OSDK::float32_t y = 0.0;   /*!< LOCAL Y COORDINATE (initialized as 0.0)*/
DJI::OSDK::float32_t z = 0.0;   /*!< LOCAL Z COORDINATE (initialized as 0.0)*/
DJI::OSDK::float32_t yaw = 0.0; /*!< LOCAL YAW COORDINATE (initialized as 0.0)*/


void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  // std::cout << "gpsPosition lat " << gpsPosition->latitude << ", long " << gpsPosition->longitude << " received.";
  gps_position_ = *gpsPosition;
}

bool moveByPosOffset(FlightTaskControl& task,const JoystickCommand &offsetDesired,
                     float posThresholdInM = 0.8,
                     float yawThresholdInDeg = 1.0);

bool moveToPos(FlightTaskControl& task,const JoystickCommand &offsetDesired,
               float posThresholdInM = 0.8,
               float yawThresholdInDeg = 1.0);

DJI::OSDK::float32_t xyzMax = 20;
DJI::OSDK::float32_t yawMax = 360;

// TODO: getTime gets the GPS time, which is used by filterTime to decide to move to waypoints
// void getTime(void);

// TODO: filterTime receives time and filters waypoints coming from Path Planning
// void filterTime(void);

// TODO: filterTime receives time and filters waypoints before calling them
// void filterTime(void);

// TODO: UTMToLocalCoord converts utm coordinates to local system
// void utmToLocalCoord(void);

// TODO: localToUTMCoord converts local coordinates to utm system
// void utmToLocalCoord(void);

// TODO: moveToPos commands drone to move somewhere
// void moveToPos(void);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;

  ros::Subscriber gpsPositionSub = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);

  task_control_client = nh.serviceClient<FlightTaskControl>("/flight_task_control");

  auto set_go_home_altitude_client = nh.serviceClient<SetGoHomeAltitude>("/set_go_home_altitude");
  auto get_go_home_altitude_client = nh.serviceClient<GetGoHomeAltitude>("get_go_home_altitude");
  auto set_current_point_as_home_client = nh.serviceClient<SetCurrentAircraftLocAsHomePoint>("/set_current_aircraft_point_as_home");
  // Enables collision avoidance for horizontal obstacles? We should test and verify.
  auto enable_horizon_avoid_client  = nh.serviceClient<SetAvoidEnable>("/set_horizon_avoid_enable");
  // Enables collision avoidance for vertical obastacles? We should test and verify.
  auto enable_upward_avoid_client   = nh.serviceClient<SetAvoidEnable>("/set_upwards_avoid_enable");
  auto get_avoid_enable_client      = nh.serviceClient<GetAvoidEnable>("get_avoid_enable_status");
  auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");
  auto emergency_brake_client       = nh.serviceClient<dji_osdk_ros::EmergencyBrake>("emergency_brake");

  set_joystick_mode_client = nh.serviceClient<SetJoystickMode>("set_joystick_mode");
  joystick_action_client   = nh.serviceClient<JoystickAction>("joystick_action");

  std::cout << "| Available commands:                                                      |" << std::endl;
  std::cout << "| [a] Monitored Takeoff + Input Position Control By Offset + Landing       |" << std::endl;
  std::cout << "| [b] Monitored Takeoff + Input Position Control By Local Coord + Landing  |" << std::endl;
  std::cout << "| [c] GPS location printout                                                |" << std::endl;

  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;

  DJI::OSDK::float32_t xRequested = 0.0;   /*!< Control with respect to the x axis of the JI::OSDK::Control::HorizontalCoordinate.*/
  DJI::OSDK::float32_t yRequested = 0.0;   /*!< Control with respect to the y axis of the DJI::OSDK::Control::HorizontalCoordinate.*/
  DJI::OSDK::float32_t zRequested = 0.0;   /*!< Control with respect to the z axis, up is positive. */
  DJI::OSDK::float32_t yawRequested = 0.0; /*!< Yaw position/velocity control w.r.t. the ground frame.*/

  float posThreshInM = 0.8;
  float yawThreshInDeg = 1.0;

  EmergencyBrake emergency_brake;
  FlightTaskControl control_task;
  ObtainControlAuthority obtainCtrlAuthority;
  
  obtainCtrlAuthority.request.enable_obtain = true;
  obtain_ctrl_authority_client.call(obtainCtrlAuthority);

  switch (inputChar)
  {
    case 'a':
      {
        /* TAKEOFF */
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }
        else if (control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("turn on Horizon_Collision-Avoidance-Enabled");
          SetAvoidEnable horizon_avoid_req;
          horizon_avoid_req.request.enable = true;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("turn on Upwards-Collision-Avoidance-Enabled");
          SetAvoidEnable upward_avoid_req;
          upward_avoid_req.request.enable = true;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          GetAvoidEnable getAvoidEnable;
          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          /* Ask for position and yaw threshold at beginning of mission. */
          std::cout << "Please enter position threshold [meters]: ";
          std::cin >> posThreshInM;
          std::cout << "Please enter yaw threshold [degrees]: ";
          std::cin >> yawThreshInDeg;
          /* Ask after each iteration if drone should keep flying and ask for another waypoint. */
          char keep_flying = 'y';
          while (keep_flying == 'y')
          {
            /* Ask for new (x, y, z, yaw). */
            std::cout << "Please enter x offset [meters]: ";
            std::cin >> xRequested;
            std::cout << "Please enter y offset [meters]: ";
            std::cin >> yRequested;
            std::cout << "Please enter z offset [meters]: ";
            std::cin >> zRequested;
            std::cout << "Please enter yaw offset [degrees]: ";
            std::cin >> yawRequested;

            if (fabs(xRequested) <= xyzMax && fabs(yRequested) <= xyzMax && fabs(zRequested) <= xyzMax && yawRequested <= yawMax)
            {
              /* MOVE TO OFFSET POSITION */
              ROS_INFO_STREAM("Using moveByPosOffset to offset x = " << xRequested << ", y = " << yRequested << ", z = " << zRequested << ", yaw = " << yawRequested << " ...");
              // globalMoveByPoseOffset({xRequested, yRequested, zRequested, yawRequested}, 4, posThreshInM, yawThreshInDeg);
              moveByPosOffset(control_task, {xRequested, yRequested, zRequested, yawRequested}, posThreshInM, yawThreshInDeg);
              ROS_INFO_STREAM("moveByPosOffset complete!");
            }
            else
            {
              std::cout << "Invalid request received. Exceeded xyzMax=" << xyzMax << "or yawMax=" << yawMax << ".";
            }
            /* Ask if we should keep flying. */
            std::cout << "Keep flying? (y/n) ";
            std::cin >> keep_flying;
          }

          ROS_INFO_STREAM("Shut down Horizon_Collision-Avoidance-Enabled");
          horizon_avoid_req.request.enable = false;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Disable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("Shut down Upwards-Collision-Avoidance-Enabled");
          upward_avoid_req.request.enable = false;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          /* LANDING */
          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          ROS_INFO_STREAM("Landing request sending ...");
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
      }
    case 'b':
      {
        /* TAKEOFF */
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }
        else if (control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("turn on Horizon_Collision-Avoidance-Enabled");
          SetAvoidEnable horizon_avoid_req;
          horizon_avoid_req.request.enable = true;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("turn on Upwards-Collision-Avoidance-Enabled");
          SetAvoidEnable upward_avoid_req;
          upward_avoid_req.request.enable = true;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          GetAvoidEnable getAvoidEnable;
          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          /* Ask for position and yaw threshold at beginning of mission. */
          std::cout << "Please enter position threshold [meters]: ";
          std::cin >> posThreshInM;
          std::cout << "Please enter yaw threshold [degrees]: ";
          std::cin >> yawThreshInDeg;
          /* Ask after each iteration if drone should keep flying and ask for another waypoint. */
          char keep_flying = 'y';
          while (keep_flying == 'y')
          {
            // moveToPosition(const Telemetry::GPSFused& desiredGPSPosition, float32_t& desiredHeight, float yawDesiredInDeg,
            //                           int timeout, float posThresholdInM, float yawThresholdInDeg)
            /* Ask for desired (longitude, latitude, z, yaw). */
            std::cout << "Please enter latitude: ";
            std::cin >> xRequested;
            std::cout << "Please enter longitude: ";
            std::cin >> yRequested;
            std::cout << "Please enter z height [meters]: ";
            std::cin >> zRequested;
            std::cout << "Please enter yaw orientation [degrees]: ";
            std::cin >> yawRequested;

              /* MOVE TO NEW LOCAL POSITION */
            ROS_INFO_STREAM("Using moveToPose to move to latitude = " << xRequested << ", longitude = " << yRequested << ", z = " << zRequested << ", yaw = " << yawRequested << " ...");
            moveToPos(control_task, {xRequested, yRequested, zRequested, yawRequested}, posThreshInM, yawThreshInDeg);
            ROS_INFO_STREAM("moveToPose complete!");

            /* Ask if we should keep flying. */
            std::cout << "Keep flying? (y/n) ";
            std::cin >> keep_flying;
          }

          ROS_INFO_STREAM("Shut down Horizon_Collision-Avoidance-Enabled");
          horizon_avoid_req.request.enable = false;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Disable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("Shut down Upwards-Collision-Avoidance-Enabled");
          upward_avoid_req.request.enable = false;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          /* LANDING */
          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          ROS_INFO_STREAM("Landing request sending ...");
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
      }
    case 'c':
      {
        char keep_checking = 'y';
        while (keep_checking == 'y')
        {
          std::cout << "gps_position_.latitude =" << gps_position_.latitude << std::endl; // * C_PI / 180.0
          std::cout << "gps_position_.longitude =" << gps_position_.longitude << std::endl; // * C_PI / 180.0

          /* Ask if we should keep checking. */
          std::cout << "Keep checking? (y/n) ";
          std::cin >> keep_checking;
        }
        // // Let's create a vector to store our waypoints in.
        // std::vector<dji_osdk_ros::WaypointV2> waypointList;
        // dji_osdk_ros::WaypointV2 startPoint;
        // dji_osdk_ros::WaypointV2 waypointV2;

        // startPoint.latitude  = gps_position_.latitude * C_PI / 180.0;
        // startPoint.longitude = gps_position_.longitude * C_PI / 180.0;
        // startPoint.relativeHeight = 15;
        // setWaypointV2Defaults(startPoint);
        // waypointList.push_back(startPoint);

        // // Iterative algorithm
        // for (int i = 0; i < polygonNum; i++) {
        //   float32_t angle = i * 2 * M_PI / polygonNum;
        //   setWaypointV2Defaults(waypointV2);
        //   float32_t X = radius * cos(angle);
        //   float32_t Y = radius * sin(angle);
        //   waypointV2.latitude = Y/EARTH_RADIUS + startPoint.latitude;
        //   waypointV2.longitude = X/(EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
        //   waypointV2.relativeHeight = startPoint.relativeHeight ;
        //   waypointList.push_back(waypointV2);
        // }
        // waypointList.push_back(startPoint);

        // return waypointList;
      }
    default:
      break;
  }

  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

  ros::spin();
  return 0;
}


bool moveByPosOffset(FlightTaskControl& task,const JoystickCommand &offsetDesired,
                    float posThresholdInM,
                    float yawThresholdInDeg)
{
  task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
  task.request.joystickCommand.x = offsetDesired.x;
  task.request.joystickCommand.y = offsetDesired.y;
  task.request.joystickCommand.z = offsetDesired.z;
  task.request.joystickCommand.yaw = offsetDesired.yaw;
  task.request.posThresholdInM   = posThresholdInM;
  task.request.yawThresholdInDeg = yawThresholdInDeg;

  task_control_client.call(task);
  return task.response.result;
}

bool moveToPos(FlightTaskControl& task,const JoystickCommand &offsetDesired,
                    float posThresholdInM,
                    float yawThresholdInDeg)
{
  task.request.task = FlightTaskControl::Request::TASK_GPS_POSITION_AND_YAW_CONTROL;
  task.request.joystickCommand.x = offsetDesired.x;
  task.request.joystickCommand.y = offsetDesired.y;
  task.request.joystickCommand.z = offsetDesired.z;
  task.request.joystickCommand.yaw = offsetDesired.yaw;
  task.request.posThresholdInM   = posThresholdInM;
  task.request.yawThresholdInDeg = yawThresholdInDeg;

  task_control_client.call(task);
  return task.response.result;
}