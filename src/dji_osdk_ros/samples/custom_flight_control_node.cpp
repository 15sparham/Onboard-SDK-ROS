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
#include <dji_osdk_ros/dji_vehicle_node.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

#include <dji_osdk_ros/SetJoystickMode.h>
#include <dji_osdk_ros/JoystickAction.h>

#include <math.h>
#include <dji_osdk_ros/vehicle_wrapper.h>
#include "UTM.h"
#include "UTM.cpp"

#include "dji_osdk_ros/WaypointUTM.h"
#include "dji_osdk_ros/WaypointUTMArray.h"

static int zone = 17;
static bool southhemi = false;

//CODE
using namespace dji_osdk_ros;

ros::ServiceClient task_control_client;
ros::ServiceClient set_joystick_mode_client;
ros::ServiceClient joystick_action_client;

dji_osdk_ros::GPSUTC time_sync_gps_utc_;
sensor_msgs::NavSatFix gps_position_;
sensor_msgs::NavSatFix rtk_position_;
std_msgs::Int16 rtk_yaw_;

DJI::OSDK::float32_t x = 0.0;   /*!< LOCAL X COORDINATE (initialized as 0.0)*/
DJI::OSDK::float32_t y = 0.0;   /*!< LOCAL Y COORDINATE (initialized as 0.0)*/
DJI::OSDK::float32_t z = 0.0;   /*!< LOCAL Z COORDINATE (initialized as 0.0)*/
DJI::OSDK::float32_t yaw = 0.0; /*!< LOCAL YAW COORDINATE (initialized as 0.0)*/

void timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC::ConstPtr& timeSyncGpsUtc)
{
  time_sync_gps_utc_ = *timeSyncGpsUtc;
}

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  gps_position_ = *gpsPosition;
}

void rtkPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& rtkPosition)
{
  rtk_position_ = *rtkPosition;
}

void rtkYawSubCallback(const std_msgs::Int16::ConstPtr& rtkYaw)
{
  rtk_yaw_ = *rtkYaw;
}

ros::Subscriber timeSyncGpsUtcSub;
ros::Subscriber gpsPositionSub;
ros::Subscriber rtkPositionSub;
ros::Subscriber rtkYawSub;

bool moveByPosOffset(FlightTaskControl& task,const JoystickCommand &offsetDesired,
                     float posThresholdInM = 0.8,
                     float yawThresholdInDeg = 1.0);

bool moveToPos(FlightTaskControl& task,const JoystickCommand &offsetDesired,
               float posThresholdInM = 0.8,
               float yawThresholdInDeg = 1.0);

DJI::OSDK::float32_t xyzMax = 20;
DJI::OSDK::float32_t yawMax = 360;

// filterTime() receives time and filters waypoints coming from Path Planning
std::vector<dji_osdk_ros::WaypointUTM> filterTime(std::vector<dji_osdk_ros::WaypointUTM> waypointArray, int min_secs_needed = 1);

// utmToGPSCoord() converts waypoint array's xy UTM coordinates to lat/lon coordinates (in radians)
void utmToGPSCoord(std::vector<dji_osdk_ros::WaypointUTM> &waypointArray);

// broadcastCoord() converts waypoint array's xy lat/lon coordinates (in radians) to UTM coordinates
// and changes first coordinate to current location.
void broadcastCoord(std::vector<dji_osdk_ros::WaypointUTM> &waypointArray);

// TODO: moveToPos commands drone to move somewhere
// void takeoffAndMoveToWaypoints(void);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "custom_flight_control_node");
  ros::NodeHandle nh;

  timeSyncGpsUtcSub = nh.subscribe("dji_osdk_ros/time_sync_gps_utc", 50, &timeSyncGpsUtcSubCallback);
  gpsPositionSub = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
  rtkPositionSub = nh.subscribe("dji_osdk_ros/rtk_position", 10, &rtkPositionSubCallback);
  rtkYawSub = nh.subscribe("dji_osdk_ros/rtk_yaw", 10, &rtkYawSubCallback);
    

  ros::Duration(1).sleep();
  ros::AsyncSpinner spinner(1);
  spinner.start();

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
  std::cout << "| [d] GPS + RTK location printout                                          |" << std::endl;
  std::cout << "| [e] Time printout                                                        |" << std::endl;
  std::cout << "| [f] Build UTM waypoint vector, convert to GPS+RTK and back               |" << std::endl;

  std::cout << "Please select command: ";
  char inputChar;
  char keep_checking = 'y';
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
        while (keep_checking == 'y')
        {
          std::cout << "gps_position_.latitude =" << gps_position_.latitude << std::endl; // * C_PI / 180.0
          std::cout << "gps_position_.longitude =" << gps_position_.longitude << std::endl; // * C_PI / 180.0

          /* Ask if we should keep checking. */
          std::cout << "Keep checking? (y/n) ";
          std::cin >> keep_checking;
        }
        break;
      }
    case 'd':
      {
        while (keep_checking == 'y')
        {
          ROS_INFO("rtk_position_(latitude, longitude, altitude) :%f, %f, %f \n",
          rtk_position_.latitude, rtk_position_.longitude, rtk_position_.altitude);
          ROS_INFO("rtk_yaw_: %d\n", rtk_yaw_.data);

          /* Ask if we should keep checking. */
          std::cout << "Keep checking? (y/n) ";
          std::cin >> keep_checking;
        }
        break;
      }
    case 'e':
      {
        while (keep_checking == 'y')
        {
          ROS_INFO("time_sync_gps_utc_(timestamp,UTCTimeData):%d.%d, %s \n",
            time_sync_gps_utc_.stamp.sec, time_sync_gps_utc_.stamp.nsec, time_sync_gps_utc_.UTCTimeData.data());

          /* Ask if we should keep checking. */
          std::cout << "Keep checking? (y/n) ";
          std::cin >> keep_checking;
        }
        break;
      }
    case 'f':
      {
        dji_osdk_ros::WaypointUTM waypointUTM;
        dji_osdk_ros::WaypointUTMArray waypointUTMArray;

        char option2 = 'a';
        std::cout << "| Available commands:                                                      |" << std::endl;
        std::cout << "| [a] Manually insert waypoints                                            |" << std::endl;
        std::cout << "| [b] Use predefined waypoints around static MAir Location                 |" << std::endl;
        std::cout << "| [c] Use predefined waypoints around current GPS+RTK Location             |" << std::endl;
        std::cout << "| [d] Manually insert waypoints + action                                   |" << std::endl;
        std::cout << "| [e] Use predefined waypoints around static MAir Location + action        |" << std::endl;
        std::cout << "| [f] Use predefined waypoints around current GPS+RTK Location + action    |" << std::endl;
        std::cin >> option2;

        if (option2 == 'a' || option2 == 'd')
        {
          while (keep_checking == 'y')
          {
            std::cout << "Input UTM waypoint" << std::endl;
            std::cout << "Please enter x: ";
            std::cin >> waypointUTM.x;
            std::cout << "Please enter y: ";
            std::cin >> waypointUTM.y;
            std::cout << "Please enter z height [meters]: ";
            std::cin >> waypointUTM.z;
            std::cout << "Please enter yaw orientation [degrees]: ";
            std::cin >> waypointUTM.yaw;
            std::cout << "Please enter time [seconds since epoch]: ";
            std::cin >> waypointUTM.ts.sec;
            std::cout << "Please enter time [nanoseconds]: ";
            std::cin >> waypointUTM.ts.nsec;

            waypointUTMArray.waypoints.push_back(waypointUTM);

            /* Ask if we should keep checking. */
            std::cout << "Keep inserting UTM waypoints? (y/n) ";
            std::cin >> keep_checking;
          }
        }
        else if (option2 == 'b' || option2 == 'c' || option2 == 'e' || option2 == 'f')
        {
          if (option2 == 'b' || option2 == 'e')
          {
            // Predefined UTM waypoint used in option b
            waypointUTM.x = 276554.086;
            waypointUTM.y = 4686028.952;
          }
          else
          {
            // Get UTM waypoint from current location in option c
            FLOAT lat = rtk_position_.latitude;
            FLOAT lon = rtk_position_.longitude;
            FLOAT lat_radians = DegToRad(lat);
            FLOAT lon_radians = DegToRad(lon);

            std::cout << "GPS+RTK coordinate (lat, lon): " << setprecision(15) << lat << ", " << lon << std::endl;

            LatLonToUTMXY(lat_radians, lon_radians, zone, waypointUTM.x, waypointUTM.y);
            std::cout << "Converted UTM coordinate: " << waypointUTM.x << ", " << waypointUTM.y << std::endl << std::endl;
          }
          double x = 0;
          double y = 0;
          for (int i = 1; i <= 5; i++)
          {
            if (i == 1)
            {
              x = 1;
              y = -1;
            }
            else if (i == 2)
            {
              x = 1;
              y = 1;
            }
            else if (i == 3)
            {
              x = -1;
              y = 1;
            }
            else if (i == 4)
            {
              x = -1;
              y = -1;
            }
            else
            {
              x = 0;
              y = 0;
            }
            waypointUTM.x += x; // Add i - 1 to move waypoints
            waypointUTM.y += y;
            waypointUTM.z = 5;
            waypointUTM.yaw = 0;
            waypointUTM.ts.sec = time_sync_gps_utc_.stamp.sec + (i*5 + 10); // Add i*5 + 10 seconds to the timestamp
            waypointUTM.ts.nsec = time_sync_gps_utc_.stamp.nsec;
            waypointUTMArray.waypoints.push_back(waypointUTM);
          }
        }
        else
        {
          break;
        }

        // Check if filterTime() works
        std::cout << "Number of UTM waypoints inserted = " << waypointUTMArray.waypoints.size() << std::endl;
        std::vector<dji_osdk_ros::WaypointUTM> waypointVector = filterTime(waypointUTMArray.waypoints);
        dji_osdk_ros::WaypointUTMArray waypointUTMArray2;
        waypointUTMArray2.waypoints = waypointVector;
        std::cout << "Number of UTM waypoints after filterTime() is = " << waypointUTMArray2.waypoints.size() << std::endl;
        
        // Take filtered waypoints and convert them from UTM to GPS+RTK
        utmToGPSCoord(waypointUTMArray2.waypoints);

        broadcastCoord(waypointUTMArray2.waypoints);

        if (option2 == 'a' || option2 == 'b' || option2 == 'c')
        {
          break;
        }


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
          for (dji_osdk_ros::WaypointUTM waypoint : waypointUTMArray2.waypoints)
          {
            ROS_INFO_STREAM("Using moveToPose to move to latitude = " << waypoint.x << ", longitude = " << waypoint.y << ", z = " << waypoint.z << ", yaw = " << waypoint.yaw << " ...");
            moveToPos(control_task, {waypoint.x, waypoint.y, waypoint.z, waypoint.yaw}, posThreshInM, yawThreshInDeg);
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

        break;
      }
    default:
      break;
  }

  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

  ros::waitForShutdown();
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

// filterTime() receives time and filters waypoints coming from Path Planning
std::vector<dji_osdk_ros::WaypointUTM> filterTime(std::vector<dji_osdk_ros::WaypointUTM> waypointVector, int min_secs_needed)
{
  std::vector<dji_osdk_ros::WaypointUTM> waypointVectorOut;
  // For each element in the input array...
  for (dji_osdk_ros::WaypointUTM waypoint : waypointVector)
  {
    // If current time + min_secs_needed is less than waypoint time, add it to the output array list.
    if (time_sync_gps_utc_.stamp.sec + min_secs_needed < waypoint.ts.sec)
    {
      waypointVectorOut.push_back(waypoint);
    }
    else if (time_sync_gps_utc_.stamp.sec + min_secs_needed == waypoint.ts.sec && time_sync_gps_utc_.stamp.nsec < waypoint.ts.nsec)
    {
      waypointVectorOut.push_back(waypoint);
    }
  }
  return waypointVectorOut;
}

// utmToGPSCoord() converts waypoint array's xy UTM coordinates to lat/lon coordinates (in radians)
void utmToGPSCoord(std::vector<dji_osdk_ros::WaypointUTM> &waypointArray)
{
  for (int i = 0; i < waypointArray.size(); i++)
  {
    FLOAT x = waypointArray[i].x;
    FLOAT y = waypointArray[i].y;

    std::cout << "UTM coordinate (x, y): " << setprecision(15) << x << ", " << y << std::endl;

    UTMXYToLatLon(x, y, zone, southhemi, waypointArray[i].x, waypointArray[i].y);
    // UTMXYToLatLon(x, y, zone, southhemi, lat, lon);

    std::cout << "Converted GPS+RTK coordinate (lat, lon): " << RadToDeg(waypointArray[i].x) << ", " << RadToDeg(waypointArray[i].y) << std::endl << std::endl;
    // std::cout << "Converted GPS+RTK coordinate: " << RadToDeg(lat) << ", " << RadToDeg(lon) << std::endl;

  }
}

// broadcastCoord() converts waypoint array's xy lat/lon coordinates (in radians) to UTM coordinates
// and changes first coordinate to current location.
void broadcastCoord(std::vector<dji_osdk_ros::WaypointUTM> &waypointArray)
{
  std::vector<dji_osdk_ros::WaypointUTM> waypointArrayOut;
  for (int i = 0; i < waypointArray.size(); i++)
  {
    dji_osdk_ros::WaypointUTM waypoint = waypointArray[i];

    if (i == 0)
    {
      std::cout << "Changing first waypoint to use current RTK position..." << std::endl;
      waypoint.x = DegToRad(rtk_position_.latitude);
      waypoint.y = DegToRad(rtk_position_.longitude);
      waypoint.z = rtk_position_.altitude;
    }

    FLOAT lat = waypoint.x;
    FLOAT lon = waypoint.y;

    std::cout << "GPS+RTK coordinate (lat, lon): " << setprecision(15) << RadToDeg(lat) << ", " << RadToDeg(lon) << std::endl;

    // LatLonToUTMXY (FLOAT lat, FLOAT lon, int zone, FLOAT& x, FLOAT& y)
    LatLonToUTMXY(lat, lon, zone, waypoint.x, waypoint.y);
    std::cout << "Converted UTM coordinate: " << waypoint.x << ", " << waypoint.y << std::endl << std::endl;

    waypointArrayOut.push_back(waypoint);
  }
}