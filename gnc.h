
/*
 * File: RPI/gnc.h
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file contains the headers, includings, states UAS,
 *             functions for the guidance, navigation, and control
 *             , and UDP sockets connected with the autopilot.
 *
 */

 #ifndef GNC_H
 #define GNC_H
 
 #include "c_library_v2/common/mavlink.h"
 #include <stdio.h>
 #include <sys/time.h>
 #include <stdlib.h>
 #include <string.h>
 #include <arpa/inet.h>
 #include <unistd.h>
 #include <math.h>
 #include <fcntl.h>
 #include <sys/select.h>
 #define M_PI 3.14159265358979323846
 #include <stdatomic.h>
 #include <termios.h>
 #include <errno.h>
 #include <signal.h>
 #include <time.h>

 // Add after the includes
#define DEBUG 1

#ifdef DEBUG
    #define DEBUG_PRINT(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) 
#endif
 
 
 #define BUFFER_LENGTH 2041
 #define DEFAULT_TARGET_SYSTEM 1     // Default to system ID 1 (typical for autopilot)
 #define DEFAULT_TARGET_COMPONENT 1  // Default to component ID 1 (flight controller)
 #define GCS_SYSTEM_ID 255          // Ground Control Station ID
 #define GCS_COMPONENT_ID 0         // GCS component ID
 
 #define DEG_TO_RAD (M_PI / 180.0)
 #define RAD_TO_DEG (180.0 / M_PI)
 #define EARTH_RADIUS 6378137.0
 #define MODE_CHANGE_TIMEOUT 500
 // Add this at the top with other defines
 #define MISSION_RETRY_COUNT 10
 
 // Add this at the top
 #define MIN_MISSION_INTERVAL 5     // 50ms minimum interval
 
 // Add this define at the top
 #define DO_JUMP_COMMAND 177  // MAV_CMD_DO_JUMP
 
 #define GUIDED_2_ATTACK 1

 #define MAX_WAYPOINTS 3

 #include <time.h>
 //#define _POSIX_C_SOURCE 199309L
 
 // If CLOCK_MONOTONIC is not defined, define it
 #ifndef CLOCK_MONOTONIC
 #define CLOCK_MONOTONIC 1
 #endif
 // Add these mode definitions at the top
 #define PLANE_MODE_MANUAL    0
 #define PLANE_MODE_GUIDED    15
 #define PLANE_MODE_AUTO      10
 #define PLANE_MODE_RTL       11
 #define PLANE_MODE_FBWA      5
 
 
 typedef struct{
 //sensors reading from the autopilot
     double gps_lat;
     double gps_lon;
     double gps_alt;
 
     double heading;
     double airspeed;
     double groundspeed;
     double alt_hud;
 
     //home position
     double home_lat;
     double home_lon;
     double home_alt;
 //end
 
     double t2m_distance; //missile to target distance 
     double t2m_altitude;
     double bearing_error;
     double bearing;
 
     //phases
     double req_lat;
     double req_lon;
     double req_alt;
     double t_displacement_x;
     double t_displacement_y;
     double t_lat;
     double t_lon;
     double t_alt;
     double t2m_heading;
     double heading_lat_uav;
     double heading_lon_uav;
     double heading_alt_uav;
     double stable_lat[4];
     double stable_lon[4];
     int stable_alt[4];
     //LOS
     double los_angle;
     double last_los_angle;
     //param
     double last_pitch_angle;
     int N_gain;
     float k_gain;
 
     double flight_path_angle; //desired flight path angle
     double last_flight_path_angle; // achieved flight path angle
     float q[4]; // quaternion
     double desired_roll_angle;
 
     int mission_state;
 
     double attitude_uav[3]; //pitch, roll, yaw

     int rc_channels;

 } sts;

 typedef struct {
    uint16_t count;
    mavlink_mission_item_int_t mission_items[MAX_WAYPOINTS];
    bool receiving;
    uint16_t received;
    uint16_t num_waypoints;
    uint16_t current_mission_seq;
} mission_data_t;
 
 typedef struct{
     mavlink_message_t msg;
 
     mavlink_heartbeat_t heartbeat_msg;
 
     mavlink_gps_raw_int_t gps_raw_int;
 
     mavlink_global_position_int_t global_position_int;
     
     mavlink_vfr_hud_t vfr_hud;
 
     mavlink_home_position_t home_position;
 
     mavlink_mission_item_reached_t mission_item_reached;
 
     mavlink_attitude_t attitude;

     mavlink_rc_channels_t rc_channels;
 
 }mavlink_str;
 
 typedef struct{
     uint8_t buf[2041];
     ssize_t len;
     int sockfd;
 }sockport;
 
 
 
 void enter_phase(sts *sts);
 void gnc_thread(sts *sts);
 //void readautopilot_thread(sockport *socket, sts *sts, mavlink_str *mavlink_str, mission_data_t *mission_data);
 void readautopilot_thread(sockport *socket, sts *sts, mavlink_str *mavlink_str);
 void sendautopilot_thread(sockport *sock, sts *sts, mavlink_str *mavlink_str); 
 
 
 #endif
 