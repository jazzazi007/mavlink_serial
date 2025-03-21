/*
 * File: RPI/readautopilot.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file contains the implementation of a MAVLink heartbeat receiver.
 *
 */

 #include "gnc.h"
 // Add this function at the top of the file
 static void request_next_mission_item(sockport *sock, uint16_t seq) {
     mavlink_message_t msg;
     uint8_t buf[BUFFER_LENGTH];
     mavlink_msg_mission_request_int_pack(
         GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
         DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
         seq,
         MAV_MISSION_TYPE_MISSION
     );
     uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
     int bytes = write(sock->sockfd, buf, len);
     //printf("Requesting mission item %d (bytes: %d)\n", seq, bytes);
     tcdrain(sock->sockfd); // Ensure the data is sent
 }

 // Modify heart_prep function to use serial
 static void heart_prep(mavlink_message_t *msg, uint8_t *buf, ssize_t *len)
 {
     mavlink_heartbeat_t heartbeat;
     heartbeat.type = MAV_TYPE_GCS;
     heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
     heartbeat.base_mode = MAV_MODE_MANUAL_ARMED;
     heartbeat.custom_mode = 0;
     heartbeat.system_status = MAV_STATE_ACTIVE;
     mavlink_msg_heartbeat_encode(1, 200, msg, &heartbeat);
     len = mavlink_msg_to_send_buffer(buf, msg);
 }

 // Modify request_mission function
 void request_mission(sockport *sock)
 {
     mavlink_message_t msg;
     uint8_t buf[BUFFER_LENGTH];
     uint16_t len;

     mavlink_msg_mission_request_list_pack(
         GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
         DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
         MAV_MISSION_TYPE_MISSION
     );

     len = mavlink_msg_to_send_buffer(buf, &msg);
     write(sock->sockfd, buf, len);
     //printf("byte number: %d\n", bytesnum);
 }

 // Modify readautopilot_thread function
 void readautopilot_thread(sockport *socket, sts *sts, mavlink_str *mavlink_str)
 {     


    // int count = 0;
//////////////
         heart_prep(&mavlink_str->msg, socket->buf, &socket->len);

         // Send heartbeat over serial
         write(socket->sockfd, socket->buf, socket->len);

         if (sts->mission_state == 0)
             request_mission(socket);

         // Check for incoming data
         struct timeval tv;
         fd_set readfds;

         tv.tv_sec = 1;
         tv.tv_usec = 0;

         FD_ZERO(&readfds);
         FD_SET(socket->sockfd, &readfds);

         int retval = select(socket->sockfd + 1, &readfds, NULL, NULL, &tv);

         if (retval == -1) {
             perror("select()");
             close(socket->sockfd);
             return NULL;
         } else if (retval) {
             // Data is available to read
             char recv_buf[1024];
             int recv_len = read(socket->sockfd, recv_buf, sizeof(recv_buf));

             if (recv_len > 0) {
                 // Decode the MAVLink message
                 mavlink_status_t status;
                 for (int i = 0; i < recv_len; i++) {
                     // Inside the message parsing loop, add more debug information
                     if (mavlink_parse_char(MAVLINK_COMM_0, recv_buf[i], &mavlink_str->msg, &status)) {
                         //printf("Received message ID: %d\n", mavlink_str->msg.msgid);
                         
                         // Print out the received message details
                        //printf("Received message with ID: %d\n", mavlink_str->msg.msgid);
                         
                         // Process the specific message (e.g., HEARTBEAT)
                         if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                             mavlink_msg_heartbeat_decode(&mavlink_str->msg, &mavlink_str->heartbeat_msg);
                             //printf("Received HEARTBEAT: Type: %d, Autopilot: %d\n", mavlink_str->heartbeat_msg.type, mavlink_str->heartbeat_msg.autopilot);
                         }
                         if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
                         {
                             mavlink_msg_gps_raw_int_decode(&mavlink_str->msg, &mavlink_str->gps_raw_int);
                             sts->gps_lat = (double)mavlink_str->gps_raw_int.lat/1e7;
                             sts->gps_lon = (double)mavlink_str->gps_raw_int.lon/1e7;
                             sts->gps_alt = (double)mavlink_str->gps_raw_int.alt/1e3;
                             //sts->gps_vel = (double)mavlink_str->gps_raw_int.vel/1e2;
                             printf("Received GPS_RAW_INT: Lat: %f, Lon: %f\n", sts->gps_lat, sts->gps_lon);
                             //printf("Received GPS_RAW_INT: Alt: %f, VD: %f\n", sts->gps_alt, sts->gps_vel);
                         }
                        if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS)
                        {
                            mavlink_msg_rc_channels_decode(&mavlink_str->msg, &mavlink_str->rc_channels);
                            sts->rc_channels = (double)mavlink_str->rc_channels.chan7_raw;
                            printf("Received RC_CHANNELS: RC: %i\n", sts->rc_channels);
                        }
                         if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                         {
                             mavlink_msg_global_position_int_decode(&mavlink_str->msg, &mavlink_str->global_position_int);
                             sts->gps_lat = (double)mavlink_str->global_position_int.lat/1e7;
                             sts->gps_lon = (double)mavlink_str->global_position_int.lon/1e7;
                             sts->gps_alt = (double)mavlink_str->global_position_int.alt/1e3;
                             printf("Received GPS_RAW_INT: Lat: %f, Lon: %f\n", sts->gps_lat, sts->gps_lon);
                             printf("Received GPS_RAW_INT: Alt: %f\n", sts->gps_alt);
                         }
                         if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_VFR_HUD)
                         {
                             mavlink_msg_vfr_hud_decode(&mavlink_str->msg, &mavlink_str->vfr_hud);
                             sts->airspeed = (double)mavlink_str->vfr_hud.airspeed;
                             sts->groundspeed = (double)mavlink_str->vfr_hud.groundspeed;
                             sts->heading = (double)mavlink_str->vfr_hud.heading;
                             sts->alt_hud = (double)mavlink_str->vfr_hud.alt;
                             //printf("Received VFR_HUD: Airspeed: %f, Groundspeed: %f\n", sts->airspeed, sts->groundspeed);
                             //printf("Received VFR_HUD: Heading: %f, Altitude: %f\n", sts->heading, sts->alt_hud);
                         }
                         if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_HOME_POSITION)
                         {
                             mavlink_msg_home_position_decode(&mavlink_str->msg, &mavlink_str->home_position);
                             sts->home_lat = (double)mavlink_str->home_position.latitude/1e7;
                             sts->home_lon = (double)mavlink_str->home_position.longitude/1e7;
                             sts->home_alt = (double)mavlink_str->home_position.altitude/1e3;
                             //printf("Received HOME_POSITION: Lat: %f, Lon: %f\n", sts->home_lat, sts->home_lon);
                             //printf("Received HOME_POSITION: Alt: %f\n", sts->home_alt);
                         }
                         if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED)
                         {
                             mavlink_msg_mission_item_reached_decode(&mavlink_str->msg, &mavlink_str->mission_item_reached);
                             printf("Received MISSION_ITEM_REACHED: Seq: %d\n", mavlink_str->mission_item_reached.seq);
                             if (mavlink_str->mission_item_reached.seq == 4)
                                 sts->mission_state = 4; // if the mission at its last point this means the attack gnc will start
                         }
                         if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
                         {
                             mavlink_msg_attitude_decode(&mavlink_str->msg, &mavlink_str->attitude);
                             sts->attitude_uav[0] = (double)mavlink_str->attitude.pitch * RAD_TO_DEG;
                             sts->attitude_uav[1] = (double)mavlink_str->attitude.roll * RAD_TO_DEG;
                             sts->attitude_uav[2] = (double)mavlink_str->attitude.yaw * RAD_TO_DEG;
                             //printf("Received ATTITUDE: Pitch: %f, Roll: %f\n", sts->attitude_uav[0], sts->attitude_uav[1]);
                             //printf("Received ATTITUDE: Yaw: %f\n", sts->attitude_uav[2]);

                         }
                         // Modify only the mission handling section in your existing code
                         /*if(sts->mission_state == 0) {
                             if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
                                 mavlink_mission_count_t mission_count;
                                 mavlink_msg_mission_count_decode(&mavlink_str->msg, &mission_count);
                                 count = (int)mission_count.count;
                                // printf("Received mission count: %d\n", count);
                                 
                                 // Request first mission item (seq 0)
                                 request_next_mission_item(socket, 0);
                             }
                             
                             if (count == 3) {
                                 if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
                                     mavlink_mission_item_int_t mission_item;
                                     mavlink_msg_mission_item_int_decode(&mavlink_str->msg, &mission_item);
                                    // printf("Received mission item: %d\n", mission_item.seq);
                                     
                                     // Keep your existing mission item handling
                                     if (mission_item.seq == 1) {
                                         // ...existing target waypoint code...
                                         if (mission_item.x != 0 && mission_item.y != 0) {
                                             sts->t_lat = (double)mission_item.x/1e7;
                                             sts->t_lon = (double)mission_item.y/1e7;
                                             sts->t_alt = mission_item.z;// VI don't forget to set the altitude of target (sea level alt)
                                            // printf("Target waypoint stored: lat=%f, lon=%f\n",
                                                //   sts->t_lat, sts->t_lon);
                                         }
                                     }
                                     if (mission_item.seq == 2) {
                                         // ...existing heading waypoint code...
                                         if (mission_item.x != 0 && mission_item.y != 0) {
                                             sts->heading_lat_uav = (double)mission_item.x/1e7;
                                             sts->heading_lon_uav = (double)mission_item.y/1e7;
                                             sts->heading_alt_uav = mission_item.z;
                                            // printf("Heading waypoint stored: lat=%f, lon=%f, alt=%.2f\n",
                                                //  sts->heading_lat_uav, sts->heading_lon_uav, mission_item.z);
                                         }
                                     }
                                     if (mission_item.seq == 3) {
                                        // Add validation to prevent zero values
                                        
                                            sts->N_gain = mission_item.param1;
                                            sts->last_los_angle = mission_item.param2;
                                            sts->last_flight_path_angle = mission_item.param3;
                                            sts->k_gain = mission_item.param4;
                                            printf("param 1 %f\n", sts->N_gain);
                                            printf("param 2 %f\n", sts->last_los_angle);
                                            printf("param 3 %f\n", sts->last_flight_path_angle);
                                            printf("param 4 %f\n", sts->k_gain);
                                            //printf("Heading waypoint stored: lat=%f, lon=%f, alt=%.2f\n",
                                              //     sts->heading_lat_uav, sts->heading_lon_uav, mission_item.z);
                                        
                                    }
                                     
                                     // Keep your existing bearing calculation
                                     if (sts->heading_lat_uav != 0 && sts->t_lat != 0) {
                                         // ...existing bearing calculation code...
                                         double lat1 = sts->t_lat * DEG_TO_RAD;
                                         double lon1 = sts->t_lon * DEG_TO_RAD;
                                         double lat2 = sts->heading_lat_uav * DEG_TO_RAD;
                                         double lon2 = sts->heading_lon_uav * DEG_TO_RAD;
                                     
                                         double dlon = lon2 - lon1;
                                         double y = sin(dlon) * cos(lat2);
                                         double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
                                         double bearing = atan2(y, x);
                                     
                                         bearing = bearing * RAD_TO_DEG;
                                         bearing = fmod((bearing + 360.0), 360.0);
                                         
                                         // Only update bearing if calculation produced valid result
                                         if (!isnan(bearing)) {
                                             sts->t2m_heading = bearing;
                                             //printf("New bearing to target calculated: %f\n", sts->t2m_heading);
                                         }
                                     }
                                     
                                     // Add automatic request for next mission item
                                     if (mission_item.seq < 2) { // Since we need items 0,1,2
                                         request_next_mission_item(socket, mission_item.seq + 1);
                                     }
                                 }
                             }
                         }

                     }*/
                 }
             }  
        }
        else 
             printf("No data\n");
     }
     printf("Exiting readautopilot_thread\n");
     close(socket->sockfd);
     return NULL;
 }