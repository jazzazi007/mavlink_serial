/*
 * File: RPI/send_autopilot.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description:
 * setup the intial state of fixed wing
 * measure relative velocity
 * calculate the line of sight
 * attitude and oriantation
 * pure proportional navigation depending on LOS and flight path
*/

#include "gnc.h"

void gps2meter(sts *sts)
{
    double dlat = sts->gps_lat - sts->t_lat;  
    double dlon = sts->gps_lon - sts->t_lon;  
    double lat1 = sts->t_lat * DEG_TO_RAD;
    double lat2 = sts->gps_lat * DEG_TO_RAD;
    double dlat_rad = dlat * DEG_TO_RAD;
    double dlon_rad = dlon * DEG_TO_RAD;

    double a = sin(dlat_rad/2) * sin(dlat_rad/2) +
               cos(lat1) * cos(lat2) * 
               sin(dlon_rad/2) * sin(dlon_rad/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    sts->t2m_distance = EARTH_RADIUS * c;
    
    //printf("distance: %.2f meters\n", sts->t2m_distance);
}

void calculate_bearing_alt(sts *sts)
{
    // Convert to radians
    double lat1 = sts->gps_lat * DEG_TO_RAD;
    double lon1 = sts->gps_lon * DEG_TO_RAD;
    double lat2 = sts->t_lat * DEG_TO_RAD;
    double lon2 = sts->t_lon * DEG_TO_RAD;

    // Calculate bearing
    double dlon = lon2 - lon1;
    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    sts->bearing = atan2(y, x);

    // Convert to degrees
    sts->bearing = sts->bearing * RAD_TO_DEG;
    // Normalize to 0-360
    sts->bearing = fmod((sts->bearing + 360.0), 360.0);

    // Calculate heading error (difference between current heading and required bearing)
    sts->bearing_error = sts->bearing - sts->heading;
    // Normalize to -180 to +180
    if (sts->bearing_error > 180) sts->bearing_error -= 360;
    if (sts->bearing_error < -180) sts->bearing_error += 360;

    sts->t2m_altitude = sts->gps_alt - sts->t_alt;

    //printf("Current heading: %.2f°, Bearing to home: %.2f°, Error: %.2f°, different alt: %.2f\n", 
      //     sts->heading, sts->bearing, sts->bearing_error, sts->t2m_altitude);
}

void los_angle(sts *sts)
{
    sts->los_angle = -atan2(sts->t2m_altitude, sts->t2m_distance) * RAD_TO_DEG;
    //printf("Line of sight angle: %.2f°\n", sts->los_angle);
}

// Modify your desired_flight_path_angle function
//void desired_flight_path_angle(sts *sts, angle_plot_t *plot)
void desired_flight_path_angle(sts *sts)
{
    los_angle(sts);
    sts->flight_path_angle = sts->N_gain * (sts->los_angle - sts->last_los_angle) - sts->last_flight_path_angle;
    //printf("last flight path angle: %.2f°, new flight path angle: %.2f°\n", sts->last_flight_path_angle, sts->flight_path_angle);
    // Update the plot with new angles
    //update_angle_plot(plot, sts->attitude_uav[0], sts->flight_path_angle);
}

void desired_roll_correction(sts *sts)
{
    // lambda LOS = bearing error from sts
    float g = 9.81; //m/s^2
    sts->desired_roll_angle = atan2(sts->airspeed * sts->k_gain * (sts->bearing_error * DEG_TO_RAD), g) * RAD_TO_DEG;
    //printf("Roll correction: %.2f°\n", sts->desired_roll_angle);
    //printf("airspeed: %.2f\n", sts->airspeed);
    //printf("bearing error: %.2f\n", sts->bearing_error);
}

void quaternion(sts *sts)
{
        //Quaternion to Euler angles: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles   
    
        float roll  = sts->desired_roll_angle * DEG_TO_RAD;
        float pitch = sts->flight_path_angle * DEG_TO_RAD;
        float yaw   = 0 * DEG_TO_RAD;
        sts->q[0] = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2); //w
        sts->q[1] = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2); //x
        sts->q[2] = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2); //y
        sts->q[3] = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2); //z
}

void gnc_thread(sts *sts) 
{

        enter_phase(sts);
        gps2meter(sts);
        calculate_bearing_alt(sts);
        //rc_drive(sts, joy, gains);
        // Update angle calculations with plot
        //desired_flight_path_angle(sts, plot);
        //printf("joy throttle %d, roll %d, pitch %d\n", joy->throttle, joy->roll, joy->pitch);
        desired_flight_path_angle(sts);
        desired_roll_correction(sts);
        quaternion(sts);
    return NULL;
}