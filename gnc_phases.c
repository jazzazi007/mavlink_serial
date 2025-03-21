/*
 * File: RPI/gnc_phases.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: Actually, missile system or liotering munition need to enter phases of guidance,
 *             navigation, and control.
 *             enter phase->mid coarse->attack phase.
 *
 */

 #include "gnc.h"
 int i =0;

static void meter2gps(sts *sts)
{
    double lat0_rad = sts->t_lat * DEG_TO_RAD;
    //double lon0_rad = sts->t_lon * DEG_TO_RAD;
    //double t2m_heading = atan2(sts->t_lat - sts->heading_lat_uav, sts->t_lon - sts->heading_lon_uav);
    //printf("t2m heading: %f\n", t2m_heading);
    //double heading = sts->t2m_heading * DEG_TO_RAD;
    double heading = 320 * DEG_TO_RAD;
    //printf("heading: %f\n", sts->t2m_heading);

    double dl_x = sts->t_displacement_x * cos(heading) - sts->t_displacement_y * sin(heading);
    double dl_y = sts->t_displacement_x * sin(heading) + sts->t_displacement_y * cos(heading);

    sts->req_lat = sts->t_lat + dl_x / EARTH_RADIUS * RAD_TO_DEG;
    sts->req_lon = sts->t_lon + dl_y / (EARTH_RADIUS * cos(lat0_rad)) * RAD_TO_DEG;
    sts->req_alt = sts->heading_alt_uav;

    //printf("req lat: %f, req lon: %f req Alt: %f\n", sts->req_lat, sts->req_lon, sts->req_alt);
}

static void entry_phase(sts *sts)
{
    //printf("Entry phase\n");
    meter2gps(sts);

    return;
}

static void stability_phase(sts *sts)
{
    int point = 1;
    double dis_x = sts->t_displacement_x;
    double dis_y = sts->t_displacement_y;

    sts->stable_lat[0] = sts->req_lat;
    sts->stable_lon[0] = sts->req_lon;
    sts->stable_alt[0] = sts->heading_alt_uav;
    while (point < 4)
    {
        
        dis_x = dis_x - 150;
        double lat0_rad = sts->t_lat * DEG_TO_RAD;
        //double lon0_rad = sts->t_lon * DEG_TO_RAD;
        //double heading = sts->t2m_heading * DEG_TO_RAD;
        double heading = 320 * DEG_TO_RAD;
        double dl_x = dis_x * cos(heading) - dis_y * sin(heading);
        double dl_y = dis_x * sin(heading) + dis_y * cos(heading);

        sts->stable_lat[point] = sts->t_lat + dl_x / EARTH_RADIUS * RAD_TO_DEG;
        sts->stable_lon[point] = sts->t_lon + dl_y / (EARTH_RADIUS * cos(lat0_rad)) * RAD_TO_DEG;
        sts->stable_alt[point] = sts->heading_alt_uav;
        point++;
    }
    printf("stability point 1: lat: %f, lon: %f, alt: %d\n", sts->stable_lat[0], sts->stable_lon[0], sts->stable_alt[0]);
    printf("stability point 2: lat: %f, lon: %f, alt: %d\n", sts->stable_lat[1], sts->stable_lon[1], sts->stable_alt[1]);
    printf("stability point 3: lat: %f, lon: %f, alt: %d\n", sts->stable_lat[2], sts->stable_lon[2], sts->stable_alt[2]);
    printf("stability point 4: lat: %f, lon: %f, alt: %d\n", sts->stable_lat[3], sts->stable_lon[3], sts->stable_alt[3]);
    return;

}

void enter_phase(sts *sts)
{
    switch (sts->mission_state)
    {
    case 0:
        i =0;
        break;
    case 1:
        entry_phase(sts);

        if (i == 0)
        {
            stability_phase(sts);
            i++;
        }
        break;
    case 2:
        break;
    case 3:
        break;
    
    default:
        break;
    }
}