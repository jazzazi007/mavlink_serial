/*
 * File: RPI/serial_connection.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file contains the implementation of a MAVLink heartbeat sender and receiver.
 *              It sends heartbeat messages to a specified IP address and UDP port, and listens
 *              for incoming MAVLink messages.
 */
#include "gnc.h"


int open_mavlink(sockport *sock)
{
    sock->sockfd = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (sock->sockfd == -1)
    {
        perror("serial port");
        return -1;
    }
    printf("Serial port opened\n");
    return 0;
}

static int try_connect_uav(int fd, int max_retries) {
    char test_buffer[1024];
    mavlink_message_t msg;
    mavlink_status_t status;
    int retry_count = 0;
    
    // Set non-blocking read with timeout
    struct termios tty;
    tcgetattr(fd, &tty);
    tty.c_cc[VTIME] = 20;  // Wait up to 2 seconds
    tty.c_cc[VMIN] = 0;    // No minimum number of bytes
    tcsetattr(fd, TCSANOW, &tty);
    
    while (retry_count < max_retries || max_retries == -1) {
        printf("Attempting to connect to UAV (attempt %d)...\n", retry_count + 1);
        
        // Try to read from the port
        int result = read(fd, test_buffer, sizeof(test_buffer));
        
        if (result > 0) {
            // Try to parse MAVLink message
            for (int i = 0; i < result; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, test_buffer[i], &msg, &status)) {
                    printf("Found valid MAVLink message (ID: %d)\n", msg.msgid);
                    
                    // Set back to non-blocking
                    tty.c_cc[VTIME] = 0;
                    tty.c_cc[VMIN] = 0;
                    tcsetattr(fd, TCSANOW, &tty);
                    
                    return 0;
                }
            }
            printf("Read %d bytes but no valid MAVLink message\n", result);
        } else if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            printf("Read error: %s (errno: %d)\n", strerror(errno), errno);
            return -1;
        }
        
        // Send heartbeat to prompt response
        mavlink_message_t heartbeat_msg;
        uint8_t buf[BUFFER_LENGTH];
        mavlink_msg_heartbeat_pack(1, 200, &heartbeat_msg, 
                                 MAV_TYPE_GCS, 
                                 MAV_AUTOPILOT_INVALID,
                                 0, 0, MAV_STATE_ACTIVE);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &heartbeat_msg);
        write(fd, buf, len);
        tcdrain(fd);
        
        sleep(1);
        retry_count++;
    }
    
    return -1;
}

int main() {
    printf("Starting...\n");
    //sleep(15);
    sockport sock;
    sts sts;
    mavlink_str mavlink_str;
   // mission_data_t mission_data;
    //memset(&mission_data, 0, sizeof(mission_data));
    memset(&sock, 0, sizeof(sock));  // Initialize memory to zero
    memset(&sts, 0, sizeof(sts));  // Initialize memory to zero
    sts.N_gain = 3.0;
    sts.last_los_angle = -12;
    sts.last_flight_path_angle = -2;
    sts.k_gain = 0.8;
    sts.t_lat = 32.157193;
    sts.t_lon = 36.266708;
    sts.t_alt = 570;
    sts.heading_alt_uav = 70;
    sts.home_lat = 32.157193;
    sts.home_lon = 36.266708;
    sts.home_alt = 575;
    sts.t_displacement_x = 600;
    sts.t_displacement_y = 0;

    sts.mission_state = 0;     //here the mission starts to defined as 0, this means the mission is still not excuted, but there is data being recieved.

    // Open serial port
    sock.sockfd = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (sock.sockfd == -1) {
        perror("Failed to open serial port");
        return -1;
    }

    // Configure serial port settings
    struct termios options;
    tcgetattr(sock.sockfd, &options);
    cfsetispeed(&options, B115200);    // Set input baud rate
    cfsetospeed(&options, B115200);    // Set output baud rate
    options.c_cflag &= ~PARENB;       // No parity
    options.c_cflag &= ~CSTOPB;       // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;           // 8 data bits
    options.c_cflag |= CLOCAL;        // Ignore modem control lines
    options.c_cflag |= CREAD;         // Enable receiver
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST;        // Raw output

    if (tcsetattr(sock.sockfd, TCSANOW, &options) < 0) {
        perror("Failed to set serial port attributes");
        close(sock.sockfd);
        return -1;
    }

    printf("Serial port opened and configured\n");

    // Test the connection with retries
    printf("Attempting to connect to UAV...\n");
    if (try_connect_uav(sock.sockfd, -1) < 0) {  // -1 means infinite retries
        perror("Failed to connect to UAV after all retries");
        close(sock.sockfd);
        return -1;
    }
    //create threads
    while (1)
    {
        readautopilot_thread(&sock, &sts, &mavlink_str);
        gnc_thread(&sts);
        sendautopilot_thread(&sock, &sts, &mavlink_str);

    }
    printf("Tasks completed.\n");
    close(sock.sockfd);
    return 0;
}