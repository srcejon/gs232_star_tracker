// A program to track stars using a Yaesu GS232 rotator.
// Jon Beniston <jon@beniston.com>
//
// The program needs the J2000 RA/Dec (Right Ascension and Declination) of the
// star to track as well as the latitude and longitude of the rotator.
// It then periodically converts this to Az/El (Azimuth and Elevation) and sends
// that via a serial port to a GS232 rotator controller.
//
// RA/Dec can either be specified on the command line or received over TCP/IP
// from Stellarium (or other software than implements the Protocol described here:
// http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt)
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <locale.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <getopt.h>

// Right ascension and declination
typedef struct ra_dec {
    double ra;
    double dec;
} ra_dec_t;

// Azimuth and Altitude
typedef struct az_alt {
    double az;
    double alt;
} az_alt_t;

static int verbose;                             // Print extra information
static int ip_port = 10001;                     // TCP/IP port for the server to listen on

// Default position is London
static double latitude = 51.507572;             // Decimal latitude of the rotator
static double longitude = -0.127772;            // Decimal longitude of the rotator

static ra_dec_t initial_target;                 // Initial target star set on command line

static char *serial_port = "/dev/ttyS0";        // Serial port to use to send commands to the rotator controller
static int serial_fd;                           // File descriptor for GS232 rotator

static int update_interval = 5;                 // How frequently (in seconds) to update the rotator when tracking

// Print a value in degrees, minutes and seconds
void print_dms(double v)
{
    double d, m, s;
    int neg;

    neg = v < 0.0;
    v = fabs(v);
    d = floor(v);
    v -= d;
    v *= 60.0;
    m = floor(v);
    v -= m;
    v *= 60.0;
    s = v;
    if (neg) {
        printf("-");
    }
    printf("%dd%d\'%.2f\"", (int)d, (int)m, s);
}

// Print a value in hours, minutes and seconds
void print_hms(double v)
{
    double d, m, s;

    v = fabs(v);
    d = floor(v);
    v -= d;
    v *= 60.0;
    m = floor(v);
    v -= m;
    v *= 60.0;
    s = v;
    printf("%dh%dm%.2f", (int)d, (int)m, s);
}

// Print RA and Dec in both decimal and HMS/DMS format
void print_ra_dec (ra_dec_t rd)
{
    printf("RA: %f (", rd.ra);
    print_hms(rd.ra);
    printf(") Dec: %f (", rd.dec);
    print_dms(rd.dec);
    printf(")\n");
}

// Print Alt and Az in both decimal and DMS format
void print_az_alt (az_alt_t aa)
{
    printf("Alt: %f (", aa.alt);
    print_dms(aa.alt);
    printf(") Az: %f (", aa.az);
    print_dms(aa.az);
    printf(")\n");
}

// Read and echo any input from the GS232 rotator
void gs232_read(int fd)
{
    char buf;
    int len;

    while ((len = read(fd, &buf, 1)) > 0) {
        printf("%c", buf);
    }
    if ((len < 0) && (errno != EWOULDBLOCK)) {
        perror("Error reading from serial port");
    }
}

// Initialise serial connection to GS232 rotator
void gs232_init(char *serial_port)
{
    struct termios ios;

    // Open serial port
    serial_fd = open(serial_port, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("Error opening serial port");
    } else {
        if (verbose) {
            printf("Opened serial port %s\n", serial_port);
        }

        // Set baud rate and data format
        if (tcgetattr(serial_fd, &ios) < 0) {
            perror("Error getting serial port attributes");
        }
        if (cfsetospeed(&ios, B9600) < 0) {
            perror("Error setting serial port baud rate");
        }
        ios.c_cflag &= ~PARENB;
        ios.c_cflag &= ~CSTOPB;
        ios.c_cflag &= ~CSIZE;
        ios.c_cflag |= CS8 | CLOCAL;
        if (tcsetattr(serial_fd, TCSANOW, &ios) < 0) {
            perror("Error setting serial port attributes");
        }

        // Make reads non-blocking
        if (fcntl(serial_fd, F_SETFL, FNDELAY) < 0) {
            perror("Error making serial port non-blocking");
        }
    }
}

// Rotate to the given altitude and azimuth
void gs232_goto(az_alt_t aa)
{
    static int below_horizon_warning = 0;
    char cmd[256];
    int az;
    int el;
    int len;

    if (verbose) {
        printf("GoTo: ");
        print_az_alt(aa);
    }

    // Don't try to rotate below horizon
    if (aa.alt < 0.0) {
        // Only output warning the first time it happens
        if (!below_horizon_warning) {
            if (verbose) {
                printf("Ignoring rotate command below horizon. Alt=%f Az=%f\n", aa.alt, aa.az);
            }
            below_horizon_warning = 1;
        }
        return;
    }
    below_horizon_warning = 0;

    az = (int)round(aa.az);  // GS232 Azimuth is integer in range [0,450]
    el = (int)round(aa.alt);  // GS232 Elevation is integer in [0,180]

    if (serial_fd > 0) {
        // Check for any input that might block output
        gs232_read(serial_fd);

        // Create command to rotate to spcified az/el
        len = sprintf(cmd, "W%03d %03d\r\n", az, el);
        if (verbose) {
            printf("ROT command: %s\n", cmd);
        }

        // Send command via serial port
        if (len != write(serial_fd, cmd, len)) {
            perror("Error writing rotate command to serial port");
        }
    }
}

// Calculate Julian date (days since January 1, 4713 BC) from Gregorian calendar date
double juliandate(int year, int month, int day, int hours, int minutes, int seconds)
{
    int julian_day;
    double julian_date;

    // From: https://en.wikipedia.org/wiki/Julian_day
    julian_day = (1461 * (year + 4800 + (month - 14)/12))/4 +(367 * (month - 2 - 12 * ((month - 14)/12)))/12 - (3 * ((year + 4900 + (month - 14)/12)/100))/4 + day - 32075;
    julian_date = julian_day + (hours/24.0 - 0.5) + minutes/(24.0*60.0) + seconds/(24.0*60.0*60.0);

    if (verbose) {
        printf("Julian date for %4d/%d/%d %d:%02d:%02d = %f\n", year, month, day, hours, minutes, seconds, julian_date);
    }

    return julian_date;
}

// Get Julian date of J2000 Epoch
double jd_j2000(void)
{
    static double j2000 = 0.0;

    if (j2000 == 0.0) {
        j2000 = juliandate(2000, 1, 1, 12, 0, 0);
    }
    return j2000;
}

// Get Julian date of B1950 Epoch
double jd_b1950(void)
{
    static double b1950 = 0.0;

    if (b1950 == 0.0) {
        b1950 = juliandate(1949, 12, 31, 22, 9, 0);
    }
    return b1950;
}

// Get Julian date of current time Epoch
double jd_now(void)
{
    time_t system_time;
    struct tm *utc_time;

    // Get current time in seconds since Unix Epoch (1970)
    time(&system_time);
    // Convert to UTC (GMT)
    utc_time = gmtime(&system_time);

    // Convert to Julian date
    return juliandate(utc_time->tm_year + 1900, utc_time->tm_mon + 1, utc_time->tm_mday,
                      utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec);
}

// Convert from degrees, minutes and seconds to decimal degrees
double dms2decdeg(double d, double m, double s)
{
    return d + m/60.0 + s/3600.0;
}

// Convert degrees to radians
double deg2rad(double degrees)
{
    return degrees * (M_PI/180.0);
}

// Convert radians to degrees
double rad2deg(double radians)
{
    return radians * (180.0/M_PI);
}

// Precess a RA/DEC between two given Epochs
ra_dec_t precess(ra_dec_t rd_in, double jd_from, double jd_to)
{
    ra_dec_t rd_out;
    double x, y, z;
    double xp, yp, zp;
    double ra_rad, dec_rad;
    double rot[3][3];      // [row][col]
    double ra_deg;

    double days_per_century = 36524.219878;
    double t0 = (jd_from - jd_b1950())/days_per_century; // Tropical centuries since B1950.0
    double t = (jd_to - jd_from)/days_per_century;     // Tropical centuries from starting epoch to ending epoch

    // From: https://www.cloudynights.com/topic/561254-ra-dec-epoch-conversion/
    rot[0][0] = 1.0 - ((29696.0 + 26.0*t0)*t*t - 13.0*t*t*t)*.00000001;
    rot[1][0] = ((2234941.0 + 1355.0*t0)*t - 676.0*t*t + 221.0*t*t*t)*.00000001;
    rot[2][0] = ((971690.0 - 414.0*t0)*t + 207.0*t*t + 96.0*t*t*t)*.00000001;
    rot[0][1] = -rot[1][0];
    rot[1][1] = 1.0 - ((24975.0 + 30.0*t0)*t*t - 15.0*t*t*t)*.00000001;
    rot[2][1] = -((10858.0 + 2.0*t0)*t*t)*.00000001;
    rot[0][2] = -rot[2][0];
    rot[1][2] = rot[2][1];
    rot[2][2] = 1.0 - ((4721.0 - 4.0*t0)*t*t)*.00000001;

    // Hours to degrees
    ra_deg = rd_in.ra*(360.0/24.0);

    // Convert to rectangular coordinates
    ra_rad = deg2rad(ra_deg);
    dec_rad = deg2rad(rd_in.dec);
    x = cos(ra_rad) * cos(dec_rad);
    y = sin(ra_rad) * cos(dec_rad);
    z = sin(dec_rad);

    // Rotate
    xp = rot[0][0]*x + rot[0][1]*y + rot[0][2]*z;
    yp = rot[1][0]*x + rot[1][1]*y + rot[1][2]*z;
    zp = rot[2][0]*x + rot[2][1]*y + rot[2][2]*z;

    // Convert back to spherical coordinates
    rd_out.ra = rad2deg(atan(yp/xp));
    if (xp < 0.0) {
        rd_out.ra += 180.0;
    } else if ((yp < 0) && (xp > 0)) {
        rd_out.ra += 360.0;
    }
    rd_out.dec = rad2deg(asin(zp));

    // Degrees to hours
    rd_out.ra /= (360.0/24.0);

    if (verbose) {
        printf("Precessed: ");
        print_ra_dec(rd_in);
        printf("To:        ");
        print_ra_dec(rd_out);
    }

    return rd_out;
}

// Convert from J2000 right ascension (decimal hours) and declination (decimal degrees) to altitude and azimuth, for location (decimal degrees) and time (Julian date)
az_alt_t ra_dec_to_az_alt(ra_dec_t rd, double latitude, double longitude, double jd)
{
    az_alt_t aa;
    double ra_deg; // Right ascension in degrees
    double lst_deg; // Local sidereal time in degrees
    double ha; // Hour angle
    double a, az;
    double dec_rad, lat_rad, ha_rad, alt_rad; // Corresponding variables as radians

    // Precess RA/DEC from J2000 Epoch to desired (typically current) Epoch
    rd = precess(rd, jd_j2000(), jd);

    double d = (jd - jd_j2000()); // Days since J2000 epoch (including fraction)
    double f = fmod(jd, 1.0); // Fractional part is decimal days
    double ut = (f+0.5)*24.0; // Universal time in decimal hours

    // Calculate local mean sidereal time (LMST) in degrees
    // https://astronomy.stackexchange.com/questions/24859/local-sidereal-time
    // 100.46 is offset for GMST at 0h UT on 1 Jan 2000
    // 0.985647 is number of degrees per day over 360 the Earth rotates in one solar day
    // Approx to 0.3 arcseconds
    lst_deg = fmod(100.46 + 0.985647 * d + longitude + (360/24) * ut, 360.0);

    // Convert right ascension from hours to degrees
    ra_deg = rd.ra * (360.0/24.0);

    // Calculate hour angle
    ha = fmod(lst_deg - ra_deg, 360.0);

    // Convert degrees to radians
    dec_rad = deg2rad(rd.dec);
    lat_rad = deg2rad(latitude);
    ha_rad = deg2rad(ha);

    // Calculate altitude and azimuth - no correction for atmospheric refraction
    // From: http://www.convertalot.com/celestial_horizon_co-ordinates_calculator.html
    alt_rad = asin(sin(dec_rad)*sin(lat_rad) + cos(dec_rad)*cos(lat_rad)*cos(ha_rad));
    a = rad2deg(acos((sin(dec_rad)-sin(alt_rad)*sin(lat_rad)) / (cos(alt_rad)*cos(lat_rad))));
    if (sin(ha_rad) < 0.0) {
        az = a;
    } else {
        az = 360.0 - a;
    }

    aa.alt = rad2deg(alt_rad);
    aa.az = az;
    return aa;
}

// Check for goto commands from the client (Stellarium) and update the rotator
void main_loop(void)
{
    int i;
    int ret;
    unsigned char buf[1024];
    fd_set fds;
    struct timeval timeout;
    int tracking = 0;
    az_alt_t aa;
    ra_dec_t rd;

    int server = -1;
    int client = -1;
    struct sockaddr_in sa, ca;
    int ca_len;

    // Timeout for waiting for packet from Stellarium
    timeout.tv_sec = update_interval;
    timeout.tv_usec = 0;

    // Start tracking immediately if target supplied on command line
    if ((initial_target.ra != 0.0) || (initial_target.dec != 0.0)) {
        rd = initial_target;
        tracking = 1;
    }

    while (1) {

        if (tracking) {
            // Convert RA/Dec to Alt/Az
            aa = ra_dec_to_az_alt(rd, latitude, longitude, jd_now());
            // Update rotator to point to Alt/Az
            gs232_goto(aa);
        }

        if (client == -1) {
            if (server == -1) {
                // Create a server socket to listen to connections from Stellarium on
                if ((server = socket(AF_INET, SOCK_STREAM, 0)) != -1) {

                    // Bind to user specified port
                    sa.sin_family = AF_INET;
                    sa.sin_addr.s_addr = INADDR_ANY;
                    sa.sin_port = htons(ip_port);
                    if (bind(server, (struct sockaddr *)&sa, sizeof(sa))) {
                        perror("Error binding");
                        exit(EXIT_FAILURE);
                    }

                    // Listen for a connection from client
                    listen(server, 5);
                    if (verbose) {
                        printf("Waiting for connection on port %d\n", ip_port);
                    }
                    ca_len = sizeof(ca);
                }
            }
            if (server != -1) {
                // Set which file descriptor we want to wait for
                FD_ZERO(&fds);
                FD_SET(server, &fds);

                // Wait for a client to connect or timeout
                if (select(FD_SETSIZE, &fds, NULL, NULL, &timeout) > 0) {
                    client = accept(server, (struct sockaddr *)&ca, &ca_len);
                    if (client < 0) {
                        perror("Error on accept");
                    } else {
                        if (verbose) {
                            printf("Client connected\n");
                        }
                    }
                }
            }
        }

        if (client != -1) {

            // Set which file descriptor we want to wait for
            FD_ZERO(&fds);
            FD_SET(client, &fds);

            // Wait until we receive a commmand from the client or timeout
            if (select(FD_SETSIZE, &fds, NULL, NULL, &timeout) > 0) {

                ret = read(client, buf, sizeof(buf));
                if (ret > 0) {
                    int msg_len;
                    int msg_type;
                    unsigned char *msg;

                    if (0 && verbose) {
                        printf("RX %d bytes : ", ret);
                        for (i = 0; i < ret; i++) {
                            printf("%02x ", 0xff&buf[i]);
                        }
                        printf("\n");
                    }

                    // Extract length and message type
                    msg_len = buf[0] | (buf[1] << 8);
                    msg_type = buf[2] | (buf[3] << 8);
                    msg = &buf[4];

                    if (msg_type == 0) {
                        int ra;
                        int dec;

                        if (msg_len == 20) {

                            // Skip time
                            msg += 8;
                            // Extract RA LSB first
                            ra = msg[0] | (msg[1] << 8) | (msg[2] << 16) | (msg[3] << 24);
                            msg += 4;
                            // Extract DEC LSB first
                            dec = msg[0] | (msg[1] << 8) | (msg[2] << 16) | (msg[3] << 24);
                            msg += 4;

                            // Convert from integer to floating point
                            rd.ra = ra*(24.0/4294967296.0);    // Convert to decimal hours
                            rd.dec = dec*(360.0/4294967296.0);  // Convert to decimal degrees
                            if (0 && verbose) {
                                printf("RA %f Dec %f\n", rd.ra, rd.dec);
                            }

                            tracking = 1;

                        } else {
                            fprintf(stderr, "Error: Unexpected number of bytes received (%d) for msg_type=0\n", msg_len);
                        }

                    } else {
                        fprintf(stderr, "Error: Unexpected type %x\n", msg_type);
                    }

                } else if (ret == 0) {
                    if (verbose) {
                        printf("Client disconnected\n");
                    }
                    close (client);
                    client = -1;
                } else {
                    perror("Error receving");
                    close (client);
                    client = -1;
                }
            }
        }
    }
}

// Command line options
static const struct option options[] = {
    {"ra",              required_argument,      NULL,           'r'},
    {"dec",             required_argument,      NULL,           'd'},
    {"latitude",        required_argument,      NULL,           'l'},
    {"longitude",       required_argument,      NULL,           'L'},
    {"update",          required_argument,      NULL,           'u'},
    {"serial-port",     required_argument,      NULL,           's'},
    {"ip-port",         required_argument,      NULL,           'i'},
    {"verbose",         no_argument,            &verbose,       1},
    {"help",            no_argument,            0,              'h'},
    {NULL,              0,                      NULL,           0}
};

int main(int argc, char *argv[])
{
    int c;
    int option_index = 0;

    // Parse command line flags
    while (-1 != (c = getopt_long(argc, argv, "", options, &option_index))) {
        switch (c) {
        case 0:
            // Flag set
            break;
        case 'r':
            initial_target.ra = atof(optarg);
            break;
        case 'd':
            initial_target.dec = atof(optarg);
            break;
        case 'l':
            latitude = atof(optarg);
            break;
        case 'L':
            longitude = atof(optarg);
            break;
        case 'u':
            update_interval = atoi(optarg);
            break;
        case 's':
            serial_port = optarg;
            break;
        case 'i':
            ip_port = atoi(optarg);
            break;
        case 'h':
            printf("GS232 Star Tracker with Stellarium Telescope Control Server\n");
            printf("Usage:\n");
            printf("%s [[--ra decimal_hours] [--dec decimal_degrees] [--latitude decimal_lat] [--longitude decimal_long] [--update time_in_seconds] [--serial-port /dev/ttyS0] [--ip-port 10001] [--verbose] [--help]\n", argv[0]);
            printf("\n");
            printf("E.g: To set:\n");
            printf("   your location as London\n");
            printf("   updating the rotator on /dev/ttyS0 every 5 seconds\n");
            printf("   with a Stellarium Telescope Control server on TCP/IP port 10001:\n");
            printf("%s --latitude 51.507572 --longitude -0.127772 --serial-port /dev/ttyS0 --update 5 --ip-port 10001\n", argv[0]);
            printf("\n");
            printf("To immediately track PSR B0329+54 from Sydney using default serial and IP port:\n");
            printf("%s --ra 3.549819 --dec 54.5791666 --latitude -33.856322 --longitude 151.215297\n", argv[0]);
            break;
        default:
            // Unknown option
            exit(EXIT_FAILURE);
        }
    }

    // Check sensible values for RA/Dec
    if ((initial_target.ra < 0.0) || (initial_target.ra >= 24.0)) {
        printf("Warning: RA is not within range [0.0,24.0)\n");
    }
    if ((initial_target.dec < -90.0) || (initial_target.dec > 90.0)) {
        printf("Warning: Dec is not within range [-90.0,90.0]\n");
    }

    // Check sensible values for position
    if ((latitude > 90.0) || (latitude < -90.0)) {
        printf("Warning: Latitude is not within range [-90.0,90.0]\n");
    }
    if ((longitude > 180.0) || (longitude < -180.0)) {
        printf("Warning: Longitude is not within range [-180.0,180.0]\n");
    }

    if (verbose) {
        printf("Position: %f (", latitude);
        print_dms(latitude);
        printf(") %f (", longitude);
        print_dms(longitude);
        printf(")\n");
    }

    gs232_init(serial_port);

    main_loop();

    return 0;
}
