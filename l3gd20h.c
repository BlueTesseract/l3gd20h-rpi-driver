#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <stdint.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define PI 3.141592654

static struct timeval tm1;

static inline void start()
{
    gettimeofday(&tm1, NULL);
}

static inline double stop()
{
    struct timeval tm2;
    gettimeofday(&tm2, NULL);

    unsigned long long t = 1000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec) / 1000;
    printf("%llu ms\n", t);
	return (double)t;
}

uint8_t getRegVal(int8_t addr, int fd_gyro) {
	uint8_t out;

	write(fd_gyro, &addr, 1);
	read(fd_gyro, &out, 1);

	return out;
}

double getAxisDataRaw(int8_t addr, int fd_gyro) {
	int8_t reg;
	int8_t data_0 = 0;
	int8_t data_1 = 0;

	reg = addr;
	write(fd_gyro, &reg, 1);
	read(fd_gyro, &data_0, 1);

	reg = addr+1;
	write(fd_gyro, &reg, 1);
	read(fd_gyro, &data_1, 1);

	// Convert the data
	int res = (data_1 * 256 + data_0);
	if(res > 32767)
		res -= 65536;
	return (double)res;
}

double getAxisData(int8_t addr, int fd_gyro, double min, double max, double avg) {
	double val = getAxisDataRaw(addr, fd_gyro);
	if (val >= min && val <= max)
		return 0.0;
	else
		return val - avg;
}

struct point {
	double x;
	double y;
	double z;
};

void rotateX(double deg_angle, struct point * p) {
	double rad_angle = (deg_angle * PI) / 180.0;
	double ca = cos(rad_angle);
	double sa = sin(rad_angle);
	double x = p->x;
	double y = p->y;
	double z = p->z;

	p->x = x;
	p->y = (ca * y) - (sa * z);
	p->z = (sa * y) + (ca * z);
}

void rotateY(double deg_angle, struct point * p) {
	double rad_angle = (deg_angle * PI) / 180.0;
	double ca = cos(rad_angle);
	double sa = sin(rad_angle);
	double x = p->x;
	double y = p->y;
	double z = p->z;

	p->x = (ca * x) + (sa * z);
	p->y = y;
	p->z = (ca * z) - (sa * x);
}

void rotateZ(double deg_angle, struct point * p) {
	double rad_angle = (deg_angle * PI) / 180.0;
	double ca = cos(rad_angle);
	double sa = sin(rad_angle);
	double x = p->x;
	double y = p->y;
	double z = p->z;

	p->x = (ca * x) - (sa * y);
	p->y = (sa * x) + (ca * y);
	p->z = z;
}

double distance(struct point r, struct point p) {
	return sqrt(((p.x-r.x)*(p.x-r.x)) + ((p.y-r.y)*(p.y-r.y)) + ((p.z - r.z)*(p.z - r.z)));
}

void main() 
{
	// Create I2C bus
	int file;
	char *bus = "/dev/i2c-1";
	if((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, L3GD20H I2C address is 0x6B
	ioctl(file, I2C_SLAVE, 0x6B);

	// Enable X, Y, Z-Axis and disable Power down mode(0x4F) - 25hz
	char config[2] = {0};
	config[0] = 0x20;
	config[1] = 0xAF;
	write(file, config, 2);

	config[0] = 0x23;
	config[1] = 0x00;
	write(file, config, 2);

	config[0] = 0x21;
	config[1] = 0x26;
	write(file, config, 2);

	sleep(1);

	double ax = 0.0, ay = 0.0, az = 0.0;
	double gx, gy, gz;
	gx = gy = gz = 0.00875;

	// Calibration
	printf("Do not move!");
	// gx = calibrateAxis(0x28, file);	
	// gy = calibrateAxis(0x2A, file);	
	// gz = calibrateAxis(0x2C, file);	
	double min_x = 0.0, max_x = 0.0, avg_x = 0.0;
	double min_y = 0.0, max_y = 0.0, avg_y = 0.0;
	double min_z = 0.0, max_z = 0.0, avg_z = 0.0;
	uint8_t status;
	double dt = 0.0;

	int d = 250;
	for (int i = 0; i < d; ++i) {
	start();
		do {
			status = getRegVal(0x27, file);
		} while(!(status&8));
		double x = getAxisDataRaw(0x28, file);
		min_x = MIN(x, min_x); max_x = MAX(x, max_x); avg_x += x;

		double y = getAxisDataRaw(0x2A, file);
		min_y = MIN(y, min_y); max_y = MAX(y, max_y); avg_y += y;

		double z = getAxisDataRaw(0x2C, file);
		min_z = MIN(z, min_z); max_z = MAX(z, max_z); avg_z += z;
		dt += stop();
	}
	avg_x /= (double)d; avg_y /= (double)d; avg_z /= (double)d;
	dt /= d;
	dt /= 1000;
	printf("Calibration completed, dt = %lf!\n", dt);

	struct point p, r;
	p.x = 1.0; p.y = 0.0; p.z = 0.0;
	r.x = 1.0; r.y = 0.0; r.z = 0.0;

	while (1){
	//	 start();

		do {
			status = getRegVal(0x27, file);
		} while(!(status&8));

		double x_data = getAxisData(0x28, file, min_x, max_x, avg_x);
		double y_data = getAxisData(0x2A, file, min_y, max_y, avg_y);
		double z_data = getAxisData(0x2C, file, min_z, max_z, avg_z);

		double	dx = x_data * gx * dt;
		double	dy = y_data * gy * dt;
		double	dz = z_data * gz * dt;

		ax += dx;
		ay += dy;
		az += dz;
		rotateX(dz, &p);
		rotateY(dy, &p);
		rotateZ(dx, &p);

		printf("%7.2f %7.2f %7.2f  =====    %7.2f    =====    %7.2f\n", ax, ay, az, 2.0 * (((asin(distance(r, p)/2.0))*180.0)/PI),
				(acos(p.x * sqrt((p.x*p.x)+(p.y*p.y)))*180.0)/PI);
//		printf("d = %7.2f p = %7.2f %7.2f %7.2f\n", ((2.0 * asin(distance(r, p)/2.0))*180)/PI, p.x, p.y, p.z);

//		usleep(34750);
	//	 stop();
	}
}
