#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>

static struct timeval tm1;

static inline void start()
{
    gettimeofday(&tm1, NULL);
}

static inline void stop()
{
    struct timeval tm2;
    gettimeofday(&tm2, NULL);

    unsigned long long t = 1000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec) / 1000;
    printf("%llu ms\n", t);
}

int getAxisData(int8_t addr, int fd_gyro) {
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
	return res;
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
	config[1] = 0x4F;
	write(file, config, 2);

	config[0] = 0x23;
	config[1] = 0x00;
	write(file, config, 2);

	sleep(1);

	double ax = 0.0, ay = 0.0, az = 0.0;
	double gx, gy, gz;
	double dt = 0.04;
	gx = gy = gz = 0.00875;
	gy = 0.0089;

	// Calibration
	printf("Do not move!");
	// gx = calibrateAxis(0x28, file);	
	// gy = calibrateAxis(0x2A, file);	
	// gz = calibrateAxis(0x2C, file);	
	printf("Calibration completed!");

	while (1){
		start();

		int x_data = getAxisData(0x28, file);
		int y_data = getAxisData(0x2A, file);
		int z_data = getAxisData(0x2C, file);

		ax += ((double)x_data) * gx * dt;
		ay += ((double)y_data) * gy * dt;
		az += ((double)z_data) * gz * dt;

		printf("%7.2f %7.2f %7.2f\n", ax, ay, az);
		usleep(35150);
		stop();
	}
}
