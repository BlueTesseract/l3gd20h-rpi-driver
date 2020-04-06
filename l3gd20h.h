#ifndef L3GD20H_H
#define L3GD20H_H

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <stdio.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define PI 3.141592654

#define L3GD20G_ADDRESS 0x6B

#define REG_CTRL1 0x20
#define DEFAULT_DRBW 0xA0
#define MASK_PWR_ENABLE 0x8
#define MASK_Z_ENABLE 0x4
#define MASK_X_ENABLE 0x2
#define MASK_Y_ENABLE 0x1
#define REG_CTRL2 0x21
	/* High Pass filter Mode */
#define DEFAULT_CTRL2 0x00
#define REG_CTRL3 0x22
#define REG_CTRL4 0x23
	/* 250 dps */
#define DEFAULT_CTRL4 0x00
#define REG_CTRL5 0x24

#define REG_DATA_STATUS 0x27

#define REG_X_DATA_HI 0x28
#define REG_Y_DATA_HI 0x2A
#define REG_Z_DATA_HI 0x2C


struct l3gd20h {
	int i2c;
	double gain_x;
	double gain_y;
	double gain_z;

	double angle_x;
	double angle_y;
	double angle_z;

	/* Calibration data */
	double avg_x;
	double avg_y;
	double avg_z;

	double min_x;
	double min_y;
	double min_z;

	double max_x;
	double max_y;
	double max_z;

	double dt;
};


int init_l3gd20h(struct l3gd20h * l);
int init_i2c(struct l3gd20h * l);
void init_axes(struct l3gd20h * l);
void init_ctrl2(struct l3gd20h * l);
void init_ctrl4(struct l3gd20h *l);
void calibrate_l3gd20h(struct l3gd20h * l, int d);
double get_axis_data_raw(char axis, struct l3gd20h * l);
void wait_for_new_data(struct l3gd20h * l);
uint8_t get_reg_val(int8_t addr, struct l3gd20h * l);
void update_axes(struct l3gd20h * l);
double get_axis_data(char axis, struct l3gd20h * l);


static inline void start(struct timeval * tm1)
{
	gettimeofday(tm1, NULL);
}


static inline double stop(struct timeval * tm1)
{
	struct timeval tm2;
	unsigned long long t /* ms */;

	gettimeofday(&tm2, NULL);

	t = 1000 * (tm2.tv_sec - tm1->tv_sec) + (tm2.tv_usec - tm1->tv_usec) / 1000;
	return (double)t;
}

#endif
