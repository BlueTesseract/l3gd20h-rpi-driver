#include "l3gd20h.h"


int init_l3gd20h(struct l3gd20h * l)
{
	if (init_i2c(l))
		return -1;

	init_axes(l);

	init_ctrl2(l);
	init_ctrl4(l);

	return 0;
}


int init_i2c(struct l3gd20h * l)
{
	char *bus = "/dev/i2c-1";

	if ((l->i2c = open(bus, O_RDWR)) < 0) {
		printf("Failed to open the bus.\n");
		l->i2c = -1;
		return -1;
	}

	ioctl(l->i2c, I2C_SLAVE, L3GD20G_ADDRESS);
	return 0;
}


void init_axes(struct l3gd20h * l)
{
	uint8_t config[2];

	config[0] = REG_CTRL1;
	config[1] = DEFAULT_DRBW | MASK_PWR_ENABLE | MASK_Z_ENABLE | MASK_X_ENABLE | MASK_Y_ENABLE;
	write(l->i2c, config, 2);

	l->angle_x = l->angle_y = l->angle_z = 0.0;
	l->gain_x = l->gain_y = l->gain_z = 0.00875;
}


void init_ctrl2(struct l3gd20h * l)
{
	uint8_t config[2];

	config[0] = REG_CTRL2;
	config[1] = DEFAULT_CTRL2;
	write(l->i2c, config, 2);
}


void init_ctrl4(struct l3gd20h *l)
{
	uint8_t config[2];

	config[0] = REG_CTRL4;
	config[1] = DEFAULT_CTRL4;
	write(l->i2c, config, 2);
}


/* int d - calibrate using d samples */
void calibrate_l3gd20h(struct l3gd20h * l, int d)
{
	struct timeval tm;
	l->min_x = 0.0; l->max_x = 0.0; l->avg_x = 0.0;
	l->min_y = 0.0; l->max_y = 0.0; l->avg_y = 0.0;
	l->min_z = 0.0; l->max_z = 0.0; l->avg_z = 0.0;
	l->dt = 0.0;

	for (int i = 0; i < d; ++i) {
		start(&tm);
		wait_for_new_data(l);
		double x = get_axis_data_raw('x', l);
		double y = get_axis_data_raw('y', l);
		double z = get_axis_data_raw('z', l);

		l->min_x = MIN(x, l->min_x); l->max_x = MAX(x, l->max_x); l->avg_x += x;
		l->min_y = MIN(y, l->min_y); l->max_y = MAX(y, l->max_y); l->avg_y += y;
		l->min_z = MIN(z, l->min_z); l->max_z = MAX(z, l->max_z); l->avg_z += z;

		l->dt += stop(&tm);
	}

	l->avg_x /= (double)d; l->avg_y /= (double)d; l->avg_z /= (double)d;
	l->dt /= d;
	l->dt /= 1000;
}


double get_axis_data_raw(char axis, struct l3gd20h * l)
{
	int8_t reg;
	int8_t data_0 = 0;
	int8_t data_1 = 0;
	int res;

	switch (axis) {
	case 'x':
		reg = REG_X_DATA_HI;
		break;
	case 'y':
		reg = REG_Y_DATA_HI;
		break;
	case 'z':
		reg = REG_Z_DATA_HI;
		break;
	}

	write(l->i2c, &reg, 1);
	read(l->i2c, &data_0, 1);

	reg++;
	write(l->i2c, &reg, 1);
	read(l->i2c, &data_1, 1);

	// Convert the data
	res = (data_1 * 256 + data_0);
	if(res > 32767)
		res -= 65536;
	return (double)res;
}


void wait_for_new_data(struct l3gd20h * l)
{
	uint8_t status;
	do {
		status = get_reg_val(REG_DATA_STATUS, l);
	} while(!(status&8));
}


uint8_t get_reg_val(int8_t addr, struct l3gd20h * l)
{
	uint8_t out;

	write(l->i2c, &addr, 1);
	read(l->i2c, &out, 1);

	return out;
}


void update_axes(struct l3gd20h * l)
{
		double x_data = get_axis_data('x', l);
		double y_data = get_axis_data('y', l);
		double z_data = get_axis_data('z', l);

		double	dx = x_data * l->gain_x * l->dt;
		double	dy = y_data * l->gain_y * l->dt;
		double	dz = z_data * l->gain_z * l->dt;

		l->angle_x += dx;
		l->angle_y += dy;
		l->angle_z += dz;
}


double get_axis_data(char axis, struct l3gd20h * l)
{
	double min, max, avg, val;

	switch (axis) {
	case 'x':
		min = l->min_x;
		max = l->max_x;
		avg = l->avg_x;
		break;
	case 'y':
		min = l->min_y;
		max = l->max_y;
		avg = l->avg_y;
		break;
	case 'z':
		min = l->min_z;
		max = l->max_z;
		avg = l->avg_z;
		break;
	}

	val = get_axis_data_raw(axis, l);
	if (val >= min && val <= max)
		return 0.0;
	else
		return val - avg;
}
