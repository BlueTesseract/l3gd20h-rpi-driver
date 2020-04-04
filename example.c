#include <stdio.h>
#include <stdlib.h>
#include "l3gd20h.h"


void main(int argc, char * argv[])
{
	struct l3gd20h * l = malloc(sizeof(*l));

	if (init_l3gd20h(l)) {
		printf("Error: Cannot init l3gd20h struct!\n");
		goto end;
	}

	calibrate_l3gd20h(l, 250);

	while (1) {
		wait_for_new_data(l);
		update_axes(l);

		printf("%7.2f %7.2f %7.2f\n", l->angle_x, l->angle_y, l->angle_z);
	}

end:
	free(l);
}
