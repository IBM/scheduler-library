#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "verbose.h"

#include "occ_grid.h"

extern unsigned time_step;


void print_local_occupancy_grid()
{
  printf("\nTime-Step %u : %s occupancy-grid\n", time_step, "LOCAL");
  printf("|IDX|DIST|-0-|-1-|-2-|-3-|-4-|-5-|-6-|\n");
  printf("|---|----|---|---|---|---|---|---|---|\n");
  for (int iy = OCC_GRID_Y_DIM-1; iy >= 0; iy--) {
    printf("|%3u|%4u|", iy, GRID_DIST_STEP_SIZE * iy);
    for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
      /*	switch (my_occ_grid[ix][iy]) {
		case OCC_GRID_UNKNOWN_VAL :     printf("???|"); break;
		case OCC_GRID_NO_OBSTACLE_VAL : printf("---|"); break;
		case OCC_GRID_OBSTACLE_VAL :    printf("XXX|"); break;
		case OCC_GRID_MY_CAR_VAL :      printf("MMM|"); break;
		default :                       printf("***|"); break;
		}*/
      printf("%s|", occ_grid_from_value_str[local_occ_grid[ix][iy]]);
    }
    printf("\n");
  }
  printf("END of LOCAL occupancy map\n");
}

void print_remote_occupancy_grid()
{
  printf("\nTime-Step %u : %s occupancy-grid\n", time_step, "REMOTE");
  printf("|IDX|DIST|-0-|-1-|-2-|-3-|-4-|-5-|-6-|\n");
  printf("|---|----|---|---|---|---|---|---|---|\n");
  for (int iy = OCC_GRID_Y_DIM-1; iy >= 0; iy--) {
    printf("|%3u|%4u|", iy, GRID_DIST_STEP_SIZE * iy);
    for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
      /*	switch (my_occ_grid[ix][iy]) {
		case OCC_GRID_UNKNOWN_VAL :     printf("???|"); break;
		case OCC_GRID_NO_OBSTACLE_VAL : printf("---|"); break;
		case OCC_GRID_OBSTACLE_VAL :    printf("XXX|"); break;
		case OCC_GRID_MY_CAR_VAL :      printf("MMM|"); break;
		default :                       printf("***|"); break;
		}*/
      printf("%s|", occ_grid_from_value_str[remote_occ_grid[ix][iy]]);
    }
    printf("\n");
  }
  printf("END of occupancy REMOTE map\n");
}

void print_fused_occupancy_grid()
{
  printf("\nTime-Step %u : %s occupancy-grid\n", time_step, "FUSED");
  printf("|IDX|DIST|-0-|-1-|-2-|-3-|-4-|-5-|-6-|\n");
  printf("|---|----|---|---|---|---|---|---|---|\n");
  for (int iy = OCC_GRID_Y_DIM-1; iy >= 0; iy--) {
    printf("|%3u|%4u|", iy, GRID_DIST_STEP_SIZE * iy);
    for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
      /*	switch (my_occ_grid[ix][iy]) {
		case OCC_GRID_UNKNOWN_VAL :     printf("???|"); break;
		case OCC_GRID_NO_OBSTACLE_VAL : printf("---|"); break;
		case OCC_GRID_OBSTACLE_VAL :    printf("XXX|"); break;
		case OCC_GRID_MY_CAR_VAL :      printf("MMM|"); break;
		default :                       printf("***|"); break;
		}*/
      printf("%s|", occ_grid_from_value_str[fused_occ_grid[ix][iy]]);
    }
    printf("\n");
  }
  printf("END of occupancy FUSED map\n");
}
