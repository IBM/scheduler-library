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
  //printf("END of LOCAL occupancy map\n");
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
  //printf("END of occupancy REMOTE map\n");
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
  //printf("END of occupancy FUSED map\n");
}



void print_side_by_side_occupancy_grids()
{
  printf("\nTime-Step %u : ALL occupancy-grids\n", time_step);
  printf("            LOCAL (MINE)                       REMOTE(YOURS)                  FUSED (TOTAL)\n");
  printf("|IDX|DIST|-0-|-1-|-2-|-3-|-4-|-5-|-6-| + |-0-|-1-|-2-|-3-|-4-|-5-|-6-| = |-0-|-1-|-2-|-3-|-4-|-5-|-6-|\n");
  printf("|---|----|---|---|---|---|---|---|---| + |---|---|---|---|---|---|---| = |---|---|---|---|---|---|---|\n");
  for (int iy = OCC_GRID_Y_DIM-1; iy >= 0; iy--) {
    printf("|%3u|%4u|", iy, GRID_DIST_STEP_SIZE * iy);
    for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
      printf("%s|", occ_grid_from_value_str[local_occ_grid[ix][iy]]);
    }
    printf(" + |");
    for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
      printf("%s|", occ_grid_from_value_str[remote_occ_grid[ix][iy]]);
    }
    printf(" = |");
    for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
      printf("%s|", occ_grid_from_value_str[fused_occ_grid[ix][iy]]);
    }
    printf("\n");
  }
  printf("|---|----|---|---|---|---|---|---|---| + |---|---|---|---|---|---|---| = |---|---|---|---|---|---|---|\n");
  //printf("END of occupancy FUSED map\n");
}



// This is a simple fusion routine that will scan from the nearest part of a lane to the
//  farthest, and use the data from the local and remote maps to form a new fused map.
// The general ruls is that we assume known-good info supercedes unknowns, basically...

unsigned occ_grid_bad_local_entries = 0;
unsigned occ_grid_bad_remote_entries = 0;
unsigned occ_grid_conflicts = 0;

int fuse_local_remote_occupancy_grids()
{
  unsigned this_occ_grid_bad_local_entries = 0;
  unsigned this_occ_grid_bad_remote_entries = 0;
  unsigned this_occ_grid_conflicts = 0;
  
  // Step through each lane.
  for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
    // Move from closest to farthest away in the lane
    for (int iy = OCC_GRID_Y_DIM-1; iy >= 0; iy--) {
      // Start with a copy of hte local information
      fused_occ_grid[ix][iy] = local_occ_grid[ix][iy];
      // And then supplement with the remote information...
      switch (local_occ_grid[ix][iy]) {
      case OCC_GRID_UNKNOWN_VAL:
	if (remote_occ_grid[ix][iy] <= OCC_GRID_MY_CAR_VAL) {
	  fused_occ_grid[ix][iy] = remote_occ_grid[ix][iy];
	} else {
	  if (occ_grid_bad_remote_entries == 0) {
	    printf("ERROR : REMOTE map has bad entry at lane %u didx %u of %u\n", ix, iy, remote_occ_grid[ix][iy]);
	  }
	  this_occ_grid_bad_remote_entries++;
	}
	break;

      case OCC_GRID_NO_OBSTACLE_VAL :
	// Local is "CLEAR" so if remote is "unknown" or "clear" we're fine, otherwise process onwards...
	if ((remote_occ_grid[ix][iy] != OCC_GRID_UNKNOWN_VAL) && (remote_occ_grid[ix][iy] != local_occ_grid[ix][iy])) {
	  // If we are marked "clear" and remote indicates THAT CAR is there, then we must take that as truth
	  if (remote_occ_grid[ix][iy] == OCC_GRID_MY_CAR_VAL) {
	    fused_occ_grid[ix][iy] = OCC_GRID_MY_CAR_VAL;
	  } else {
	    fused_occ_grid[ix][iy] = OCC_GRID_ERROR_VAL;
	    if (occ_grid_conflicts == 0) {
	      printf("ERROR : LOCAL and REMOTE maps conflict at lane %u didx %u : %u vs %u\n", ix, iy, local_occ_grid[ix][iy], remote_occ_grid[ix][iy]);
	    }
	    this_occ_grid_conflicts++;
	  }
	}
	break;

      case OCC_GRID_OBSTACLE_VAL :
	// Local is "OBSTACLE" so if remote is "unknown" or "clear" we're fine, otherwise process onwards...
	if ((remote_occ_grid[ix][iy] != OCC_GRID_UNKNOWN_VAL) && (remote_occ_grid[ix][iy] != local_occ_grid[ix][iy])) {
	  fused_occ_grid[ix][iy] = OCC_GRID_ERROR_VAL;
	  if (occ_grid_conflicts == 0) {
	    printf("ERROR : LOCAL and REMOTE maps conflict at lane %u didx %u : %u vs %u\n", ix, iy, local_occ_grid[ix][iy], remote_occ_grid[ix][iy]);
	  }
	  this_occ_grid_conflicts++;
	}
	break;

      case OCC_GRID_MY_CAR_VAL :
	// Local is "MY_CAR" so if remote is "unknown" or "clear" we're fine, otherwise process onwards...
	if ((remote_occ_grid[ix][iy] != OCC_GRID_UNKNOWN_VAL) && (remote_occ_grid[ix][iy] != OCC_GRID_NO_OBSTACLE_VAL)) {
	  // If we are marked "clear" and remote indicates THAT CAR is there, then we must take that as truth
	  /* if (remote_occ_grid[ix][iy] == OCC_GRID_MY_CAR_VAL) { */
	  /*   fused_occ_grid[ix][iy] = OCC_GRID_ERROR_VAL; */
	  /* } else { */
	  fused_occ_grid[ix][iy] = OCC_GRID_ERROR_VAL;
	  if (occ_grid_conflicts == 0) {
	    printf("ERROR : LOCAL and REMOTE maps conflict at lane %u didx %u : %u vs %u\n", ix, iy, local_occ_grid[ix][iy], remote_occ_grid[ix][iy]);
	  }
	  this_occ_grid_conflicts++;
	  /* } */
	}
	break;

	/* if ((remote_occ_grid[ix][iy] != OCC_GRID_UNKNOWN_VAL) && (remote_occ_grid[ix][iy] != local_occ_grid[ix][iy])) { */
	/*   fused_occ_grid[ix][iy] = OCC_GRID_ERROR_VAL; */
	/*   if (occ_grid_conflicts == 0) { */
	/*     printf("ERROR : LOCAL and REMOTE maps conflict at lane %u didx %u : %u vs %u\n", ix, iy, local_occ_grid[ix][iy], remote_occ_grid[ix][iy]); */
	/*   } */
	/*   occ_grid_conflicts++; */
	/* } */
	/* break; */
      default:
	if (occ_grid_bad_local_entries == 0) {
	  printf("ERROR : LOCAL map has bad entry at lane %u didx %u of %u\n", ix, iy, local_occ_grid[ix][iy]);
	}
	this_occ_grid_bad_local_entries++;
      }
    }
  }
  // Accumulate global error counts...
  occ_grid_bad_local_entries += this_occ_grid_bad_local_entries;
  occ_grid_bad_remote_entries += this_occ_grid_bad_remote_entries;
  occ_grid_conflicts += this_occ_grid_conflicts;


  // Return indicating the total errors this fusion...
  return (this_occ_grid_bad_local_entries + this_occ_grid_bad_remote_entries + this_occ_grid_conflicts);
}


