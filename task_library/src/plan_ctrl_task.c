/*
 * Copyright 2019 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

 //#define VERBOSE
#include "verbose.h"

#include "plan_ctrl_task.h"
#include "scheduler.h"

std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> plan_ctrl_profile;

extern "C" {
  void print_plan_ctrl_metadata_block_contents(void * mb_ptr) {
    task_metadata_entry * mb = (task_metadata_entry *) mb_ptr;
    print_base_metadata_block_contents(mb);
    plan_ctrl_io_t * plan_ctrl_data_p = (plan_ctrl_io_t *) (mb->data_space);
    printf("  PLAN_CTRL: in_object     = %u\n", *(plan_ctrl_data_p->obj_label));
    printf("  PLAN_CTRL: in_distance   = %.1f\n", *(plan_ctrl_data_p->distance_ptr));
    printf("  PLAN_CTRL: in_message    = %u\n", *(plan_ctrl_data_p->message_id));
    printf("  PLAN_CTRL: in_veh_state  : Active %u Lane %u Speed %.1f\n",
      plan_ctrl_data_p->current_vehicle_state->active, plan_ctrl_data_p->current_vehicle_state->lane,
      plan_ctrl_data_p->current_vehicle_state->speed);
    printf("  PLAN_CTRL: out_veh_state : Active %u Lane %u Speed %.1f\n",
      plan_ctrl_data_p->new_vehicle_state->active, plan_ctrl_data_p->new_vehicle_state->lane,
      plan_ctrl_data_p->new_vehicle_state->speed);
  }
}

extern "C" {
  void output_plan_ctrl_task_type_run_stats(void * sptr_ptr, unsigned my_task_type,
    unsigned total_accel_types) {
    scheduler_datastate * sptr = (scheduler_datastate *) sptr_ptr;

    printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n",
      my_task_type, sptr->task_name_str[my_task_type], sptr->freed_metadata_blocks[my_task_type],
      total_accel_types);
    // The PLAN_CTRL/CNN Task Timing Info
    unsigned total_plan_ctrl_comp_by[total_accel_types + 1];
    uint64_t total_plan_ctrl_call_usec[total_accel_types + 1];
    for (int ai = 0; ai <= total_accel_types; ai++) {
      total_plan_ctrl_comp_by[ai] = 0;
      total_plan_ctrl_call_usec[ai] = 0;
    }
    for (int ai = 0; ai < total_accel_types; ai++) {
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_type,
          sptr->task_name_str[my_task_type], ai, sptr->accel_name_str[ai]);
      }
      for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
        plan_ctrl_timing_data_t * plan_ctrl_timings_p = (plan_ctrl_timing_data_t *) &(
          sptr->master_metadata_pool[bi].task_timings[my_task_type]);
        unsigned this_comp_by = (unsigned) (
          sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
        uint64_t this_plan_ctrl_call_usec = (uint64_t) (plan_ctrl_timings_p->call_sec[ai]) * 1000000 +
          (uint64_t) (plan_ctrl_timings_p->call_usec[ai]);
        if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
          printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu usec\n", bi, ai, sptr->accel_name_str[ai],
            this_comp_by, this_plan_ctrl_call_usec);
        }
        else {
          if ((this_comp_by + this_plan_ctrl_call_usec) != 0) {
            printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu\n", bi, ai, sptr->accel_name_str[ai],
              this_comp_by, this_plan_ctrl_call_usec);
          }
        }
        // Per acceleration (CPU, HWR)
        total_plan_ctrl_comp_by[ai] += this_comp_by;
        total_plan_ctrl_call_usec[ai] += this_plan_ctrl_call_usec;
        // Overall Total
        total_plan_ctrl_comp_by[total_accel_types] += this_comp_by;
        total_plan_ctrl_call_usec[total_accel_types] += this_plan_ctrl_call_usec;
      } // for (bi = 1 .. numMetatdataBlocks)
    }   // for (ai = 0 .. total_accel_types)

    printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type,
      sptr->task_name_str[my_task_type]);
    printf("     PnC-call  run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_plan_ctrl_call_usec[ai] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_plan_ctrl_comp_by[ai],
        total_plan_ctrl_call_usec[ai], avg);
    }
    {
      double avg = (double) total_plan_ctrl_call_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_plan_ctrl_comp_by[total_accel_types],
        total_plan_ctrl_call_usec[total_accel_types], avg);
    }
  }
}

extern "C" {
  void execute_on_cpu_plan_ctrl_accelerator(void * plan_ctrl_io_ptr) {
    plan_ctrl_io_t * plan_ctrl_data_p = (plan_ctrl_io_t *) (plan_ctrl_io_ptr);

    DEBUG(printf("In the plan_and_control task : label %u %s distance %.1f (T1 %.1f T2 %.1f T3 %.1f) message %u\n",
      *(plan_ctrl_data_p->obj_label), object_names[*(plan_ctrl_data_p->obj_label)],
      *(plan_ctrl_data_p->distance_ptr),
      PNC_THRESHOLD_1, PNC_THRESHOLD_2, PNC_THRESHOLD_3, *(plan_ctrl_data_p->message_id)));
    DEBUG(printf("Plan-Ctrl: current Vehicle-State : Active %u Lane %u Speed %.1f\n",
      plan_ctrl_data_p->current_vehicle_state->active, plan_ctrl_data_p->current_vehicle_state->lane,
      plan_ctrl_data_p->current_vehicle_state->speed));


    // Start with outpu vehicle state is a copy of input vehicle state...
    plan_ctrl_data_p->new_vehicle_state = plan_ctrl_data_p->current_vehicle_state;
    if (!plan_ctrl_data_p->current_vehicle_state->active) {
      // Our car is broken and burning, no plan-and-control possible -- nothing to
      // do
    }
    else if ( //(*(plan_ctrl_data_p->obj_label) != no_object) && // For safety, assume every return is from SOMETHING we should not hit!
      ((*(plan_ctrl_data_p->distance_ptr) <= PNC_THRESHOLD_1)
#ifdef USE_SIM_ENVIRON
        || ((plan_ctrl_data_p->current_vehicle_state->speed < car_goal_speed) &&
          (*(plan_ctrl_data_p->distance_ptr) <= PNC_THRESHOLD_2))
#endif
        )) {
      // This covers all cases where we have an obstacle "too close" ahead of us
      if (*(plan_ctrl_data_p->distance_ptr) <= IMPACT_DISTANCE) {
        // We've crashed into an obstacle...
        printf("WHOOPS: We've suffered a collision on time_step %u!\n", plan_ctrl_data_p->time_step);
        // fprintf(stderr, "WHOOPS: We've suffered a collision on time_step %u!\n", plan_ctrl_data_p->time_step);
        plan_ctrl_data_p->new_vehicle_state->speed = 0.0;
        plan_ctrl_data_p->new_vehicle_state->active = false; // We should add visualizer stuff for this!
      }
      else {
        // Some object ahead of us that needs to be avoided.
        DEBUG(printf("  In lane %s with object %u at %.1f\n",
          lane_names[plan_ctrl_data_p->current_vehicle_state->lane], *(plan_ctrl_data_p->obj_label),
          *(plan_ctrl_data_p->distance_ptr)));
        switch (*(plan_ctrl_data_p->message_id)) {
        case safe_to_move_right_or_left:
          /* Bias is move right, UNLESS we are in the Right lane and would then
           * head into the RHazard Lane */
          if (plan_ctrl_data_p->current_vehicle_state->lane < right) {
            DEBUG(printf("   In %s with Safe_L_or_R : Moving Right\n",
              lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
            plan_ctrl_data_p->new_vehicle_state->lane = (lane_t) ((int) plan_ctrl_data_p->new_vehicle_state->lane
              + 1);
          }
          else {
            DEBUG(printf("   In %s with Safe_L_or_R : Moving Left\n",
              lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
            plan_ctrl_data_p->new_vehicle_state->lane = (lane_t) ((int) plan_ctrl_data_p->new_vehicle_state->lane
              - 1);
          }
          break; // prefer right lane
        case safe_to_move_right_only:
          DEBUG(printf("   In %s with Safe_R_only : Moving Right\n",
            lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
          plan_ctrl_data_p->new_vehicle_state->lane = (lane_t) ((int) plan_ctrl_data_p->new_vehicle_state->lane
            + 1);
          break;
        case safe_to_move_left_only:
          DEBUG(printf("   In %s with Safe_L_Only : Moving Left\n",
            lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
          plan_ctrl_data_p->new_vehicle_state->lane = (lane_t) ((int) plan_ctrl_data_p->new_vehicle_state->lane
            - 1);
          break;
        case unsafe_to_move_left_or_right:
#ifdef USE_SIM_ENVIRON
          if (plan_ctrl_data_p->current_vehicle_state->speed > car_decel_rate) {
            plan_ctrl_data_p->new_vehicle_state->speed = plan_ctrl_data_p->current_vehicle_state->speed -
              car_decel_rate; // was / 2.0;
            DEBUG(printf("   In %s with No_Safe_Move -- SLOWING DOWN from %.2f to %.2f\n",
              lane_names[plan_ctrl_data_p->current_vehicle_state->lane],
              plan_ctrl_data_p->current_vehicle_state->speed, plan_ctrl_data_p->new_vehicle_state->speed));
          }
          else {
            DEBUG(printf("   In %s with No_Safe_Move -- Going < 15.0 so STOPPING!\n",
              lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
            plan_ctrl_data_p->new_vehicle_state->speed = 0.0;
          }
#else
          DEBUG(printf("   In %s with No_Safe_Move : STOPPING\n",
            lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
          plan_ctrl_data_p->new_vehicle_state->speed = 0.0;
#endif
          break; /* Stop!!! */
        default:
          printf(" ERROR  In %s with UNDEFINED MESSAGE: %u\n",
            lane_names[plan_ctrl_data_p->current_vehicle_state->lane], *(plan_ctrl_data_p->message_id));
          // exit( -6);
        }
      } // end of "we have some obstacle too close ahead of us"
    }
    else {
      // No obstacle-inspired lane change, so try now to occupy the center lane
      switch (plan_ctrl_data_p->current_vehicle_state->lane) {
      case lhazard:
      case left:
        if ((*(plan_ctrl_data_p->message_id) == safe_to_move_right_or_left) ||
          (*(plan_ctrl_data_p->message_id) == safe_to_move_right_only)) {
          DEBUG(printf("  In %s with Can_move_Right: Moving Right\n",
            lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
          plan_ctrl_data_p->new_vehicle_state->lane = (lane_t) ((int) plan_ctrl_data_p->new_vehicle_state->lane
            + 1);
        }
        break;
      case center:
        // No need to alter, already in the center
        break;
      case right:
      case rhazard:
        if ((*(plan_ctrl_data_p->message_id) == safe_to_move_right_or_left) ||
          (*(plan_ctrl_data_p->message_id) == safe_to_move_left_only)) {
          DEBUG(printf("  In %s with Can_move_Left : Moving Left\n",
            lane_names[plan_ctrl_data_p->current_vehicle_state->lane]));
          plan_ctrl_data_p->new_vehicle_state->lane = (lane_t) ((int) plan_ctrl_data_p->new_vehicle_state->lane
            - 1);
        }
        break;
      }
#ifdef USE_SIM_ENVIRON
      if ((plan_ctrl_data_p->current_vehicle_state->speed <
        car_goal_speed) && // We are going slower than we want to, and
       //((*(plan_ctrl_data_p->obj_label) == no_object) || // There is no object ahead of us -- don't need;
        (*(plan_ctrl_data_p->distance_ptr) >= PNC_THRESHOLD_2)) { // Any object is far enough away
        if (plan_ctrl_data_p->current_vehicle_state->speed <= (car_goal_speed - car_accel_rate)) {
          plan_ctrl_data_p->new_vehicle_state->speed += 15.0;
        }
        else {
          plan_ctrl_data_p->new_vehicle_state->speed = car_goal_speed;
        }
        DEBUG(printf("  Going %.2f : slower than target speed %.2f : Speeding up to %.2f\n",
          plan_ctrl_data_p->current_vehicle_state->speed, 50.0, plan_ctrl_data_p->new_vehicle_state->speed));
      }
#endif
    } // end of plan-and-control logic functions...

    DEBUG(printf(
      "Plan-Ctrl:     new Vehicle-State : Active %u Lane %u Speed %.1f\n",
      plan_ctrl_data_p->new_vehicle_state->active, plan_ctrl_data_p->new_vehicle_state->lane,
      plan_ctrl_data_p->new_vehicle_state->speed));
  }
}

void set_up_plan_ctrl_task_on_accel_profile_data() {
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
    plan_ctrl_profile[sizeof(label_t) + sizeof(message_t) + sizeof(distance_t)][ai] = ACINFPROF;
  }
#ifdef COMPILE_TO_ESP
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  plan_ctrl_profile[sizeof(label_t) + sizeof(message_t) + sizeof(distance_t)][SCHED_CPU_ACCEL_T] = 500; // Picked a small value...
#else
  plan_ctrl_profile[sizeof(label_t) + sizeof(message_t) + sizeof(distance_t)][SCHED_CPU_ACCEL_T] = 1; // Picked a small value...
#endif
  DEBUG(printf("\n%18s : %18s %18s %18s %18s\n", "PnC-PROFILES", "CPU", "FFT-HWR", "VIT-HWR",
    "CV-HWR");
  printf("%15s :", "pnc_profile");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", plan_ctrl_profile[sizeof(label_t) + sizeof(message_t) + sizeof(distance_t)][0][ai]); }
  printf("\n\n"));
}


// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
extern "C" {
  void plan_ctrl_auto_finish_routine(void * mb_ptr) {
    task_metadata_entry * mb = (task_metadata_entry *) mb_ptr;
    TDEBUG(scheduler_datastate * sptr = mb->scheduler_datastate_pointer;
    printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n",
      mb->block_id, sptr->task_name_str[mb->task_type], sptr->task_criticality_str[mb->crit_level],
      sptr->accel_name_str[mb->accelerator_type], mb->accelerator_id));
    DEBUG(printf("  MB%u auto Calling free_task_metadata_block\n", mb->block_id));
    free_task_metadata_block(mb);
  }
}

// NOTE: This routine DOES NOT copy out the data results -- a call to
//   calculate_peak_distance_from_fmcw now results in alteration ONLY
//   of the metadata task data; we could send in the data pointer and
//   over-write the original input data with the PLAN_CTRL results (As we used
//   to) but this seems un-necessary since we only want the final "distance"
//   really.
extern "C" {
  void finish_plan_ctrl_execution(void * plan_ctrl_metadata_block_ptr, void * args) {
    task_metadata_entry * plan_ctrl_metadata_block = (task_metadata_entry *) plan_ctrl_metadata_block_ptr;
    // vehicle_state_t *new_vehicle_state)


    // We've finished the execution and lifetime for this task; free its metadata
    DEBUG(printf("  MB%u Calling free_task_metadata_block\n", plan_ctrl_metadata_block->block_id));
    free_task_metadata_block(plan_ctrl_metadata_block);
  }
}