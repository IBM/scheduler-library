#ifndef H_HPVM_TASKS_H
#define H_HPVM_TASKS_H

#ifdef HPVM

#include "base_types.h"
#include "hpvm.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>

typedef struct __attribute__((__packed__)) {
  message_size_t msg_size;
  ofdm_param *ofdm_ptr;
  size_t ofdm_size;
  frame_param *frame_ptr;
  size_t frame_ptr_size;
  uint8_t *in_bits;
  size_t in_bit_size;
  message_t *message_id;
  size_t msg_id_size;
  char *out_msg_text;
  size_t out_msg_text_size;
  label_t in_label;
  label_t *obj_label;
  size_t obj_label_size;
  uint32_t log_nsamples;
  float *inputs_ptr;
  size_t inputs_ptr_size;
  distance_t *distance_ptr;
  size_t distance_ptr_size;
  unsigned time_step;
  unsigned repeat_factor;
  vehicle_state_t* current_vehicle_state;
  size_t current_vehicle_state_size;
  vehicle_state_t *new_vehicle_state;
  size_t new_vehicle_state_size;
} RootIn;

void vit_leaf(message_size_t msg_size, ofdm_param *ofdm_ptr, size_t ofdm_size,
              frame_param *frame_ptr, size_t frame_ptr_size, uint8_t *in_bits,
              size_t in_bit_size, message_t *message_id, size_t msg_id_size,
              char *out_msg_text, size_t out_msg_text_size);

void cv_leaf(label_t in_label, label_t *obj_label, size_t obj_label_size);


void radar_leaf(uint32_t log_nsamples,float *inputs_ptr, size_t inputs_ptr_size,
                distance_t *distance_ptr, size_t distance_ptr_size
                );

void MiniERARootWrapper(message_size_t msg_size, ofdm_param *ofdm_ptr,
                 size_t ofdm_size, frame_param *frame_ptr,
                 size_t frame_ptr_size, uint8_t *in_bits, size_t in_bit_size,
                 message_t *message_id, size_t msg_id_size, char *out_msg_text,
                 size_t out_msg_text_size, label_t in_label, label_t *obj_label,
                 size_t obj_label_size, uint32_t log_nsamples, float *inputs_ptr,
                 size_t inputs_ptr_size, distance_t *distance_ptr,
                 size_t distance_ptr_size,
                 unsigned time_step, unsigned repeat_factor,
                 vehicle_state_t *current_vehicle_state,
                 size_t current_vehicle_state_size,
                 vehicle_state_t *new_vehicle_state,
                 size_t new_vehicle_state_size);

void MiniERARoot(message_size_t msg_size, ofdm_param *ofdm_ptr,
                 size_t ofdm_size, frame_param *frame_ptr,
                 size_t frame_ptr_size, uint8_t *in_bits, size_t in_bit_size,
                 message_t *message_id, size_t msg_id_size, char *out_msg_text,
                 size_t out_msg_text_size, label_t in_label, label_t *obj_label,
                 size_t obj_label_size, uint32_t log_nsamples, float *inputs_ptr,
                 size_t inputs_ptr_size, distance_t *distance_ptr,
                 size_t distance_ptr_size,
                 unsigned time_step,
                 unsigned repeat_factor,
                 vehicle_state_t *current_vehicle_state,
                 size_t current_vehicle_state_size,
                 vehicle_state_t *new_vehicle_state,
                 size_t new_vehicle_state_size);


void pnc_leaf(unsigned time_step, unsigned repeat_factor,  label_t *obj_label, size_t obj_label_size,
              distance_t *distance_ptr, size_t distance_ptr_size,
              message_t *message_id, size_t msg_id_size, vehicle_state_t* current_vehicle_state, size_t current_vehicle_state_size,
              vehicle_state_t* new_vehicle_state, size_t new_vehicle_state_size ,char *out_msg_text, size_t out_msg_text_size
              );
 



void hpvm_launch(RootIn *DFGArgs, label_t *cv_tr_label, unsigned time_step,
                 unsigned log_nsamples, float *radar_inputs,
                 distance_t *distance, message_size_t vit_msgs_size,
                 vit_dict_entry_t *vdentry_p, message_t *message,
                 char *out_msg_text, vehicle_state_t *vehicle_state,
                 vehicle_state_t *new_vehicle_state,
                 unsigned pandc_repeat_factor);

RootIn *hpvm_initialize();

void hpvm_cleanup(RootIn *);

#ifdef HPVM_BASE_CRIT

void hpvm_launch_base_VIT(int vit_base_msg_size, vit_dict_entry_t *base_vdentry);

void hpvm_launch_base_RADAR(unsigned base_log_nsamples, float* base_radar_inputs);

void hpvm_launch_base_CV(label_t cv_tr_label);


typedef struct __attribute__((__packed__)) {
  message_size_t msg_size;
  ofdm_param *ofdm_ptr;
  size_t ofdm_size;
  frame_param *frame_ptr;
  size_t frame_ptr_size;
  uint8_t *in_bits;
  size_t in_bit_size;
  message_t *message_id;
  size_t msg_id_size;
  char *out_msg_text;
  size_t out_msg_text_size;
} VitRootIn;

typedef struct __attribute__((__packed__)) {
  uint32_t log_nsamples;
  float *inputs_ptr;
  size_t inputs_ptr_size;
  distance_t *distance_ptr;
  size_t distance_ptr_size;

} RadarRootIn;

typedef struct __attribute__((__packed__)) {
  label_t in_label;
  label_t *obj_label;
  size_t obj_label_size;
} CVRootIn;


void VITRootWrapper(
  message_size_t msg_size, ofdm_param *ofdm_ptr, size_t ofdm_size,
  frame_param *frame_ptr, size_t frame_ptr_size, uint8_t *in_bits,
  size_t in_bit_size, message_t *message_id, size_t msg_id_size, char *out_msg_text,
  size_t out_msg_text_size
        );

void VITRoot(
  message_size_t msg_size, ofdm_param *ofdm_ptr, size_t ofdm_size,
  frame_param *frame_ptr, size_t frame_ptr_size, uint8_t *in_bits,
  size_t in_bit_size, message_t *message_id, size_t msg_id_size, char *out_msg_text,
  size_t out_msg_text_size
        );


void RadarRootWrapper(
  uint32_t log_nsamples, float *inputs_ptr, size_t inputs_ptr_size,
  distance_t *distance_ptr, size_t distance_ptr_size);

void RadarRoot(
  uint32_t log_nsamples, float *inputs_ptr, size_t inputs_ptr_size,
  distance_t *distance_ptr, size_t distance_ptr_size);

void CVRootWrapper(
  label_t in_label, label_t *obj_label, size_t obj_label_size);

void CVRoot(
  label_t in_label, label_t *obj_label, size_t obj_label_size);

#endif

#endif

#endif
