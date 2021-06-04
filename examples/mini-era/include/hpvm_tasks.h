#ifdef HPVM

#include "hpvm.h"


void vit_leaf(message_size_t msg_size, ofdm_param* ofdm_ptr, size_t ofdm_size,
        frame_param* frame_ptr, size_t frame_ptr_size,
        uint8_t* in_bits, size_t in_bit_size,
        message_t* message_id, size_t msg_id_size,
        char* out_msg_text, size_t out_msg_text_size);


void cv_leaf(label_t in_label,
        label_t* obj_label, size_t obj_label_size);

void radar_leaf(distance_t* distance_ptr, size_t distance_ptr_size,
            float* inputs_ptr, size_t inputs_ptr_size,
            uint32_t log_nsamples);

void pnc_leaf(
        vehicle_state_t* vehicle_state_ptr, size_t vehicle_state_size,
        vehicle_state_t* new_vehicle_state, size_t new_vehicle_state_size,
        distance_t* distance_ptr, size_t distance_ptr_size,
        label_t* obj_label, size_t obj_label_size,
        message_t* message_id, size_t msg_id_size,
        char* out_msg_text, size_t out_msg_text_size,
        unsigned time_step, unsigned repeat_factor 
        );

#endif
