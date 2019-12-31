#include "usbd_audio.h"
#include "buffer_control.h"


volatile int8_t g_i8PlayBuffe_idx_IN     = 0;
volatile int8_t g_i8PlayBuffe_idx_OUT    = 0;
volatile int8_t g_i8RecordBuffer_idx_IN  = 0;
volatile int8_t g_i8RecordBuffer_idx_OUT = 0;

uint32_t g_au32PLayBuffer   [ PLAY_BUFFER_DEPTH   ] [ PLAY_BUFFER_LENHTH_IN_BYTES   / 4]   = { 0 }; //1ms data
uint32_t g_au32RecordBuffer [ RECORD_BUFFER_DEPTH ] [ RECORD_BUFFER_LENHTH_IN_BYTES / 4]   = { 0 }; //1ms data






