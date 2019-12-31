#ifndef __USBD_BUFFER_CONTROL_H__
#define __USBD_BUFFER_CONTROL_H__
#include "NuMicro.h"
#include "usbd_audio.h"


#define PLAY_BUFFER_DEPTH 8
#define RECORD_BUFFER_DEPTH 8


extern volatile int8_t g_i8PlayBuffe_idx_IN;
extern volatile int8_t g_i8PlayBuffe_idx_OUT;
extern volatile int8_t g_i8RecordBuffer_idx_IN;
extern volatile int8_t g_i8RecordBuffer_idx_OUT;


/* PLAYrate *data width * channel / 1000ms   */
#define PLAY_BUFFER_LENHTH_IN_BYTES   (PLAY_RATE * 2 * PLAY_CHANNELS / 1000) //1 ms data
#define RECORD_BUFFER_LENHTH_IN_BYTES (PLAY_RATE * 2 * PLAY_CHANNELS / 1000) //1 ms data                                                   

/* return the number of x ms data*/
__STATIC_INLINE uint32_t get_1ms_SamplesInPlayBuf(void)
{
    int32_t i32Tmp;

    i32Tmp  = g_i8PlayBuffe_idx_IN;
    i32Tmp -= g_i8PlayBuffe_idx_OUT;

    if (i32Tmp < 0)
        i32Tmp += PLAY_BUFFER_DEPTH;

    return (uint32_t)i32Tmp;
}

/* return the number of x ms data*/
__STATIC_INLINE uint32_t get_1ms_SamplesInRecordBuf(void)
{
    int32_t i32Tmp;

    i32Tmp  = g_i8RecordBuffer_idx_IN;
    i32Tmp -= g_i8RecordBuffer_idx_OUT;

    if (i32Tmp < 0)
        i32Tmp += RECORD_BUFFER_DEPTH;

    return (uint32_t)i32Tmp;
}

extern uint32_t g_au32PLayBuffer[]   [ PLAY_BUFFER_LENHTH_IN_BYTES   / 4 ]; /* /4: in word*/
extern uint32_t g_au32RecordBuffer[] [ RECORD_BUFFER_LENHTH_IN_BYTES / 4 ]; /* /4: in word*/

#endif

