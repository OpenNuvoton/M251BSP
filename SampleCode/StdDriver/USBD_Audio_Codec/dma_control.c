#include "NuMicro.h"
#include "dma_control.h"
#include "buffer_control.h"


uint32_t u32DMA_Setting_I2S_TX = 0;
uint32_t u32DMA_Setting_I2S_RX = 0;

void PDMA_IRQHandler(void)
{
    volatile uint32_t status;
    volatile uint32_t tdsts;

    status = PDMA_GET_INT_STATUS(PDMA);
    tdsts = PDMA_GET_TD_STS(PDMA);

    if (status & 0x2)   /* TD */
    {

        if (tdsts & (0x1 << I2S_DMA_PLAY_CH))
        {
            g_i8PlayBuffe_idx_OUT++;

            if (g_i8PlayBuffe_idx_OUT >= PLAY_BUFFER_DEPTH)
            {
                g_i8PlayBuffe_idx_OUT = 0;
            }

            PDMA->DSCT[I2S_DMA_PLAY_CH].CTL = u32DMA_Setting_I2S_TX; //reload
            PDMA_SET_SRC_ADDR(PDMA, I2S_DMA_PLAY_CH, (uint32_t)&g_au32PLayBuffer[g_i8PlayBuffe_idx_OUT][0]);
            PDMA_CLR_TD_FLAG(PDMA, 0x1 << I2S_DMA_PLAY_CH);

        }
        else if (tdsts & (0x1 << I2S_DMA_RECORD_CH))
        {
            g_i8RecordBuffer_idx_IN++;

            if (g_i8RecordBuffer_idx_IN >= RECORD_BUFFER_DEPTH)
            {
                g_i8RecordBuffer_idx_IN = 0;
            }

            PDMA->DSCT[I2S_DMA_RECORD_CH].CTL = u32DMA_Setting_I2S_RX; //reload
            PDMA_SET_DST_ADDR(PDMA, I2S_DMA_RECORD_CH, (uint32_t)&g_au32RecordBuffer[g_i8RecordBuffer_idx_IN][0]);
            PDMA_CLR_TD_FLAG(PDMA, 0x1 << I2S_DMA_RECORD_CH);

        }
        else
        {
            printf("DMA error\n");

            while (1);
        }


    }
    else if (status & 0x1)     //abort
    {
        printf("ABORT: ISR  %08X \n",  status);
    }
    else
    {
        printf("unknown PDMA status code\n");
    }

}

void initI2S_PDMA(void)
{
    SYS->IPRST0 |= SYS_IPRST0_PDMARST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk;

    PDMA_Open(PDMA, (1 << I2S_DMA_PLAY_CH) | (1 << I2S_DMA_RECORD_CH));

    /*Set I2S TX DMA*/
    PDMA_SetTransferCnt(PDMA, I2S_DMA_PLAY_CH, PDMA_WIDTH_32, PLAY_BUFFER_LENHTH_IN_BYTES / 4);
    PDMA_SetTransferAddr(PDMA, I2S_DMA_PLAY_CH, (uint32_t)&g_au32PLayBuffer[g_i8PlayBuffe_idx_OUT][0], PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    PDMA_SetTransferMode(PDMA, I2S_DMA_PLAY_CH, PDMA_SPI0_TX, 0, 0);
    PDMA_SetBurstType(PDMA, I2S_DMA_PLAY_CH, PDMA_REQ_SINGLE, 0);
    PDMA_EnableInt(PDMA, I2S_DMA_PLAY_CH, PDMA_INT_TRANS_DONE);
    u32DMA_Setting_I2S_TX = PDMA->DSCT[I2S_DMA_PLAY_CH].CTL;

    /*Set I2S RX DMA*/
    PDMA_SetTransferCnt(PDMA, I2S_DMA_RECORD_CH, PDMA_WIDTH_32, RECORD_BUFFER_LENHTH_IN_BYTES / 4);
    PDMA_SetTransferAddr(PDMA, I2S_DMA_RECORD_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)&g_au32RecordBuffer[g_i8RecordBuffer_idx_IN][0], PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA, I2S_DMA_RECORD_CH, PDMA_SPI0_RX, 0, 0);
    PDMA_SetBurstType(PDMA, I2S_DMA_RECORD_CH, PDMA_REQ_SINGLE, 0);
    PDMA_EnableInt(PDMA, I2S_DMA_RECORD_CH, PDMA_INT_TRANS_DONE);
    u32DMA_Setting_I2S_RX = PDMA->DSCT[I2S_DMA_RECORD_CH].CTL;


    /*Record PDMA always start*/
    SPII2S_ENABLE_RX(SPI0);
    SPII2S_ENABLE_RXDMA(SPI0);

}



