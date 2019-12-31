#include "NuMicro.h"
#include "usbd_audio.h"
#include "buffer_control.h"


/*******************************************************************/
typedef enum
{
    E_RS_NONE,          // no resampling
    E_RS_UP,            // up sampling
    E_RS_DOWN           // down sampling
} RESAMPLE_STATE_T;

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of WAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    /* Send START */
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    /* Send device address */
    I2C_SET_DATA(I2C0, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C0, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send data */
    I2C_SET_DATA(I2C0, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send STOP */
    I2C_STOP(I2C0);
}


void WAU8822_Setup(void)
{

    I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);
    I2C_WriteWAU8822(1,  0x0FF);
    I2C_WriteWAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */

    I2C_WriteWAU8822(6,  0x14D);   /* Divide by 2, 48K@16Bit */
    I2C_WriteWAU8822(7,  0x000);   /* 48K for internal filter coefficients */

    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x0FF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteWAU8822(45, 0x0bf);   /* LAUXIN connected, and its Gain value is 0dB */
    I2C_WriteWAU8822(46, 0x1bf);   /* RAUXIN connected, and its Gain value is 0dB */
    I2C_WriteWAU8822(47, 0x175);   /* LAUXIN connected, and its Gain value is 0dB, MIC is +6dB */
    I2C_WriteWAU8822(48, 0x175);   /* RAUXIN connected, and its Gain value is 0dB, MIC is +6dB */

    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    I2C_WriteWAU8822(52, 0x039);   /* HP Volume */
    I2C_WriteWAU8822(53, 0x139);   /* HP Volume */

    I2C_WriteWAU8822(54, 0x140);   /* LSPKOUT Volume */
    I2C_WriteWAU8822(55, 0x140);   /* RSPKOUT Volume */

}



uint8_t I2cWrite_MultiByteforNAU88L25(uint8_t chipadd, uint16_t subaddr, const uint8_t *p, uint32_t len)
{
    /* Send START */
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    /* Send device address */
    I2C_SET_DATA(I2C0, chipadd);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C0, (uint8_t)(subaddr >> 8));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C0, (uint8_t)(subaddr));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send data */
    I2C_SET_DATA(I2C0, p[0]);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send data */
    I2C_SET_DATA(I2C0, p[1]);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send STOP */
    I2C_STOP(I2C0);

    return  0;
}


uint8_t I2C_WriteNAU88L25(uint16_t addr, uint16_t dat)
{
    uint8_t Tx_Data0[2];

    Tx_Data0[0] = (uint8_t)(dat >> 8);
    Tx_Data0[1] = (uint8_t)(dat & 0x00FF);

    return (I2cWrite_MultiByteforNAU88L25(0x1A << 1, addr, &Tx_Data0[0], 2));
}


void NAU88L25_Reset(void)
{
    I2C_WriteNAU88L25(0,  0x1);
    I2C_WriteNAU88L25(0,  0);   // Reset all registers
    CLK_SysTickDelay(10000);

    printf("NAU88L25 Software Reset.\n");
}

void NAU88L25_Setup(void)
{
    I2C_WriteNAU88L25(0x00, 0x0000);
    I2C_WriteNAU88L25(0x01, 0x07D4);
    I2C_WriteNAU88L25(0x02, 0x0000);
    I2C_WriteNAU88L25(0x03, 0x8053);
    I2C_WriteNAU88L25(0x04, 0x0081);
    I2C_WriteNAU88L25(0x05, 0x624D);
    I2C_WriteNAU88L25(0x06, 0xF010);
    I2C_WriteNAU88L25(0x07, 0x0410);
    I2C_WriteNAU88L25(0x08, 0x1000);
    I2C_WriteNAU88L25(0x09, 0x6000);
    I2C_WriteNAU88L25(0x0C, 0x0048);
    I2C_WriteNAU88L25(0x1C, 0x0002);
    I2C_WriteNAU88L25(0x1D, 0x301A);
    I2C_WriteNAU88L25(0x1E, 0x0000);
    I2C_WriteNAU88L25(0x2B, 0x00D2);
    I2C_WriteNAU88L25(0x2C, 0x0084);
    I2C_WriteNAU88L25(0x30, 0x00CF);
    I2C_WriteNAU88L25(0x31, 0x1000);
    I2C_WriteNAU88L25(0x33, 0x00D3);
    I2C_WriteNAU88L25(0x34, 0x02D3);
    I2C_WriteNAU88L25(0x50, 0x2007);
    I2C_WriteNAU88L25(0x66, 0x0060);
    I2C_WriteNAU88L25(0x68, 0xC300);
    I2C_WriteNAU88L25(0x6A, 0x0083);
    I2C_WriteNAU88L25(0x72, 0x0260);
    I2C_WriteNAU88L25(0x73, 0x332C);
    I2C_WriteNAU88L25(0x74, 0x4502);
    I2C_WriteNAU88L25(0x76, 0x3140);
    I2C_WriteNAU88L25(0x7F, 0x553F);
    I2C_WriteNAU88L25(0x80, 0x0420);

    printf("NAU88L25 Configured done.\n");
}

#if NAU8822
void AdjustCodecPll(RESAMPLE_STATE_T r)
{
    static uint16_t tb[3][3] = {{0x00C, 0x093, 0x0E9}, // 8.192
        {0x00E, 0x1D2, 0x1E3},  // * 1.005 = 8.233
        {0x009, 0x153, 0x1EF}
    }; // * .995 = 8.151
    static RESAMPLE_STATE_T current = E_RS_NONE;
    int i, s;

    if (r == current)
        return;
    else
        current = r;

    switch (r)
    {
        case E_RS_UP:
            s = 1;
            break;

        case E_RS_DOWN:
            s = 2;
            break;

        case E_RS_NONE:
        default:
            s = 0;
    }

    for (i = 0; i < 3; i++)
        I2C_WriteWAU8822(37 + i, tb[s][i]);
}

#else
/* adjust codec PLL */
__STATIC_INLINE void AdjustCodecPll(RESAMPLE_STATE_T r)
{
    /* Sample rate = 48KH*/
    /* 8.192, 8.192*1.005 = 8.23296, 8.192*0.995 = 8.15104 */
    /* 0.192   * 2^16 = 0x3126 */
    /* 0.23296 * 2^16 = 0x3BA3 */
    /* 0.15104 * 2^16 = 0x26AB */
    static uint16_t tb0[3] = {0x624D, 0x73C6, 0x50D2};


    static RESAMPLE_STATE_T current = E_RS_NONE;
    int s;

    if (r == current)
    {
        return;
    }
    else
    {
        current = r;
    }

    switch (r)
    {
        case E_RS_UP:
            s = 1;
            break;

        case E_RS_DOWN:
            s = 2;
            break;

        case E_RS_NONE:
        default:
            s = 0;
    }

    I2C_WriteNAU88L25(0x0005, tb0[s]);

}


#endif

void AdjFreq(void)
{
    uint32_t u32Size;

    if (g_u8PlayEn == 0)
        return;

    u32Size = get_1ms_SamplesInPlayBuf();


    if ((u32Size >= (PLAY_BUFFER_DEPTH / 2 - 1)) && (u32Size <= (PLAY_BUFFER_DEPTH / 2 + 1)))
    {
        AdjustCodecPll(E_RS_NONE);
    }
    else if (u32Size >= (PLAY_BUFFER_DEPTH - 2))
    {
        AdjustCodecPll(E_RS_UP);
    }
    else
    {
        AdjustCodecPll(E_RS_DOWN);
    }

}


