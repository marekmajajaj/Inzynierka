/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <signal.h>

#include "mailbox.h"

/*
 * Check more about Raspberry Pi's register mapping at:
 * https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 * https://elinux.org/BCM2835_registers
 */
#define PAGE_SIZE 4096

#define PERI_BUS_BASE 0x7E000000
#define PERI_PHYS_BASE 0x3F000000
#define BUS_TO_PHYS(x) ((x) & ~0xC0000000)

#define CM_BASE 0x00101000
#define CM_LEN 0xA8
#define CM_PWM 0xA0
#define CLK_CTL_BUSY (1 << 7)
#define CLK_CTL_KILL (1 << 5)
#define CLK_CTL_ENAB (1 << 4)
#define CLK_CTL_SRC(x) ((x) << 0)

#define CLK_SRCS 2

#define CLK_CTL_SRC_OSC 1
#define CLK_CTL_SRC_PLLD 6

#define CLK_OSC_FREQ 19200000
#define CLK_OSC_FREQ_2711 54000000
#define CLK_PLLD_FREQ 500000000
#define CLK_PLLD_FREQ_2711 750000000

#define CLK_DIV_DIVI(x) ((x) << 12)

#define BCM_PASSWD (0x5A << 24)

#define PWM_BASE 0x0020C000
#define PWM_LEN 0x28
#define PWM_FIFO 0x18

/* PWM control bits */
#define PWM_CTL 0
#define PWM_STA 1
#define PWM_DMAC 2
#define PWM_RNG1 4
#define PWM_DAT1 5
#define PWM_RNG2 8
#define PWM_DAT2 9

#define PWM_CTL_MSEN2 (1 << 15)
#define PWM_CTL_PWEN2 (1 << 8)
#define PWM_CTL_MSEN1 (1 << 7)
#define PWM_CTL_CLRF1 (1 << 6)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

#define PWM_DMAC_ENAB (1 << 31)
#define PWM_DMAC_PANIC(x) ((x) << 8)
#define PWM_DMAC_DREQ(x) (x)

#define GPIO_BASE 0x00200000
#define GPIO_LEN 0x60
#define GPIO_FUNC_SELECT_CLEAR(x) ~(7 << ((x) * 3))
#define GPIO_FUNC_SELECT_IN(x) (0 << ((x) * 3))
#define GPIO_FUNC_SELECT_OUT(x) (1 << ((x) * 3))
#define GPIO_FUNC_SELECT_ALT0(x) (4 << ((x) * 3))
#define GPIO_FUNC_SELECT_ALT1(x) (5 << ((x) * 3))
#define GPIO_FUNC_SELECT_ALT2(x) (6 << ((x) * 3))
#define GPIO_FUNC_SELECT_ALT3(x) (7 << ((x) * 3))
#define GPIO_FUNC_SELECT_ALT4(x) (3 << ((x) * 3))
#define GPIO_FUNC_SELECT_ALT5(x) (2 << ((x) * 3))
#define GPIO_LEVEL0 0x34
#define GPIO_LEVEL1 0x38
#define GPIO_ENABLE(x) (1 << (x))


// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MEM_FLAG_DIRECT (1 << 2)
#define MEM_FLAG_COHERENT (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)

#define TICK_CNT 100
#define CB_CNT (TICK_CNT * 2)

#define CLK_DIVI 1
#define CLK_MICROS 1


typedef struct CLKCtrlReg
{
    // See https://elinux.org/BCM2835_registers#CM
    uint32_t ctrl;
    uint32_t div;
} CLKCtrlReg;

typedef struct PWMCtrlReg
{
    uint32_t ctrl;     // 0x0, Control
    uint32_t status;   // 0x4, Status
    uint32_t dma_cfg;  // 0x8, DMA configuration
    uint32_t padding1; // 0xC, 4-byte padding
    uint32_t range1;   // 0x10, Channel 1 range
    uint32_t data1;    // 0x14, Channel 1 data
    uint32_t fifo_in;  // 0x18, FIFO input
    uint32_t padding2; // 0x1C, 4-byte padding again
    uint32_t range2;   // 0x20, Channel 2 range
    uint32_t data2;    // 0x24, Channel 2 data
} PWMCtrlReg;

typedef struct GPIOCtrlReg
{
    uint32_t fun_sel0;   // 0x0,  Function select 0-9
    uint32_t fun_sel1;   // 0x4,  Function select 10-19
    uint32_t fun_sel2;   // 0x8,  Function select 20-29
    uint32_t fun_sel3;   // 0xC,  Function select 30-39
    uint32_t fun_sel4;   // 0x10, Function select 40-49
    uint32_t fun_sel5;   // 0x14, Function select 50-53
    uint32_t padding1;   // 0x18, 4-byte padding
    uint32_t out_set0;   // 0x1C, Output set 0-31
    uint32_t out_set1;   // 0x20, Output set 32-53
    uint32_t paddind2;   // 0x24, 4-byte paddin
    uint32_t out_clr0;   // 0x28, Output clear 0-31
    uint32_t out_clr1;   // 0x2C, Output clear 32-53
    uint32_t padding3;   // 0x30, 4-byte paddin
    uint32_t lvl0;       // 0x34, Pin level 0-31
    uint32_t lvl1;       // 0x38, Pin level 32-53
    uint32_t padding4;   // 0x3C, 4-byte paddin
    uint32_t ev_stat0;   // 0x40, Event detect status 0-31
    uint32_t ev_stat1;   // 0x44, Event detect status 32-53
    uint32_t padding5;   // 0x48, 4-byte paddin
    uint32_t rise_en0;   // 0x4C, Enable rising edge detect 0-31
    uint32_t rise_en1;   // 0x50, Enable rising edge detect 32-53
    uint32_t padding6;   // 0x54, 4-byte paddin
    uint32_t fall_en0;   // 0x58, Enable falling edge detect 0-31
    uint32_t fall_en1;   // 0x5C, Enable falling edge detect 32-53
} GPIOCtrlReg;

volatile PWMCtrlReg *pwm_reg;
volatile CLKCtrlReg *clk_reg;
volatile GPIOCtrlReg *gpio_reg;

void *map_peripheral(uint32_t addr, uint32_t size)
{
    int mem_fd;
    // Check mem(4) about /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    {
        perror("Failed to open /dev/mem: ");
        exit(-1);
    }

    uint32_t *result = (uint32_t *)mmap(
        NULL,
        size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        PERI_PHYS_BASE + addr);

    close(mem_fd);

    if (result == MAP_FAILED)
    {
        perror("mmap error: ");
        exit(-1);
    }
    return result;
}

void init_hw_clk()
{
    // See Chanpter 6.3, BCM2835 ARM peripherals for controlling the hardware clock
    // Also check https://elinux.org/BCM2835_registers#CM for the register mapping

    // kill the clock if busy
    printf("1 cm_crtl: %.32b\n", clk_reg->ctrl);
    if (clk_reg->ctrl & CLK_CTL_BUSY)
    {
        do
        {
            clk_reg->ctrl = BCM_PASSWD | CLK_CTL_KILL;
        } while (clk_reg->ctrl & CLK_CTL_BUSY);
    }

    printf("2 cm_crtl: %.32b\n", clk_reg->ctrl);
    // Set clock source to oscillator (19.2 MHz)
    clk_reg->ctrl = BCM_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_OSC);
    usleep(10);
    
    printf("3 cm_crtl: %.32b\n", clk_reg->ctrl);
    // Divide by 6 to get 3.2 MHz
    clk_reg->div = BCM_PASSWD | CLK_DIV_DIVI(3);
    usleep(10);
    
    printf("4 cm_crtl: %.32b\n", clk_reg->ctrl);
    // Enable the clock
    clk_reg->ctrl |= (BCM_PASSWD | CLK_CTL_ENAB);
    usleep(10);
    printf("5 cm_crtl: %.32b\n", clk_reg->ctrl);
}

void init_pwm()
{
    // reset PWM
    pwm_reg->ctrl = 0;
    usleep(100);
    //pwm_reg->status = -1;
    //usleep(500);

    /*
     * set number of bits to transmit
     * e.g, if CLK_MICROS is 5, since we have set the frequency of the
     * hardware clock to 100 MHZ, then the time taken for `100 * CLK_MICROS` bits
     * is (500 / 100) = 5 us, this is how we control the DMA sampling rate
     */
    //pwm_reg->range1 = 100 * CLK_MICROS;
    pwm_reg->range1 = 6;
    usleep(100);
    
    pwm_reg->data1 = 3;
    usleep(100);
    
    pwm_reg->range2 = 32*6;
    usleep(100);
    
    pwm_reg->data2 = 16*6;
    usleep(100);

    // enable PWM DMA, raise panic and dreq thresholds to 15
    //pwm_reg->dma_cfg = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(15);
    //usleep(10);

    // clear PWM fifo
    //pwm_reg->ctrl = PWM_CTL_CLRF1;
    //usleep(500);

    // enable PWM channel 1 and use fifo
    pwm_reg->ctrl = PWM_CTL_PWEN1 | PWM_CTL_MSEN1 | PWM_CTL_PWEN2 | PWM_CTL_MSEN2;
    usleep(100);
    
    //pwm_reg->status = -1;
    
}

void init_gpio()
{
    // PWM on ports 12 and 13
    gpio_reg->fun_sel1 &= GPIO_FUNC_SELECT_CLEAR(8);
    gpio_reg->fun_sel1 &= GPIO_FUNC_SELECT_CLEAR(9);
    gpio_reg->fun_sel1 |= (GPIO_FUNC_SELECT_ALT5(8) | GPIO_FUNC_SELECT_ALT5(9));
    
    // Wypisanie aktualnych funkcji
    printf("fun_sel0 %.32b \nfun_sel1 %.32b \nfun_sel2 %.32b \nfun_sel3 %.32b \nfun_sel4 %.32b \nfun_sel5 %.32b\n\n",
        gpio_reg->fun_sel0, gpio_reg->fun_sel1, gpio_reg->fun_sel2, gpio_reg->fun_sel3, gpio_reg->fun_sel4, gpio_reg->fun_sel5);
}

int main()
{
    int i;
    uint32_t *dane = malloc(sizeof(uint32_t) * 200);
    
	pwm_reg = map_peripheral(PWM_BASE, PAGE_SIZE);

    uint8_t *cm_base_ptr = map_peripheral(CM_BASE, CM_LEN);
    clk_reg = (CLKCtrlReg *)(cm_base_ptr + CM_PWM);

    gpio_reg = map_peripheral(GPIO_BASE, PAGE_SIZE);
    
    init_hw_clk();
    
    init_gpio();
    
    init_pwm();
    
    printf("cm_crtl: %.32b\n", clk_reg->ctrl);
    printf("pwm_crtl: %.32b\n", pwm_reg->ctrl);
    printf("pwm_stat: %.32b\n", pwm_reg->status);
    
    for (i = 0; i < 200; i++)
    {
        dane[i] = gpio_reg->lvl0;
        //dane[i] = pwm_reg->data2;
        //usleep(100);
    }
    
    for (i = 0; i < 200; i++)
    {
        printf("%3d: %.32b\n", i, dane[i]);
    }
    
    free(dane);
}
