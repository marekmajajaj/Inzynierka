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
#include <math.h>

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

#define DMA_BASE 0x00007000
#define DMA_CHANNEL 6
#define DMA_OFFSET 0x100
#define DMA_ADDR (DMA_BASE + DMA_OFFSET * (DMA_CHANNEL >> 2))

/* DMA CS Control and Status bits */
#define DMA_ENABLE (0xFF0 / 4)
#define DMA_CHANNEL_RESET (1 << 31)
#define DMA_CHANNEL_ABORT (1 << 30)
#define DMA_WAIT_ON_WRITES (1 << 28)
#define DMA_PANIC_PRIORITY(x) ((x) << 20)
#define DMA_PRIORITY(x) ((x) << 16)
#define DMA_INTERRUPT_STATUS (1 << 2)
#define DMA_END_FLAG (1 << 1)
#define DMA_ACTIVE (1 << 0)
#define DMA_DISDEBUG (1 << 28)

/* DMA control block "info" field bits */
#define DMA_NO_WIDE_BURSTS (1 << 26)
#define DMA_PERIPHERAL_MAPPING(x) ((x) << 16)
#define DMA_BURST_LENGTH(x) ((x) << 12)
#define DMA_SRC_IGNORE (1 << 11)
#define DMA_SRC_DREQ (1 << 10)
#define DMA_SRC_WIDTH (1 << 9)
#define DMA_SRC_INC (1 << 8)
#define DMA_DEST_IGNORE (1 << 7)
#define DMA_DEST_DREQ (1 << 6)
#define DMA_DEST_WIDTH (1 << 5)
#define DMA_DEST_INC (1 << 4)
#define DMA_WAIT_RESP (1 << 3)
#define DMA_2D_MODE (1 << 1)
#define DMA_IRQ_EN (1 << 0)

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
#define PWM_CTL_USEF2 (1 << 13)
#define PWM_CTL_POLA2 (1 << 12)
#define PWM_CTL_SBIT2 (1 << 11)
#define PWM_CTL_RPTL2 (1 << 10)
#define PWM_CTL_MODE2 (1 << 9)
#define PWM_CTL_PWEN2 (1 << 8)
#define PWM_CTL_MSEN1 (1 << 7)
#define PWM_CTL_CLRF1 (1 << 6)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_POLA1 (1 << 4)
#define PWM_CTL_SBIT1 (1 << 3)
#define PWM_CTL_RPTL1 (1 << 2)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

#define PWM_DMAC_ENAB (1 << 31)
#define PWM_DMAC_PANIC(x) ((x) << 8)
#define PWM_DMAC_DREQ(x) (x)

#define GPIO_BASE 0x00200000
#define GPIO_LEN 0xB4
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

//#define TICK_CNT 100
//#define CB_CNT (TICK_CNT * 2)

#define TICK_PWM 24
#define TICK_DONE 25
#define TICK_DUMMY 26

#define CB_DELAY 18 // 16 or higher(multiple of 2s)
#define CB_START 1 // Used to control 24bit data delay

#define CLK_DIVI 12
#define CLK_SEL CLK_CTL_SRC_OSC
#define CLK_MICROS 1

#define SMPL_RATE (19200000/CLK_DIVI/2/64)
#define SMPL_TO_COLLECT (SMPL_RATE*10)
//#define SMPL_TO_COLLECT 10
#define SMPL_MICS_NUMBER 16

#define RESOLUTION_VERT 11
#define RESOLUTION_HOR 11
#define ANGLE_MAX_VERT M_PI_4   // pi/4
#define ANGLE_MAX_HOR M_PI_4

#define PIN_D1 6
#define PIN_D2 16
#define PIN_D3 1
#define PIN_D4 5
#define PIN_D5 7
#define PIN_D6 0
#define PIN_D7 8
#define PIN_D8 11
#define PIN_D9 25
#define PIN_D10 9
#define PIN_D11 24
#define PIN_D12 10
#define PIN_D13 23
#define PIN_D14 22
#define PIN_D15 18
#define PIN_D16 27
#define PIN_D17 17
#define PIN_D18 4
#define PIN_D19 3
#define PIN_D20 2

#define FIR_DELAY_LENGTH 21

typedef struct DMACtrlReg
{
    uint32_t cs;      // DMA Channel Control and Status register
    uint32_t cb_addr; // DMA Channel Control Block Address
} DMACtrlReg;

typedef struct DMAControlBlock
{
    uint32_t tx_info;    // Transfer information
    uint32_t src;        // Source (bus) address
    uint32_t dest;       // Destination (bus) address
    uint32_t tx_len;     // Transfer length (in bytes)
    uint32_t stride;     // 2D stride
    uint32_t next_cb;    // Next DMAControlBlock (bus) address
    uint32_t padding[2]; // 2-word padding
} DMAControlBlock;

typedef struct DMAMemHandle
{
    void *virtual_addr; // Virutal base address of the page
    uint32_t bus_addr;  // Bus adress of the page, this is not a pointer because it does not point to valid virtual address
    uint32_t mb_handle; // Used by mailbox property interface
    uint32_t size;
} DMAMemHandle;

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
    uint32_t paddind2;   // 0x24, 4-byte padding
    uint32_t out_clr0;   // 0x28, Output clear 0-31
    uint32_t out_clr1;   // 0x2C, Output clear 32-53
    uint32_t padding3;   // 0x30, 4-byte padding
    uint32_t lvl0;       // 0x34, Pin level 0-31
    uint32_t lvl1;       // 0x38, Pin level 32-53
    uint32_t padding4;   // 0x3C, 4-byte padding
    uint32_t ev_stat0;   // 0x40, Event detect status 0-31
    uint32_t ev_stat1;   // 0x44, Event detect status 32-53
    uint32_t padding5;   // 0x48, 4-byte padding
    uint32_t rise_en0;   // 0x4C, Enable rising edge detect 0-31
    uint32_t rise_en1;   // 0x50, Enable rising edge detect 32-53
    uint32_t padding6;   // 0x54, 4-byte padding
    uint32_t fall_en0;   // 0x58, Enable falling edge detect 0-31
    uint32_t fall_en1;   // 0x5C, Enable falling edge detect 32-53
    uint32_t padding7;   // 0x60, 4-byte padding
    uint32_t high_en0;   // 0x64, Enable high detect 0-31
    uint32_t high_en1;   // 0x68, Enable high detect 32-53
    uint32_t padding8;   // 0x6C, 4-byte padding
    uint32_t low_en0;    // 0x70, Enable low detect 0-31
    uint32_t low_en1;    // 0x74, Enable low detect 32-53
    uint32_t padding9;   // 0x78, 4-byte padding
    uint32_t a_rise_en0; // 0x7C, Enable async. rising edge detect 0-31
    uint32_t a_rise_en1; // 0x80, Enable async. rising edge detect 32-53
    uint32_t padding10;  // 0x84, 4-byte padding
    uint32_t a_fall_en0; // 0x88, Enable async. falling edge detect 0-31
    uint32_t a_fall_en1; // 0x8C, Enable async. falling edge detect 32-53
    uint32_t padding11;  // 0x90, 4-byte padding
    uint32_t pull_en;    // 0x94, Pull-up/down enable
    uint32_t pull_encl0; // 0x98, Pull-up/down enable clock
    uint32_t pull_encl1; // 0x9C, Pull-up/down enable clock
} GPIOCtrlReg;

int mailbox_fd = -1;
DMAMemHandle *dma_cbs;
DMAMemHandle *dma_ticks;
//DMAMemHandle *dma_pwm_data;
volatile DMACtrlReg *dma_reg;
volatile PWMCtrlReg *pwm_reg;
volatile CLKCtrlReg *clk_reg;
volatile GPIOCtrlReg *gpio_reg;

DMAMemHandle *dma_malloc(unsigned int size)
{
    if (mailbox_fd < 0)
    {
        mailbox_fd = mbox_open();
        assert(mailbox_fd >= 0);
    }

    // Make `size` a multiple of PAGE_SIZE
    size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

    DMAMemHandle *mem = (DMAMemHandle *)malloc(sizeof(DMAMemHandle));
    // Documentation: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
    mem->mb_handle = mem_alloc(mailbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
    mem->bus_addr = mem_lock(mailbox_fd, mem->mb_handle);
    mem->virtual_addr = mapmem(BUS_TO_PHYS(mem->bus_addr), size);
    mem->size = size;

    assert(mem->bus_addr != 0);

    fprintf(stderr, "MBox alloc: %d bytes, bus: %08X, virt: %08X\n", mem->size, mem->bus_addr, (uint32_t)mem->virtual_addr);

    return mem;
}

void dma_free(DMAMemHandle *mem)
{
    if (mem->virtual_addr == NULL)
        return;

    unmapmem(mem->virtual_addr, mem->size);
    mem_unlock(mailbox_fd, mem->mb_handle);
    mem_free(mailbox_fd, mem->mb_handle);
    mem->virtual_addr = NULL;
}

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

void dma_alloc_buffers()
{
    /* 
     * 16   for filling pwm fifo
     * 64*2 for timing
     * 1    for info about finished 24bit collection
     */
    dma_cbs = dma_malloc((CB_DELAY + 64*2 + 1) * sizeof(DMAControlBlock));
    
    /*
     * 24 for data
     * 1  for PWM data sent to fifo
     * 1  for info about finished 24bit collection
     * 1  for dummy data
     */
    dma_ticks = dma_malloc((24 + 1 + 1 + 1) * sizeof(uint32_t));
}

static inline DMAControlBlock *ith_cb_virt_addr(int i) { return (DMAControlBlock *)dma_cbs->virtual_addr + i; }

static inline uint32_t ith_cb_bus_addr(int i) { return dma_cbs->bus_addr + i * sizeof(DMAControlBlock); }

static inline uint32_t *ith_tick_virt_addr(int i) { return (uint32_t *)dma_ticks->virtual_addr + i; }

static inline uint32_t ith_tick_bus_addr(int i) { return dma_ticks->bus_addr + i * sizeof(uint32_t); }

void dma_init_cbs()
{
    int i;
    DMAControlBlock *cb;
    
    // Fill fifo
    for (i = 0; i < CB_DELAY; i++)
    {
        cb = ith_cb_virt_addr(i);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
        cb->src = ith_tick_bus_addr(TICK_PWM); // PWM data
        cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO;
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr(i + 1);
    }
    
    // Tick block (i=16)
    i=CB_DELAY;
    cb = ith_cb_virt_addr(i);
    cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
    cb->src = PERI_BUS_BASE + GPIO_BASE + GPIO_LEVEL0;
    cb->dest = ith_tick_bus_addr(TICK_DUMMY);
    cb->tx_len = 4;
    cb->next_cb = ith_cb_bus_addr(i + 1);

    // Delay block (i=17)
    i++;
    cb = ith_cb_virt_addr(i);
    cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
    cb->src = ith_tick_bus_addr(TICK_PWM); // PWM data
    cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO;
    cb->tx_len = 4;
    cb->next_cb = ith_cb_bus_addr(i + 1);
    
    for (i = (CB_DELAY+2)/2; i < (24+(CB_DELAY+2)/2); i++) // Blocks 18-65
    {
        // Tick block
        cb = ith_cb_virt_addr(2 * i);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cb->src = PERI_BUS_BASE + GPIO_BASE + GPIO_LEVEL0;
        cb->dest = ith_tick_bus_addr(i-(CB_DELAY+2)/2);
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr(2 * i + 1);

        // Delay block
        cb = ith_cb_virt_addr(2 * i + 1);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
        cb->src = ith_tick_bus_addr(TICK_PWM); // PWM data
        cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO;
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr(2 * i + 2);
    }
    
    // Info block (i=66)
    i = 24*2 + CB_DELAY + 2;
    cb = ith_cb_virt_addr(i);
    cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
    cb->src = ith_tick_bus_addr(TICK_PWM); // PWM data
    cb->dest = ith_tick_bus_addr(TICK_DONE);
    cb->tx_len = 4;
    cb->next_cb = ith_cb_bus_addr(i+1);
    
    for (i = (24*2 + CB_DELAY + 2)/2; i < (63+(CB_DELAY+2)/2); i++) // Blocks 67-144
    {
        // Tick block
        cb = ith_cb_virt_addr(2 * i + 1);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cb->src = PERI_BUS_BASE + GPIO_BASE + GPIO_LEVEL0;
        cb->dest = ith_tick_bus_addr(TICK_DUMMY);
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr(2 * i + 2);

        // Delay block
        cb = ith_cb_virt_addr(2 * i + 2);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
        cb->src = ith_tick_bus_addr(TICK_PWM); // PWM data
        cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO;
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr(2 * i + 3);
    }
    // Go back to block 8
    i = 63 * 2 + CB_DELAY + 2;
    cb = ith_cb_virt_addr(i);
    cb->next_cb = ith_cb_bus_addr(CB_DELAY);
    
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
    usleep(100);
    
    printf("3 cm_crtl: %.32b\n", clk_reg->ctrl);
    // Divide by 3 to get 6.4 MHz
    clk_reg->div = BCM_PASSWD | CLK_DIV_DIVI(CLK_DIVI);
    usleep(100);
    
    printf("4 cm_crtl: %.32b\n", clk_reg->ctrl);
    // Enable the clock
    clk_reg->ctrl |= (BCM_PASSWD | CLK_CTL_ENAB);
    usleep(100);
    printf("5 cm_crtl: %.32b\n", clk_reg->ctrl);
}

void init_pwm()
{
    // reset PWM
    pwm_reg->ctrl = 0;
    usleep(100);
    //pwm_reg->status = -1;
    //usleep(500);

    pwm_reg->range1 = 2;
    usleep(100);
    
    //pwm_reg->data1 = 1;
    //usleep(100);
    
    pwm_reg->range2 = 128;
    usleep(100);
    
    pwm_reg->data2 = 64;
    //pwm_reg->data2 = 4;
    usleep(100);

    // enable PWM DMA
    pwm_reg->dma_cfg = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(15);
    usleep(100);

    // Channel 1 use fifo in serializer mode, channel 2 use data with MS in PWM mode
    pwm_reg->ctrl = PWM_CTL_SBIT1 | PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_POLA2 | PWM_CTL_MSEN2;
    usleep(100);
}

void pwm_clear_fifo()
{
    // clear PWM fifo
    pwm_reg->ctrl |= PWM_CTL_CLRF1;
    usleep(100);
    
    // Single shot operation, no need to clear the bit
}

void pwm_start()
{
    pwm_reg->ctrl |= PWM_CTL_PWEN1 | PWM_CTL_PWEN2;
    usleep(100);
    pwm_reg->status = -1;
    usleep(100);
}

void pwm_end()
{
    pwm_reg->ctrl &= ~(PWM_CTL_PWEN1 | PWM_CTL_PWEN2);
    usleep(100);
}

void init_gpio()
{
    printf("gpio levels: %.32b\n", gpio_reg->lvl0);
    // GPIO reset
    gpio_reg->fun_sel0 = 0; // 0-9
    usleep(100);
    gpio_reg->fun_sel1 = 0; // 10-19
    usleep(100);
    gpio_reg->fun_sel2 &= ~(GPIO_FUNC_SELECT_CLEAR(9) & GPIO_FUNC_SELECT_CLEAR(8)); // 20-27
    usleep(100);
    gpio_reg->rise_en0 &= 0xF0000000; // 0-27
    usleep(100);
    gpio_reg->fall_en0 &= 0xF0000000;
    usleep(100);
    gpio_reg->high_en0 &= 0xF0000000;
    usleep(100);
    gpio_reg->low_en0 &= 0xF0000000;
    usleep(100);
    gpio_reg->a_rise_en0 &= 0xF0000000;
    usleep(100);
    gpio_reg->a_fall_en0 &= 0xF0000000;
    usleep(100);
    
    printf("fun_sel0 %.32b \nfun_sel1 %.32b \nfun_sel2 %.32b \nfun_sel3 %.32b \nfun_sel4 %.32b \nfun_sel5 %.32b\n\n",
        gpio_reg->fun_sel0, gpio_reg->fun_sel1, gpio_reg->fun_sel2, gpio_reg->fun_sel3, gpio_reg->fun_sel4, gpio_reg->fun_sel5);
    
    // PWM on pins 12 and 13
    gpio_reg->fun_sel1 |= GPIO_FUNC_SELECT_ALT0(2) | GPIO_FUNC_SELECT_ALT0(3);
    
    // TX/RX on pin 14 and 15 (they were there originally)
    gpio_reg->fun_sel1 |= GPIO_FUNC_SELECT_ALT5(4) | GPIO_FUNC_SELECT_ALT5(5);
    
    // Falling edge detection on pin 12
    gpio_reg->a_rise_en0 |= 1 << 12;
    
    // Wypisanie aktualnych funkcji
    printf("fun_sel0 %.32b \nfun_sel1 %.32b \nfun_sel2 %.32b \nfun_sel3 %.32b \nfun_sel4 %.32b \nfun_sel5 %.32b\n\n",
        gpio_reg->fun_sel0, gpio_reg->fun_sel1, gpio_reg->fun_sel2, gpio_reg->fun_sel3, gpio_reg->fun_sel4, gpio_reg->fun_sel5);

    printf("gpio levels: %.32b\n", gpio_reg->lvl0);

}

void dma_start()
{
    // Set data for PWM (0b10101010...)
    *(ith_tick_virt_addr(TICK_PWM)) = 0xAAAAAAAA;
    
    // Clear info data
    *(ith_tick_virt_addr(TICK_DONE)) = 0;

    // Reset the DMA channel
    dma_reg->cs = DMA_CHANNEL_ABORT;
    dma_reg->cs = 0;
    dma_reg->cs = DMA_CHANNEL_RESET;
    dma_reg->cb_addr = 0;

    // Make cb_addr point to the first DMA control block and enable DMA transfer
    dma_reg->cb_addr = ith_cb_bus_addr(CB_START);
    dma_reg->cs = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_DISDEBUG;
    dma_reg->cs |= DMA_WAIT_ON_WRITES | DMA_ACTIVE | DMA_INTERRUPT_STATUS | DMA_END_FLAG;
}

void dma_end()
{
    // Shutdown DMA channel, otherwise it won't stop after program exits
    dma_reg->cs |= DMA_CHANNEL_ABORT;
    usleep(100);
    dma_reg->cs &= ~DMA_ACTIVE;
    dma_reg->cs |= DMA_CHANNEL_RESET;
    usleep(100);

    // Release the memory used by DMA, otherwise the memory will be leaked after program exits
    dma_free(dma_ticks);
    dma_free(dma_cbs);

    free(dma_ticks);
    free(dma_cbs);
}

double calculateDelay(double micX, double micY, double freq, double wave_speed, double angle_theta, double angle_phi)
{
    // Wspolrzedne sferyczne do wektora, r = 1
    // Traktujemy micY jako micZ i micX jako -micY
    double normX = -cos(angle_theta)*sin(angle_phi);
    double normY = sin(angle_theta);

    // Iloczyn skalarny wektora normalnego i ujemnych wzpolrzednych mikrofonu
    double dist = (normX * -micX) + (normY * -micY);

    // Obliczenie opoznienia
    return dist/wave_speed*freq;
}

void filter_iir(int32_t* x, int32_t* y, int lenX)
{
    int i;
    const double a2 = -0.353327144716459,
        b1 = 0.676663572358229,
        b2 = -0.676663572358229;
    double x1 = 0.0, y1 = 0.0;
    double xtmp, ytmp;

    for(i = 0; i < lenX; i++)
    {
        xtmp = (double)x[i];
        ytmp = b1 * xtmp + b2 * x1 - a2 * y1;

        x1 = xtmp;
        y1 = ytmp;

        y[i] = (int32_t)ytmp;
    }
}

inline double sinc(double x)
{
    if(x == 0)
    	return 1.0;

    return sin(M_PI*x)/M_PI/x;
}

inline double fraction_part(double x)
{
    return x-floor(x);
}

inline double blackman_window(double x, int n)
{
    return 0.42 - 0.5*cos(2*M_PI*x/(n-1)) + 0.08*cos(4*M_PI*x/(n-1));
}

void calculate_delay_fir(double* h, int n, double delay)
{
    int i;
    uint32_t m = (n-1)/2;
    for(i = 0; i < n; i++)
    {
        h[i] = sinc(i - fraction_part(delay) - m) * blackman_window(i, n);
    }
}

void apply_delay(int32_t* x, int32_t* y, double delay, int lenX, int lenH)
{
    int i, j;
    double *h = malloc(sizeof(double) * lenH);
    double tmp;
    int delay_int = (int)delay;
    // Opoznienie, czesc calkowita
    for(i = 0; i < delay_int; i++)
    {
        y[i] = 0;
    }

    // Opoznienie, czesc ulamkowa
    calculate_delay_fir(h, lenH, delay);
    for(i = 0; i < lenX - delay_int; i++)
    {
    	tmp = 0.0;
        for(j = 0; j < lenH; j++)
        {
            if(i - j < 0)
                continue;
            tmp += (double)x[i-j] * h[j];
        }
        y[i+delay_int] = (int32_t)tmp;
    }

    free(h);
}

double rms(int32_t* x, uint32_t lenX)
{
    int i;
    double res = 0.0;
    for(i = 0; i < lenX; i++)
    {
        res += (double)x[i]*(double)x[i]/lenX;
    }
    return sqrt(res);
}

int main()
{
    // ---------------------------------- TWORZENIE ZMIENNYCH ----------------------------------

    uint32_t i, j;
    volatile uint32_t k;
    uint32_t i_start;
    volatile uint32_t *done;
    uint32_t *ticks = malloc(sizeof(uint32_t) * SMPL_TO_COLLECT * 24);
    int32_t **mic_data = malloc(sizeof(int32_t *) * SMPL_MICS_NUMBER);
    int32_t **mic_mod = malloc(sizeof(int32_t *) * SMPL_MICS_NUMBER);
    int32_t **mic_tmp;
    uint32_t tmp = 0;
    uint8_t data_pins[] = {PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7, PIN_D8, PIN_D9, PIN_D10, PIN_D11, PIN_D12, PIN_D13, PIN_D14, PIN_D15, PIN_D16, PIN_D17, PIN_D18, PIN_D19, PIN_D20};

    double mic_posX[20];
    double mic_posY[20];
    double *beam_angle_theta = malloc(sizeof(double) * RESOLUTION_VERT);
    double *beam_angle_phi = malloc(sizeof(double) * RESOLUTION_HOR);
    double angle_step;
    double ***mic_delay = malloc(sizeof(double **) * SMPL_MICS_NUMBER);
    double delay_min = 999999.0;
    
    FILE *file_out, *file_out_filt, *file_pos;

    if(ticks == 0 || mic_data == 0 || mic_mod == 0 || beam_angle_theta == 0 || beam_angle_phi == 0 || mic_delay == 0)
    {
        printf("Memory allocation error");
        goto code_error_end;
    }

    for(i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        mic_data[i] = malloc(sizeof(int32_t) * SMPL_TO_COLLECT);
        mic_mod[i] = malloc(sizeof(int32_t) * SMPL_TO_COLLECT);
        mic_delay[i] = malloc(sizeof(double *) * RESOLUTION_HOR);
        if(mic_data[i] == 0 || mic_mod[i] == 0 || mic_delay[i] == 0)
        {
            printf("Memory allocation error");
            goto code_error_end;
        }
        for(j = 0; j < RESOLUTION_HOR; j++)
        {
            mic_delay[i][j] = malloc(sizeof(double) * RESOLUTION_VERT);
            if(mic_delay[i][j] == 0)
            {
                printf("Memory allocation error");
                goto code_error_end;
            }
        }
    }
    
    // ---------------------------------- OTWIERANIE PLIKOW ----------------------------------
    
    file_out = fopen("output.raw", "wb");
    file_out_filt = fopen("output_filt.raw", "wb");
    
    if(file_out == 0 || file_out_filt == 0)
    {
        printf("Opening output file error");
        goto code_error_end;
    }

    // Otwarcie pliku z pozycjami mikrofonow
    if(SMPL_MICS_NUMBER == 16)
    {
        file_pos = fopen("mic_pos16.txt", "r");
    }
    else
    {
        file_pos = fopen("mic_pos20.txt", "r");
    }
    
    if(file_pos == 0)
    {
        printf("Opening position file error");
        goto code_error_end;
    }

    // Pobranie danych z pliku
    for (i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        fscanf(file_pos, "%lf %lf", &(mic_posX[i]), &(mic_posY[i]));
        //printf("%lf %lf\n", mic_posX[i], mic_posY[i]);
    }

    fclose(file_pos);

    // ---------------------------------- PRZYGOTOWANIE DANYCH ----------------------------------

    // Dlugosc azymutalna (phi)
    // Zaczecie od srodka
    if(RESOLUTION_HOR % 2 == 1)
    {
    	i_start = (RESOLUTION_HOR - 1) / 2;
    }
    else
    {
        i_start = RESOLUTION_HOR / 2;
    }
    
    // Obliczenie katow (dla prawej polowy)
    for(i = i_start; i < RESOLUTION_HOR-1; i++)
    {
        if(RESOLUTION_HOR % 2 == 1)
        {
            beam_angle_phi[i] = (double)(i - i_start) * ANGLE_MAX_HOR / (RESOLUTION_HOR - 1.0d) * 2.0d;
        }
        else
        {
            beam_angle_phi[i] = (double)((i - i_start) * 2 + 1) * ANGLE_MAX_HOR / (RESOLUTION_HOR - 2.0d);
        }
    }
    beam_angle_phi[RESOLUTION_HOR - 1] = ANGLE_MAX_HOR;

    // Kopiowanie katow do lewej polowy
    for(i = 0; i < i_start; i++)
    {
        beam_angle_phi[i] = -beam_angle_phi[RESOLUTION_HOR - 1 - i];
        printf("phi%d: %lf\n",i,beam_angle_phi[i]);
    }


    // Odleglosc zenitalna (theta)
    // Zaczecie od srodka
    if(RESOLUTION_VERT % 2 == 1)
    {
    	i_start = (RESOLUTION_VERT - 1) / 2;
    }
    else
    {
        i_start = RESOLUTION_VERT / 2;
    }

    // Obliczenie katow (dla prawej polowy)
    for(i = i_start; i < RESOLUTION_VERT-1; i++)
    {
        if(RESOLUTION_VERT % 2 == 1)
        {
            beam_angle_theta[i] = (double)(i - i_start) * ANGLE_MAX_VERT / (RESOLUTION_VERT - 1.0d) * 2.0d + M_PI_2;
        }
        else
        {
            beam_angle_theta[i] = (double)((i - i_start) * 2 + 1) * ANGLE_MAX_VERT / (RESOLUTION_VERT - 2.0d) + M_PI_2;
        }
    }
    beam_angle_theta[RESOLUTION_VERT - 1] = ANGLE_MAX_VERT + M_PI_2;

    // Kopiowanie katow do lewej polowy
    for(i = 0; i < i_start; i++)
    {
        beam_angle_theta[i] = -beam_angle_theta[RESOLUTION_VERT - 1 - i];
        printf("theta%d: %lf\n",i,beam_angle_theta[i]);

    }


    // Obliczenie opoznien
    for(i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        for(j = 0; j < RESOLUTION_HOR; j++)
        {
            for(k = 0; k < RESOLUTION_VERT; k++)
            {
                mic_delay[i][j][k] = calculateDelay(mic_posX[i], mic_posY[i], SMPL_RATE, 340.0d, beam_angle_theta[j], beam_angle_phi[k]);
                if(mic_delay[i][j][k] < delay_min)
                    delay_min = mic_delay[i][j][k];
            }
        }
    }

    // ---------------------------------- REJESTRY ----------------------------------
    
    uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
    dma_reg = (DMACtrlReg *)(dma_base_ptr + DMA_CHANNEL * 0x100);
    
    //uint32_t *dane = malloc(sizeof(uint32_t) * 200);
    pwm_reg = map_peripheral(PWM_BASE, PAGE_SIZE);

    uint8_t *cm_base_ptr = map_peripheral(CM_BASE, CM_LEN);
    clk_reg = (CLKCtrlReg *)(cm_base_ptr + CM_PWM);

    gpio_reg = map_peripheral(GPIO_BASE, PAGE_SIZE);
    
    dma_alloc_buffers();
    printf("DMA buffers allocated\n");
    usleep(100);

    dma_init_cbs();
    printf("DMA control blocks initiated\n");
    usleep(100);

    init_hw_clk();
    printf("Clock initiated\n");
    usleep(100);
    
    init_gpio();
    printf("GPIO initiated\n");
    usleep(100);
    
    init_pwm();
    printf("PWM initiated\n");
    usleep(100);

    // ---------------------------------- ROZPOCZECIE AKWIZYCJI ----------------------------------
    
    pwm_clear_fifo();
    printf("PWM FIFO cleared\n");
    usleep(100);
    
    dma_start();
    printf("DMA started\n");
    printf("tick pwm: %.8X\n", *(ith_tick_virt_addr(TICK_PWM)));

    pwm_start();
    printf("PWM started\n");
    
/*
    i = 0;
    gpio_reg->ev_stat0 = 1 << 12;
    while (i < 200)
    {
        if ((gpio_reg->ev_stat0 & (1 << 12)) == 0)
            continue;
        dane[i] = gpio_reg->lvl0;
        gpio_reg->ev_stat0 = 1 << 12;
        i++;
    }
    
    
    for (i = 0; i < 200; i++)
    {
        printf("%3d: %.32b\n", i, dane[i]);
    }
    
    printf("%d\n", k);

*/
    done = ith_tick_virt_addr(TICK_DONE);

    k = 0;
    for (i = 0; i < SMPL_TO_COLLECT; i++)
    {
        while (*done == 0)
        {
            //printf("%.8X\n", *(ith_tick_virt_addr(TICK_DONE)));
            //usleep(1);
            ;
        }
        // Wait at least 2^18 clock cycles (this loop executes every 64 cycles)
        if(k < 10000)
        {
            k++;
            *(ith_tick_virt_addr(TICK_DONE)) = 0;
            i--;
            continue;
        }
        
        //memcpy(&(ticks[i*24]), ith_tick_virt_addr(i*24), 24 * sizeof(uint32_t));
        for(j = 0; j < 24; j++)
        {
            ticks[i*24 + j] = *(ith_tick_virt_addr(j));
        }
        *(ith_tick_virt_addr(TICK_DONE)) = 0;
    }

    printf("Aquisition ended\n");

    // ---------------------------------- PROCESOWANIE DANYCH ----------------------------------
    
    pwm_end();
    printf("PWM stopped\n");

    dma_end();
    printf("DMA stopped\n");

    for (i = 0; i < SMPL_TO_COLLECT; i++)
    {
        //printf("DMA %10d: %.32b\n", i, ticks[i]);
        for (j = 0; j < SMPL_MICS_NUMBER; j++)
        {
            mic_data[j][i] = 0;
            for (k = 0; k < 24; k++)
            {
                // Get pin value
                tmp = (ticks[i*24 + k] & (1 << data_pins[j])) ? 1 : 0;
                // Insert into correct bit
                mic_data[j][i] |= tmp << (23 - k);

                /*
                if (k < 8)
                    mic_data[j][i*3] |= tmp << (7 - k);
                else if (k < 16 && k >= 8)
                    mic_data[j][i*3 + 1] |= tmp << (15 - k);
                else
                    mic_data[j][i*3 + 2] |= tmp << (23 - k);
                */
            }
            // Sign extension
            tmp = 0b1 << 23;
            mic_data[j][i] = (int32_t)(((uint32_t)mic_data[j][i] ^ tmp) - tmp);
            //mic_data[j][i] = (uint32_t)((int32_t)mic_data[j][i] * 256);
        }
    }

    printf("Initial data preparation finished\n");

    /*
    for(i = 0; i < SMPL_TO_COLLECT; i++)
    {
        printf("%4d: %d\n", i, mic_data[2][i]);
    }
    */
    
    /*
    for(i = 0; i < SMPL_TO_COLLECT * 24; i++)
    {
        printf("%5d: %.32b\n", i, ticks[i]);
    }
    */
    
    for(i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        fwrite(mic_data[i], sizeof(int32_t), SMPL_TO_COLLECT, file_out);
    }

    printf("Written data to file output.raw\n");


    // --------------------------------- WSTĘPNY FILTR IIR ---------------------------------
    

    for(i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        filter_iir(&(mic_data[i][0]), &(mic_mod[i][0]), SMPL_TO_COLLECT);
    }
    mic_tmp = mic_data;
    mic_data = mic_mod;
    mic_mod = mic_tmp;
    mic_tmp = 0;

    
    for(i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        fwrite(mic_data[i], sizeof(int32_t), SMPL_TO_COLLECT, file_out_filt);
    }

    /*
    printf("dma stat: %.32b\n", dma_reg->cs);

    printf("cb delay: %d\n", CB_DELAY);
    */

    // ---------------------------------- KONIEC PROGRAMU ----------------------------------

    free(ticks);
    for(i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        free(mic_data[i]);
        free(mic_mod[i]);
        for(j = 0; j < RESOLUTION_HOR; j++)
        {
            free(mic_delay[i][j]);
        }
        free(mic_delay[i]);
    }
    free(mic_data);
    free(mic_mod);
    free(beam_angle_theta);
    free(beam_angle_phi);
    if(file_out != 0)
    	fclose(file_out);
    if(file_out_filt != 0)
    	fclose(file_out_filt);
    return 0;

    // ----------------------------- KONIEC PROGRAMU: ERROR EDITION -----------------------------

code_error_end:
    if(file_out != 0)
        fclose(file_out);
    if(file_out_filt != 0)
    	fclose(file_out_filt);
    if(file_pos != 0)
        fclose(file_pos);
    for(i = 0; i < SMPL_MICS_NUMBER; i++)
    {
        free(mic_data[i]);
        free(mic_mod[i]);
        for(j = 0; j < RESOLUTION_HOR; j++)
        {
            free(mic_delay[i][j]);
        }
        free(mic_delay[i]);
    }
    free(mic_data);
    free(mic_mod);
    free(beam_angle_theta);
    free(beam_angle_phi);
    free(ticks);
    return -1;
}
