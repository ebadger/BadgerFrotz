
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"

#include <math.h>
#include <vector>
#include "font.h"
#include <hardware/flash.h>
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/types.h"
#include "pico/stdio.h"
#include "scancodes.h"
#include <string.h>
#include <cstdio>
#include "art.h"
#include "binaries.h"

void badgerfrotz_main(const char *game);
void startpios();
void pausepios();
void configdma();
void startdma();
void pausedma();


bool _dmarunning = false;

#define XRES  640
#define YRES  480
#define BPP   4
#define PBB   (8/BPP)

#define FONT_SIZE_X 8
#define FONT_SIZE_Y 16

// VGA timing constants
#define H_ACTIVE   655    //327    // (active + frontporch - 1) - one cycle delay for mov
#define V_ACTIVE   479    // (active - 1)
#define RGB_ACTIVE 319    // (horizontal active)/2 - 1

#define SCAN_BUFFER_WIDTH   (XRES/2)
#define TERMINAL_BUFFER  80 * 31

// Length of the pixel array, and number of DMA transfers
//#define TXCOUNT (XRES*YRES) / (8 / BPP) // Total pixels/2 (since we have 2 pixels per byte)


uint8_t scanline_buffer[2][SCAN_BUFFER_WIDTH + 4];
uint16_t _scanline = 0;

uint8_t _keystate[256] = {0};
uint32_t _terminalOut[TERMINAL_BUFFER] = {0};

uint8_t _xpos = 0;
uint8_t _ypos = 0;
bool    _cursorBlinkState = false;

bool    _beginFlash = false;
bool    _flashReady = false;
bool    _flashDone = false;
uint32_t _flashOffset = 0;

//uint8_t vga_buffer[TXCOUNT];
//uint8_t * address_pointer = (uint8_t *)vga_buffer;
//uint8_t * address_pointer2 = (uint8_t *)vga_buffer;
uint8_t * scanline_pointer = (uint8_t *)scanline_buffer[0];
uint32_t * scanline_pointer32 = (uint32_t *)scanline_buffer[0];

// Manually select a few state machines from pio instance pio0.
uint hsync_sm = 0;
uint vsync_sm = 1;
uint rgb_sm = 2;

// DMA channels - 0 sends color data, 1 reconfigures and restarts 0
int rgb_chan_0 = 0;
int rgb_chan_1 = 1;

uint hsync_offset;
uint vsync_offset;
uint rgb_offset;
uint tones_offset;
uint64_t spinlock = 0;

int _bits = 0;
int _cycles = 0;
bool _finit = false;


#define GPIO_KB_DATA    8
#define GPIO_KB_CLOCK   9

#define HSYNC           16
#define VSYNC           17
#define RED_PIN         18
#define GREEN_PIN       19
#define BLUE_PIN        20

uint32_t _line = 0;
uint32_t _frame = 0;

queue_t  _queueKeyBits;
queue_t  _queueTerminal;

mutex_t _mutexLock;

uint64_t _lastPS2Tick = 0;
bool     _clearPS2State = false;
bool     _keydown = true;
uint8_t  _scancode = 0;

static const char sc2ascii[] = {
/*                                                                             
       0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F          
*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  '`', 0x00, /* 0     
*/ 0x00, 0x00, 0x00, 0x00, 0x00,  'q',  '1', 0x00, 0x00, 0x00,  'z',  's',  'a',  'w',  '2', 0x00, /* 1     
*/ 0x00,  'c',  'x',  'd',  'e',  '4',  '3', 0x00, 0x00,  ' ',  'v',  'f',  't',  'r',  '5', 0x00, /* 2     
*/ 0x00,  'n',  'b',  'h',  'g',  'y',  '6', 0x00, 0x00, 0x00,  'm',  'j',  'u',  '7',  '8', 0x00, /* 3     
*/ 0x00,  ',',  'k',  'i',  'o',  '0',  '9', 0x00, 0x00,  '.',  '/',  'l',  ';',  'p',  '-', 0x00, /* 4     
*/ 0x00, 0x00, '\'', 0x00,  '[',  '=', 0x00, 0x00, 0x00, 0x00, 0x0D,  ']', 0x00, '\\', 0x00, 0x00, /* 5     
*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 6     
*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* 7
*/ };

static const char sc2ascii_shifted[] = { /*
       0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
  */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  '~', 0x00, /* 0
  */ 0x00, 0x00, 0x00, 0x00, 0x00,  'Q',  '!', 0x00, 0x00, 0x00,  'Z',  'S',  'A',  'W',  '@', 0x00, /* 1
  */ 0x00,  'C',  'X',  'D',  'E',  '$',  '#', 0x00, 0x00,  ' ',  'V',  'F',  'T',  'R',  '%', 0x00, /* 2
  */ 0x00,  'N',  'B',  'H',  'G',  'Y',  '^', 0x00, 0x00, 0x00,  'M',  'J',  'U',  '&',  '*', 0x00, /* 3
  */ 0x00,  '<',  'K',  'I',  'O',  ')',  '(', 0x00, 0x00,  '>',  '?',  'L',  ':',  'P',  '_', 0x00, /* 4
  */ 0x00, 0x00, 0x22, 0x00,  '{',  '+', 0x00, 0x00, 0x00, 0x00, 0x0D,  '}', 0x00,  '|', 0x00, 0x00, /* 5
  */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 6
  */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* 7
*/ };

static const char sc2ascii_ctrl[] = { /*
       0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
  */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  '~', 0x00, /* 0
  */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,  '!', 0x00, 0x00, 0x00, 0x1A, 0x13, 0x01, 0x17,  '@', 0x00, /* 1
  */ 0x00, 0x03, 0x18, 0x04, 0x05,  '$',  '#', 0x00, 0x00,  ' ', 0x16, 0x06, 0x14, 0x12,  '%', 0x00, /* 2
  */ 0x00, 0x0E, 0x02, 0x08, 0x07, 0x19,  '^', 0x00, 0x00, 0x00, 0x0D, 0x0A, 0x15,  '&',  '*', 0x00, /* 3
  */ 0x00,  '<', 0x0B, 0x09, 0x0F,  ')',  '(', 0x00, 0x00,  '>',  '?', 0x0C,  ':', 0x10,  '_', 0x00, /* 4
  */ 0x00, 0x00, 0x22, 0x00,  '{',  '+', 0x00, 0x00, 0x00, 0x00, 0x0D,  '}', 0x00,  '|', 0x00, 0x00, /* 5
  */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 6
  */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* 7
*/ };


void os_reboot()
{
    watchdog_enable(1, false);
    while(1);
}


static uint64_t _avgtime;
static uint32_t _samples;
static uint64_t _starttime;
static uint64_t _endtime;

void drawScanline(uint16_t line, uint8_t b)
{
    uint16_t y =  (line / FONT_SIZE_Y);
    uint16_t row = y * 80;
    uint8_t fontrow = line % FONT_SIZE_Y;
    uint32_t bits = 0;
    uint32_t vaddr = 0;

    uint8_t xcaret = _xpos;
    uint8_t ycaret = _ypos;

    if (xcaret >= 80)
    {
        xcaret=0;
        ycaret++;
    }
    
#ifdef PROFILE    
    _starttime = time_us_64();
#endif

    for(int x = 0; x < 80; x++)
    {
        uint16_t addr = x + row;
        uint32_t c = _terminalOut[addr];
        bits = _font8x16_precalc[c & 0xFF][fontrow];

        if (c & 0x100)
        {
            bits = ~bits;
        }

        if (_cursorBlinkState && xcaret == x && ycaret == y)
        {
            bits = ~bits;
        }

        memcpy(&scanline_buffer[b][x<<2], &bits, 4);
    }

#ifdef PROFILE
    _endtime = time_us_64();
    _avgtime += _endtime - _starttime;
    _samples++;
    
    if(_samples % 1000 == 0)
    {
        _avgtime /= _samples;
        printf("avgtime in us: %lld\r\n", _avgtime);
        _samples = 0;
        _avgtime = 0;
    }
#endif
}

void hblank()
{
#ifdef PROFILE_HBLANK
    //pio_interrupt_clear(pio0, PIO0_IRQ_0);
    _endtime = time_us_64();
    _avgtime += _endtime - _starttime;
    _samples++;
    
    if(_samples % 1000 == 0)
    {
        _avgtime /= _samples;
        printf("avgtime in us: %lld\r\n", _avgtime);
        _samples = 0;
        _avgtime = 0;
    }    
    _starttime = time_us_64();
#endif 

    drawScanline(_line, _line % 2);
    scanline_pointer = scanline_buffer[_line % 2]; 
    _line++;

    if(_line == 480)
    {
        _line = 0;
    }    
}

void vblank()
{
    #if PROFILE_VBLANK
        //pio_interrupt_clear(pio0, PIO0_IRQ_0);
        _endtime = time_us_64();
        _avgtime += _endtime - _starttime;
        _samples++;
        
        if(_samples % 60 == 0)
        {
            _avgtime /= _samples;
            printf("avgtime in us: %lld\r\n", _avgtime);
            _samples = 0;
            _avgtime = 0;
        }    
        _starttime = time_us_64();
    #endif 
    _frame++;
    _line = 0;

    // 16ms per frame
    // 250ms cursor blnk frame means 16 frames
    if (_frame % 32 ==0)
    {
        _cursorBlinkState = !_cursorBlinkState;
    }

}

void setCaretPos(uint8_t xpos, uint8_t ypos)
{
   _xpos = xpos;
   _ypos = ypos;
}

void advancechar()
{
    _xpos++;

    if (_xpos >= 80)
    {
        _ypos++;
        _xpos = 0;
    }
}

bool handlespecialchars(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        _ypos++;
        _xpos = 0;
    }
    else if (byte == 8)
    {
        if (_xpos > 0)
        {
            _terminalOut[_xpos + (_ypos * 80)] = ' ';
            //drawTerminalChar(' ', _xpos, _ypos, false);
            _xpos--;
        }
    }
    else
    {
        return false;
    }

    return true;
}

void checkscroll()
{
    if (_ypos >= 30)
    {
        memcpy(_terminalOut, &_terminalOut[80], 2320 * sizeof(uint32_t));
        memset(&_terminalOut[2319], 0, 80 * sizeof(uint32_t));
        _ypos = 29;
        //drawTerminal();
    }
}

uint16_t os_GetCell(uint8_t xpos, uint8_t ypos)
{
     return _terminalOut[xpos + (ypos * 80)] & 0xFFFF;
}

void os_SetCell(uint8_t xpos, uint8_t ypos, uint16_t cell)
{
     _terminalOut[xpos + (ypos * 80)] = cell;
}

void os_reset_screen(void)
{
    memset(_terminalOut, 32, TERMINAL_BUFFER * sizeof(uint32_t));
}

void PrintAt(uint8_t xpos, uint8_t ypos, const char *string, bool inverse)
{
    uint8_t _xtemp = _xpos;
    uint8_t _ytemp = _ypos;

    _xpos = xpos;
    _ypos = ypos;

    for(int i = 0 ; i < strlen(string); i++)
    {
        uint16_t c = string[i];

        if (c == '\r')
        {
            _ypos++;
            continue;
        }

        if (c == '\n')
        {
            _xpos = 0;
            continue;
        }

        if (inverse)
        {
            c |= 0x100;
        }

        _terminalOut[_xpos + (_ypos * 80)] = c;
        //drawTerminalChar(string[i], _xpos, _ypos, c & 0x100);
        advancechar();
        checkscroll();
    }

    //_xpos = _xtemp;
    //_ypos = _ytemp;
}

void PrintString(const char *string, bool inverse)
{
    PrintAt(_xpos, _ypos, string, inverse);
}

void PrintChar(uint8_t byte, bool inverse)
{
    bool printable = false;
    uint32_t c;

    printf ("%c", byte);
    if(!handlespecialchars(byte))
    {
        advancechar();
        printable = true;
    }

    checkscroll();

    if(printable)
    {
        c = byte;
        if (inverse)
        {
            c |= 0x100;
        }

        _terminalOut[_xpos + (_ypos * 80)] = c;
        //drawTerminalChar(byte, _xpos, _ypos, inverse);
    }
}

void __not_in_flash_func(gpio_callback)(uint gpio, uint32_t events) 
{
    if (gpio == VSYNC)
    {
        vblank();
    }

    if (gpio == GPIO_KB_CLOCK)
    {
        if ((time_us_64() - _lastPS2Tick) > 150000) // greater than 150ms?
        {
            // clear ps2 keyboard state
            _clearPS2State = true;
        }

        _lastPS2Tick = time_us_64();

        bool bit = gpio_get(GPIO_KB_DATA);
        
        queue_add_blocking(&_queueKeyBits, &bit );
    }
}

void __not_in_flash_func(core1_loop)() 
{
    while(true)
    {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT && _finit)
        {
            queue_add_blocking(&_queueTerminal, &c);
            //PrintChar(c, false);
        }

        if (_beginFlash)
        {
            uint32_t ints = save_and_disable_interrupts();
            pausepios();
            pausedma();
            _flashDone = false;
            _flashReady = true;
            while (_flashDone == false)
            {
                sleep_ms(1);
            }
            _beginFlash = false;
            
            startpios();
            configdma();
            startdma();
            restore_interrupts(ints);
        }
    }
}

void core1()
{
    multicore_lockout_victim_init();
    queue_init(&_queueKeyBits, 1, 1024);

    gpio_set_dir(GPIO_KB_CLOCK, false);
    gpio_set_dir(GPIO_KB_DATA, false);
    gpio_set_input_enabled(GPIO_KB_CLOCK, true);
    gpio_set_input_enabled(GPIO_KB_DATA, true);
    gpio_set_input_hysteresis_enabled(GPIO_KB_CLOCK, true);
    gpio_set_input_hysteresis_enabled(GPIO_KB_DATA, true);
    gpio_pull_up(GPIO_KB_CLOCK);
    gpio_pull_up(GPIO_KB_DATA);
 
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_init(GPIO_KB_CLOCK);
    gpio_init(GPIO_KB_DATA);

    gpio_set_dir(GPIO_KB_CLOCK, false);
    gpio_set_dir(GPIO_KB_DATA, false);

    gpio_set_irq_enabled_with_callback(GPIO_KB_CLOCK, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(VSYNC, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    configdma();
    startpios();
    startdma();

    hblank();

    //pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, hblank);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio0_hw->inte0 |= PIO_IRQ0_INTE_SM1_BITS;

    core1_loop();
}

void pausedma()
{
    _dmarunning = false;
    hw_clear_bits(&dma_hw->ch[rgb_chan_0].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[rgb_chan_1].al1_ctrl, DMA_CH1_CTRL_TRIG_EN_BITS);
    dma_channel_abort(rgb_chan_0);
    dma_channel_abort(rgb_chan_1);
}

void startdma()
{
   // _dmarunning = true;
   // hw_set_bits(&dma_hw->ch[rgb_chan_0].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
   // hw_set_bits(&dma_hw->ch[rgb_chan_1].al1_ctrl, DMA_CH1_CTRL_TRIG_EN_BITS);
 
    _dmarunning = true;
    dma_start_channel_mask((1u << rgb_chan_0)) ;

}

void pausepios()
{
    pio_set_sm_mask_enabled(pio0,  ((1u << hsync_sm) | (1u << vsync_sm) | (1u << rgb_sm)), false);
    pio_remove_program(pio0, &hsync_program, hsync_offset );
    pio_remove_program(pio0, &vsync_program, vsync_offset );
    pio_remove_program(pio0, &rgb_program, rgb_offset );
}

void configdma()
{
    // Channel Zero (sends color data to PIO VGA machine)
    dma_channel_config c0 = dma_channel_get_default_config(rgb_chan_0);  // default configs
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_8);              // 8-bit txfers
    channel_config_set_read_increment(&c0, true);                        // yes read incrementing
    channel_config_set_write_increment(&c0, false);                      // no write incrementing
    channel_config_set_dreq(&c0, DREQ_PIO0_TX2) ;                        // DREQ_PIO0_TX2 pacing (FIFO)
    channel_config_set_chain_to(&c0, rgb_chan_1);                        // chain to other channel

  
    dma_channel_configure(
        rgb_chan_0,                 // Channel to be configured
        &c0,                        // The configuration we just created
        &pio0->txf[rgb_sm],         // write address (RGB PIO TX FIFO)
        &scanline_pointer,          // The initial read address (pixel color array)
        SCAN_BUFFER_WIDTH,          // Number of transfers; in this case each is 1 byte.
        false                       // Don't start immediately.
    );

    // Channel One (reconfigures the first channel)
    dma_channel_config c1 = dma_channel_get_default_config(rgb_chan_1);   // default configs
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);              // 32-bit txfers
    channel_config_set_read_increment(&c1, false);                        // no read incrementing
    channel_config_set_write_increment(&c1, false);                       // no write incrementing
    channel_config_set_chain_to(&c1, rgb_chan_0);                         // chain to other channel


    dma_channel_configure(
        rgb_chan_1,                         // Channel to be configured
        &c1,                                // The configuration we just created
        &dma_hw->ch[rgb_chan_0].read_addr,  // Write address (channel 0 read address)
        &scanline_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers, in this case each is 4 byte
        false                               // Don't start immediately.
    );
}
void startpios()
{
    hsync_offset = pio_add_program(pio0, &hsync_program);
    vsync_offset = pio_add_program(pio0, &vsync_program);
    rgb_offset = pio_add_program(pio0, &rgb_program);

    hsync_program_init(pio0, hsync_sm, hsync_offset, HSYNC);
    vsync_program_init(pio0, vsync_sm, vsync_offset, VSYNC);
    rgb_program_init(pio0, rgb_sm, rgb_offset, RED_PIN);


// Initialize PIO state machine counters. This passes the information to the state machines
// that they retrieve in the first 'pull' instructions, before the .wrap_target directive
// in the assembly. Each uses these values to initialize some counting registers.
    pio_sm_put_blocking(pio0, hsync_sm, H_ACTIVE);
    pio_sm_put_blocking(pio0, vsync_sm, V_ACTIVE);
    pio_sm_put_blocking(pio0, rgb_sm, RGB_ACTIVE);

    pio_enable_sm_mask_in_sync(pio0, ((1u << hsync_sm) | (1u << vsync_sm) | (1u << rgb_sm)));
}

void __not_in_flash_func(os_tick) (void)
{    
        bool b;
        while (queue_try_remove(&_queueKeyBits, &b))
        {
            if (_clearPS2State)
            {
                _bits = 0;
                _scancode = 0;
                _clearPS2State = false;
            }

            if (_bits > 0 && _bits <= 8)
            {
                _scancode |= b << (_bits - 1);
            }

            if (++_bits % 11 == 0)
            {
                if (_scancode == 0xF0)
                {
                    _keydown = false;
                }
                else if (_scancode == 0xE0)
                {
                    // extended key
                }
                else
                {
                    if (_keydown)
                    {
                        uint8_t key = 0;
                        if (_keystate[SC_LEFTCTRL] || _keystate[SC_RIGHTCTRL])
                        {
                            key = sc2ascii_ctrl[_scancode & 0x7F];
                        }
                        else if (_keystate[SC_LEFTSHIFT] || _keystate[SC_RIGHTSHIFT])
                        {
                            key = sc2ascii_shifted[_scancode & 0x7F];
                        }
                        else
                        {
                            key = sc2ascii[_scancode & 0x7F];
                        }
        
                        if (key != 0 && _finit)
                        {
                            if (key == 13)
                            {
                                key = 10;
                            }

                            queue_add_blocking(&_queueTerminal, &key);

                            //PrintChar(key,false);
                            /*
                            if (_keystate[0x1F] != 0)
                            {
                                PrintChar(key, true);
                            }
                            else
                            {
                                PrintChar(key, false);
                            }
                            */
                        }
                    }

                    _keystate[_scancode] = _keydown ? 1 : 0;
                    _keydown = true;
                }

                _bits = 0;
                _scancode = 0;
            }
        }        

    if (!_finit)
    {  
        for(int i = 0; i < count_of(embedded_files); i++)
        {
            const int START_SAVE_FLASH = 1964;  // flash ends at 2048?  4K block per save game as of now (1964 = 2011136)
                                                // TODO: erase a block of flash once, write save games as 1K blocks instead of 4K blocks
                                                // Include a cookie to prevent restoring garbage if nothing has been saved yet.
                                                // use cookie to determine if save game flash area should be erased and do that one time only.
                                                // 1964 means 2044-1964 / 4K blocks = 20 save games (A through T) 

            if (_keystate[_scancode_map[i]])    // A through ...
            {
                _finit = true;
                _flashOffset = (START_SAVE_FLASH + (4 * i)) * 1024;
                badgerfrotz_main(embedded_files[i].name);
                os_reboot();
                _finit = false;
            }
        }

    }
   
    _cycles++;
}

int os_getchar()
{
    uint8_t byte;
    while(false == queue_try_remove(&_queueTerminal, &byte))
    {
        os_tick();
    }

    return byte;
}


void PrintTitleScreen()
{
    char buf[2400] = {};
    int linemax = 0;

    PrintAt(0,0, (char *)_titlescreen, false);
    
    for (int i = 0; i < count_of(embedded_files); i++)
    {
        char line[128] = {};
        sprintf(line, " %c: %s\r\n", 'A' + i, embedded_files[i].description);
        strcat(buf,line);
    }

    strcat(buf, "\r\n Press a key to play: ");

    PrintAt(0, 9, buf, false);
}

int main(int , char **)
{
    multicore_lockout_victim_init();
    queue_init(&_queueTerminal, 1, 1024);

    //mutex_init(&_mutexLock);

    //vreg_set_voltage(VREG_VOLTAGE_1_10);
    set_sys_clock_khz(250000, true);

    stdio_init_all();
    //setup_default_uart();
    //stdio_usb_init();


    multicore_launch_core1(core1);

    //PrintString((char *)_artHHG, false);
    PrintTitleScreen();
    
    while(true)
    {
        os_tick();
    }

    return 0;
}

int os_read_savefile(std::vector<uint8_t> &vecBytes)
{
    uint8_t *pFlash = (uint8_t *) XIP_BASE + _flashOffset;
    uint32_t size = *(uint32_t *)pFlash;

    if (size >> 16 != 0xEBAD)
    {
        // no cookie, not valid save file
        return 0;
    }

    size = size & 0xFFFF;
    
    pFlash += 4;
    vecBytes.resize(size);
    for(int i = 0; i < size; i++)
    {
        vecBytes[i] = pFlash[i];
    }

    return size & 0xFFFF;
}

void __not_in_flash_func(os_write_savefile)(std::vector<uint8_t> &vecBytes)
{
    int writesize = vecBytes.size() + sizeof(uint32_t);
    writesize += writesize % FLASH_BLOCK_SIZE;
    int erasesize = writesize + writesize % FLASH_SECTOR_SIZE;

    uint8_t *data = new uint8_t[writesize];
    uint8_t *pData = data;
    uint32_t *pVal = (uint32_t *)data;

    memset(data, 0, writesize);

    *pVal = (vecBytes.size() & 0xFFFF) | (0xEBAD << 16);

    pData += sizeof(uint32_t);

    for(int i = 0; i < vecBytes.size(); i++)
    {
        pData[i] = vecBytes[i];
    }

    _flashDone = false;
    _flashReady = false;

    _beginFlash = true;            
    while(_flashReady == false) 
    {
        sleep_ms(1);
    }

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(_flashOffset, erasesize); 
    flash_range_program(_flashOffset, data, writesize);  
    restore_interrupts(ints);

    delete [] data;

    _flashDone = true;
}