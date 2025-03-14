/**
\brief SCuM-specific definition of the "board" bsp module.

\author Tengfei Chang <tengfei.chang@inria.fr>, August 2016.
*/

#include "memory_map.h"
#include "board.h"
#include "debugpins.h"
#include "opendefs.h"
// bsp modules
#include "adc.h"
#include "leds.h"
#include "uart.h"
#include "radio.h"
#include "eui64.h"
#include "sctimer.h"
#include "optical.h"
#include "scm3c_hw_interface.h"
//=========================== definition ======================================

#define CRC_VALUE           (*((unsigned int *) 0x0000FFFC))
#define CODE_LENGTH         (*((unsigned int *) 0x0000FFF8))

//=========================== variables =======================================

extern char send_packet[127];
extern unsigned int ASC[38];
extern unsigned int ASC_FPGA[38];

extern unsigned int RX_channel_codes[16];
extern unsigned int TX_channel_codes[16];

// Bootloader will insert length and pre-calculated CRC at these memory addresses    
#define crc_value         (*((unsigned int *) 0x0000FFFC))
#define code_length       (*((unsigned int *) 0x0000FFF8))

// LC freq = 2.405G
unsigned int LC_target = 2001200; //2002083;//2004000;//2004000; //801900; 
unsigned int LC_code = 360;

unsigned int HF_CLOCK_fine = 15;
unsigned int HF_CLOCK_coarse = 5;
unsigned int RC2M_superfine = 16;

// MF IF clock settings
unsigned int IF_clk_target = 1600000;
unsigned int IF_coarse = 22;
unsigned int IF_fine = 17;

unsigned int cal_iteration = 0;
unsigned int run_test_flag = 0;
unsigned int num_packets_to_test = 1;

unsigned short doing_initial_packet_search;
unsigned short current_RF_channel;
unsigned short do_debug_print = 0;

// ADC configuration.
static const adc_config_t g_adc_config = {
    .reset_source = ADC_RESET_SOURCE_FSM,
    .convert_source = ADC_CONVERT_SOURCE_FSM,
    .pga_amplify_source = ADC_PGA_AMPLIFY_SOURCE_FSM,
    .pga_gain = 0,
    .settling_time = 0,
    .bandgap_reference_tuning_code = 1,
    .const_gm_tuning_code = 0xFF,
    .vbat_div_4_enabled = FALSE,
    .ldo_enabled = TRUE,
    .input_mux_select = ADC_INPUT_MUX_SELECT_EXTERNAL_SIGNAL,
    .pga_bypass = TRUE,
};

//=========================== prototypes ======================================

unsigned reverse(unsigned x);
unsigned int crc32c(unsigned char *message, unsigned int length);

void board_optical_calibration(void);
void build_channel_table(unsigned int channel_11_LC_code);

//=========================== interrupt ========================================

//=========================== main =============================================

extern int mote_main(void);

int main(void) {
    return mote_main();
}

//=========================== public ==========================================

void board_init(void) {
    uint8_t eui[8];
    uint32_t calc_crc;
    
    printf("Initializing...");
        
    // Set up mote configuration
    // This function handles all the analog scan chain setup
    initialize_mote();

    // Configure the ADC.
    adc_config(&g_adc_config);
    adc_enable_interrupt();
    analog_scan_chain_write();
    analog_scan_chain_load();

    // Check CRC to ensure there were no errors during optical programming
    printf("\r\n-------------------\r\n");
    printf("Validating program integrity..."); 
    
    calc_crc = crc32c(0x0000,CODE_LENGTH);
    
    if(calc_crc == CRC_VALUE){
        printf("CRC OK\r\n");
    } else{
        printf("\r\nProgramming Error - CRC DOES NOT MATCH - Halting Execution\r\n");
        while(1);
    }
    
    // set priority of interrupts
    
    IPR0 = 0xFF;    // uart has lowest priority
    IPR6 = 0x0F;    // priority for radio
    IPR7 = 0x00;    // priority for rf_timer
    
    // initialize bsp modules
    debugpins_init();
    leds_init();
    uart_init();
    sctimer_init();
    radio_init();
    eui64_get(eui);
    
    // After bootloading the next thing that happens is frequency calibration using optical
    printf("Calibrating frequencies...\r\n");
    
    // Initial frequency calibration will tune the frequencies for HCLK, the RX/TX chip clocks, and the LO

    // For the LO, calibration for RX channel 11, so turn on AUX, IF, and LO LDOs
    // by calling radio rxEnable
    radio_rxEnable();
    
    // Enable optical SFD interrupt for optical calibration
    optical_enable();
    
    // Wait for optical cal to finish
    while(optical_getCalibrationFinshed() == 0);

    printf("Cal complete\r\n");
}

void board_sleep(void) {
    uint16_t i;
    
    for (i=0;i<0xf;i++);
    // not sure how to enter a sleep mode
}

void board_reset(void) {
    // not sure how the reset is triggered
}

//=========================== private =========================================
