#ifndef CC1101_H
#define CC1101_H


  // Command strobes
  #define CC1101_SRES           0x30u // Reset chip
  #define CC1101_SFSTXON        0x31u // Enable and calibrate frequency synthesizer
  #define CC1101_SXOFF          0x32u // Turn off crystal oscillator
  #define CC1101_SCAL           0x33u // Calibrate frequency synthesizer and turn it off
  #define CC1101_SRX            0x34u // Enable rx
  #define CC1101_STX            0x35u // In IDLE state: enable TX
  #define CC1101_SIDLE          0x36u // Exit RX/TX, turn off frequency synthesizer
  // Note: CC1101 datasheet skips register at 0x37u. Use is unavailable.
  #define CC1101_SWOR           0x38u // Start automatic RX polling sequence
  #define CC1101_SPWD           0x39u // Enter power down mode when CSn goes high
  #define CC1101_SFRX           0x3Au // Flush the RX FIFO buffer
  #define CC1101_SFTX           0x3Bu // Flush the TX FIFO buffer
  #define CC1101_SWORRST        0x3Cu // Reset real time clock to Event1 value
  #define CC1101_SNOP           0x3Du // No operation

  // Status registers
  #define CC1101_PARTNUM        0x30u // Chip ID
  #define CC1101_VERSION        0x31u // Chip ID
  #define CC1101_FREQEST        0x32u // Frequency offset estimate from demodulator
  #define CC1101_LQI            0x33u // Demodulator estimate for link quality
  #define CC1101_RSSI           0x34u // Received signal strength indication
  #define CC1101_MARCSTATE      0x35u // Main radio control state machine state
  #define CC1101_WORTIME1       0x36u // High byte of WOR time
  #define CC1101_WORTIME0       0x37u // Low byte of WOR time
  #define CC1101_PKTSTATUS      0x38u // Current GDOx status and packet status
  #define CC1101_VCO_VC_DAC     0x39u // Current setting from PLL calibration module
  #define CC1101_TXBYTES        0x3Au // Underflow and number of bytes
  #define CC1101_RXBYTES        0x3Bu // Overflow and number of bytes
  #define CC1101_RCCTRL1_STATUS 0x3Cu // Last RC oscillator calibration result
  #define CC1101_RCCTRL0_STATUS 0x3Du // Last RC oscillator calibration result

  // PA table register
  #define CC1101_PATABLE        0x3Eu // RF output power level

  // RX/TX FIFO registers
  #define CC1101_RXFIFO         0x3Fu // Receive FIFO buffer (read-only)
  #define CC1101_TXFIFO         0x3Fu // Transmit FIFO buffer (write-only)

  // -----------------------------------------------------------------------------

  // Configuration register addresses
  /**
   *  Note: For the CC110L, some of the following registers are reserved. Please
   *  see the CC110L Value Line Transceiver User's Guide (swrs109a) for more
   *  information.
   */
  #define CC1101_REG_IOCFG2         0x00u // GDO2 Output Pin Configuration
  #define CC1101_REG_IOCFG1         0x01u // GDO1 Output Pin Configuration
  #define CC1101_REG_IOCFG0         0x02u // GDO0 Output Pin Configuration
  #define CC1101_REG_FIFOTHR        0x03u // RX FIFO and TX FIFO Thresholds
  #define CC1101_REG_SYNC1          0x04u // Sync Word, High Byte
  #define CC1101_REG_SYNC0          0x05u // Sync Word, Low Byte
  #define CC1101_REG_PKTLEN         0x06u // Packet Length
  #define CC1101_REG_PKTCTRL1       0x07u // Packet Automation Control
  #define CC1101_REG_PKTCTRL0       0x08u // Packet Automation Control
  #define CC1101_REG_ADDR           0x09u // Device Address
  #define CC1101_REG_CHANNR         0x0Au // Channel Number
  #define CC1101_REG_FSCTRL1        0x0Bu // Frequency Synthesizer Control
  #define CC1101_REG_FSCTRL0        0x0Cu // Frequency Synthesizer Control
  #define CC1101_REG_FREQ2          0x0Du // Frequency Control Word, High Byte
  #define CC1101_REG_FREQ1          0x0Eu // Frequency Control Word, Middle Byte
  #define CC1101_REG_FREQ0          0x0Fu // Frequency Control Word, Low Byte
  #define CC1101_REG_MDMCFG4        0x10u // Modem Configuration
  #define CC1101_REG_MDMCFG3        0x11u // Modem Configuration
  #define CC1101_REG_MDMCFG2        0x12u // Modem Configuration
  #define CC1101_REG_MDMCFG1        0x13u // Modem Configuration
  #define CC1101_REG_MDMCFG0        0x14u // Modem Configuration
  #define CC1101_REG_DEVIATN        0x15u // Modem Deviation Setting
  #define CC1101_REG_MCSM2          0x16u // Main Radio Control State Machine Configuration
  #define CC1101_REG_MCSM1          0x17u // Main Radio Control State Machine Configuration
  #define CC1101_REG_MCSM0          0x18u // Main Radio Control State Machine Configuration
  #define CC1101_REG_FOCCFG         0x19u // Frequency Offset Compensation Configuration
  #define CC1101_REG_BSCFG          0x1Au // Bit Synchronization Configuration
  #define CC1101_REG_AGCCTRL2       0x1Bu // AGC Control
  #define CC1101_REG_AGCCTRL1       0x1Cu // AGC Control
  #define CC1101_REG_AGCCTRL0       0x1Du // AGC Control
  #define CC1101_REG_WOREVT1        0x1Eu // High Byte Event0 Timeout
  #define CC1101_REG_WOREVT0        0x1Fu // Low Byte Event0 Timeout
  #define CC1101_REG_WORCTRL        0x20u // Wake On Radio Control
  #define CC1101_REG_FREND1         0x21u // Front End RX Configuration
  #define CC1101_REG_FREND0         0x22u // Front End TX Configuration
  #define CC1101_REG_FSCAL3         0x23u // Frequency Synthesizer Calibration
  #define CC1101_REG_FSCAL2         0x24u // Frequency Synthesizer Calibration
  #define CC1101_REG_FSCAL1         0x25u // Frequency Synthesizer Calibration
  #define CC1101_REG_FSCAL0         0x26u // Frequency Synthesizer Calibration
  #define CC1101_REG_RCCTRL1        0x27u // RC Oscillator Configuration
  #define CC1101_REG_RCCTRL0        0x28u // RC Oscillator Configuration
  #define CC1101_REG_FSTEST         0x29u // Frequency Synthesizer Calibration Control
  #define CC1101_REG_PTEST          0x2Au // Production Test
  #define CC1101_REG_AGCTEST        0x2Bu // AGC Test
  #define CC1101_REG_TEST2          0x2Cu // Various Test Settings
  #define CC1101_REG_TEST1          0x2Du // Various Test Settings
  #define CC1101_REG_TEST0          0x2Eu // Various Test Settings

  // -----------------------------------------------------------------------------

  // Configuration register field masks
  // IOCFG2
  #define CC1101_GDO2_INV                   0x40u
  #define CC1101_GDO2_CFG                   0x3Fu
  // IOCFG1
  #define CC1101_GDO1_DS                    0x80u
  #define CC1101_GDO1_INV                   0x40u
  #define CC1101_GDO1_CFG                   0x3Fu
  // IOCFG0
  #define CC1101_GDO0_TEMP_SENSOR_ENABLE    0x80u
  #define CC1101_GDO0_INV                   0x40u
  #define CC1101_GDO0_CFG                   0x3Fu
  // FIFOTHR
  #define CC1101_ADC_RETENTION              0x40u
  #define CC1101_CLOSE_IN_RX                0x30u
  #define CC1101_FIFO_THR                   0x0Fu
  // SYNC1
  #define CC1101_SYNC_MSB                   0xFFu
  // SYNC0
  #define CC1101_SYNC_LSB                   0xFFu
  // PKTLEN
  #define CC1101_PACKET_LENGTH              0xFFu
  // PKTCTRL1
  #define CC1101_PQT                        0xE0u
  #define CC1101_CRC_AUTOFLUSH              0x08u
  #define CC1101_APPEND_STATUS              0x04u
  #define CC1101_ADR_CHK                    0x03u
  // PKTCTRL0
  #define CC1101_WHITE_DATA                 0x40u
  #define CC1101_PKT_FORMAT                 0x30u
  #define CC1101_CRC_EN                     0x04u
  #define CC1101_LENGTH_CONFIG              0x03u
  // ADDR
  #define CC1101_DEVICE_ADDR                0xFFu
  // CHANNR
  #define CC1101_CHANNR_CHAN                0xFFu
  // FSCTRL1
  #define CC1101_FREQ_IF                    0x1Fu
  // FSCTRL0
  #define CC1101_FREQOFF                    0xFFu
  // FREQ2
  #define CC1101_FREQ_23_22                 0xC0u
  #define CC1101_FREQ_21_16                 0x3Fu
  // FREQ1
  #define CC1101_FREQ_15_8                  0xFFu
  // FREQ0
  #define CC1101_FREQ_7_0                   0xFFu
  // MDMCFG4
  #define CC1101_CHANBW_E                   0xC0u
  #define CC1101_CHANBW_M                   0x30u
  #define CC1101_DRATE_E                    0x0Fu
  // MDMCFG3
  #define CC1101_DRATE_M                    0xFFu
  // MDMCFG2
  #define CC1101_DEM_DCFILT_OFF             0x80u
  #define CC1101_MOD_FORMAT                 0x70u
  #define CC1101_MANCHESTER_EN              0x08u
  #define CC1101_SYNC_MODE                  0x07u
  // MDMCFG1
  #define CC1101_FEC_EN                     0x80u
  #define CC1101_NUM_PREAMBLE               0x70u
  #define CC1101_CHANSPC_E                  0x03u
  // MDMCFG0
  #define CC1101_CHANSPC_M                  0xFFu
  // DEVIATN
  #define CC1101_DEVIATION_E                0x70u
  #define CC1101_DEVIATION_M                0x07u
  // MCSM2
  #define CC1101_RX_TIME_RSSI               0x10u
  #define CC1101_RX_TIME_QUAL               0x08u
  #define CC1101_RX_TIME                    0x07u
  // MCSM1
  #define CC1101_CCA_MODE                   0x30u
  #define CC1101_RXOFF_MODE                 0x0Cu
  #define CC1101_TXOFF_MODE                 0x03u
  // MCSM0
  #define CC1101_FS_AUTOCAL                 0x30u
  #define CC1101_PO_TIMEOUT                 0x0Cu
  #define CC1101_PIN_CTRL_EN                0x02u
  #define CC1101_XOSC_FORCE_ON              0x01u
  // FOCCFG
  #define CC1101_FOC_BS_CS_GATE             0x20u
  #define CC1101_FOC_PRE_K                  0x18u
  #define CC1101_FOC_POST_K                 0x04u
  #define CC1101_FOC_LIMIT                  0x03u
  // BSCFG
  #define CC1101_BS_PRE_K                   0xC0u
  #define CC1101_BS_PRE_KP                  0x30u
  #define CC1101_BS_POST_K                  0x08u
  #define CC1101_BS_POST_KP                 0x04u
  #define CC1101_BS_LIMIT                   0x03u
  // AGCCTRL2
  #define CC1101_MAX_DVGA_GAIN              0xC0u
  #define CC1101_MAX_LNA_GAIN               0x38u
  #define CC1101_MAGN_TARGET                0x07u
  // AGCCTRL1
  #define CC1101_AGC_LNA_PRIORITY           0x40u
  #define CC1101_CARRIER_SENSR_REL_THR      0x30u
  #define CC1101_CARRIER_SENSE_ABS_THR      0x0Fu
  // AGCCTRL0
  #define CC1101_HYST_LEVEL                 0xC0u
  #define CC1101_WAIT_TIME                  0x30u
  #define CC1101_AGC_FREEZE                 0x0Cu
  #define CC1101_FILTER_LENGTH              0x03u
  // WOREVT1
  #define CC1101_EVENT0_15_8                0xFFu
  // WOREVT0
  #define CC1101_EVENT0_7_0                 0xFFu
  // WORCTRL
  #define CC1101_RC_PD                      0x80u
  #define CC1101_EVENT1                     0x70u
  #define CC1101_RC_CAL                     0x08u
  #define CC1101_WOR_RES                    0x03u
  // FREND1
  #define CC1101_LNA_CURRENT                0xC0u
  #define CC1101_LNA2MIX_CURRENT            0x30u
  #define CC1101_LODIV_BUF_CURRENT          0x0Cu
  #define CC1101_MIX_CURRENT                0x03u
  // FREND0
  #define CC1101_LODIV_BUF_CURRENT_TX       0x30u
  #define CC1101_PA_POWER                   0x07u
  // FSCAL3
  #define CC1101_FSCAL3_7_6                 0xC0u
  #define CC1101_CHP_CURR_CAL_EN            0x30u
  #define CC1101_FSCAL3_3_0                 0x0Fu
  // FSCAL2
  #define CC1101_VCO_CORE_H_EN              0x20u
  #define CC1101_FSCAL2_7_0                 0x1Fu
  // FSCAL1
  #define CC1101_FSCAL1_7_0                 0x3Fu
  // FSCAL0
  #define CC1101_FSCAL0_7_0                 0x7Fu
  // RCCTRL1
  #define CC1101_RCCTRL_1                   0x7Fu
  // RCCTRL0
  #define CC1101_RCCTRL_0                   0x7Fu
  // FSTEST
  #define CC1101_FSTEST_7_0                 0xFFu
  // PTEST
  #define CC1101_PTEST_7_0                  0xFFu
  // AGCTEST
  #define CC1101_AGCTEST_7_0                0xFFu
  // TEST2
  #define CC1101_TEST2_7_0                  0xFFu
  // TEST1
  #define CC1101_TEST1_7_0                  0xFFu
  // TEST0
  #define CC1101_TEST0_7_2                  0xFCu
  #define CC1101_VCO_SEL_CAL_EN             0x02u
  #define CC1101_TEST0_0                    0x01u

  // Status register field masks
  // PARTNUM
  #define CC1101_PARTNUM_7_0                0xFFu
  // VERSION
  #define CC1101_VERSION_7_0                0xFFu
  // FREQOFF_EST
  #define CC1101_FREQOFF_EST                0xFFu
  // LQI
  #define CC1101_CRC_OK                     0x80u
  #define CC1101_LQI_EST                    0x7Fu
  // RSSI
  #define CC1101_RSSI_7_0                   0xFFu
  // MARC_STATE
  #define CC1101_MARC_STATE                 0x1Fu
  // WORTIME1
  #define CC1101_TIME_15_8                  0xFFu
  // WORTIME0
  #define CC1101_TIME_7_0                   0xFFu
  // PKTSTATUS
  #define CC1101_PKTSTATUS_CRC_OK           0x80u
  #define CC1101_PKSTATUS_CS                0x40u
  #define CC1101_PKTSTATUS_PQT_REACHED      0x20u
  #define CC1101_PKTSTATUS_CCA              0x10u
  #define CC1101_PKSTATUS_SFD               0x08u
  #define CC1101_PKTSTATUS_GDO2             0x04u
  #define CC1101_PKTSTATUS_GDO0             0x01u
  // VCO_VC_DAC
  #define CC1101_VCO_VC_DAC_7_0             0xFFu
  // TXBYTES
  #define CC1101_TXFIFO_UNDERFLOW           0x80u
  #define CC1101_NUM_TXBYTES                0x7Fu
  // RXBYTES
  #define CC1101_RXFIFO_OVERFLOW            0x80u
  #define CC1101_NUM_RXBYTES                0x7Fu
  // RCCTRL1_STATUS
  #define CC1101_RCCTRL1_STATUS_7_0         0xFFu
  // RCCTRL0_STATUS
  #define CC1101_RCCTRL0_STATUS_7_0         0xFFu
  // Status byte
  #define CC1101_FIFO_BYTES_AVAILABLE       0x0Fu
  // Burst/single access masks
  #define CC1101_WRITE_SINGLE               0x00u
  #define CC1101_WRITE_BURST                0x40u
  #define CC1101_READ_SINGLE                0x80u
  #define CC1101_READ_BURST                 0xC0u

  // -----------------------------------------------------------------------------

  // Physical FIFO size
  #define CC1101_RXFIFO_SIZE        64 // Receive hardware FIFO absolute size
  #define CC1101_TXFIFO_SIZE        64 // Transmit hardware FIFO absolute size

  // Maximum timeout error ticks
  #define CC1101_MAX_TIMEOUT        2000

  // -----------------------------------------------------------------------------
  
struct sCC1101
{
  uint8 iocfg2;   // GDO2 output pin configuration
  uint8 iocfg1;   // GDO1 output pin configuration  
  uint8 iocfg0;   // GDO0 output pin configuration
  uint8 fifothr;  // RXFIFO and TXFIFO thresholds
  uint8 sync1;    // Sync word, high byte
  uint8 sync0;    // Sync word, low byte
  uint8 pktlen;   // Packet length
  uint8 pktctrl1; // Packet automation control
  uint8 pktctrl0; // Packet automation control
  uint8 addr;     // Device address
  uint8 channr;   // Channel number
  uint8 fsctrl1;  // Frequency synthesizer control
  uint8 fsctrl0;  // Frequency synthesizer control
  uint8 freq2;    // Frequency control word, high byte
  uint8 freq1;    // Frequency control word, middle byte
  uint8 freq0;    // Frequency control word, low byte
  uint8 mdmcfg4;  // Modem configuration 4
  uint8 mdmcfg3;  // Modem configuration 3
  uint8 mdmcfg2;  // Modem configuration 2
  uint8 mdmcfg1;  // Modem configuration 1
  uint8 mdmcfg0;  // Modem configuration 0
  uint8 deviatn;  // Modem deviation setting
  uint8 mcsm2;    // Main radio control state machine configuration
  uint8 mcsm1;    // Main radio control state machine configuration
  uint8 mcsm0;    // Main radio control state machine configuration
  uint8 foccfg;   // Frequency offset compensation configuration
  uint8 bscfg;    // Bit synchronization configuration
  uint8 agcctrl2; // AGC control 2
  uint8 agcctrl1; // AGC control 1
  uint8 agcctrl0; // AGC control 0
  uint8 worevt1;  // High byte event0 timeout
  uint8 worevt0;  // Low byte event0 timeout
  uint8 worctrl;  // Wake on radio control
  uint8 frend1;   // Front end RX configuration
  uint8 frend0;   // Front end TX configuration
  uint8 fscal3;   // Frequency synthesizer calibration
  uint8 fscal2;   // Frequency synthesizer calibration
  uint8 fscal1;   // Frequency synthesizer calibration
  uint8 fscal0;   // Frequency synthesizer calibration
  uint8 rcctrl1;  // RC oscillator configuration
  uint8 rcctrl0;  // RC oscillator configuration
  uint8 fstest;   // Frequency synthesizer calibration control
  uint8 ptest;    // Production test
  uint8 agctest;  // AGC test
  uint8 test2;    // Various test settings 2
  uint8 test1;    // Various test settings 1
  uint8 test0;    // Various test settings 0
};  
#endif