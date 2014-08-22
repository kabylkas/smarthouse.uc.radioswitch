#include "msp430g2231.h"
#include "types.h"
#include "cc1101.h"
// Device data
#define HOUSE_ADDR     0x11
#define DEVICE_ADDR    0x22

// Debugging flags
#define DEBUG           1
// LaunchPad board support package spi device definitions.
#define SPI_CSN_OUT     P1OUT
#define SPI_CSN_DIR     P1DIR
#define SPI_MISO_IN     P1IN
#define SPI_CSN_PIN     BIT4
#define SPI_CLK_PIN     BIT5
#define SPI_MOSI_PIN    BIT6
#define SPI_MISO_PIN    BIT7
#define SWITCH_OUT      P1OUT
#define SWITCH_PIN      BIT0
#define SWITCH_DIR      P1DIR

static const struct sCC1101 gCC1101Settings = {
  0x29,               // GDO2 output pin configuration.                     
  0x2E,               // GDO1 output pin configuration.
  0x06,               // GDO0 output pin configuration.
  0x47,               // RXFIFO and TXFIFO thresholds.
  0xD3,               // Sync word, high byte
  0x91,               // Sync word, low byte
  0xFF,               // Packet length.                                     
  0x04,               // Packet automation control.                         
  0x05,               // Packet automation control.
  0x00,               // Device address.
  0x00,               // Channel number.
  0x06,               // Frequency synthesizer control.
  0x00,               // Frequency synthesizer control.
  0x10,               // Frequency control word, high byte.
  0xB1,               // Frequency control word, middle byte.
  0x3B,               // Frequency control word, low byte.
  0xF6,               // Modem configuration.
  0x83,               // Modem configuration.
  0x1B,               // Modem configuration.
  0x22,               // Modem configuration.
  0xF8,               // Modem configuration.
  0x15,               // Modem deviation setting (when FSK modulation is enabled).
  0x07,               // Main Radio Control State Machine configuration.
  0x30,               // Main Radio Control State Machine configuration.    
  0x18,               // Main Radio Control State Machine configuration.
  0x16,               // Frequency Offset Compensation Configuration.
  0x6C,               // Bit synchronization Configuration.
  0x03,               // AGC control.
  0x40,               // AGC control.
  0x91,               // AGC control.
  0x87,               // High byte Event 0 timeout
  0x6B,               // Low byte Event 0 timeout
  0xFB,               // Wake On Radio control
  0x56,               // Front end RX configuration.
  0x17,               // Front end RX configuration.
  0xE9,               // Frequency synthesizer calibration.
  0x2A,               // Frequency synthesizer calibration.
  0x00,               // Frequency synthesizer calibration.
  0x1F,               // Frequency synthesizer calibration.
  0x41,               // RC oscillator configuration
  0x00,               // RC oscillator configuration  
  0x59,               // Frequency synthesizer calibration control
  0x7F,               // Production test
  0x3F,               // AGC test
  0x81,               // Various test settings.
  0x35,               // Various test settings.
  0x09                // Various test settings.
};

static uint8 gPaTable[8] = {0x00,0x12,0x0e,0x34,0x60,0xc5,0xc1,0xc0};

void delay(uint16 n)
{
  while (n>0) 
  {
    __delay_cycles(2);
    n--;
  }
}

// SPI control implementations for the LaunchPad platform.
void SpiInit()
{
  // Setup CSn line.
  SPI_CSN_DIR |= SPI_CSN_PIN;
  //SPI_CSN_SEL &= ~SPI_CSN_PIN;
  SPI_CSN_OUT |= SPI_CSN_PIN;
  
  // Setup the USI peripheral for SPI operation.
  USICTL0 |= USISWRST; // freeze the peripheral for config
  USICTL0 |= USIPE7 +  USIPE6 + USIPE5 + USIMST + USIOE; // Port, SPI master
  USICTL1 |= USIIE + USICKPH;
  USICKCTL |= USIDIV_1 + USISSEL_2;// + USICKPL;      // Clock select: SMCLK
  USICTL0 &= ~USISWRST; // release the peripheral
}

void SwitchInit()
{
  SWITCH_DIR |= SWITCH_PIN;
  
  SWITCH_OUT |= SWITCH_PIN;
  delay(50000);
  SWITCH_OUT &= ~SWITCH_PIN;
  delay(50000);
  SWITCH_OUT |= SWITCH_PIN;
  delay(50000);
  SWITCH_OUT &= ~SWITCH_PIN;
}

void SpiRead(uint8 address, uint8 *pBuffer, uint8 count)
{
  uint8 i;
  //chip select
  SPI_CSN_OUT &= ~SPI_CSN_PIN;
  // Look for CHIP_RDYn from radio.
  while (SPI_MISO_IN & SPI_MISO_PIN);

  // When USIIFG = 0 and USICNTx > 0, clock generation is enabled and the 
  // master will begin clocking in/out data using USISR.
  // send address
  USISRL = address;
  USICNT = 8;
  while (!(USICTL1 & USIIFG));
  // start reading data
  for (i = 0; i < count; i++)
  {
    USISRL = 0xFF;
    USICNT = 8;
    while (!(USICTL1 & USIIFG));
    *(pBuffer+i) = USISRL;
  }
  //chip deselect
  SPI_CSN_OUT |= SPI_CSN_PIN;
}

void SpiWrite(uint8 address, uint8 *pBuffer, uint8 count)
{
  uint8 i;
  
  //chip select
  SPI_CSN_OUT &= ~SPI_CSN_PIN;
  // Look for CHIP_RDYn from radio. While SO is high
  while (SPI_MISO_IN & SPI_MISO_PIN);

  // When USIIFG = 0 and USICNTx > 0, clock generation is enabled and the 
  // master will begin clocking in/out data using USISR.
  // send address
  USISRL = address;
  USICNT = 8;
  while (!(USICTL1 & USIIFG));
  // start reading data
  for (i = 0; i < count; i++)
  {
    USISRL = *(pBuffer+i);
    USICNT = 8;
    while (!(USICTL1 & USIIFG));
  }
  
  //chip deselect
  SPI_CSN_OUT |= SPI_CSN_PIN;
}

void CC1101Write(uint8 address, uint8 *buffer, uint8 count)
{
  if (count > 1)
  {
    address = address | CC1101_WRITE_BURST;
  }
  else
  {
    address = address | CC1101_WRITE_SINGLE;
  }
  
  SpiWrite(address, buffer, count);
}

void CC1101Read(uint8 address, uint8 *buffer, uint8 count)
{
  if (count > 1)
  {
    address = address | CC1101_READ_BURST;
  } 
  else
  {
    address = address | CC1101_READ_SINGLE;
  }
  
  SpiRead(address, buffer, count);
}

void CC1101Config(const struct sCC1101* config)
{
  uint8 i;
  uint8* conf = (uint8*)(config);
  for (i=0; i<sizeof(struct sCC1101)/sizeof(uint8); i++)
  {
    CC1101Write(
      i, //address of the config registers
      (conf+i), //pointer to configurations
      1 // number of registers to configure
    );
  }
}

void CC1101Strobe(uint8 command)
{
  SpiWrite(command, NULL, 0);
}

uint8 CC1101GetStatus()
{
  CC1101Strobe(CC1101_SNOP|CC1101_READ_SINGLE);
  return USISRL;
}

int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  // Set up the clock
  BCSCTL1 = CALBC1_1MHZ;                    
  DCOCTL = CALDCO_1MHZ; 
  
  SpiInit();
  SwitchInit();
  
  CC1101Strobe(CC1101_SRES);
  delay(255);
  CC1101Config(&gCC1101Settings);
  CC1101Write(CC1101_PATABLE, gPaTable, 8);
  CC1101Strobe(CC1101_SIDLE);
  uint8 rxData[5], k;
  
  while(1)
  {
    delay(255);
    CC1101Strobe(CC1101_SIDLE);
    CC1101Strobe(CC1101_SFRX);
    CC1101Strobe(CC1101_SFRX);
    CC1101Strobe(CC1101_SRX);
    
    uint8 did_not_get = 1;
    while (did_not_get)
    {
      k = CC1101GetStatus();
      if (k&CC1101_FIFO_BYTES_AVAILABLE)
      {
        did_not_get = 0;
      }
      delay(10000);
    }
    
    // Get data from FIFO
    CC1101Read(CC1101_RXFIFO, rxData, 4);
    
    // Process data
    if (rxData[1] == HOUSE_ADDR)
    {
      if (rxData[2] == DEVICE_ADDR)
      {
        if (rxData[3] == 0xAA)
        {
          SWITCH_OUT |= SWITCH_PIN; // on
        }
        
        if (rxData[3] == 0x55)
        {
          SWITCH_OUT &= ~SWITCH_PIN; // off          
        }
      }
    }
  }
}
