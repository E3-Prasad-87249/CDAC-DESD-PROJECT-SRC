/* SPI Slave Demo

 *
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 *
 
 */
#include <SPI.h>
#include<stdint.h>  

#define SPI_SCK 	13
#define SPI_MISO 	12
#define SPI_MOSI 	11
#define SPI_SS 		10

char dataBuff[500];

// Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);
  pinMode(MISO, OUTPUT);
    
  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));

  /* Return Data Register */
  return SPDR;
}


//sends one byte of data 
void SPI_SlaveTransmit(char data)
{
  /* Start transmission */
  SPDR = data;

  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}
  

// The setup() function runs right after reset.
void setup() 
{
  // Initialize serial communication 
  Serial.begin(9600);
  // Initialize SPI Slave.
  SPI_SlaveInit();
  Serial.println("Ardruino Slave Debug Interface is Initialized!");
}

// The loop function runs continuously after setup().
void loop() 
{
  uint32_t i;
  uint16_t dataLen = 0;
  //Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS));
  i = 0;
  dataLen = SPI_SlaveReceive();
  for(i = 0 ; i < dataLen ; i++)
  {
    dataBuff[i] =  SPI_SlaveReceive();
  }

  //  Serial.println(String(i,HEX));
  dataBuff[i] = '\0'; 
  Serial.print("Rcvd dbg_msg as:");
  Serial.println(dataBuff);
}


   
   
