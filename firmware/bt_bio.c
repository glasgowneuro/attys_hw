#include <msp430.h>
#include <stdio.h>
#include "mpu9250.h"

#define ADC_CLOCK_SLOW 250
#define ADC_CLOCK_FAST 2

unsigned char uart_rx_avail = 0;
unsigned char uart_rx_char = 0;

unsigned char spi_rx_avail = 0;
unsigned char spi_rx_char = 0;

unsigned int accel_x = 0;
unsigned int accel_y = 0;
unsigned int accel_z = 0;

unsigned int mag_x = 0;
unsigned int mag_y = 0;
unsigned int mag_z = 0;

unsigned int gyr_x = 0;
unsigned int gyr_y = 0;
unsigned int gyr_z = 0;

unsigned int timestamp = 0;

unsigned int powergoodoff = 10;

int temperature;

unsigned char adc_stat;
unsigned char gyro_stat;
unsigned char mag_stat;

uint32_t adc_ch1,adc_ch2;

unsigned char adc_reg0 = 0x55;

uint8_t mpu9250_AK8963_whoami = 0;
uint8_t mpu9250_whoami = 0;

#define CONFIG_BUFFER_SIZE 32
unsigned char config_buffer[CONFIG_BUFFER_SIZE];
unsigned int config_ptr = 0;

// brute force delay
void delay(unsigned int n)
{
	unsigned int i;

	for(i=0;i<n;i++) {
		volatile unsigned long int r = i*10;
	}
}


// make the power LED go off for a short while
void flashPowerLED()
{
	// toggles the power LED for about a second
	powergoodoff = 10;
}


// is called from the main interrupt and is counted
// down till zero and then the LED is switched back on
void fatalLED()
{
	while (1) {
		// switch it off and count down
		delay(30000);
		P2OUT &= ~0x80;
		delay(30000);
		P2OUT |= 0x80;
	}
}


// is called from the main interrupt and is counted
// down till zero and then the LED is switched back on
void controlPowerLED()
{
	// flash power LED
	if (powergoodoff) {
		// switch it off and count down
		P2OUT &= ~0x80;
		powergoodoff--;
	} else {
		// switch it on
		P2OUT |= 0x80;
	}
}


unsigned char spi_txrx(unsigned char tx)
{
	volatile unsigned char rx;

	// in case a transfer is still ongoing (it shouldn't)
	while (!(IFG2 & UCB0TXIFG));
	// trasmit a byte
	UCB0TXBUF = tx;
	// wait for the byte to arrive
	while (!(IFG2 & UCB0RXIFG));
	rx = UCB0RXBUF;
	return rx;
}




// ADS1292

void adc_init_spi(unsigned char d)
{
	UCB0BR0 = d;
	// inactive state of the SPI is low
	UCB0CTL0 &= ~UCCKPL;
	// Phase=0, data is already ready after /CS
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

}

void adc_command(unsigned char value)
{
	adc_init_spi(ADC_CLOCK_SLOW);

	// CS to low
	P1OUT &= ~BIT4;

	spi_txrx(value);

	delay(1000);

	// CS to high
	P1OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}



void adc_write_reg(unsigned char index, unsigned char value)
{
	adc_init_spi(ADC_CLOCK_SLOW);

	// CS to low
	P1OUT &= ~BIT4;

	spi_txrx(index | 0x40);
	spi_txrx(0x00);
	spi_txrx(value);

	delay(1000);

	// CS to high
	P1OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}



// ADS1292
unsigned char adc_read_reg(unsigned char index)
{
	unsigned char ret;

	adc_init_spi(ADC_CLOCK_SLOW);

	// CS to low
	P1OUT &= ~BIT4;

	spi_txrx(index | 0x20);
	spi_txrx(0x00);
	ret = spi_txrx(0x00);

	delay(1000);

	// CS to high
	P1OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;

	return ret;
}



void adc_read_data()
{
	uint32_t b2,b1,b0;

	adc_init_spi(ADC_CLOCK_FAST);
	
	// CS to low
	P1OUT &= ~BIT4;

	adc_stat = spi_txrx(0x00);
	spi_txrx(0x00);
	spi_txrx(0x00);

	b2 = spi_txrx(0x00);
	b1 = spi_txrx(0x00);
	b0 = spi_txrx(0x00);
	adc_ch1 = ((b2 << 16) | (b1 << 8) | b0) ^ 0x00800000;

	b2 = spi_txrx(0x00);
	b1 = spi_txrx(0x00);
	b0 = spi_txrx(0x00);
	adc_ch2 = ((b2 << 16) | (b1 << 8) | b0) ^ 0x00800000;

	// CS to high
	P1OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}


void initADC()
{
	// stop cont data read
	// adc_command(0x11);
	// read ID
	// do {
	//	adc_reg0 = adc_read_reg(0);
	// } while (adc_reg0 == 0x73);
	// 250Hz sampling rate
       	adc_write_reg(0x01,0b00000001);
	// switch the REF on
	adc_write_reg(0x02,0b10100000);
	// start cont data read
	// adc_command(0x10);
	// set start to high
	P1OUT |= BIT3;
}


// reads / writes from a register
// xmtype defines if it's the gyroscope or the magnetometer
unsigned char mpu9250_txrx(unsigned char readreg,
                        unsigned char addr, 
			unsigned char c)
{
	unsigned char rx;

	// 1MHz SPI clock
	UCB0BR0 = 16;
	// inactive state of the SPI is high
	UCB0CTL0 |= UCCKPL;
	// Phase=0, data is changed on the first edge
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

	// CS
	P2OUT &= ~BIT4;

	if (readreg) {
		addr |= 0x80;
		c = 0;
	} else {
		addr &= 0x7f;
	}

	rx=spi_txrx(addr);
	rx=spi_txrx(c);

	// CS
	P2OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;

	return rx;
}


// reads from multiple registers
void mpu9250_rx_multi(unsigned char addr, 
		      unsigned char n,
		      unsigned char *rx)
{
	int i;

	// 1MHz SPI clock
	UCB0BR0 = 16;
	// inactive state of the SPI is high
	UCB0CTL0 |= UCCKPL;
	// Phase=0, data is changed on the first edge
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

	// CS
	P2OUT &= ~BIT4;

	addr |= 0x80;
	spi_txrx(addr);

	for(i=0; i<n; i++) {
		*rx=spi_txrx(0);
		rx++;
	}

	// CS
	P2OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}


uint8_t mpu9250_readRegister(const uint8_t register_addr) {
	return mpu9250_txrx(1,register_addr,0);
}


uint8_t mpu9250_writeRegister(const uint8_t register_addr,
			      const uint8_t value) {
	return mpu9250_txrx(0,register_addr,value);
}


int16_t mpu9250_readRegisters(const uint8_t msb_register, 
			      const uint8_t lsb_register) {
    uint8_t msb = mpu9250_readRegister(msb_register);
    uint8_t lsb = mpu9250_readRegister(lsb_register);
    return (((int16_t)msb) << 8) | lsb;
}


int mpu9250_checkwhoIam() {
	int i = 0;
	mpu9250_whoami = 0;
	for(i=0;i<32000;i++) {
		mpu9250_whoami = mpu9250_readRegister(MPU9250_WHO_AM_I);
		if ( mpu9250_whoami == WHOAMI_DEFAULT_VAL ) return 1;
		if ( mpu9250_whoami == WHOAMI_RESET_VAL ) return 1;
	}
	return 0;
}	


// should return 48h
uint8_t mpu9250_AK8963_read_whoami_reg() {
	uint8_t response;
	
	// talk to slave at address AK8963_I2C_ADDR, READ OP
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);
	// read from register AK8963_WIA
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG, AK8963_WIA);
	// read one byte from slave 0
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL, 0x81);
	delay(10);
	response=mpu9250_readRegister(MPUREG_EXT_SENS_DATA_00);
	return response;
}


int mpu9250_AK8963_checkwhoIam() {
	int i;
	mpu9250_AK8963_whoami = 0;
	for(i=0;i<32000;i++) {
		mpu9250_AK8963_whoami = mpu9250_AK8963_read_whoami_reg();
		if ( mpu9250_AK8963_whoami == 0x48 ) return 1;
	}
	return 0;
}	


void mpu9250_setFullScaleGyroRange(const uint8_t range) {
	// normal operation
        mpu9250_writeRegister(MPU9250_GYRO_CONFIG, range);
}


void mpu9250_setFullScaleAccelRange(const uint8_t range) {
	// normal operation
        mpu9250_writeRegister(MPU9250_ACCEL_CONFIG, range);
}



void initmpu9250() {
	// master reset
	mpu9250_writeRegister(MPU9250_PWR_MGMT_1,0x80);
	delay(0xffff);
	delay(0xffff);
	// wake it up with internal oscillator
	mpu9250_writeRegister(MPU9250_PWR_MGMT_1,0x00);
	delay(0xffff);
	delay(0xffff);
	// switch clock source to PLL
	mpu9250_writeRegister(MPU9250_PWR_MGMT_1,0x01);
	delay(0xffff);
	delay(0xffff);
	// check that we are alive and know who we are
	gyro_stat = mpu9250_checkwhoIam();
	delay(0xffff);

	// switch on all sensors
	mpu9250_writeRegister(MPU9250_PWR_MGMT_2,0x00);
	delay(0xffff);

	// sampling rate 1kHz, 90Hz bandwidth
	mpu9250_writeRegister(MPU9250_CONFIG,0x02);
	delay(0xffff);

	// /8 = 125Hz
	mpu9250_writeRegister(MPU9250_SMPLRT_DIV,7);
	delay(0xffff);

	// set the range of the gyro and accelerometer
	mpu9250_setFullScaleAccelRange(MPU9250_FULL_SCALE_8G);
	delay(0xffff);
       	mpu9250_setFullScaleGyroRange(MPU9250_GYRO_FULL_SCALE_2000DPS);
	delay(0xffff);

	// Enable the I2C Master I/F module
	mpu9250_writeRegister(MPU9250_USER_CTRL,0x20);
	delay(0xffff);
	// 400kHz clock for I2C
        mpu9250_writeRegister(MPUREG_I2C_MST_CTRL,0x0D);
	delay(0xffff);

	// magnetometer reset
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG,AK8963_CNTL2);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_DO,0x01);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL,0x81);
	delay(0xffff);

	// check if it's back to normal operation and knows...
	mag_stat = mpu9250_AK8963_checkwhoIam();
	delay(0xffff);

	// set the magnetometer to contious mode 1
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG,AK8963_CNTL1);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_DO,0x12);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL,0x81);
	delay(0xffff);

	// initiate a block read from the magnetometer to the MPU
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);
	// read the high byte of the X direction
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG, AK8963_HXL);
	// read 7 bytes (x,y,z,status)
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL, 0x87);
	delay(0xffff);
}


void mpu9250_read_data() {
	uint8_t rx[20];

	mpu9250_rx_multi( MPU9250_ACCEL_XOUT_H, 20, rx );

	accel_x = (((uint16_t)(rx[0])<<8) | (uint16_t)(rx[1])) ^ 0x8000;
	accel_y = (((uint16_t)(rx[2])<<8) | (uint16_t)(rx[3])) ^ 0x8000;
	accel_z = (((uint16_t)(rx[4])<<8) | (uint16_t)(rx[5])) ^ 0x8000;

	uint16_t raw = ((((uint16_t)(rx[6])))<<8) | ((uint16_t)rx[7]);
	temperature = 15 + raw/321;

	gyr_x = (((uint16_t)(rx[8])<<8) | (uint16_t)(rx[9])) ^ 0x8000;
	gyr_y = (((uint16_t)(rx[10])<<8) | (uint16_t)(rx[11])) ^ 0x8000;
	gyr_z = (((uint16_t)(rx[12])<<8) | (uint16_t)(rx[13])) ^ 0x8000;

	mag_x = (((uint16_t)(rx[14])<<8) | (uint16_t)(rx[15])) ^ 0x8000;
	mag_y = (((uint16_t)(rx[16])<<8) | (uint16_t)(rx[17])) ^ 0x8000;
	mag_z = (((uint16_t)(rx[18])<<8) | (uint16_t)(rx[19])) ^ 0x8000;
}

// receive a character from the serial port and process it (commands)
__attribute__((interrupt(USCIAB0RX_VECTOR)))
void USCI0RX_ISR(void)
{
	if (!(IFG2 & UCA0RXIFG)) return;
	flashPowerLED();
	// store the received char
       	uart_rx_char = UCA0RXBUF;
	// character available
	uart_rx_avail = 1;
	// store it in the buffer
	config_buffer[config_ptr] = uart_rx_char;
	// increment pointer if possible
	if (config_ptr<CONFIG_BUFFER_SIZE)
		config_ptr++;
	// check for carrige return or line feed
	if ((uart_rx_char == 13) || (uart_rx_char == 10)) {
		// add zero to terminate string properly
		config_buffer[config_ptr] = 0;
		// check if we have an '=' sign and
		// at least one character after the '=' sign
		if ( (config_buffer[1] = '=') && 
		     (config_ptr>1) ) {
			switch (config_buffer[0]) {
			case 'a':
			case 'A':
				flashPowerLED();
				break;
			case 'm':
			case 'M':
				flashPowerLED();
				break;
			case 'g':
			case 'G':
				flashPowerLED();
				break;
			}
		}
		// always reset the buffer after a cr or lf
		config_ptr = 0;
		// set the string to zero (being paranoid)
		config_buffer[0] = 0;
		config_buffer[1] = 0;
		config_buffer[2] = 0;
	}
}


// transmit a character to the bluetooth module
// check that RTS is low
void uart_tx(unsigned char c)
{
	int timeout = 30000;
	// is RTS high? Then let's wait till it goes low.
	while ( (P2IN & 0x02 ) && (timeout>0) ) {timeout--;};
	// Timeout? Let's discard the data
	if (timeout==0) return;
	// USCI_A0 TX buffer ready?
	while (!(IFG2&UCA0TXIFG));
	// TX -> RXed character
	UCA0TXBUF = c;             
}

// receive data and wait for it. In case of a timeout we
// return 0xff
unsigned char uart_rx()
{
	int timeout = 3000;
	while ((!uart_rx_avail) && (timeout>0)) {timeout--;};
	uart_rx_avail = 0;
	if (timeout) 
		return uart_rx_char;
	else
		return 0xff;
}


// send a standard string to the serial output
void sendText(const unsigned char *txt)
{
	while ((*txt)!=0)
	{
		uart_tx(*txt);
		txt++;
	}
}


__attribute__((interrupt(PORT2_VECTOR)))
void port2ISR(void)
{
	char tmp[256];

	// not data ready which has caused it
	if (!(P2IFG & BIT3)) return;

	// get the data from the ADC converter
	adc_read_data();

	mpu9250_read_data();

	// send it to the host
	// the '-1' makes it easier to detect a broken sample
	sprintf(tmp,"-1,%u,",timestamp++);
	sendText(tmp);

	if (gyro_stat) {
		sprintf(tmp,"%u,%u,%u,%u,%u,%u,%u,",
			accel_x,accel_y,accel_z,
			gyr_x,gyr_y,gyr_z,temperature);
	} else {
		sprintf(tmp,"-1,-1,-1,-1,-1,-1,-1,");
	}	
	sendText(tmp);

	if (mag_stat) {
		sprintf(tmp,"%u,%u,%u,",
			mag_x,mag_y,mag_z);
	} else {
		sprintf(tmp,"-1,-1,-1,");	
	}
	sendText(tmp);

	if (adc_stat == 0xc0) {
		sprintf(tmp,"%lu,%lu",
			adc_ch1,adc_ch2
			);
	} else {
		sprintf(tmp,"-1,-1");
	}
	sendText(tmp);
	sendText("\n\r");

	controlPowerLED();

	P2IFG &= ~BIT3;
}





// init all the ports of the MSP430
static inline void initMSP430()
{
	int i;

	WDTCTL = WDTPW + WDTHOLD;
	DCOCTL = 0;
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

 	// first, get everything into a nice state
	P1SEL  = 0x00;
	P2SEL  = 0x00;
	
	P1SEL2  = 0x00;
	P2SEL2  = 0x00;
	
	P1REN = 0x00;
	P2REN = 0x00;
  
	P1DIR = 0x00;
	P2DIR = 0x00;
	
	P1OUT = 0x00;
	P2OUT = 0x00;
 
        P1IES  = 0x00;
        P1IE   = 0x00;

        P2IES  = 0x00;
        P2IE   = 0x00;
 
	// Bluetooth RESET, active low
	P2DIR |= 0x01;
	delay(1000);
	P2OUT |= 0x01;
	delay(1000);
	P2OUT &= ~0x01;
	// set it to high
	delay(1000);
	P2OUT |= 0x01;

	// BT CTS
	P2DIR |= 0x04;
	// keep it low so that we can receive data, we are always ready
	P2OUT &= ~0x04;

	// LED, power good, P2.7
	P2DIR |= 0x80;
	P2OUT |= 0x80;

	// bluetooth /PG input, P2.6
	P2REN |= 0x40;
	P2OUT |= 0x40;

	// UART
	P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	P1SEL2 = BIT1 + BIT2;                      
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK, 16MHz
	// division factor for 16MHz @ 115200baud, N = 138
	UCA0BR0 = 138;
	UCA0BR1 = 0;
	// UCBRSx = 7, UCBRFx = 0 for fraction
	UCA0MCTL = UCBRS2 + UCBRS1 + UCBRS0;
	// **Initialize USCI state machine**
	UCA0CTL1 &= ~UCSWRST;
       	IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
	uart_rx_char = UCA0RXBUF;

	// SPI
	// CS_G
	P2DIR |= BIT5;
	P2OUT |= BIT5;
	// CS_M
	P2DIR |= BIT4;
	P2OUT |= BIT4;
	// CS_ADC
	P1DIR |= BIT4;
	P1OUT |= BIT4;

	// START_ADC line to zero (not running)
	P1DIR |= BIT3;
	P1OUT &= ~BIT3;
	// RESET ADC
	P1DIR |= BIT0;
	P1OUT |= BIT0;
	delay(1000);
	P1OUT &= ~BIT0;
	delay(1000);
	P1OUT |= BIT0;
	delay(0xffff);

	// config the SPI
	P1SEL |= BIT5 + BIT6 + BIT7;
	P1SEL2 |= BIT5 + BIT6 + BIT7;
	UCB0CTL0 |= UCCKPL + UCCKPH + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
	UCB0CTL1 |= UCSSEL_2;                     // SMCLK
	UCB0BR0 = 2;                              // 8MHz SPI clock
	UCB0BR1 = 0;                              //
	UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	
	P2OUT &= ~BIT5;                           // Now with SPI signals initialized,
	P2OUT |= BIT5;                            // reset slave
	
	P2OUT &= ~BIT4;                           // Now with SPI signals initialized,
	P2OUT |= BIT4;                            // reset slave

}

void enableInterrupts()
{

        // P2.3 interrupt enabled for serial
        P2IE |= BIT3;
        // P2.3 high/low edge
        P2IES |= BIT3;   
        // P2.3 IFG cleared. We are ready to go
        P2IFG &= ~BIT3;

	// start interrupts
	__eint();

}








// main program
void main(void)
{
	// init all ports and switch off the watchdog
	initMSP430();

	initADC();

	initmpu9250();

	enableInterrupts();

	// low power mode 0: CPU is off. Everything else is on.
	_BIS_SR(LPM0_bits + GIE);
		
	// this should never be reached because the CPU is off
	// at this point. Everything is handled by interrupts. :)
	while (1);
}
