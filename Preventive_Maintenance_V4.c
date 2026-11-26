/* --------------------------------------------------
Preventive Maintenance Monitoring System
Board: MSP-EXP430F5529LP
File: main.c
Version: 3.0.0 - Consolidated warning logic
Author(s): Sinclair, Iaian, Arthur, Julius, Wai Loon
-------------------------------------------------- */

#include <msp430.h>
#include <stdint.h>
#include <math.h>

/* I2C addresses / registers */
#define INA219_ADDR          0x40
#define INA_CONFIG_REG       0x00
#define INA_SHUNT_REG        0x01
#define ADXL_ADDR            0x53
#define ADXL_DEVID           0x00
#define ADXL_BW_RATE         0x2C
#define ADXL_POWER_CTL       0x2D
#define ADXL_DATA_FORMAT     0x31
#define ADXL_DATAX0          0x32
#define ADXL_FIFO_CTL        0x38
#define ADXL_FIFO_STATUS     0x39

/* Sampling / calibration */
#define SAMPLES              128U
#define FIFO_BATCH           8U
#define CAL_WINDOWS          20U
#define ALERT_WINDOWS        3U
#define I2C_TIMEOUT          100000UL

/* Timer_A0 runs at 32768 Hz ACLK: 1638 counts is about 50 ms. */
#define TIMER_50MS_COUNTS    1638U
#define MONITOR_TICKS        20U     // 20 x 50 ms = about 1 second


/* Thresholds: units are 0.1 mg unless stated otherwise. */
#define RMS_TOL_PCT          20UL    /* at least +-20% of baseline */
#define RMS_SPREAD_MULT      1UL     /* also account for window variation */
#define RMS_TOL_FLOOR        20UL    /* at least +-2.0 mg */
#define IMPACT_MULT          3UL     /* 3 times max calibration peak to peak */
#define IMPACT_MIN           2500UL  /* minimum possible impact threshold: 250 mg */

/* Current and temperature warning settings (experimental) */
#define CURRENT_TOL_PCT      10L    // Allow +-10% of healthy current
#define CURRENT_TOL_MIN      100L   // Minimum tolerance of 10 mA
#define TEMP_RISE_LIMIT      50L    // Warn at 5.0 C above baseline
#define CURRENT_OBSTRUCTION  3000L  // 300.0 mA
#define CURRENT_STOPPED      50L    // Within +-5.0 mA of zero

/* --------------------------------------------------
UART A0: sole 3.3 V UART connection to FireBeetle ESP32

Author: Julius 
Version: 3.0.0
Responsibilities:
- Configure UART A0 and transmit HELLO, CAL, BASE and DATA messages.
- Include separate X, Y and Z RMS values in each DATA message.
P3.3 (TX) -> ESP32 GPIO16 (RX2)
P3.4 (RX) <- ESP32 GPIO17 (TX2) [optional]
Common GND required. Do not connect GPIO to 5 V.

Data line: DATA,current_x10_mA,temp_x10_C,xRms,yRms,zRms,totalRms,status,impact,currentStatus,tempStatus,machineStatus,overallStatus
Example: DATA,1752,321,120,180,130,252,0,0,0,0,0,0
All RMS values are in units of 0.1 mg.
status: 0=normal; 1=elevated/checking; 2=above normal;
        3=low/checking; 4=below normal
impact: 0=none, 1=possible impact event this window
-------------------------------------------------- */
void ESP_UART_Init(void)
{
    P3SEL |= BIT3 | BIT4;          /* UCA0 TX/RX */
    UCA0CTL1 = UCSWRST | UCSSEL_2;
    UCA0BR0 = 109;                 /* 9600 at ~1.048 MHz SMCLK */
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_2;
    UCA0CTL1 &= ~UCSWRST;
}

void ESP_UART_Char(char c)
{
    while (!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = c;
}

void ESP_UART_Print(const char *s)
{
    while (*s) ESP_UART_Char(*s++);
}

void ESP_UART_Number(unsigned long n)
{
    char digits[11];
    unsigned int i = 0;
    do {
        digits[i++] = (char)('0' + (n % 10UL));
        n /= 10UL;
    } while (n);
    while (i) ESP_UART_Char(digits[--i]);
}

void ESP_UART_Signed(long n)
{
    if (n < 0) {
        ESP_UART_Char('-');
        ESP_UART_Number((unsigned long)(-n));
    } else {
        ESP_UART_Number((unsigned long)n);
    }
}

/* --------------------------------------------------
Function: ESP_UART_SendData

Author: Wai Loon
Version: 3.0.0

Responsibilities:
- Send current, temperature and individual vibration RMS values to ESP32
- Include total vibration RMS, warning status and impact event

Parameters:
current_x10 - Current in 0.1 mA
 temp_x10  - Temperature in 0.1 C
xRms, yRms, zRms, rms_x10 - Vibration RMS in 0.1 mg
status     - Vibration warning status
impact     - Impact event flag
currentStatus - Current warning (same codes as vibration)
tempStatus - Temperature warning (0 normal, 1 checking, 2 high)
machineStatus - Combined fault code (0 none, 1 checking, 2 obstruction, 3 stopped, 4 looseness)
overallStatus - 0 healthy, 1 weak
-------------------------------------------------- */
void ESP_UART_SendData(int16_t current_x10, long temp_x10,
                       uint32_t xRms, uint32_t yRms, uint32_t zRms,
                       uint32_t rms_x10, unsigned int status,
                       unsigned int impact, unsigned int currentStatus,
                       unsigned int tempStatus, unsigned int machineStatus,
                       unsigned int overallStatus)
{
    ESP_UART_Print("DATA,");
    ESP_UART_Signed((long)current_x10); // Send current
    ESP_UART_Char(',');
    ESP_UART_Signed(temp_x10);         // Send temperature
    ESP_UART_Char(',');
    ESP_UART_Number(xRms);             // Send X-axis RMS
    ESP_UART_Char(',');
    ESP_UART_Number(yRms);             // Send Y-axis RMS
    ESP_UART_Char(',');
    ESP_UART_Number(zRms);             // Send Z-axis RMS
    ESP_UART_Char(',');
    ESP_UART_Number(rms_x10);          // Send combined RMS
    ESP_UART_Char(',');
    ESP_UART_Number(status);           // Send vibration warning status
    ESP_UART_Char(',');
    ESP_UART_Number(impact);           // Send impact event flag
    ESP_UART_Char(',');
    ESP_UART_Number(currentStatus);    // Send current warning
    ESP_UART_Char(',');
    ESP_UART_Number(tempStatus);       // Send temperature warning
    ESP_UART_Char(',');
    ESP_UART_Number(machineStatus);     // Send combined fault status
    ESP_UART_Char(',');
    ESP_UART_Number(overallStatus);     // 0 healthy, 1 weak
    ESP_UART_Print("\r\n");
}

void ESP_UART_SendCalWindow(unsigned int index, uint32_t rms_x10)
{
    ESP_UART_Print("CAL,");
    ESP_UART_Number(index);
    ESP_UART_Char(',');
    ESP_UART_Number((unsigned long)rms_x10);
    ESP_UART_Print("\r\n");
}

void ESP_UART_SendBaseline(uint32_t baseline, uint32_t lower,
                           uint32_t upper)
{
    ESP_UART_Print("BASE,");
    ESP_UART_Number(baseline);
    ESP_UART_Char(',');
    ESP_UART_Number(lower);
    ESP_UART_Char(',');
    ESP_UART_Number(upper);
    ESP_UART_Print("\r\n");
}

/* --------------------------------------------------
Function: I2C_Init

Author: Julius
Version: 2.3.0
Purpose: Configure MSP430 USCI_B0 as the shared I2C master.
Parameters: None
Returns: None
-------------------------------------------------- */
void I2C_Init(void)
{
    P3SEL |= BIT0 | BIT1;          // Select I2C function for SDA and SCL
    UCB0CTL1 = UCSWRST | UCSSEL_2; // Hold USCI_B0 in reset and select SMCLK
    UCB0CTL0 = UCMST | UCMODE_3 | UCSYNC; // I2C master in synchronous mode
    UCB0BR0 = 32;                 // Approx. 32.8 kHz when SMCLK is 1.048 MHz
    UCB0BR1 = 0;
    UCB0I2CSA = INA219_ADDR;      // Initial slave address (updated per transaction)
    UCB0CTL1 &= ~UCSWRST;         // Release USCI_B0 from reset
}

/* --------------------------------------------------
Function: I2C_RecoverBus

Author: Sinclair
Version: 2.3.0
Purpose:
Release a stuck SDA line by generating up to nine clock pulses,
then a STOP condition. Restore USCI_B0 afterward.
Parameters: None
Returns: 1 = recovered, 0 = bus still stuck
Note: External I2C pull-up resistors must be present.
-------------------------------------------------- */
unsigned char I2C_RecoverBus(void)
{
    unsigned int i, wait;
    unsigned char success = 1;

    UCB0CTL1 |= UCSWRST;          // Stop the I2C peripheral before GPIO recovery
    P3SEL &= ~(BIT0 | BIT1);      // Select GPIO control for SDA and SCL
    P3OUT &= ~(BIT0 | BIT1);      // Output LOW whenever a pin is driven
    P3DIR &= ~(BIT0 | BIT1);      // Release both pins (external pull-ups pull HIGH)
    __delay_cycles(1000);

    if (!(P3IN & BIT1))           // SCL must be HIGH before clocking SDA free
        success = 0;

    if (success)
    {
        for (i = 0; i < 9 && !(P3IN & BIT0); i++)
        {
            P3DIR |= BIT1;       // Pull SCL LOW
            __delay_cycles(1000);
            P3DIR &= ~BIT1;      // Release SCL HIGH

            for (wait = 0; wait < 1000 && !(P3IN & BIT1); wait++)
                __delay_cycles(10); // Allow SCL to rise or sensor to stretch clock

            if (!(P3IN & BIT1))  // Stop if SCL cannot be released
            {
                success = 0;
                break;
            }
            __delay_cycles(1000);
        }
    }

    if (success && (P3IN & BIT0))
    {
        P3DIR |= BIT0;           // Drive SDA LOW while SCL is HIGH
        __delay_cycles(1000);
        P3DIR &= ~BIT0;          // Release SDA HIGH to generate STOP
        __delay_cycles(1000);
        if ((P3IN & (BIT0 | BIT1)) != (BIT0 | BIT1))
            success = 0;         // Check both lines are released HIGH
    }
    else
        success = 0;

    I2C_Init();                   // Return pins to their I2C function
    __delay_cycles(1000);
    if (UCB0STAT & UCBBUSY)
        success = 0;
    return success;
}

/* --------------------------------------------------
Function: I2C_WriteRegister

Author: Sinclair
Version: 2.5.0
Purpose: Write one or more bytes to a sensor register using I2C.
Parameters:
addr  - Sensor I2C address
reg   - Register address
data  - Array containing the bytes to send
count - Number of data bytes to send
Returns: 1 = successful, 0 = NACK, timeout or busy bus
-------------------------------------------------- */
unsigned char I2C_WriteRegister(uint8_t addr, uint8_t reg,
                                const uint8_t *data, uint8_t count)
{
    unsigned long timeout = I2C_TIMEOUT;
    uint8_t i;

    if (data == 0 || count == 0) return 0; // Reject invalid data
    if (UCB0STAT & UCBBUSY) return 0;      // Do not start if I2C bus is busy

    UCB0I2CSA = addr;                  // Select the sensor address
    UCB0IFG &= ~UCNACKIFG;            // Clear previous NACK flag
    UCB0CTL1 |= UCTR | UCTXSTT;       // Set transmit mode and generate START

    timeout = I2C_TIMEOUT;
    while (!(UCB0IFG & UCTXIFG))       // Wait until TX buffer is ready
    {
        if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
    }
    if (UCB0IFG & UCNACKIFG) goto fail;
    UCB0TXBUF = reg;                  // Send sensor register address

    for (i = 0; i < count; i++)        // Send each data byte in order
    {
        timeout = I2C_TIMEOUT;
        while (!(UCB0IFG & UCTXIFG))   // Wait for space in TX buffer
        {
            if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
        }
        if (UCB0IFG & UCNACKIFG) goto fail;
        UCB0TXBUF = data[i];          // Transmit current data byte
    }

    timeout = I2C_TIMEOUT;
    while (!(UCB0IFG & UCTXIFG))       // Wait for last data byte to be sent
    {
        if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
    }
    if (UCB0IFG & UCNACKIFG) goto fail;
    UCB0CTL1 |= UCTXSTP;              // Generate STOP condition

    timeout = I2C_TIMEOUT;
    while (UCB0CTL1 & UCTXSTP)         // Wait for STOP to finish
    {
        if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
    }
    return (UCB0IFG & UCNACKIFG) ? 0 : 1;

fail:
    UCB0CTL1 |= UCTXSTP;              // Try to end failed communication
    timeout = I2C_TIMEOUT;
    while ((UCB0CTL1 & UCTXSTP) && --timeout); // Avoid infinite wait
    return 0;
}

/* --------------------------------------------------
Function: I2C_ReadRegister

Author: Sinclair
Version: 2.3.0
Purpose: Read one or more consecutive bytes from a sensor register.
Parameters:
addr  - Sensor I2C address
reg   - First register address
bytes - Destination array for received bytes
count - Number of bytes to read (must be at least one)
Returns: 1 = successful, 0 = NACK, timeout or busy bus
-------------------------------------------------- */
unsigned char I2C_ReadRegister(uint8_t addr, uint8_t reg,
                               uint8_t *bytes, uint8_t count)
{
    uint8_t i;
    unsigned long timeout = I2C_TIMEOUT;

    if (count == 0 || bytes == 0) return 0; // Reject invalid read request
    while (UCB0STAT & UCBBUSY)         // Wait until the bus is idle
    {
        if (--timeout == 0) return 0;
    }

    UCB0I2CSA = addr;                  // Select the sensor
    UCB0IFG &= ~UCNACKIFG;            // Clear previous NACK
    UCB0CTL1 |= UCTR | UCTXSTT;       // START in transmit mode

    timeout = I2C_TIMEOUT;
    while (!(UCB0IFG & UCTXIFG))       // Wait to transmit register address
    {
        if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
    }
    if (UCB0IFG & UCNACKIFG) goto fail;
    UCB0TXBUF = reg;                  // Select the register to read

    timeout = I2C_TIMEOUT;
    while (!(UCB0IFG & UCTXIFG))       // Wait until address byte is sent
    {
        if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
    }
    if (UCB0IFG & UCNACKIFG) goto fail;

    UCB0CTL1 &= ~UCTR;                // Switch to receive mode
    UCB0CTL1 |= UCTXSTT;              // Send repeated START to read data

    timeout = I2C_TIMEOUT;
    while (UCB0CTL1 & UCTXSTT)         // Wait for repeated START to complete
    {
        if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
    }
    if (UCB0IFG & UCNACKIFG) goto fail;

    if (count == 1) UCB0CTL1 |= UCTXSTP; // Single byte: NACK/STOP before reading

    for (i = 0; i < count; i++)
    {
        timeout = I2C_TIMEOUT;
        while (!(UCB0IFG & UCRXIFG))   // Wait for the next 8-bit byte
        {
            if ((UCB0IFG & UCNACKIFG) || --timeout == 0) goto fail;
        }
        bytes[i] = UCB0RXBUF;         // Copy received byte into output array

        if (count > 1 && i == count - 2)
            UCB0CTL1 |= UCTXSTP;      // NACK final byte and generate STOP
    }

    timeout = I2C_TIMEOUT;
    while (UCB0CTL1 & UCTXSTP)         // Wait for STOP to complete
    {
        if (--timeout == 0) return 0;
    }
    return 1;

fail:
    UCB0CTL1 |= UCTXSTP;              // Attempt to end failed transfer
    timeout = I2C_TIMEOUT;
    while ((UCB0CTL1 & UCTXSTP) && --timeout); // Do not hang on error
    return 0;
}

/* --------------------------------------------------
Function: INA219_Init

Author: Arthur
Version: 2.5.0
Purpose: Configure 128x bus/shunt ADC averaging in continuous mode.
Parameters: None
Returns: 1 = successful, 0 = communication failure
-------------------------------------------------- */
unsigned char INA219_Init(void)
{
    uint16_t config = 0x3FFF;          // 128x averaging, continuous mode
    uint8_t data[2];

    data[0] = (uint8_t)(config >> 8);  // Extract high byte (0x3F)
    data[1] = (uint8_t)(config & 0xFF); // Extract low byte (0xFF)

    return I2C_WriteRegister(INA219_ADDR, INA_CONFIG_REG, data, 2);
}

/* --------------------------------------------------
INA219 - shunt register 0x01

Author: Arthur
Version: 2.1.0
Responsibilities:
- Convert the shunt register to tenths of milliamps.
For an R100 (0.1 ohm) shunt, each raw count = 0.1 mA.
-------------------------------------------------- */
unsigned char INA219_ReadCurrent(int16_t *current_x10_mA)
{
    uint8_t data[2];
    if (!I2C_ReadRegister(INA219_ADDR, INA_SHUNT_REG, data, 2))
        return 0;
    *current_x10_mA = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    return 1;
}

/* --------------------------------------------------
MCP9700

Author: Iaian
Version: 2.1.0
Responsibilities:
- Configure ADC12 A0 on P6.0 and calculate the temperature using the average of 16 sampled reading

Formulas:
- Vmv = ADC reading * Vref(3.3v)) /4095 
- VOUT ~= 500mV + 10mV/deg C. (According to specsheet) 
-------------------------------------------------- */
void ADC_Init(void)
{
    P6SEL |= BIT0;                          // Select peripheral function
    ADC12CTL0 = ADC12SHT0_8 | ADC12ON;      // Configure sample and hold time & turn on ADC12 module
    ADC12CTL1 = ADC12SHP;                   // Select sampling timer
    ADC12CTL2 = ADC12RES_2;                 // Set resolution to 12bits (0-4095)
    ADC12MCTL0 = ADC12INCH_0 | ADC12SREF_0; // Select A0 channel that is connected to P6.0 and select AVCC & AVSS as reference voltage
    ADC12CTL0 |= ADC12ENC;                  // Enable ADC conversion
}

long ReadTemp(void)
{
    int i;
    unsigned int average = 16;
    unsigned long total = 0;

    //Start ADC conversion and take 16 ADC reading adding them together
    for (i = 0; i < average; i++){
        ADC12IFG &= ~BIT0;           // Clear ADC12 interrupt flag
        ADC12CTL0 |= ADC12SC;        // Start converting MCP9700 analog voltage to digital value
        while (!(ADC12IFG & BIT0));  // Wait until conversion is finished
        total += ADC12MEM0;          // Read and store the ADC reading
        __delay_cycles(1000);
    }
    
    //Calculate the average ADC value
    unsigned long adc = total / average;

    //Convert ADC results into milivolts 
    unsigned long voltage_mV = (adc * 3300)/4095L;

    //Convert milivolts to temperature in 1dp C
    return voltage_mV - 500L;     
}

/* --------------------------------------------------
ADXL345
Author: Sinclair
Version: 2.1.0
Responsibilities:
- Configure accelerometer, read FIFO and calculate RMS/impact data.: 200 Hz, full-resolution +/-4 g, stream FIFO.
At 200 Hz: 128 samples ~= 0.64 s. FIFO read in batches of 8.
-------------------------------------------------- */
unsigned char ADXL345_Init(void)
{
    uint8_t device_id;
    uint8_t value;                    // ADXL345 registers use one data byte

    if (!I2C_ReadRegister(ADXL_ADDR, ADXL_DEVID, &device_id, 1)) return 0;
    if (device_id != 0xE5) return 0;  // Verify sensor identity

    value = 0x00; // Put ADXL345 in standby before changing settings
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_POWER_CTL, &value, 1)) return 0;

    value = 0x0B; // Set output data rate to 200 Hz
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_BW_RATE, &value, 1)) return 0;

    value = 0x09; // Full resolution, +/-4 g measurement range
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_DATA_FORMAT, &value, 1)) return 0;

    value = 0x00; // Clear FIFO before enabling stream mode
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_FIFO_CTL, &value, 1)) return 0;

    value = 0x80; // Set FIFO to stream mode
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_FIFO_CTL, &value, 1)) return 0;

    value = 0x08; // Enable accelerometer measurements
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_POWER_CTL, &value, 1)) return 0;

    return 1;
}

unsigned char ADXL345_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    if (!I2C_ReadRegister(ADXL_ADDR, ADXL_DATAX0, data, 6))
        return 0;
    *x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    *y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    *z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    return 1;
}

/* Integer square root */
uint32_t IntegerSqrt(uint64_t n)
{
    uint64_t result = 0;
    uint64_t bit = (uint64_t)1 << 62;
    while (bit > n)
        bit >>= 2;
    while (bit) {
        if (n >= result + bit) {
            n -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)result;
}

/* --------------------------------------------------
Function: CalculateVibrationRMS

Purpose:
Calculate vibration RMS by subtracting the mean
acceleration from every sample.
-------------------------------------------------- */
void CalculateVibrationRMS(const int16_t x[], const int16_t y[],
                           const int16_t z[],
                           uint32_t *xRms, uint32_t *yRms,
                           uint32_t *zRms, uint32_t *totalRms)
{
    float meanX = 0, meanY = 0, meanZ = 0;
    float sumSqX = 0, sumSqY = 0, sumSqZ = 0;
    float dx, dy, dz;
    float rmsX, rmsY, rmsZ;
    unsigned int i;

    // Calculate mean acceleration
    for (i = 0; i < SAMPLES; i++) {
        meanX += x[i];
        meanY += y[i];
        meanZ += z[i];
    }

    meanX /= SAMPLES;
    meanY /= SAMPLES;
    meanZ /= SAMPLES;

    // Subtract mean and square differences
    for (i = 0; i < SAMPLES; i++) {
        dx = x[i] - meanX;
        dy = y[i] - meanY;
        dz = z[i] - meanZ;

        sumSqX += dx * dx;
        sumSqY += dy * dy;
        sumSqZ += dz * dz;
    }

    // Convert from ADXL345 raw counts to 0.1 mg units
    rmsX = sqrtf(sumSqX / SAMPLES) * 39.0f;
    rmsY = sqrtf(sumSqY / SAMPLES) * 39.0f;
    rmsZ = sqrtf(sumSqZ / SAMPLES) * 39.0f;

    *xRms = (uint32_t)(rmsX + 0.5f);
    *yRms = (uint32_t)(rmsY + 0.5f);
    *zRms = (uint32_t)(rmsZ + 0.5f);
    *totalRms = (uint32_t)(sqrtf(rmsX * rmsX + rmsY * rmsY + rmsZ * rmsZ) + 0.5f);
}

/* RMS and impact peak values use 0.1 mg units. */
unsigned char MeasureVibration(uint32_t *xRms, uint32_t *yRms,
                               uint32_t *zRms, uint32_t *totalRms,
                               uint32_t *peakToPeak)
{
    unsigned int collected = 0;
    unsigned int j;
    unsigned int poll;
    uint8_t fifo_status;
    uint8_t entries;
    int16_t x, y, z;
    int16_t minX = 32767, minY = 32767, minZ = 32767;
    int16_t maxX = -32768, maxY = -32768, maxZ = -32768;
    int16_t xSamples[SAMPLES], ySamples[SAMPLES], zSamples[SAMPLES];
    uint32_t peakX, peakY, peakZ;
    uint8_t fifoConfig;

    /* Reset FIFO so window contains fresh samples. */
    fifoConfig = 0x00; // Disable FIFO to clear previous measurements
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_FIFO_CTL, &fifoConfig, 1)) return 0;
    fifoConfig = 0x80; // Re-enable FIFO in stream mode
    if (!I2C_WriteRegister(ADXL_ADDR, ADXL_FIFO_CTL, &fifoConfig, 1)) return 0;

    while (collected < SAMPLES) {
        /* The 200 Hz sensor generates 8 samples in about 40 ms. */
        for (poll = 0; poll < 100; poll++) {
            if (!I2C_ReadRegister(ADXL_ADDR, ADXL_FIFO_STATUS,
                                  &fifo_status, 1)) return 0;
            entries = (uint8_t)(fifo_status & 0x3F);
            if (entries >= FIFO_BATCH)
                break;
            __delay_cycles(5000);
        }
        if (poll == 100 || entries >= 31)
            return 0; /* timed out or FIFO dangerously full */

        for (j = 0; j < FIFO_BATCH; j++) {
            if (!ADXL345_ReadXYZ(&x, &y, &z))
                return 0;
            xSamples[collected] = x;
            ySamples[collected] = y;
            zSamples[collected] = z;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
            collected++;
        }
    }

    CalculateVibrationRMS(xSamples, ySamples, zSamples,
                          xRms, yRms, zRms, totalRms);

    /* Largest single-axis peak-to-peak, used ONLY for impact detection. */
    peakX = (uint32_t)((int32_t)maxX - minX) * 39UL;
    peakY = (uint32_t)((int32_t)maxY - minY) * 39UL;
    peakZ = (uint32_t)((int32_t)maxZ - minZ) * 39UL;
    *peakToPeak = peakX;
    if (peakY > *peakToPeak) *peakToPeak = peakY;
    if (peakZ > *peakToPeak) *peakToPeak = peakZ;
    return 1;
}

/* --------------------------------------------------
Function: StopWithError
Author: Julius
Version: 2.1.0

Purpose:
Send a structured error message through UART A0 to the ESP32,
then stop monitoring so invalid measurements are not transmitted.
Parameters:
message - Text explaining the failure (must not contain newline).
Returns: None; function does not return.
-------------------------------------------------- */
void StopWithError(const char *message)
{
    ESP_UART_Print("ERROR,");                /* Identify a terminal failure */
    ESP_UART_Print(message);                  /* Report the failing operation */
    ESP_UART_Print(",I2C_");
    ESP_UART_Print((UCB0STAT & UCBBUSY) ? "BUSY" : "IDLE");
    ESP_UART_Print("\r\n");
    while (1) { /* Stop to prevent reporting invalid readings */ }
}


#define WITHIN_BASELINE 0U
#define ABOVE_CHECKING 1U
#define ABOVE_NORMAL 2U
#define BELOW_CHECKING 3U
#define BELOW_NORMAL 4U
#define NO_FAULT 0U
#define FAULT_CHECKING 1U
#define FAN_OBSTRUCTION 2U
#define FAN_STOPPED 3U
#define MECHANICAL_LOOSENESS 4U
#define HEALTHY 0U
#define WEAK 1U

static unsigned int statusCode, impactEvent, currentStatus, tempStatus, machineStatus, overallHealth;
static uint32_t baselineRms, lowerLimit, upperLimit, impactThreshold;
static long baselineCurrent_x10, currentLower, currentUpper;
static long baselineTemp_x10, tempUpper;
static int vibrationCount = 0, currentCount = 0, tempCount = 0;
static unsigned char impactArmed = 1;
static unsigned int machineCandidate = 0, machineCount = 0;

/* --------------------------------------------------
Function: Calibration_Run
Author: Sinclair
Version: 3.0.0
Responsibilities:
- Measure the healthy fan and calculate all warning limits
- Reset warning counters when a new baseline is established
Parameters: None
Returns: None (halts on communication failure)
-------------------------------------------------- */
void Calibration_Run(void)
{
    unsigned int i;
    int16_t current_x10;
    long currentSum = 0, tempSum = 0, currentTolerance;
    uint32_t xRms, yRms, zRms, rms, peak;
    uint32_t rmsSum = 0, minRms = 0, maxRms = 0, maxPeak = 0;
    uint32_t tolerance, spread;

    __delay_cycles(2000000); // Allow the fan speed to settle

    for (i = 0; i < CAL_WINDOWS; i++) {
        if (!MeasureVibration(&xRms, &yRms, &zRms, &rms, &peak))
            StopWithError("ADXL345 FIFO/vibration calibration failed");
        if (!INA219_ReadCurrent(&current_x10))
            StopWithError("INA219 read failed during calibration");

        rmsSum += rms;                 // Add healthy vibration readings
        currentSum += current_x10;     // Add healthy current readings
        tempSum += ReadTemp();         // Add healthy temperature readings

        // Store first RMS reading as minimum and maximum
        if (i == 0) {
            minRms = rms;
            maxRms = rms;
        } else {
            if (rms < minRms) minRms = rms;
            if (rms > maxRms) maxRms = rms;
        }
        if (peak > maxPeak) maxPeak = peak; // Track largest peak in every window
        ESP_UART_SendCalWindow(i + 1, rms);
    }

    // Calculate average healthy readings
    baselineRms = rmsSum / CAL_WINDOWS;
    baselineCurrent_x10 = currentSum / (long)CAL_WINDOWS;
    baselineTemp_x10 = tempSum / (long)CAL_WINDOWS;

    // Prevent calibration with the fan stopped
    if (baselineCurrent_x10 < 100L)
        StopWithError("Calibration requires positive fan current above 10 mA");

    // Calculate upper and lower limit
    tolerance = baselineRms * RMS_TOL_PCT / 100UL;
    lowerLimit = (baselineRms > tolerance) ? baselineRms - tolerance : 0;
    upperLimit = baselineRms + tolerance;

    // Current: allow +-10% of average current, or at least 10 mA
    currentTolerance = baselineCurrent_x10 * CURRENT_TOL_PCT / 100L;
    if (currentTolerance < CURRENT_TOL_MIN)
        currentTolerance = CURRENT_TOL_MIN;
    currentLower = baselineCurrent_x10 - currentTolerance;
    if (currentLower < 0) currentLower = 0;
    currentUpper = baselineCurrent_x10 + currentTolerance;

    // Temperature: detect a rise of 5 C above the calibrated temperature
    tempUpper = baselineTemp_x10 + TEMP_RISE_LIMIT;

    // Impact: use the existing peak-to-peak threshold
    impactThreshold = maxPeak * IMPACT_MULT;
    if (impactThreshold < IMPACT_MIN) impactThreshold = IMPACT_MIN;

    // Clear previous warning history after calibration
    vibrationCount = 0;
    currentCount = 0;
    tempCount = 0;
    machineCandidate = 0;
    machineCount = 0;
    impactArmed = 1;

    ESP_UART_SendBaseline(baselineRms, lowerLimit, upperLimit);
    // Report additional calibration limits in tenths of mA and C
    ESP_UART_Print("LIMITS,");
    ESP_UART_Signed(baselineCurrent_x10); ESP_UART_Char(',');
    ESP_UART_Signed(currentLower); ESP_UART_Char(',');
    ESP_UART_Signed(currentUpper); ESP_UART_Char(',');
    ESP_UART_Signed(baselineTemp_x10); ESP_UART_Char(',');
    ESP_UART_Signed(tempUpper); ESP_UART_Print("\r\n");
}

/* --------------------------------------------------
Function: Warning_Check
Author: Arthur
Version: 3.0.0
Responsibilities:
- Check individual and combined sensor conditions
- Confirm sustained abnormalities over consecutive windows
- Detect impacts immediately and determine overall health
Parameters: current and temperature in tenths; rms and peak in 0.1 mg
Returns: None; updates warning results for UART transmission
-------------------------------------------------- */
void Warning_Check(int16_t current_x10, long temperature_x10,
                   uint32_t rms, uint32_t peak)
{
    unsigned char lowRms = (rms < lowerLimit);
    unsigned char highRms = (rms > upperLimit);
    unsigned char lowCurrent = (current_x10 < currentLower);
    unsigned char highCurrent = (current_x10 > currentUpper);
    unsigned char normalCurrent = (!lowCurrent && !highCurrent);
    unsigned char zeroCurrent = (current_x10 >= -CURRENT_STOPPED &&
                                 current_x10 <= CURRENT_STOPPED);
    unsigned char highTemp = (temperature_x10 > tempUpper);
    unsigned int candidate = NO_FAULT;

    overallHealth = HEALTHY;
    currentStatus = WITHIN_BASELINE;
    statusCode = WITHIN_BASELINE;
    tempStatus = WITHIN_BASELINE;
    machineStatus = NO_FAULT;
    impactEvent = 0;

    // Current: confirm consecutive abnormal readings in the same direction
    if (highCurrent) {
        if (currentCount < 0) currentCount = 0;
        if (currentCount < (int)ALERT_WINDOWS) currentCount++;
        currentStatus = ABOVE_CHECKING;
        if (currentCount >= (int)ALERT_WINDOWS) {
            currentStatus = ABOVE_NORMAL;
            overallHealth = WEAK;
        }
    } else if (lowCurrent) {
        if (currentCount > 0) currentCount = 0;
        if (currentCount > -(int)ALERT_WINDOWS) currentCount--;
        currentStatus = BELOW_CHECKING;
        if (currentCount <= -(int)ALERT_WINDOWS) {
            currentStatus = BELOW_NORMAL;
            overallHealth = WEAK;
        }
    } else {
        currentCount = 0;
    }

    // Vibration: confirm consecutive abnormal readings in the same direction
    if (highRms) {
        if (vibrationCount < 0) vibrationCount = 0;
        if (vibrationCount < (int)ALERT_WINDOWS) vibrationCount++;
        statusCode = ABOVE_CHECKING;
        if (vibrationCount >= (int)ALERT_WINDOWS) {
            statusCode = ABOVE_NORMAL;
            overallHealth = WEAK;
        }
    } else if (lowRms) {
        if (vibrationCount > 0) vibrationCount = 0;
        if (vibrationCount > -(int)ALERT_WINDOWS) vibrationCount--;
        statusCode = BELOW_CHECKING;
        if (vibrationCount <= -(int)ALERT_WINDOWS) {
            statusCode = BELOW_NORMAL;
            overallHealth = WEAK;
        }
    } else {
        vibrationCount = 0;
    }

    // Combined current and vibration conditions
    if (lowRms && current_x10 >= CURRENT_OBSTRUCTION) {
        candidate = FAN_OBSTRUCTION;
    } else if (lowRms && zeroCurrent) {
        candidate = FAN_STOPPED;
    } else if (highRms && normalCurrent) {
        candidate = MECHANICAL_LOOSENESS;
    }

    // Confirm the same combined condition for consecutive windows
    if (candidate == NO_FAULT) {
        machineCandidate = NO_FAULT;
        machineCount = 0;
    } else {
        if (candidate != machineCandidate) {
            machineCandidate = candidate;
            machineCount = 0;
        }
        if (machineCount < ALERT_WINDOWS) machineCount++;
        machineStatus = FAULT_CHECKING;
        if (machineCount >= ALERT_WINDOWS) {
            machineStatus = candidate;
            overallHealth = WEAK;
        }
    }

    // Independent temperature warning
    if (highTemp) {
        if (tempCount < (int)ALERT_WINDOWS) tempCount++;
        tempStatus = ABOVE_CHECKING;
        if (tempCount >= (int)ALERT_WINDOWS) {
            tempStatus = ABOVE_NORMAL;
            overallHealth = WEAK;
        }
    } else {
        tempCount = 0;
    }

    // Report sudden impacts immediately; rearm after vibration settles
    if (peak > impactThreshold && impactArmed) {
        impactEvent = 1;
        impactArmed = 0;
        overallHealth = WEAK;
    }
    if (peak < impactThreshold / 2UL) impactArmed = 1;
}

/* --------------------------------------------------
Timer_A0 and S1 Recalibration Button

Author: Julius
Version: 2.4.0
Responsibilities:
- Use Timer_A0 to count 50 ms intervals using ACLK
- Schedule sensor monitoring approximately every 1 second
- Check button S1 on P2.1 after a 50 ms debounce interval
- Request recalibration without running sensor code inside an ISR
- Wake the MSP430 from LPM0 when work is ready
-------------------------------------------------- */

volatile unsigned char monitorDue = 0;          // Timer requests a sensor update
volatile unsigned char recalibrateDue = 0;      // S1 requests a new baseline
volatile unsigned char buttonChecking = 0;      // S1 is being debounced
volatile unsigned char buttonHeld = 0;          // Prevent repeat presses while held
volatile unsigned int monitorTicks = 0;         // Number of elapsed 50 ms ticks

/* --------------------------------------------------
Function: Timer_Init

Author: Iaian
Version: 2.4.0
Purpose: Configure Timer_A0 to interrupt approximately every 50 ms.
Parameters: None
Returns: None
-------------------------------------------------- */
void Timer_Init(void)
{
    TA0CTL = TASSEL_1 | MC_0 | TACLR;       // Select ACLK and stop/reset timer
    TA0CCR0 = TIMER_50MS_COUNTS - 1U;       // Set interval to about 50 ms
    TA0CCTL0 = CCIE;                        // Enable Timer_A0 CCR0 interrupt
    TA0CTL = TASSEL_1 | MC_1 | TACLR;       // Start Timer_A0 in up mode
}

/* --------------------------------------------------
Function: Button_Init

Author: Iaian
Version: 2.4.0
Purpose: Configure the LaunchPad's built-in S1 (P2.1) as a button interrupt.
Parameters: None
Returns: None
-------------------------------------------------- */
void Button_Init(void)
{
    P2SEL &= ~BIT1;       // Select GPIO function on P2.1
    P2DIR &= ~BIT1;       // Set P2.1 as an input
    P2REN |= BIT1;        // Enable pull resistor
    P2OUT |= BIT1;        // Select pull-up: button press reads LOW
    P2IES |= BIT1;        // Detect falling edge when S1 is pressed
    P2IFG &= ~BIT1;       // Clear previous button interrupt flag
    P2IE |= BIT1;         // Enable S1 interrupt
}

/* --------------------------------------------------
Function: Timer_CheckButton

Author: Iaian
Version: 2.4.0
Purpose: Debounce S1 and wait for its release before accepting a new press.
Parameters: None
Returns: None
Called by: Timer_A0 ISR, once per 50 ms tick
-------------------------------------------------- */
void Timer_CheckButton(void)
{
    if (buttonChecking)
    {
        buttonChecking = 0;               // 50 ms has passed since S1 interrupt
        if (!(P2IN & BIT1))               // Check S1 is still pressed
        {
            recalibrateDue = 1;          // Request calibration in main(), not ISR
            buttonHeld = 1;              // Do not retrigger while S1 is held
        }
        else
        {
            P2IFG &= ~BIT1;              // Clear switch bounce flag
            P2IE |= BIT1;                // False press: accept a new press
        }
    }
    else if (buttonHeld && (P2IN & BIT1))  // Check that S1 has been released
    {
        buttonHeld = 0;                  // Button is ready for another press
        P2IFG &= ~BIT1;                   // Clear any old button event
        P2IE |= BIT1;                     // Enable next S1 interrupt
    }
}

/* --------------------------------------------------
ISR: Port2_ISR

Author: Julius
Version: 2.4.0
Purpose: Detect the built-in S1 button and start its debounce check.
-------------------------------------------------- */
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    if (P2IFG & BIT1)
    {
        P2IFG &= ~BIT1;        // Clear S1 interrupt flag
        P2IE &= ~BIT1;         // Disable S1 interrupts during debounce
        buttonChecking = 1;    // Timer will confirm the press after 50 ms
    }
}

/* --------------------------------------------------
ISR: Timer_A0_ISR

Author: Julius
Version: 2.4.0
Purpose: Schedule sensor updates and check button debounce every 50 ms.
-------------------------------------------------- */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    Timer_CheckButton();                     // Handle S1 debounce and release
    monitorTicks++;                         // Count elapsed 50 ms intervals

    if (monitorTicks >= MONITOR_TICKS)
    {
        monitorTicks = 0;                   // Start the next 1-second interval
        monitorDue = 1;                     // Request monitoring in main()
    }

    if (monitorDue || recalibrateDue)
        __bic_SR_register_on_exit(LPM0_bits); // Wake CPU only when work is due
}

/* --------------------------------------------------
Main Program
Author: Everyone
Version: 2.2.0

Responsibilities:
-Initialize shared MSP430 peripherals and sensors.
- Recover I2C bus when required.
- Run healthy-state calibration.
- Acquire measurements and evaluate warnings.
- Send only structured HELLO, CAL, BASE, DATA and ERROR lines to ESP32.
-------------------------------------------------- */
int main(void)
{
    int16_t current_x10;
    long temperature_x10;
    uint32_t xRms, yRms, zRms, rms, peak;

    WDTCTL = WDTPW | WDTHOLD;
    ESP_UART_Init();
    ADC_Init();
    __delay_cycles(300000); /* Sensor power supply settling time */
    I2C_Init();

    if (UCB0STAT & UCBBUSY) {
        if (!I2C_RecoverBus())
            StopWithError("I2C startup recovery failed");
    }

    if (!INA219_Init()) {
        if (!I2C_RecoverBus())
            StopWithError("I2C recovery after INA219 init failed");
        if (!INA219_Init())
            StopWithError("INA219 128x init failed on retry");
    }

    ESP_UART_Print("HELLO,MSP430F5529\r\n");

    if (UCB0STAT & UCBBUSY)
        StopWithError("I2C bus already BUSY at startup");
    if (!ADXL345_Init()) {
        if (!I2C_RecoverBus() || !ADXL345_Init())
            StopWithError("ADXL345 init/recovery failed");
    }
    if (!INA219_ReadCurrent(&current_x10))
        StopWithError("INA219 read failed");

    Button_Init();                   // Built-in S1 (P2.1) requests recalibration
    Timer_Init();                    // Timer_A0 provides 50 ms scheduling ticks
    __enable_interrupt();           // Enable the button and timer interrupts

    Calibration_Run();              // Collect initial healthy baseline

    __disable_interrupt();
    recalibrateDue = 0;             // Ignore button presses during startup calibration
    monitorDue = 1;                // Take the first monitoring measurement now
    monitorTicks = 0;              // Restart timing after initial calibration
    __enable_interrupt();

    while (1)
    {
        if (recalibrateDue)
        {
            __disable_interrupt();
            recalibrateDue = 0;     // Accept one calibration request
            monitorDue = 0;        // Skip an outdated monitoring request
            __enable_interrupt();

            Calibration_Run();       // Run calibration in main, not inside ISR
        }
        else if (monitorDue)
        {
            __disable_interrupt();
            monitorDue = 0;         // Clear the scheduled measurement request
            __enable_interrupt();

            if (!MeasureVibration(&xRms, &yRms, &zRms, &rms, &peak))
                StopWithError("ADXL345 vibration measurement failed");
            if (!INA219_ReadCurrent(&current_x10))
                StopWithError("INA219 current measurement failed");
            temperature_x10 = ReadTemp();

            Warning_Check(current_x10, temperature_x10, rms, peak);
            ESP_UART_SendData(current_x10, temperature_x10,
                              xRms, yRms, zRms, rms, statusCode, impactEvent,
                              currentStatus, tempStatus, machineStatus, overallHealth);
        }

        /* Sleep only when no new button or timer request is pending. */
        __disable_interrupt();
        if (!monitorDue && !recalibrateDue)
            __bis_SR_register(LPM0_bits | GIE); // Sleep atomically until ISR wakes us
        else
            __enable_interrupt();               // Process pending work without sleeping
    }
}
