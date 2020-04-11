#include "mbed.h"
 
// Read temperature from TMP275

// I2C communication initialization with corresponded I2C pin names (see PinNames.h of the type of the board)
I2C i2c(I2C_SDA , I2C_SCL );
// Initialization of LED from a Nucleo board
// LED1 is the address of the led from F303RE (see PinNames.h of your board)
DigitalOut myled(LED1);
// USER_BUTTON is the address of the led from F303RE (see PinNames.h of your board)
DigitalIn mybutton(USER_BUTTON);
// To print readings from sensor via USB easily. UART pin names are required (see PinNames.h)
Serial pc(SERIAL_TX, SERIAL_RX);

// 7 LSbits where slave address is allocated (8th bit of uint8_t is 0)
static const uint8_t TMP275_ADDR = 0x48;
// 7 MSbits where slave address and 1 LSbit for reading are allocated 
static const uint8_t TMP275_R_ADDR = TMP275_ADDR << 1;
// 7 MSbits where slave address and 1 LSbit for writing are allocated 
static const uint8_t TMP275_W_ADDR = TMP275_R_ADDR | 1;
// Pointer regiter addresses
static const uint8_t REG_TEMP = 0x00; // Temperature Register Address
static const uint8_t REG_CONF = 0x01; // Configuraton Register Address

int main(){
    // Buffer where data package will be stored
    char buf[2];
    // 2 bytes where temperature readings will be allocated properly
    int16_t val;
    // Temperature in degrees
    float temp_c;
    // Acknowledgement between board and sensor communication
    int ack;
    // Time in seconds between readings
    int8_t time = 1;
    
    // Make sure which are the reading and writing frames to start a reading or writing procedure
    pc.printf("Reading frame: 0x%x\n\r", TMP275_R_ADDR);
    pc.printf("Writing frame: 0x%x\n\r", TMP275_W_ADDR);
    
    // ----------------------------------------
    // ---------Configuration Register---------
    // ----------------------------------------
    
    // Now pointer register will be as configuration register:
    // Pointer address byte is stored in the LSbyte of the buffer
    buf[0] = REG_CONF;
    pc.printf("Current pointer address: 0x%02x\n\r", buf[0]);
    // Set the pointer address byte as a configuration register address byte
    ack = i2c.write(TMP275_W_ADDR, &buf[0], 1);
    
    // Now the current configuration will be read:
    // Configuration register format will be stored in the MSbyte of the buffer
    //
    // Summary about what is the meaning of each bit of this byte:
    // Bit0: ShutDown Mode SD: allows saving maximum power
    // Bit1: Thermostat Mode TM: Comparator or Interrupt mode
    // Bit2: Polarity P: ALERT pin output
    // Bit3: Fault Queue F0: Consecutive faults
    // Bit4: Fault Queue F1: Consecutive faults
    // Bit5: Converter Resolution R0: Resolution and converstion time
    // Bit6: Converter Resolution R1: Resolution and conversion time
    // Bit7: One-Shot OS: Single temperature conversion
    // "More details in the TMP275 Datasheet"
    ack = i2c.read(TMP275_R_ADDR, &buf[1], 1);
    pc.printf("Current configuration of the sensor: 0x%02x, ACK: %d\n\r", buf[1], ack);
    
    // Here the configuration frame will be changed 
    // (or not, since it is depending on the current configuration read before)
    buf[1] = 0x60;
    // When master(board) is writing, it always needed to include a poiter address before configuration frame
    // Address+Writing+P.Register+Conf.Format: These bits are sent by the board to TMP275,
    // and an Acknowledge frame is sent by TMP275 to the board
    ack = i2c.write(TMP275_W_ADDR, buf, 2);
    
    // To make sure the configuration once changes are done:
    // Read the configuration, and pointer register is not needed in this case
    // Address+Reading+Conf.Format: Thses bits are sent by the board to TMP275,
    // Configuration frame will be allocated in the MSbyte of the buffer,
    // and Acknowledge frame is sent by TMP275 to the board
    ack = i2c.read(TMP275_R_ADDR, &buf[1], 1);
    pc.printf("Configuration of the sensor after changes: 0x%02x, ACK: %d\n\r", buf[1], ack);
    
    // ----------------------------------------
    // ----------Temperature Register----------
    // ----------------------------------------
    // Now pointer register will be as temperature register:
    buf[0] = REG_TEMP;
    pc.printf("Current pointer address: 0x%02x\n\r", buf[0]);
    ack = i2c.write(TMP275_W_ADDR, &buf[0], 1);
    pc.printf("Reply of the writing: 0x%02x\n\r", buf[0]);

    // If we want to stop this program, press the USER_BUTTON
    while(mybutton){
        if(!ack){
            // Here temperature readings
            // Temperature will be stored in 2 bytes.
            // So, a shifting and a combination of both will be needed to know temperature value.
            ack = i2c.read( TMP275_R_ADDR, buf, 2);
            // LSbyte: All bits have information about a part of the reading.
            // But we need to free up space for information of the MSbyte.
            pc.printf("LSbyte: 0x%x\n\r", buf[0]);
            pc.printf("LSbyte: 0x%x\n\r", buf[0] << 4);
            // MSbyte: The 4 LSbits are not useful, therefore they will be removed
            // by a shifting of 4 MSbits from left to right
            pc.printf("MSbyte: 0x%x\n\r", buf[1]);
            pc.printf("MSbyte: 0x%x\n\r", buf[1] >> 4);
            
            // Shifting and combining
            val = (buf[0]) << 4 | (buf[1] >> 4);
            //As a result of shifting and combining both bytes, we will get: 
            pc.printf("Val: 0x%x\n\r", val);
    
            // Negative temperatures
            // 0xE70 represents -25 degrees
            //val = 0xE70; // To check if it works uncomment the left side of this line.
            if ( val > 0x7FF ){
                // Convert to 2's complement
                val |= 0xF000; // This solution works just for 16 bits
                // For 32 bits ==> val |= FFFFF000
                // Don't forget to change the type of variable of "val", if you want to use 32 bits.
                // General solution for any quantity of bytes: val = (~val + 1)*(-1)
                // Now it is a int16_t variable.
            }
        
            // Temperature value in bytes needs to be converted to float temperature value (in Celsius)
            // In this case, as we are using 12 bits of resolution, just a 0.0625 scale factor is required
            // IMPORTANT: To get the maximum of accuracy, that is 0.0625 degrees, 0.0625 scale factor is required, 
            // therefore 12 bits of resolution are required. 
            // However, the more resolution is not involved to the more temperature accuracy.
            // That means that if you are using 12 bits, 0.0625 scale factor is required, 
            // but for example 0.5 degrees of accuracy can be obtained if the sensor is configured.
            // IMPORTANT: So, resolution is related with temperature accuracy. But your sensor have to be configured.
            temp_c = val * 0.0625;
            pc.printf("Temperature: %.2f ºC\n\r", temp_c);
            
            // Signal from board LED when a reading temperature is done
            // When the LED turns on 2 times quickly, it means sensor reads a sample
            myled = !myled;
            wait(0.125);
            myled = !myled;
            wait(0.125);
            myled = !myled;
            wait(0.125);
            myled = !myled;
            wait(time - 3*0.125);
        }
        else {
            pc.printf("Connection is failed\n\r");
        }
        pc.printf("Button: %d\n\r", mybutton.read());
    }
    // Signal LED when program is stopped
    myled = 1;
    wait(2);
    myled = 0;
}