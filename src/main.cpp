#include "mbed.h"
 
// Read temperature from TMP275

// I2C communication initialization. SDA and SCL pins are configured automatically
I2C i2c(I2C_SDA , I2C_SCL );
// Initialization of LED from a Nucleo board
// LED1 is the address of the led from F303RE, see PinNames.h of your board
DigitalOut myled(LED1);
// USER_BUTTON is the address of the led from F303RE, see PinNames.h of your board
DigitalIn mybutton(USER_BUTTON);
// To print readings from sensor via USB easily, PUTTY is required
Serial pc(SERIAL_TX, SERIAL_RX);

// 7 bits where slave address is allocated (8th bit of uint8_t is 0)
static const uint8_t TMP275_ADDR = 0x48;
// 7 bits where slave address and 1 bit for reading are allocated 
static const uint8_t TMP275_R_ADDR = TMP275_ADDR << 1;
// 7 bits where slave address and 1 bit for writing are allocated 
static const uint8_t TMP275_W_ADDR = TMP275_R_ADDR | 1;
// Pointer regiter addresses
static const uint8_t REG_TEMP = 0x00; // Temperature Register Address
static const uint8_t REG_CONF = 0x01; // Configuraton Register Address

int main(){
    char buf[2];
    int val;
    float temp_c;
    int ack;
    
    // Make sure which are the reading and writing frames to start a reading or writing procedure
    pc.printf("Reading frame: 0x%x\n\r", TMP275_R_ADDR);
    pc.printf("Writing frame: 0x%x\n\r", TMP275_W_ADDR);
    
    // ----------------------------------------
    // ---------Configuration Register---------
    // ----------------------------------------
    
    // Now pointer register will be as configuration register:
    // Pointer address byte is stored in the FIRST byte of the buffer
    buf[0] = REG_CONF;
    pc.printf("Current pointer address: 0x%02x\n\r", buf[0]);
    // Set the pointer address byte as a configuration register address byte
    ack = i2c.write(TMP275_W_ADDR, &buf[0], 1);
    
    // Now the current configuration will be read:
    // Configuration register format will be stored in the SECOND byte of the buffer
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
    //
    // Example: If we obtain in buf[1]:
    // buf[1] = Bit7Bit6Bit5Bit4Bit3Bit2Bit1Bit0 = 01100000 = 0x60
    // "More details in the TMP275 Datasheet"
    ack = i2c.read(TMP275_R_ADDR, &buf[1], 1);
    pc.printf("Current configuration of the sensor: 0x%02x, ACK: %d\n\r", buf[1], ack);
    
    // Here the configuration can be changed
    buf[1] = 0x60;
    // When master(board) is writing needs always include poiter address before configuration frame
    // Address+Writing+P.Register+Conf.Format bits are sent by the board to TMP275,
    // and Acknowledge frame is sent by TMP275 to the board
    ack = i2c.write(TMP275_W_ADDR, buf, 2);
    
    // To make sure the configuration once changes are done
    // To read the configuration, pointer register is not needed in this case
    // Address+Reading+Conf.Format bits are sent by the board to TMP275,
    // Configuration frame will be allocated in SECOND frame,
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
    pc.printf("Current pointer addresssssss: 0x%02x\n\r", buf[0]);
    // If we want to stop this program, press the USER_BUTTON
    while(mybutton){
        if(!ack){
            // Here temperature readings
            // Temperature will be stored in 2 bytes. So, they will be combined.
            i2c.read( TMP275_R_ADDR, buf, 2);
            // First byte: All bits have information.
            // But we need to free up space for information of the second byte.
            pc.printf("Byte 0: 0x%x\n\r", buf[0]);
            pc.printf("Byte 0: 0x%x\n\r", buf[0] << 4);
            // Second byte: The 4 lasts bits are not useful
            pc.printf("Byte 1: 0x%x\n\r", buf[1]);
            pc.printf("Byte 1: 0x%x\n\r", buf[1] >> 4);
            //Combine the bytes: 
            val = (buf[0]) << 4 | (buf[1] >> 4);
            pc.printf("Val: %x\n\r", val);
    
            
            // Convert to 2's complement, since temperature can be negative
            //if ( val > 0x7FF ) {
            //  val |= 0xF000;
            //}

            // Convert to float temperature value (Celsius)
            temp_c = val * 0.0625;
            pc.printf("Temperature: %.2f ºC\n\r", temp_c);
            // Convert temperature to decimal format
            //temp_c *= 100;
            //pc.printf("Temperature: %u.%u ºC\n\r", 
            //            (unsigned int)temp_c/100, 
            //            (unsigned int)temp_c%100);
            //pc.printf("\033[6A");
            
            float time = 0.125;
            myled = !myled;
            wait(time);
            myled = !myled;
            wait(time);
            myled = !myled;
            wait(time);
            myled = !myled;
            wait(1-time);
        }
        else {
            pc.printf("Connection is failed\n\r");
            pc.printf("\033[1A");
        }
        pc.printf("Button: %d\n\r", mybutton.read());
    }
}