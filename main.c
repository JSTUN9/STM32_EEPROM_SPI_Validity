#include "main.h"

#include "Time_Delays.h"
#include "Clk_Config.h"
#include "LCD_Display.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"

//Sensor I2C Address
#define ACCELADR 0x98

//EEPROM I2C Address
#define EEPROMADR 0xA0

//GPIO
void configure_gpio(void);

//Joystick Configuration
void joystick_configure(void);
uint32_t joystick_up(void);
uint32_t joystick_down(void);
uint32_t joystick_left(void);
uint32_t joystick_right(void);
uint32_t joystick_centre(void);

//I2C
void i2c_1_configure(void);

//Accelerometer
void config_accelerometer(void);
char read_accelerometer(void);
int read_accelerometer2(void);

//Display functions
void display_tilt(uint8_t sample); //convenience function to display tilt register
void display_tilt2(char* full_packet);
void packet_display(char* array, char i); //display contents of packet after page-mode read
//void read_packet(char* array,char* payload, char i, char count, int destination, int source, uint16_t length);
void read_full_packet_2 (char* full_packet, char count, bool results, char* checksum);

//EEPROM functions
void config_eeprom (void);
void eeprom_write(uint8_t data);
void eeprom_write_page_mode(char* array);
uint8_t eeprom_read(void);
void eeprom_read_page_mode(char* array);
void form_full_packet(char* array, int destination,int source, int length, char* payload, char* checksum);


// checksum
void checksumCount(char* array, char* checksum) ;
bool compareCheckSum(char* array, char* checksum);
	
int main(void){
	//Init
  SystemClock_Config();/* Configure the system clock to 84.0 MHz */
	SysTick_Config_MCE2(us);	
	
	
	//Configure LCD
	Configure_LCD_Pins();
	Configure_SPI1();
	Activate_SPI1();
	Clear_Screen();
	Initialise_LCD_Controller();
	set_font((unsigned char*) Arial_12);
		
	//Configure GPIO
	configure_gpio();
	joystick_configure();
	
	i2c_1_configure(); //Configure I2C and set up the GPIO pins it uses
	config_accelerometer(); // Configure Accelerometer & setup GPIO pins it uses
	config_eeprom(); // Configure EEPROM & setup GPIO pins it uses
	uint8_t sample = 0; //int to store tilt register value

	
	
	//other variables as necessary:
	char full_packet[66] = {0};
	//66-byte array to contain packet
	// this stores an array of bytes, as size of char = 1 byte
	// when input is 2 digit hex, it is autoamtically stored as 1 byte ascII code
	char checksum [2] = {0};
	char payload[54] = {0};
	//int leftCount = 0;
	int packet_disp_count = 0;
	int destination = 0xcccccccc ;
	int source = 0xdddddddd;
	int length = 0x36 ;
	
	
	//Main Loop
	while (1){
		if(joystick_centre()){
			// read accelerometer data to be displayed
			
			sample = read_accelerometer(); //Reads accelerometer sensor
			
			put_string(0,0,"             "); //Report successful data read
			put_string(0,0,"Sampled");
			put_string(0,15,"             ");
			LL_mDelay(500000);
			// delay to stop double press
			
			display_tilt(sample);
			// displays acc value
			
			payload[0] = sample ;
			// stores acce value in payload array
			
			form_full_packet( full_packet, destination, source, length, payload, checksum);
			// forms packet using accelerometer value
			
		} else if(joystick_right()){
			// reads acce value, forms packet then writes to EEPROM
			
			uint8_t sample = read_accelerometer2(); 
			payload[0] = sample ;
			// stores acce value in payload array
			
			form_full_packet( full_packet, destination, source, length, payload, checksum);
			// forms full packet without checksum values
			checksumCount(full_packet,checksum);
			// calculates checksum
			form_full_packet( full_packet, destination, source, length, payload, checksum);
			// adds checksum value back into full packet
			
			
			eeprom_write_page_mode(full_packet); 
			// writes whole packet into EEPROM
			
			put_string(0,0,"             "); //Report successful write
			put_string(0,0,"Written");
			put_string(0,15,"             ");
			LL_mDelay(500000);
			// delay to stop double press
			
			display_tilt2(full_packet);
			// displays acce value ( full_packet[10])
			
			
			// used as check to see if checksum has correct value
			/*
			char outputString[18];
			sprintf(outputString,"%x%x",checksum[1],checksum[0]);
			put_string(0,0,"             "); //Report successful write
			put_string(0,0,"Written");
			put_string(0,15,"             ");
			put_string(0,15,outputString);
			*/

		} else if(joystick_left()){
			// reads packet from EEPROM
			
			// resets full_packet value to show that all values are purely read from EEPROM
			for (int i = 0; i < 66; ++i){
			full_packet[i] = 0;
			}
			
			eeprom_read_page_mode(full_packet); 
			
			put_string(0,0,"             "); //Report successful read
			put_string(0,0,"Retrieved");
			put_string(0,15,"             ");
			
			LL_mDelay(500000);
			// delay to stop double press & To show that we retrieved
			
			display_tilt2(full_packet);

			
			

		} else if(joystick_up()){  //SCROLL UP FOR PAGE MODE
			// scroll through packet 
			bool results ;
			
			checksumCount(full_packet, checksum);
			results = compareCheckSum(full_packet, checksum);
			
			//read_packet(packet, payload, leftCount, packet_disp_count, destination, source, length);
			read_full_packet_2 (full_packet, packet_disp_count, results, checksum);
			
			packet_disp_count += 1;
			if (packet_disp_count > 5) {
				packet_disp_count = 5;
			}
			LL_mDelay(500000);
			// delay to stop double press
 
		} else if(joystick_down()){  //SCROLL DOWN FOR PAGE MODE
			// scroll through packet 
			bool results ;
		
			checksumCount(full_packet, checksum);
			results = compareCheckSum(full_packet, checksum);
			
			//read_packet(packet, payload, leftCount, packet_disp_count, destination, source, length);
			read_full_packet_2 (full_packet, packet_disp_count, results, checksum);
			
			packet_disp_count -= 1;
			if (packet_disp_count < 0) {
				packet_disp_count = 0;
			}
			LL_mDelay(500000);
			// delay to stop double press
		}
		
  }
}

/* A function to display the tilt register value from packet*/

void display_tilt2(char* full_packet){
	full_packet += 10;
	char data_bin[8] = {0};
	char outputString[18];
	for(int j = 0; j < 8; j++){
		data_bin[j] = (*full_packet & (0x80 >> j))>0;
		
	}
	put_string(0,0,"             ");
	put_string(0,0,"Tilt Reg");
	put_string(0,15,"             ");
	sprintf(outputString,"%x%x%x%x%x%x%x%x", data_bin[0], data_bin[1], data_bin[2], data_bin[3], data_bin[4], data_bin[5], data_bin[6], data_bin[7]);
	put_string(0,15,outputString);
}


// function to display tilt register value from uint8
void display_tilt(uint8_t sample){
	char data_bin[8] = {0};
	char outputString[18]; //Buffer to store text in for LCD
	
	//Converts 8 bits into array for display purposes
	for(int j = 0; j < 8; j++){
	        data_bin[j] = (sample & (0x80 >> j)) > 0;
	}
		
	put_string(0,0,"             ");
	put_string(0,0,"Tilt Reg");
	put_string(0,15,"             ");
	sprintf(outputString,"%x%x%x%x%x%x%x%x", data_bin[0], data_bin[1], data_bin[2], data_bin[3], data_bin[4], data_bin[5], data_bin[6], data_bin[7]);
	put_string(0,15,outputString);
}	



/* A function to display either the first or second copy of the tilt register value
   depending on joystick up/down presses */
void packet_display(char* array, char i) {
        //as per display_tilt:
  char data_bin[8] = {0};
	char outputString[18]; //Buffer to store text in for LCD
	char outputStringS[18];
	//Converts 8 bits into array for display purposes
	for(int j = 0; j < 8; j++){
	        data_bin[j] = (array[i] & (0x80 >> j)) > 0;
	}
	
	put_string(0,0,"             ");
	//put_string(0,0,"Tilt Reg");
	sprintf(outputStringS,"%x", i );
	// %x prints the hexadecimal
	put_string(0,0,outputStringS);
	put_string(0,15,"             ");
	sprintf(outputString,"%x%x%x%x%x%x%x%x", data_bin[0], data_bin[1], data_bin[2], data_bin[3], data_bin[4], data_bin[5], data_bin[6], data_bin[7]);
	put_string(0,15,outputString);       

} //end packet_display



void configure_gpio(void){
	//Configures the GPIO pins by enabling the peripherial clocks on the ports uses by the shield
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); 
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); 
}	

void i2c_1_configure(void){
	//Enable I2C1 Clock
	 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  
	// Configure SCL as: Alternate function, High Speed, Open Drain, Pull Up
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
        LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
	
        // Configure SDA as: Alternate, High Speed, Open Drain, Pull Up
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
        LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
  
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  
        LL_I2C_Disable(I2C1);
        LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
        LL_I2C_ConfigSpeed(I2C1, 84000000, 100000, LL_I2C_DUTYCYCLE_2);  //set speed to 100 kHz
        LL_I2C_Enable(I2C1);
}	

void config_accelerometer(void){
        //Sets accelerometer to active mode
        LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

        LL_I2C_TransmitData8(I2C1, ACCELADR); //ADDRESS + WRITE
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x07); //Transmit mode register address
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1, 0x01); //Set device to active mode (writes a 1 to mode register bit 0)
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_GenerateStopCondition(I2C1);       //STOP
}	
void config_eeprom(void){
//Sets eeprom to active mode
        LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

        LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x07); //Transmit mode register address
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1, 0x01); //Set device to active mode (writes a 1 to mode register bit 0)
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_GenerateStopCondition(I2C1);       //STOP
	
}

char read_accelerometer(void){
	int data ;
	LL_I2C_GenerateStartCondition (I2C1); // START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, ACCELADR); // address + write
	while (!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x03); // set pointer register to tilt register
	
	while (!LL_I2C_IsActiveFlag_TXE (I2C1));
	LL_I2C_GenerateStartCondition (I2C1);
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	LL_I2C_TransmitData8 (I2C1, ACCELADR+1); // Address + 1 is to indicate that it is a read
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);
	
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK); // ACK Incoming Data
	while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // sets when received data is copied in received data register, resets when data register is read
	data = LL_I2C_ReceiveData8(I2C1); // data byte
	LL_I2C_GenerateStopCondition(I2C1); // STOP

	return data;
}

int read_accelerometer2(void){
	int data ;
	LL_I2C_GenerateStartCondition (I2C1); // START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, ACCELADR); // address + write
	while (!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x03); // set pointer register to tilt register
	
	while (!LL_I2C_IsActiveFlag_TXE (I2C1));
	LL_I2C_GenerateStartCondition (I2C1);
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	LL_I2C_TransmitData8 (I2C1, ACCELADR+1); // Address + 1 is to indicate that it is a read
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);
	
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK); // ACK Incoming Data
	while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // sets when received data is copied in received data register, resets when data register is read
	data = LL_I2C_ReceiveData8(I2C1); // data byte
	LL_I2C_GenerateStopCondition(I2C1); // STOP

	return data;
}
void joystick_configure(void){
	//This function configures all the GPIO pins that are connected to the joystick on the mbed shield
	
	LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT); 	//set PA4 as Input
	LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);     //set PA4 as NO pull
  
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 	//set PA4 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);     //set PA4 as NO pull	
	
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT); 	//set PA4 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);     //set PA4 as NO pull	
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 	//set PA4 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);     //set PA4 as NO pull	
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); 	//set PA4 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);     //set PA4 as NO pull	
        //repeat the above for (GPIOB, pins 0 and 5) and (GPIOC, pins 0 and 1)

}	

uint32_t joystick_up(void) {
	//Returns 1 if the joystick is pressed up, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4));
}

uint32_t joystick_down(void) {
	//Returns 1 if the joystick is pressed down, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0));
}

uint32_t joystick_left(void) {
	//Returns 1 if the joystick is pressed left, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_1));
}

uint32_t joystick_right(void) {
	//Returns 1 if the joystick is pressed right, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_0));
}
uint32_t joystick_centre(void) {
	//Returns 1 if the joystick is pressed in the centre, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5));
}

void eeprom_write(uint8_t data){
	//Writes the tilt register sample value to the EEPROM
	
	LL_I2C_GenerateStartCondition(I2C1); //START
	while(!LL_I2C_IsActiveFlag_SB(I2C1));
  
	LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1); // clear address flag

	LL_I2C_TransmitData8(I2C1, 0x00); //POINTER HIGH
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x00); //POINTER LOW
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, data); //DATA
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
	
	LL_I2C_GenerateStopCondition(I2C1); //STOP
}

uint8_t eeprom_read(void){
	//Reads the stored tilt register value from the EEPROM
	uint8_t data;
	
	LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

        LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);
	
        LL_I2C_TransmitData8(I2C1, 0x00); //MEMORY POINTER HIGH BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
	LL_I2C_TransmitData8(I2C1, 0x00); //MEMORY POINTER LOW BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
	LL_I2C_GenerateStartCondition(I2C1); //RE-START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));
	
        LL_I2C_TransmitData8(I2C1, EEPROMADR+1); //ADDRESS + READ
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
	data = LL_I2C_ReceiveData8(I2C1);  //DATA BYTE
	LL_I2C_GenerateStopCondition(I2C1); //STOP
	
	return data;
}


//Page mode write
void eeprom_write_page_mode(char* array){
     //write 50 copies of data in page mode (i.e. 32, then the remainder)
	//int count = 0;
	char* array_address;
	char outputString[18];
			int x = 0;


				LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));
        
        LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x00); //POINTER HIGH
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1, 0x00); //POINTER LOW
				// j only has value after page change
				
				// writes first 32 bytes
				for (int i=0; i < 32; i+=1){
				array_address = array + i ;
				
				uint8_t data = *array_address ;
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1,data ); //DATA
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
				}
				
        LL_I2C_GenerateStopCondition(I2C1); //
		
				
				// acknowledge polling
				LL_I2C_ClearFlag_AF(I2C1);
				// we clear the acknowledge failure flag
				put_string(0,15,"           ");
				sprintf(outputString,"%x",x);
				put_string(0,15,outputString);
				//LL_mDelay(500000);
				
				while (1){// permanent loop
				LL_I2C_GenerateStartCondition(I2C1); //START
				while(!LL_I2C_IsActiveFlag_SB(I2C1));
				LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
				while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
				LL_I2C_ClearFlag_ADDR(I2C1);
				LL_mDelay(200);
				if(!LL_I2C_IsActiveFlag_AF(I2C1)){
					// breaks if we receive 0 (no acknowledge failure flag)
				break;
				}
				LL_I2C_ClearFlag_AF(I2C1);
				// we clear the acknowledge failure flag
				}
				
				

				// 2nd page
				// repeats same as above
				LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));
        
        LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x00); //POINTER HIGH
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1, 0x20); //POINTER LOW
				// j only has value after page change
				
				for (int i=0; i < 32 ; i+=1){
				array_address = array+32 + i ;
				uint8_t data = *array_address ;
        while(!LL_I2C_IsActiveFlag_TXE(I2C1)); // wait for PointerLOW transmission complete

        LL_I2C_TransmitData8(I2C1,data ); //DATA
        while(!LL_I2C_IsActiveFlag_TXE(I2C1)); // wait for transmit flag
				}
				
        LL_I2C_GenerateStopCondition(I2C1); //
				
				// acknowledge polling
				
				LL_I2C_ClearFlag_AF(I2C1);
				
				put_string(0,15,"             ");
				sprintf(outputString,"%x",x);
				put_string(0,15,outputString);
				//LL_mDelay(500000);
				
				while (1){
				LL_I2C_GenerateStartCondition(I2C1); //START
				while(!LL_I2C_IsActiveFlag_SB(I2C1));
				LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
				while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
				LL_I2C_ClearFlag_ADDR(I2C1);
				LL_mDelay(5000);
				if(!LL_I2C_IsActiveFlag_AF(I2C1)){
				break;
				}
				LL_I2C_ClearFlag_AF(I2C1);
				}
				
				// 3rd page
				
				LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));
        
        LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x00); //POINTER HIGH
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1, 0x40); //POINTER LOW
				// j only has value after page change
				
				for (int i=0; i < 2 ; i+=1){
				array_address = array+64 + i ;
				//count +=1;
				uint8_t data = *array_address ;
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1,data ); //DATA
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
				}
				
				LL_I2C_GenerateStopCondition(I2C1); // STOP

}
	
	//LL_mDelay(5000); // pause between page changes

//Page mode read
void eeprom_read_page_mode(char* array) {
	// reads in page mode
	// when char* is used, compiler automatically converts array into pointer to
//	address of first element
    
	uint8_t data;
	
	char* array_address;
		
	LL_I2C_GenerateStartCondition(I2C1); //START
	while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);
	
	LL_I2C_TransmitData8(I2C1, 0x00); //MEMORY POINTER HIGH BYTE
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
	LL_I2C_TransmitData8(I2C1, 0x00); //MEMORY POINTER LOW BYTE
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
	LL_I2C_GenerateStartCondition(I2C1); //RE-START
  while(!LL_I2C_IsActiveFlag_SB(I2C1));
	
  LL_I2C_TransmitData8(I2C1, EEPROMADR+1); //ADDRESS + READ
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);
	
	for (int i=0; i<65 ; i+=1){
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
	data = LL_I2C_ReceiveData8(I2C1);  //DATA BYTE
	//LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA

	// change the value at pointer address
	array_address = array + i;
	
	*array_address = data;
	// changes value at array_address to new data
		
 }
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
	data = LL_I2C_ReceiveData8(I2C1);  //DATA BYTE
	array_address = array + 65;
	*array_address = data;
	
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	
	LL_I2C_GenerateStopCondition(I2C1); //STOP
	
	i2c_1_configure();	
}

// function to scroll through parts of packet
void read_full_packet_2 (char* full_packet, char count, bool results, char* checksum){
	
	// print destination
	if (count == 5){
		char* array_address = full_packet;
		// temp storage for array address 
		char outputString[18];
		char databin[4];
		put_string(0,0,"               ");
	
		put_string(0,0,"     dest      ");
		
		for (int i = 0 ; i <4 ; i++){
			// read from packet
			databin[i] = *array_address;
			array_address += 1;
		}
		put_string(0,15,"             ");
		sprintf(outputString,"%x%x%x%x",databin[3],databin[2],databin[1],databin[0]);
		put_string(0,15,outputString);
	}
	
	// print Source
	if (count == 4){
		char* array_address = full_packet;
		array_address += 4;
		char outputString[18];
		char databin[4];
		put_string(0,0,"               ");
	
		put_string(0,0,"     src      ");
		
		for (int i = 0 ; i <4 ; i++){
			// read from packet
			databin[i] = *array_address;
			array_address += 1;
		}
		put_string(0,15,"             ");
		sprintf(outputString,"%x%x%x%x",databin[3],databin[2],databin[1],databin[0]);
		put_string(0,15,outputString);
	}
	
	// print length
	if (count == 3){
		char* array_address = full_packet;
		array_address += 8;
		char outputString[18];
		char databin[2];
		put_string(0,0,"               ");
	
		put_string(0,0,"     length     ");
		
		for (int i = 0 ; i <2 ; i++){
			// read from packet
			databin[i] = *array_address;
			array_address += 1;
		}
		put_string(0,15,"             ");
		sprintf(outputString,"%x%x",databin[1],databin[0]);
		put_string(0,15,outputString);
	}
	
	// print 1st value of payload
	if (count == 2){
		char* array_address = full_packet;
		array_address += 10;
		char outputString[18];
		char databin[8];
		put_string(0,0,"               ");
	
		put_string(0,0,"    Payload     ");
		
			//Converts 8 bits into array for display purposes
		for(int j = 0; j < 8; j++){
	       databin[j] = (*array_address & (0x80 >> j)) > 0;
		}
			// read from packet
		put_string(0,15,"             ");
		sprintf(outputString,"%x%x%x%x%x%x%x%x",databin[0],databin[1],databin[2],databin[3],databin[4],databin[5],databin[6],databin[7]);
		put_string(0,15,outputString);
	}
	
	// print checksum
	if (count == 1){
		char* array_address = full_packet;
		array_address += 64;
		
		char outputString[18];
		char databin[8];
		put_string(0,0,"               ");
	
		put_string(0,0,"Checksum");
		
		for (int i = 0 ; i <2 ; i++){
			// read from packet
			databin[i] = *array_address;
			array_address += 1;
		}
		put_string(0,15,"             ");
		sprintf(outputString,"%x%x",databin[1],databin[0]);
		put_string(0,15,outputString);
	}
	
	// recalculates checksum and check if it is the same as checksum in package
	if (count == 0){
		char* array_address = full_packet;
		array_address += 64;
		
		if(results){
			// compares checksum array & checksum in packet
			// display OK
			
			char outputString[18];
			char databin[8];
			put_string(0,0,"               ");
			put_string(0,0,"Checksum OK");
		
			// displays checksum value 
		for (int i = 0 ; i <2 ; i++){
			// read from packet
			databin[i] = *array_address;
			array_address += 1;
		}
		put_string(0,15,"             ");
		sprintf(outputString,"%x%x",databin[1],databin[0]);
		put_string(0,15,outputString);
		}
		else{
			// display ERROR & New calculated checksum value
			
			char outputString[18];
			char databin[8];
			put_string(0,0,"               ");
			put_string(0,0,"Checksum Error");
		
		for (int i = 0 ; i <2 ; i++){
			// read from packet
			databin[i] = *checksum;
			checksum += 1;
		}
		put_string(0,15,"             ");
		sprintf(outputString,"%x%x",databin[1],databin[0]);
		put_string(0,15,outputString);
		}
	
	}
	
	
}

// function to form the packet array
void form_full_packet(char* array, int destination,int source, int length, char* payload, char* checksum){
	
	// Destination
	for (int i = 0 ; i < 4 ; i++){
		
		*array = (destination & 0xFF);
		// masks first 8 bits to be read
		array += 1; 
		// move array pointer forward
		destination = destination >> 8;
		// shift bits by 1 byte
	}
	
	// source
	for (int i = 0 ; i < 4 ; i++){
		
		*array = (source & 0xFF);
		// masks first 8 bits to be read
		
		source = source >> 8;
		
		// shift bits by 1 byte 
		
		array += 1; 
		// move array pointer forward
	}
	
	// length
	for (int i = 0 ; i < 2 ; i++){
		
		*array = (length & 0xFF);
		// masks first 8 bits to be read
		
		length = length >> 8;
		
		// shift bits by 1 byte 
		
		array += 1; 
		// move array pointer forward
	}
	
	// payload
	for (int i =  0 ; i <54 ; i++){
		*array = *payload ; 
		
		payload += 1;
		array += 1;
	}
	
	// checksum
	for (int i =  0 ; i <2 ; i++){
		*array = *checksum ; 
		
		checksum += 1;
		array += 1;
	}
}

void checksumCount(char* array, char* checksum){
	int x = 0;
	char* array2;
	
	char hex1 = 0; // stores 3rd hex digit
	int hex2 = 0; // stores 4th hex digit
	char carry = 0;
	
	// reads each hex value and adds it to value x (sum of all 16 bit values)
	for (int i = 0 ; i <64 ; i+=2){
		// iterates by 2 chars every time
		array2 = array + 1;
		hex1 = *array2 % 16;
		// gets remainder (LSB hex value)
		hex2 = *array2/16;
		// gets integer divide (MSB hex value)
		x = x + *array + hex1*256+ hex2*4096;
		// sum = sum + char + 2nd char split into 2 hex values
		array = array +2 ;
		// shifts array address by 2 chars
	}
	
	carry = x >> 16;
	// shift bits by 16 bits, to get 5th hex digit
	
	// this is to remove the 5th hex digit (e.g. 2fffd -> fffd)
	x = x- carry *65536; // deduct by the carry hex digit * the value of a 5th pos hex
	
	// this is to add back the removed carry digit (e.g. x = fffd+2)
	x = x+carry; 
	// we mask the value of checksum to size of char and add into 1st array position
	*checksum = x & 0xff ;
	// moves array address pointer to 2nd position
	checksum += 1;
	
	// reads 2nd char into checksum
	*checksum = (x>>8) & 0xff ;
	
	/*
				char outputString[18];
			sprintf(outputString,"%x",x);
			put_string(0,0,"             "); //Report successful write
			put_string(0,0,"Written");
			put_string(0,15,"             ");
			put_string(0,15,outputString);
	LL_mDelay(50000000);
	*/
	
}

// compares checksum value stored in packet & stored in temp array checksum
bool compareCheckSum(char* array, char* checksum){
	array = array+64;
	// moves address pointer to 65th position
	bool results; // initate value results
	
	// return true if both hex digits match
	if (*array == *checksum)
	{
		array +=1;
		checksum +=1;
		if (*array == *checksum)
		{
			results = true;
		}
	}
	
	// false otherwise
	else{
		results = false;
	}
	return results;
}



