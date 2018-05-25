#ifndef ZODIAC_GPIO_MESSAGE_H
#define ZODIAC_GPIO_MESSAGE_H

#define GPIO_NUM152  (152)
#define ENABLE_SDI1  (0)
#define ENABLE_SDI2  (1)


typedef struct SET_Gpio
{
	long  msgtype;
	struct DATA
	{
		unsigned int gpioNum;                    //type 0
		//unsigned int direction;  //0 in  1 out         type1
		unsigned int value; //type2
	}Data;
}Set_Gpio;

enum
{
	// GPIONUM,                 //type 0
	 NUM_VALUE=1,  //0 in  1 out         type1
	// VALUE  //type2
};



void set_gpioNum_Value(int gpio_num,int gpio_value);
//void set_gpioDirection(int gpio_direction);
//void set_gpioValue(int gpio_value);

void init_GPIO_IPCMessage();
void delete_GPIO_IPCMessage();
void SendMessage(Set_Gpio* value);

#endif
