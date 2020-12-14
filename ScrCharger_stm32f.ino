
#include <SPI.h>


//SCR의 테이타 쉬트의 ONTIME은 100A/us이다
// 50으로도 충분하다.
#define TIMER2_SET_PERIOD	10  // 10us
// this value is not depand on timer2
#define SCRONTIME 			100

#define SCR_MIN_STARTTIME	100
//8300us = 8.3ms
#define SCR_MIN_STARTTIME   80 
#define SCR_MAX_STARTTIME   830 
#define SCR_DEFAULT_STARTTIME 820	
//매 10ms마다 값을 읽는다. 10x1000
#define READ_AMPERE_PERIOD	1000	

#define MAX_CHARGING_VOLTAGE	82	
#define MAX_CHARGING_AMPERE		82	
#define DEBUG		
int readZeroCrossing_PC14 = PC14;
int AmperePin =PA2;
int VoltagePin =PA0;
int scrOnOff_PA1 = PA1;
float chargingAmpere=0;
float goalAmpere=0;
float chargingVoltage=0;
float goalVoltage=0;
float setChargingVoltage=0;
float setChargingAmpere=0;
volatile int risingStartFlag;

HardwareTimer timer(2);
SPIClass SPI3(2);
void handler_tm2(void);
void readAmpereAndVoltage();

int scrStartDelayTime = SCR_DEFAULT_STARTTIME;//7000ms soft start를 시키기 위한 조건이다.
int startTimer2 = false;
							//측정결과 최소 시간은 HIGH를 인식하고 나서 1.5ms이상이 지난후 ON을 시켜야 한다.
int scrStartDelayTimeSet =  SCR_DEFAULT_STARTTIME;
unsigned long isrCount=0;
void handler_tm2() 
{
	// SCR을 얼만큰 지연 시켜서 시작할지를 결정한다.
	// scrStartDelayTime을 zerocrossing 에서 인터럽트에서 시작된다.
	noInterrupts();
	isrCount++;
	//주기적으로 전압과 전류를 읽는다.
	if(!(isrCount % READ_AMPERE_PERIOD)){
		readAmpereAndVoltage();
	};
#ifdef DEBUG
	if( !(isrCount % 100000)){  // 10,000* 100 =  1,000,000 = 1S
		Serial.print("Amp: "); Serial.print(chargingAmpere ); Serial.print("	vol: "); Serial.println((float)chargingVoltage ); Serial.println(analogRead(VoltagePin));
	}
#endif
	//제로크로싱일 일어났고, 이제 전원공급이 시작되었다.
	if(startTimer2 && scrStartDelayTime > 0  &&  
			chargingVoltage <= MAX_CHARGING_VOLTAGE	 && 
			chargingAmpere  <= MAX_CHARGING_AMPERE	 &&
			goalAmpere > 0 &&
			goalVoltage> 0 
		){
		scrStartDelayTime-- ;
		digitalWrite(scrOnOff_PA1,0);
		if(  scrStartDelayTime == 0 ){
			//SCR을 켜준다.
			//일정시간 동안 온을 유지한다.
			Serial.println("SCR ON");
			for(int i=0;i< SCRONTIME ;i++)digitalWrite(scrOnOff_PA1,1);
			digitalWrite(scrOnOff_PA1,0);
			Serial.println("SCR OFF");
			//테스트 코드이다. 이 코드는 제로크로싱에서 담당한다.
			scrStartDelayTime=scrStartDelayTimeSet ;  
			startTimer2 = false;
		}
	}
	interrupts();
}

void setup() {
    //pinMode(scrOnOff_PA1, PWM);
    pinMode(scrOnOff_PA1, OUTPUT);
    pinMode(AmperePin, INPUT_ANALOG);
    pinMode(VoltagePin, INPUT_ANALOG);
    pinMode(PC13, OUTPUT);
    pinMode(readZeroCrossing_PC14, INPUT);
	
    Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
    Timer2.setPeriod(TIMER2_SET_PERIOD);  
    Timer2.attachCompare1Interrupt(handler_tm2);
    
    // low-High zero crossing
    attachInterrupt(digitalPinToInterrupt(readZeroCrossing_PC14), button_ISR_CHANGE, RISING);//FALLING CHANGE
    digitalWrite(PC13, HIGH); // SCR OFF
	digitalWrite(scrOnOff_PA1,LOW);

    Serial.println("System Start..");
    //setupSPI();
    //attachInterrupt(0, ISR, FALLING);
    //SPI3.attachInterrupt();
    //SPI3.onReceive(spiOnReceive);
	Serial.begin(115200);
	SPI3.setModule(3);
}
void setupSPI(){
  
  //SPI3.begin(); //Initialize the SPI_1 port.
  //SPI3.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  //SPI3.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  //SPI3.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  pinMode(BOARD_SPI1_ALT_NSS_PIN   , OUTPUT);
  
  SPI3.beginTransactionSlave(SPISettings(18000000, MSBFIRST, SPI_MODE0, DATA_SIZE_8BIT));
}
void button_ISR_CHANGE()
{
  scrStartDelayTime=scrStartDelayTimeSet ;  // 3ms
  startTimer2 =true;
  //측정 결과 정확히 83마다 되는 것을 확인하였다
  //isrCount=0;
	/*
  if( digitalRead(readZeroCrossing_PC14)) //상승 변화 이었으면
  {
    risingStartFlag = 1;
    scrStartDelayTime=SCR_DEFAULT_STARTTIME	;  // 3ms
  }
  else  //하강 인터럽트이다.
  {
    risingStartFlag = 0;
  }
  */
}

void SerialReadInput(){
	if (Serial.available() > 0) {
		String readString;
		readString = Serial.readString();
		scrStartDelayTimeSet = readString.toInt();//(atoi(readString);
		if(scrStartDelayTimeSet < SCR_MIN_STARTTIME)
			scrStartDelayTimeSet =SCR_MIN_STARTTIME;
		if(scrStartDelayTimeSet > SCR_MAX_STARTTIME)
			scrStartDelayTimeSet = SCR_MAX_STARTTIME	;

		Serial.println(scrStartDelayTimeSet );
	}
}
void readAmpereAndVoltage()
{
    float ampere  = (float)(analogRead(AmperePin)) * 3.3/4096 ;
	// input 2.3v ideal 2.5V real 2.79V
	//보정    25mv/A
	ampere -=0.29;
	ampere = (ampere-2.5)/0.025; 
    chargingAmpere = (float)chargingAmpere*0.9 + (float)ampere*0.1;

    float voltage = (float)(analogRead(VoltagePin)) * 3.3/4096 ;
	voltage = (float)voltage *(417.49/10.0); //보정값을 적용 하였다.
    chargingVoltage = chargingVoltage*0.9 + voltage*0.1;
}
// uint8_t msg = SPI3.transfer(++count);
void loop() {
    SerialReadInput();
    digitalWrite(PC13,!digitalRead(PC13));
    //sendSPI();
    delay(100);// every 100ms
}
void sendSPI()
{
  byte data;
  Serial.print("send spi data");
  digitalWrite(BOARD_SPI1_ALT_NSS_PIN  , LOW); // manually take CSN low for SPI_1 transmission
  //data = SPI3.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI3.write(0x55);
  digitalWrite(BOARD_SPI1_ALT_NSS_PIN , HIGH); // manually take CSN high between spi transmissions
}

void spiOnReceive (){
  byte data;
  Serial.print("Spi Interrupt occurd");
  data = SPI3.transfer(0x55); // 데이타를 받았다고 보내주고..
  if(data == 0x01)
    data = SPI3.transfer16(chargingAmpere);
  else if(data == 0x02)
    data = SPI3.transfer16(chargingVoltage);
  else 
    data = SPI3.transfer16(0xFFFF);
}

      //scrTrunOnDuty = map(sensorValue, 0, 4095, 0, 65535);
    //TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(scrOnOff_PA1), PinMap_PWM);

    //pwmWrite(scrOnOff_PA1, scrTrunOnDuty);
    //pwmWrite(scrOnOff_PA1, scrTrunOnDuty);
    //if(risingStartFlag)
    //{
      //risingStartFlag = 0;
      //delay(4); //이 값을 변화 시켜서 전압값을 제어한다.
      //pwmWrite(scrOnOff_PA1, scrTrunOnDuty);
      //delay(10);
      //pwmWrite(scrOnOff_PA1, 0);
      //Serial.println("Button Pressed!");
    //}
    //if(digitalRead(readZeroCrossing_PC14)==LOW)
    //    pwmWrite(scrOnOff_PA1,0);
      //digitalWrite(PC13, HIGH);   // turn the LED on (HIGH is the voltage level)
      //delay(500);              // wait for a second
      //digitalWrite(PC13, LOW);    // turn the LED off by making the voltage LOW
      //delay(500);              // wait for a second
      //Serial.println(digitalRead(readZeroCrossing_PC14));
      //Serial.println(analogRead(AmperePin));
    //Serial.println(scrTrunOnDuty);
    //Serial.println(timer.getPrescaleFactor());
//C:\Users\STELLA\Documents\ArduinoData\packages\stm32duino\hardware\STM32F1\2020.10.19\system\libmaple
/*C:\Users\STELLA\Documents\ArduinoData\packages\stm32duino\hardware\STM32F1\2020.10.19\system\libmaple\include\libmaple
 * C:\Users\STELLA\Documents\ArduinoData\packages\stm32duino\hardware\STM32F1\2020.10.19\variants\generic_stm32f103c\board\board.h
#define BOARD_NR_SPI              2
#define BOARD_SPI1_NSS_PIN        PA4
#define BOARD_SPI1_MOSI_PIN       PA7
#define BOARD_SPI1_MISO_PIN       PA6
#define BOARD_SPI1_SCK_PIN        PA5

#define BOARD_SPI1_ALT_NSS_PIN    PA15
#define BOARD_SPI1_ALT_MOSI_PIN   PB5
#define BOARD_SPI1_ALT_MISO_PIN   PB4
#define BOARD_SPI1_ALT_SCK_PIN    PB3

#define BOARD_SPI2_NSS_PIN        PB12
#define BOARD_SPI2_MOSI_PIN       PB15
#define BOARD_SPI2_MISO_PIN       PB14
#define BOARD_SPI2_SCK_PIN        PB13
*/

