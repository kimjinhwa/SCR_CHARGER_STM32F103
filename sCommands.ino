#include <SerialCommands.h>

char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
	sender->GetSerial()->print("Unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
	sender->GetSerial()->print(">");
}


void readSettingVoltage(SerialCommands* sender)
{
	String readString(sender->Next());	
	if(readString==NULL || readString.length()<=0){
		sender->GetSerial()->println("ERROR NO_PWM");
		return;
	}
	goalVoltage= readString.toInt();//(atoi(readString);
	sender->GetSerial()->print("Set Charge Voltage : ");
	sender->GetSerial()->println(goalVoltage);
	sender->GetSerial()->print(">");
}

void readSettingAmpere(SerialCommands* sender)
{
	String readString(sender->Next());	
	if(readString==NULL || readString.length()<=0){
		sender->GetSerial()->println("ERROR NO_PWM");
		sender->GetSerial()->print(">");
		return;
	}
	goalAmpere = readString.toInt();//(atoi(readString);
	sender->GetSerial()->print("Set Charge Ampere: ");
	sender->GetSerial()->println(goalAmpere);
	sender->GetSerial()->print(">");
}
void readSettingScrDelayTimeSet(SerialCommands* sender)
{
	String readString(sender->Next());	
	if(readString==NULL || readString.length()<=0){
		sender->GetSerial()->println("ERROR NO_PWM");
		sender->GetSerial()->print(">");
		return;
	}
	scrStartDelayTimeSet = readString.toInt();//(atoi(readString);
	if(scrStartDelayTimeSet < SCR_MIN_STARTTIME)
		scrStartDelayTimeSet =SCR_MIN_STARTTIME;
	if(scrStartDelayTimeSet > SCR_MAX_STARTTIME)
		scrStartDelayTimeSet = SCR_MAX_STARTTIME	;
	sender->GetSerial()->print("Set Scr On Delay Time : ");
	sender->GetSerial()->println(scrStartDelayTimeSet );
	sender->GetSerial()->print(">");
}
void showRunning(SerialCommands* sender)
{
	String readString(sender->Next());	
	if(readString==NULL || readString.length()<=0){
		sender->GetSerial()->println("ERROR NO_PWM");
		sender->GetSerial()->print(">");
		return;
	}
	DEBUG =	readString.toInt();//(atoi(readString);
	sender->GetSerial()->println("Setting OK");
	sender->GetSerial()->print(">");
}
void help(SerialCommands* sender)
{
	sender->GetSerial()->println("vol  [xxx] : set charge voltage" );
	sender->GetSerial()->println("amp  [xxx] : set charge ampere" );
	sender->GetSerial()->println("scr  [xxx] : set scr on delay time" );
	sender->GetSerial()->println("show [1:0] : show current status " );
	sender->GetSerial()->println("set        : show current setting value" );
	sender->GetSerial()->print(">");
}

void showSetStatus(SerialCommands* sender)
{
	sender->GetSerial()->println("Setted Value");
	sender->GetSerial()->print("VOL : ");
	sender->GetSerial()->println(goalVoltage);
	sender->GetSerial()->println("AMP : ");
	sender->GetSerial()->print(goalAmpere);
	sender->GetSerial()->print("SCR : ");
	sender->GetSerial()->println(scrStartDelayTimeSet);
	sender->GetSerial()->print(">");
}
SerialCommand cmdSetVoltage("vol",readSettingVoltage);
SerialCommand cmdSetAmpere("amp",readSettingAmpere);
SerialCommand cmdSetScrTimeset("scr",readSettingScrDelayTimeSet);
SerialCommand cmdshowRunning("show",showRunning);
SerialCommand cmdshowSetStatus_("set",showSetStatus);
SerialCommand cmdHelp("?",help);
SerialCommand cmdHelp2("help",help);

void serialCommandSetup()
{
	serial_commands_.SetDefaultHandler(cmd_unrecognized);
	serial_commands_.AddCommand(&cmdSetVoltage);
	serial_commands_.AddCommand(&cmdSetAmpere);
	serial_commands_.AddCommand(&cmdSetScrTimeset);
	serial_commands_.AddCommand(&cmdshowRunning);
	serial_commands_.AddCommand(&cmdshowSetStatus_);
	serial_commands_.AddCommand(&cmdHelp);
	serial_commands_.AddCommand(&cmdHelp2);

}
void serialReadCommand(){
	serial_commands_.ReadSerial();
}
