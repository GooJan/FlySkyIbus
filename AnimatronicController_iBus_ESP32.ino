// https://github.com/playfultechnology/arduino-animatronic/blame/master/AnimatronicController_iBus_ESP32/AnimatronicController_iBus_ESP32.ino

/**
 * Realtime Arduino Animatronic Controller (10 channel, iBus version)
 *
 * This Arduino script reads serial data from the iBus interface of a RC receiver such as the Flysky FS-iA6B,
 * It then applies various logic to these inputs based on a chosen animatronic model (tentacle, eye, etc.) and assigns corresponding
 * positions of an array of servo motors via a PCA9685 PWM controller.
 */

// FlySky iBus interface adapted from
// https://gitlab.com/timwilkinson/FlySkyIBus
#include "src/FlySkyIBus/FlySkyIBus_servo.h"
// Instantiate an IBus object to process IBus data
FlySkyIBusServo iBus;

// Wire library used for I2C communication
#include <Wire.h>

// PCA9685 16-channel PWM controller
// See: https://github.com/NachtRaveVL/PCA9685-Arduino
#include "src/PCA9685/PCA9685.h"
// Initialise the PWM controller which will send output to the servos
PCA9685 pwmController;
// Helper function to calculate pulse required for given angle
PCA9685_ServoEvaluator pwmServo;

// For OLED Display
#include <lcdgfx.h>
// OLED display
// rstPin
DisplaySSD1306_128x64_I2C display(-1);

// How many channels to read from the iBus packet (max 14)
const byte numInputChannels = 10;
// Array of input channel values read from the iBus packet received from FS-iA6B
int channelInput[numInputChannels];

// This is the number of output channels (i.e. servo motors) to be controlled by the script
// Note that the logic applied by the script means that there is not a 1:1 mapping between inputs and outputs
// so this number can be greater or less than the number of inputs
const byte numOutputChannels = 10;
// Array of output channel values to send to PCA9685
int channelOutput[numOutputChannels];

// Define some common animatronic behaviour types
enum Behaviour {
	Passthrough, // Pass all inputs unchanged to corresponding numbered outputs
};
// Define the behavior which we'd like the model to follow
Behaviour behaviour = Behaviour::Passthrough;

// 统计帧率 FPS
unsigned long lastMillis;
unsigned long thisMillis;
unsigned long frameCount;

// 设置 LED 引脚
int led_pin = 2;
int oled_rst_pin = 21;

void setup() {

	// Hardware USB serial connection to the Arduino IDE monitor
	Serial.begin(115200);
	Serial.println("Ready!");

  // OLED 复位
	pinMode(oled_rst_pin, OUTPUT);
	digitalWrite(oled_rst_pin, LOW);
  delay(10);
	digitalWrite(oled_rst_pin, HIGH);
  delay(10);

	// 设定引脚为输出模式
	pinMode(led_pin, OUTPUT);
	digitalWrite(led_pin, LOW);

	// Serial connection to the FS-IA6B receiver
	// baud, mode, rxPin, txPin
	Serial1.begin(115200, SERIAL_8N1, 16, 17);
	// Attach the IBus object to the ibusSerial interface
	iBus.begin(Serial1);

	// Initialise I2C interface used for the PCA9685 PWM controller
	// sda, scl, frequency
	Wire.begin(18, 19, 400000);

	// Initialise PCA9685
	pwmController.resetDevices();
	pwmController.init();
	// 50Hz provides 20ms standard servo phase length
	pwmController.setPWMFrequency(50);

	// /* Select the font to use with menu and all font functions */
	display.setFixedFont( ssd1306xled_font6x8 );
	display.begin();
	display.clear();
	display.printFixed(0,  0, "Remote Control", STYLE_NORMAL);
}

void loop() {

	// iBus data is sent is discrete packets. We'll call IBus.loop() on every
	// frame to check whether a new packet has been received. If so, it will return true.
	if(iBus.loop()) {

		for(int i=0; i<numInputChannels; i++) {
			channelInput[i] = iBus.get_channel(i);
		}

		// Now perform whatever logic is necessary to convert from the array of input channels
		// into the array of output values
		ApplyLogic();

		// Now, send the output values to the PWM servo controller
		for(int i=0; i<numOutputChannels; i++) {
			pwmController.setChannelPWM(i, channelOutput[i]);
		}
	}

	fps(100);
}

void fps(int milli_seconds) {

	frameCount ++;
	thisMillis = millis();

	if (thisMillis - lastMillis > milli_seconds) {

		digitalWrite(led_pin, LOW);

		char s[21];

		snprintf(s, 21, "Ch1:%u   Ch2:%u", channelInput[0], channelInput[1]);
		display.printFixed(0,  16, s, STYLE_NORMAL);
		snprintf(s, 21, "Ch3:%u   Ch4:%u", channelInput[2], channelInput[3]);
		display.printFixed(0,  24, s, STYLE_NORMAL);
		snprintf(s, 21, "Ch5:%u   Ch6:%u", channelInput[4], channelInput[5]);
		display.printFixed(0,  32, s, STYLE_NORMAL);
		snprintf(s, 21, "Ch7:%u   Ch8:%u", channelInput[6], channelInput[7]);
		display.printFixed(0,  40, s, STYLE_NORMAL);
		snprintf(s, 21, "Ch9:%u   ChA:%u", channelInput[8], channelInput[9]);
		display.printFixed(0,  48, s, STYLE_NORMAL);

		snprintf(s, 21, "FPS:%u", frameCount * 1000 / (thisMillis - lastMillis));
		display.printFixed(0,  56, s, STYLE_NORMAL);

		frameCount = 0;
		lastMillis = thisMillis;

	}
	digitalWrite(led_pin, HIGH);
}

// This function processes inputs and calculates the appropriate outputs
// based on the selected model
void ApplyLogic() {

	// Behaviour depends on chosen model
	switch(behaviour) {

	case Behaviour::Passthrough: {
		int n = min(numInputChannels, numOutputChannels);
		for(int i=0; i<n; i++) {
			// The ON period of almost all RC pulses range from 1000us to 2000us.
			// We'll remap this to an angle from -90 to +90
			if(channelInput[i] >= 1000 && channelInput[i] <= 2000) {
				int X = map(channelInput[i], 1000, 2000, -90, 90);
				channelOutput[i] = pwmServo.pwmForAngle(X);
			}
		}
	}
	break;

	default:
		break;
	}
}
