/*Nome ALUNO A- Simão Luís 2212430
Nome ALUNO B- Tomás Rei 2202333
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEA- Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos

Pretende-se  neste  trabalho  prático  a  implementação  de um  algoritmo  que  permita aquisição de sinais atravez de sensores e respetica apresentação numa aplicação, bem como num Display.

LINK: https://youtu.be/3I9LBhsy9s0
*/


#define BLUETOOTH
#include <math.h>
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"
#include "config.h"
#include <esp32_can.h>
#include <SPI.h>
#include "MyLogger.h"
#include "ELM327_Emulator.h"
#include <iso-tp.h>
#include "obd2_codes.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Definir os pinos utilizados
#define ADC_1_4 32
#define ADC_1_6 34
#define ADC_1_7 35
#define ADC_RESOLUTION 12
#define BUTTON_OIL_PRESSURE 16
#define LED 17

// Definir os terminais de DC e CS do tft
#define TFT_SCK 18
#define TFT_MISO 19
#define TFT_CS 15
#define TFT_DC 2
#define TFT_MOSI 23
#define TFT_RST -1

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST, TFT_MISO); //definir pinos tft

// Distância entre caracteres em pixels
#define DISTC 12

EEPROMSettings settings;
ELM327Emu elmEmulator;

//definir estruturas
typedef struct {
	uint8_t pidnum;
	uint8_t mode;
	bool bHeader;
} xData_PID;
QueueHandle_t xQueueELM327_emulator;

template<class T> inline Print& operator <<(Print &obj, T arg) {
	obj.print(arg);
	return obj;
}

void setPromiscuousMode() {
	CAN0.watchFor();
}
//declarar as funcoes
const char *pcTextELM327_emulator = "Task ELM327 emulator ";
void ELM327_emulator_Task(void *pvParameters);
void vADC_INTAKE_TEMP(void *pvParameters);
void vADC_THROTTLE_POS(void *pvParameters);
void vADC_COOL_TEMP(void *pvParameters);
void vOIL_PRESSURE(void *pvParameters);
void vTFT(void *pvParameters);

//Defenicao do prototipo da interrupcao
void IRAM_ATTR oil_pressure();

//Queue para transferir valor de temperatura do ar de admissao entre tarefas
QueueHandle_t intakeairtempQueue;
//Queue para transferir valor de percentagem da borboleta entre tarefas
QueueHandle_t throttlepositionQueue;
//Queue para transferir valor de temperatura do liquido de refrigeracao entre tarefas
QueueHandle_t cooltempQueue;

//Criacao do semaforo para prioridade do botao
SemaphoreHandle_t OIL_PRESSURE_Semaphore;

//Criacao do semaforo mutex que permite que o TFT corra ate ao fim
SemaphoreHandle_t xMutex;

void setup() {

	//Input de sensores/
	pinMode(ADC_1_4, INPUT);
	pinMode(ADC_1_6, INPUT);
	pinMode(ADC_1_7, INPUT);

	//pino led
	pinMode(LED, OUTPUT);

	//Outputs para o TFT
	pinMode(TFT_CS, OUTPUT);
	pinMode(TFT_DC, OUTPUT);
	pinMode(TFT_SCK, OUTPUT);
	pinMode(TFT_MOSI, OUTPUT);
	pinMode(TFT_RST, OUTPUT);
	pinMode(TFT_MISO, INPUT);

	//Input butao oil pressure
	pinMode(BUTTON_OIL_PRESSURE, INPUT_PULLUP);

	//Anexar uma interrupcao ao pino button oil pressure a interrupcao vai chamar a funcao
	attachInterrupt(digitalPinToInterrupt(BUTTON_OIL_PRESSURE), oil_pressure,
			FALLING);
	interrupts(); //permite interrupcoes

	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1); // handler das prioridades

	Serial.begin(115200); // Definir o baudrate
	Serial.print("Build number: ");
	Serial.println(CFG_BUILD_NUM);

	//texto inicial do display
	tft.begin();
	tft.fillScreen(ILI9341_BLACK);

	tft.setCursor(0, 10);
	tft.setTextColor(ILI9341_WHITE);
	tft.setTextSize(2);
	tft.println(" Sistemas Eletricos e ");
	tft.println(" Eletronicos de Veiculos ");
	tft.println(" 2022/2023");
	tft.setTextSize(2);
	tft.println(" 2212430 Simao Luis");
	tft.println(" 2202333 Tomas Rei");

	tft.drawRect(5, 5, 300, 90, ILI9341_GREEN);
	tft.drawRect(6, 6, 298, 88, ILI9341_GREEN);

	tft.setCursor(40, 120);
	tft.setTextSize(2);
	tft.println("Engenharia Automovel");

	tft.setCursor(10, 180);
	tft.setTextSize(2);
	tft.println("Temp ar admissao");

	tft.setCursor(10, 160);
	tft.setTextSize(2);
	tft.println("Temp refrigeracao");

	tft.setCursor(10, 200);
	tft.setTextSize(2);
	tft.println("Posicao borboleta");

	//criar a queue para valor de temperatura de admissao
	intakeairtempQueue = xQueueCreate(3, sizeof(float));
	//criar a queue para valor de percentagem da borboleta
	throttlepositionQueue = xQueueCreate(3, sizeof(float));
	//criar a queue para valor de temperatura de refrigeracao
	cooltempQueue = xQueueCreate(3, sizeof(float));
	//criar a queue
	xQueueELM327_emulator = xQueueCreate(3, sizeof(xData_PID));
	// criar o semaforo
	vSemaphoreCreateBinary(OIL_PRESSURE_Semaphore);
	// criar a mutex
	xMutex = xSemaphoreCreateMutex();

		elmEmulator.ELMlibxQueueELM327 = xQueueELM327_emulator;
	xTaskCreatePinnedToCore(ELM327_emulator_Task, "ELM327 emulator Task", 8192,
			(void*) pcTextELM327_emulator, 6, NULL, 1);

	//Criacao de tarefa ADC para leitura da temperatura do ar de admissao
	xTaskCreatePinnedToCore(vADC_INTAKE_TEMP, "ADC_INTAKE_TEMP", 1024, NULL, 3,
			NULL, 1);
	//Criacao de tarefa ADC para leitura da posicao da borboleta
	xTaskCreatePinnedToCore(vADC_THROTTLE_POS, "ADC_THROTTLE_POS", 1024, NULL,
			3, NULL, 1);
	//Criacao de tarefa ADC para leitura da temperatura do liquido de refrigeracao
	xTaskCreatePinnedToCore(vADC_COOL_TEMP, "ADC_COOL_TEMP", 1024, NULL, 3,
			NULL, 1);
	//Criacao de tarefa para o TFT
	xTaskCreatePinnedToCore(vTFT, "TFT", 4096, NULL, 1, NULL, 1);
	//Criacao de tarefa para o botao oil pressure
	xTaskCreatePinnedToCore(vOIL_PRESSURE, "OIL_PRESSURE", 800, NULL, 1, NULL,
			1);
}

void ELM327_emulator_Task(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	xData_PID lReceived_Data_PID;
	CAN_FRAME incoming, outcoming;
	uint32_t my_PIDs = 0;

	char buff[30];
	//inicializar variaveis
	float in_air_temp = 0, throttle_pos = 0, eng_cool_temp = 0, eng_cool_temp2 =
			0;
	static unsigned long int rpm = 0, rpm_tmp = 0, speed = 0,
			eng_cool_temp_tmp = 0, eng_cool_temp_tmp2 = 0, throttle_pos_tmp = 0,
			in_air_temp_tmp = 0;

	settings.version = EEPROM_VER;
	settings.CAN0Speed = 500000;
	settings.CAN0_Enabled = false;
	settings.my_PID = 0xFFFFFFFF;
	settings.valid = 0; //not used right now
	strcpy(settings.btName, "SEEV_OBD2");
	settings.logLevel = 4; //info instead of debug
	MyLogger::setLoglevel((MyLogger::LogLevel) settings.logLevel);

	Serial.print(pcTaskName);

	if (settings.CAN0_Enabled) {
		CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5); //rx, tx
		CAN0.enable();
		CAN0.begin(settings.CAN0Speed, 255); //
		Serial.print("Enabled CAN0 with speed ");
		Serial.println(settings.CAN0Speed);
		CAN0.setListenOnlyMode(false);
		CAN0.watchFor();
		CAN1.disable();
		setPromiscuousMode();
	} else {
		CAN0.disable();
		CAN1.disable();
	}

	elmEmulator.setup();

	// Selecao dos PIDs utilizados
	my_PIDs = my_PIDs + (1 << (32 - PID_ENGINE_RPM));
	my_PIDs = my_PIDs + (1 << (32 - PID_VEHICLE_SPEED));
	my_PIDs = my_PIDs + (1 << (32 - PID_ENGINE_COOLANT_TEMP));
	my_PIDs = my_PIDs + (1 << (32 - PID_THROTTLE_POS));
	my_PIDs = my_PIDs + (1 << (32 - PID_INTAKE_AIR_TEMP));

	for (;;) {

		xStatus = xQueueReceive(xQueueELM327_emulator, &lReceived_Data_PID, 0); // verificar a existencia de novos valores na queue
		if (xStatus != NULL) {
			if (lReceived_Data_PID.pidnum == PID_SUPPORTED1) {

				outcoming.id = 0x7E8;
				outcoming.length = 0x06;

				outcoming.data.byte[1] = 0x40 + lReceived_Data_PID.mode;
				outcoming.data.byte[2] = lReceived_Data_PID.pidnum;
				outcoming.data.byte[3] = (uint8_t) (my_PIDs >> 24 & 0x000000FF);
				outcoming.data.byte[4] = (uint8_t) (my_PIDs >> 16 & 0x000000FF);
				outcoming.data.byte[5] = (uint8_t) (my_PIDs >> 8 & 0x000000FF);
				outcoming.data.byte[6] = (uint8_t) (my_PIDs >> 0 & 0x000000FF);
				elmEmulator.sendOBDReply(outcoming);
			} else if (lReceived_Data_PID.pidnum == PID_ENGINE_RPM) {

				//icrementacao valor de rpm
				rpm = rpm + 10;
				if (rpm > 10000)
					rpm = 0;
				rpm_tmp = rpm * 4;

				// enviar o valor para a aplicacao
				outcoming.id = 0x7E8;
				outcoming.length = 0x04;

				outcoming.data.byte[1] = 0x40 + lReceived_Data_PID.mode;
				outcoming.data.byte[2] = lReceived_Data_PID.pidnum;
				outcoming.data.byte[3] = (uint8_t) (rpm_tmp >> 8 & 0x000000FF);
				outcoming.data.byte[4] = (uint8_t) (rpm_tmp & 0x000000FF);
				elmEmulator.sendOBDReply(outcoming);

			} else if (lReceived_Data_PID.pidnum == PID_VEHICLE_SPEED) {

				//icrementacao valor de velocidade
				speed++;
				if (speed > 255)
					speed = 0;

				// enviar o valor para a aplicacao
				outcoming.id = 0x7E8;
				outcoming.length = 0x03;

				outcoming.data.byte[1] = 0x40 + lReceived_Data_PID.mode;
				outcoming.data.byte[2] = lReceived_Data_PID.pidnum;
				outcoming.data.byte[3] = (uint8_t) (speed & 0x000000FF);
				elmEmulator.sendOBDReply(outcoming);

			} else if (lReceived_Data_PID.pidnum == PID_ENGINE_COOLANT_TEMP) {

				xQueueReceive(cooltempQueue, &eng_cool_temp, 0); //recebe valor da queue

				eng_cool_temp_tmp = (int) round(eng_cool_temp + 40); //arredonada o valor e soma 40 para respeitar o protocolo PID e converte para inteiro

				// enviar o valor para a aplicacao
				outcoming.id = 0x7E8;
				outcoming.length = 0x03;
				outcoming.data.byte[1] = 0x40 + lReceived_Data_PID.mode;
				outcoming.data.byte[2] = lReceived_Data_PID.pidnum;
				outcoming.data.byte[3] = (uint8_t) (eng_cool_temp_tmp
						& 0x000000FF);
				elmEmulator.sendOBDReply(outcoming);
			}

			else if (lReceived_Data_PID.pidnum == PID_THROTTLE_POS) {

				xQueueReceive(throttlepositionQueue, &throttle_pos, 0); // recebe o valor da queue

				throttle_pos_tmp = (int) round(throttle_pos * 255 / 100); // arredonada o valor e converte para inteiro

				// enviar o valor para a aplicacao
				outcoming.id = 0x7E8;
				outcoming.length = 0x03;
				outcoming.data.byte[1] = 0x40 + lReceived_Data_PID.mode;
				outcoming.data.byte[2] = lReceived_Data_PID.pidnum;
				outcoming.data.byte[3] = (uint8_t) (throttle_pos_tmp
						& 0x000000FF);
				elmEmulator.sendOBDReply(outcoming);
			}

			else if (lReceived_Data_PID.pidnum == PID_INTAKE_AIR_TEMP) {


				xQueueReceive(intakeairtempQueue, &in_air_temp, 0); // recebe o valor da queue

				in_air_temp_tmp = (int) round(in_air_temp + 40); //arredonada o valor e soma 40 para respeitar o protocolo PID e converte para inteiro

				// enviar o valor para a aplicacao
				outcoming.id = 0x7E8;
				outcoming.length = 0x03;
				outcoming.data.byte[1] = 0x40 + lReceived_Data_PID.mode;
				outcoming.data.byte[2] = lReceived_Data_PID.pidnum;
				outcoming.data.byte[3] =
						(uint8_t) (in_air_temp_tmp & 0x000000FF);
				elmEmulator.sendOBDReply(outcoming);
			} else {
				outcoming.id = 0x000;
				outcoming.length = 0x00;
				elmEmulator.sendOBDReply(outcoming);
			}
		}
		if (settings.CAN0_Enabled == true) {

			if (CAN0.available() > 0) {
				CAN0.read(incoming);
				elmEmulator.processFrame(incoming);
			}
		}
		elmEmulator.loop();

		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_PERIOD_MS)); // coloca a tarefa em idle
	}
}
void vADC_INTAKE_TEMP(void *pvParameters) { // Inicio tarefa sensor analogico temperatura de admissao
	TickType_t xLastWakeTime;
	//inicializar variaveis
	unsigned short int analog_value;
	float voltage_value;
	float temp_value;
	analogReadResolution(ADC_RESOLUTION); // faz a conversao ADC

	for (;;) {

		analog_value = (analogRead(ADC_1_4)); // Equacao de conversao analogico temperatura de admissao
		TickType_t xLastWakeTime;
		voltage_value = ((float) analog_value) * (5000.0 / 4095.0); //converte leitura de ADC em valores de tensao
		temp_value = voltage_value * 0.1; //converter tensao em temperatura

		xQueueSend(intakeairtempQueue, &temp_value, 0); // escrever o valor na queue

		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_PERIOD_MS)); // coloca a tarefa em idle
	}
}

void vADC_THROTTLE_POS(void *pvParameters) { // Inicio tarefa sensor analogico-digital posicao da borboleta
	TickType_t xLastWakeTime;

	//inicializar variaveis
	unsigned short int analog_value;
	float voltage_value;
	float percentage_value;
	analogReadResolution(ADC_RESOLUTION); // faz a conversao ADC

	for (;;) {

		analog_value = (analogRead(ADC_1_6)); // Equacao de conversao analogico-digital posicao da borboleta
		voltage_value = ((float) analog_value) * (3300.0 / 4095.0); //converte leitura de ADC em valores de tensao
		percentage_value = voltage_value * 0.03; // converter tensao em percentagem

		xQueueSend(throttlepositionQueue, &percentage_value, 0); // escrever o valor da queue

		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_PERIOD_MS)); // coloca a tarefa em idle
	}
}
void vADC_COOL_TEMP(void *pvParameters) { // Inicio tarefa sensor analogico temperatura refrigeracao
	TickType_t xLastWakeTime;

	//inicializar variaveis
	unsigned short int analog_value;
	float voltage_value;
	float temp_value;
	analogReadResolution(ADC_RESOLUTION); // faz a conversao ADC

	for (;;) {

		analog_value = (analogRead(ADC_1_7)); // Equacao de conversao analogico-digital
		voltage_value = ((float) analog_value) * (5000 / 4095.0); //converte leitura de ADC em valores de tensao
		temp_value = voltage_value * 0.1; //converter tensao em temperatura

		xQueueSend(cooltempQueue, &temp_value, 0); // escrever o valor na queue

		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_PERIOD_MS)); // coloca a tarefa em idle
	}
}

void vTFT(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	//inicializar variaveis
	float eng_coool_temp = 1, intake_air_temp = 1, tps = 1;
	float last_eng_coool_temp = 0, last_intake_air_temp = 0, last_tps = 0;
	tft.setRotation(0);

	for (;;) {
		xSemaphoreTake(xMutex, portMAX_DELAY); //restringir o display para a funcao vtft
		{
			xQueuePeek(intakeairtempQueue, &intake_air_temp, 0); //visualizar o valor da queue
			xQueuePeek(cooltempQueue, &eng_coool_temp, 0); //visualizar o valor da queue
			xQueuePeek(throttlepositionQueue, &tps, 0); //visualizar o valor da queue


			if ( last_intake_air_temp != intake_air_temp ) { // conferir o valor se este for diferente do anterior
				tft.fillRect(225, 180, 140, 14, ILI9341_BLACK); // limpar o sitio onde escrevemos
				tft.setTextSize(2); // definir o tamanho de letra a escrever
				tft.setCursor(225, 180);//posicionar o cursor no sitio certo
				tft.println((int)round(intake_air_temp)); // converter o valor para inteiro e arredondar o valor

			}

			if ( last_eng_coool_temp != eng_coool_temp ) { // conferir o valor se este for diferente do anterior
				tft.fillRect(225, 160, 140, 14, ILI9341_BLACK); // limpar o sitio onde escrevemos
				tft.setCursor(225, 160); //posicionar o cursor no sitio certo
				tft.setTextSize(2); // definir o tamanho de letra a escrever
				tft.println((int)round(eng_coool_temp));; // converter o valor para inteiro e arredondar o valor

			}

			if ( last_tps != tps ) { // conferir o valor se este for diferente do anterior
				tft.fillRect(225, 200, 140, 14, ILI9341_BLACK); // limpar o sitio onde escrevemos
				tft.setCursor(225, 200); //posicionar o cursor no sitio certo
				tft.setTextSize(2); // definir o tamanho de letra a escrever
				tft.println((int)round(tps)); // converter o valor para inteiro e arredondar o valor

			}

			last_intake_air_temp = intake_air_temp; // atualizar o valor
			last_eng_coool_temp = eng_coool_temp; // atualizar o valor
			last_tps = tps; // atualizar o valor

		}
		xSemaphoreGive(xMutex); //restringir o display para a funcao vtft
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_PERIOD_MS)); // coloca a tarefa em idle
	}
}
// Inicio tarefa sensor botao pressao de oleo
void vOIL_PRESSURE(void *pvParameters) {
	//inicializar variaveis
	short int led = 1;
	xSemaphoreTake(OIL_PRESSURE_Semaphore, 0); //define esta tarefa com ISR

	for (;;) {

		xSemaphoreTake(OIL_PRESSURE_Semaphore, portMAX_DELAY); //define esta tarefa com ISR

		led = !digitalRead(LED); // comutar o valor de estado do led

		digitalWrite(LED, led); // muda o estado logico do pino
	}
}

void oil_pressure() {
	static signed portBASE_TYPE xHigherPriorityTaskWoken;

	//por defeito nao se forca a alta prioridade, assume se a ISR como tendo prioridade baixa
	xHigherPriorityTaskWoken = pdFALSE;

	//ativa a tarefa ativando o semaforo
	xSemaphoreGiveFromISR(OIL_PRESSURE_Semaphore, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken == pdTRUE) {
		//garantir que o processador volta a tarefa de alta prioridade onde estava
		portYIELD_FROM_ISR();
	}
}

void loop() {
	vTaskDelete( NULL);
}
