//Wifi
#include <esp_wpa2.h>
#include <WiFi.h>
//Thingspeak
#include "ThingSpeak.h"
//Microfono
#include <driver/i2s.h>
//Filtro IIR
#include "simpleDSP.h"
//FFT header
#include "FFT.h"

//Credenciales Wifi
const char* ssid = "Tec";
#define EAP_IDENTITY "Correo"
#define EAP_PASSWORD "Contraseña"

WiFiClient  client;

//Credenciales ThingSpeak
unsigned long channelID = 1950129;
unsigned long myChannelNumber = 2;
const char * myWriteAPIKey = "K6URTH7F4A6WP3A5";
const char * myReadAPIKey = "41NCFI06BYF32RTQ";

//Parametros del microfono

#include <driver/i2s.h>
// you shouldn't need to change these settings
#define SAMPLE_BUFFER_SIZE 2048
#define SAMPLE_RATE 8000
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_23
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_22
#define I2S_MIC_SERIAL_DATA GPIO_NUM_21

// don't mess around with this
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 1024,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};

int32_t raw_samples[SAMPLE_BUFFER_SIZE];            //muestras del microfono
float muestras_anteriores[SAMPLE_BUFFER_SIZE];
float muestras_actuales[SAMPLE_BUFFER_SIZE];
float muestras_completas[SAMPLE_BUFFER_SIZE];

float coefB[7] =
{
  0.6306,
  -1.2612,
  -0.6306,
  2.5225,
  -0.6306,
  -1.2612,
  0.6306
};


float coefA[7] =
{
  1,
  -2.1285,
  0.2949,
  1.8242,
  -0.8057,
  -0.3947,
  0.2099
};

IIR iir1;

//Arreglos para la fft

float fft_input[SAMPLE_BUFFER_SIZE];
float fft_output[SAMPLE_BUFFER_SIZE];

//Variables para el tiempo

unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

void setup()
{
  //inicializacion del monitor serial
  Serial.begin(115200);

  //conexion a wifi

  // WPA2 enterprise magic starts here
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);   //init wifi mode
  Serial.printf("Connecting to WiFi: %s ", ssid);
  //esp_wifi_sta_wpa2_ent_set_ca_cert((uint8_t *)incommon_ca, strlen(incommon_ca) + 1);
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  //esp_wifi_sta_wpa2_ent_enable();
  esp_wpa2_config_t configW = WPA2_CONFIG_INIT_DEFAULT();
  esp_wifi_sta_wpa2_ent_enable(&configW);
  // WPA2 enterprise magic ends here
  WiFi.begin(ssid);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  //Inicializa I2S para el microfono

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);

  //Se llenan muestras anteriores con 0

  for (int a = 0; a < 1024; a++)
  {
    muestras_anteriores[a] = 0;
  }

  //Inicializa ThingSpeak
  ThingSpeak.begin(client);

}

void loop()
{
  //Se leen las muestras del microfono
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(int8_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int8_t);

  //Se inicializa el filtro IIR
  iirInit(&iir1, 7, coefB, 7, coefA);

  //Se inicializa la FFT
  fft_config_t *real_fft_plan = fft_init(SAMPLE_BUFFER_SIZE, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

  //Se llena el arreglo de todas la muestras con muestras anteriores y actuales

  for (int b = 0; b < 2048; b++)
  {
    //Se aplica el filtro a las muestras del microfono
    muestras_actuales[b] =  iirFilt(&iir1, raw_samples[b] / 16777216);
    if (b < 1024)
    {
      muestras_completas[b] = muestras_anteriores[b + 1024];
    }
    else {
      muestras_completas[b] = raw_samples[b - 1024];
    }
    muestras_anteriores[b] = raw_samples[b];   // Se pasan las muestras actuales a muestras anteriores
    real_fft_plan->input[b] = muestras_completas[b]; //Se llena el bufer de entrada de la FFT con todas la muestras
  }

  //Se ejecuta la fft y tambien llena el bufer de salida
  fft_execute(real_fft_plan);

  float suma = 0;
  for (int k = 1 ; k < real_fft_plan->size / 2 ; k++)
  {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
    float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
    suma = suma + pow(mag, 2);
  }

  //Se calculan los Decibeles

  //suma = suma / 1;
  float p = ((float)1 / (SAMPLE_BUFFER_SIZE / 2)) * suma;
  float dba = 10 * log(p) + 55;
  dba -= 370;
  if (dba >= 40 && dba <= 50 || dba >= 60 && dba <= 80 || dba >= 90 && dba <= 110) 
  {
    if (dba > 30 && dba < 40)
    {
      dba = dba + 8;
    }
    Serial.println(dba, 3);
  }

  fft_destroy(real_fft_plan);
  delay(500);

  // Se escribe a ThingSpeak
  if ((millis() - lastTime) > timerDelay && dba > 70)
  {
    float x = ThingSpeak.writeField(myChannelNumber, 2, dba, myWriteAPIKey);

    if (x == 200)
    {
      Serial.println("Channel update successful.");
    }
    else
    {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    lastTime = millis();
  }


}
