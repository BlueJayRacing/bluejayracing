#include <Arduino.h>

#include "ad717x.hpp"

#include "ad5626.hpp"

#define ADC_CS_PIN 10

#define CENTER 0x800000

AD717X ad7175;
int32_t data_val;

double voltage_val;

int num = 0;
int start_millis;

void setup() {
	delay(2000);
	Serial.begin(115200);
	SPI.begin();

	pinMode(1, OUTPUT);

  	ad717x_init_param_t ad7175_params;

	ad717x_analog_inputs_t chan_0_inputs;
	chan_0_inputs.ainp.pos_input = AIN10;
	chan_0_inputs.ainp.neg_input = AIN9;

	ad717x_channel_map_t chan_0_map = {true, 0, chan_0_inputs};
	ad717x_channel_setup_t chan_0_setup = {true, false, false, AVDD_AVSS};
	ad717x_filtcon_t filt_con = {false, false, SPS27_DB47_MS36P7, SINC5_SINC1, SPS_100};
	ad717x_filt_pga_setup_t setup = {0.5, chan_0_setup, filt_con};

	ad7175_params.active_device = ID_AD7175_8;
	ad7175_params.ref_en = true;
	ad7175_params.mode = CONTINUOUS;

	ad7175_params.chan_map.push_back(chan_0_map);
	ad7175_params.setups.push_back(setup);

  	ad7175.init(ad7175_params, &SPI, ADC_CS_PIN);

	start_millis = millis();
}

void loop() {
	ad7175.waitForReady(0xFFFFFFFF);
	ad7175.readData(&data_val);

	voltage_val = ((double) (data_val) - CENTER) / CENTER * 5;
	num++;

	Serial.println(voltage_val, 6);
}