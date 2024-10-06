#include <Arduino.h>

#include "ad7175_8_regs.hpp"
#include "ad717x.hpp"

#include "ad5626.hpp"

#define ADC_CS_PIN 26
#define ADC_NUM_CHANNELS 4

#define CENTER 0x800000

AD717X ad7175;
int32_t data_val;

double voltage_val;

void setup() {
	Serial.begin(115200);
	SPI.begin();

	pinMode(1, OUTPUT);

  	ad717x_init_param ad7175_params;
	memset(&ad7175_params, 0, sizeof(ad717x_init_param));

	/* Pass in device registers */
	ad7175_params.regs = ad7175_8_regs;
	ad7175_params.num_regs = sizeof(ad7175_8_regs) / sizeof(ad717x_st_reg);

	ad7175_params.active_device = ID_AD7175_8;
	ad7175_params.ref_en = true;

	ad7175_params.num_channels = 16;
	ad7175_params.num_setups = 1;

	ad717x_analog_inputs chan_0_inputs;

	chan_0_inputs.ainp.pos_analog_input = AIN10;
	chan_0_inputs.ainp.neg_analog_input = AIN9;

	ad7175_params.chan_map[0] = {true, 0, chan_0_inputs};

	ad7175_params.setups[0] = {true, false, false, AVDD_AVSS};

	ad7175_params.pga[0] = 1;

	ad7175_params.filter_configuration[0].odr = sps_10;

	ad7175_params.mode = CONTINUOUS;

  	ad7175.init(&ad7175_params, &SPI, ADC_CS_PIN);
}

void loop() {
	ad7175.waitForReady(0xFFFFFFFF);
	ad7175.readData(&data_val);

	voltage_val = ((double) (data_val - CENTER)) / 8388608 * 5;
	Serial.print("New data value: ");
	Serial.print(voltage_val);
	Serial.println("\n");
}