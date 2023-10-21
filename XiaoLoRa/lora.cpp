#include "lora.h"

#ifdef RP2040
#define LogPrintf printf
#else
#include <Arduino.h>
void LogPrintf(const char *format, ...) {
  char buf[PRINTF_BUF];
  va_list ap;
  va_start(ap, F(format));
  vsnprintf(buf, sizeof(buf), format, ap); 
  va_end(ap);
  Serial.print(buf);
};
#endif


Lora::Lora() {
    LogPrintf("[Lora] Loading driver...\n");

    #ifdef RP2040
    context.busy = 6;
    context.reset = 0;
    context.nss = 1;
    context.irq = 25;
    context.sck = 18;
    context.mosi = 19;
    context.miso = 20;
    #else
    context.busy = 0;
    context.reset = 1;
    context.nss = 7;
    context.irq = 2;
    context.sck = 8;
    context.mosi = 10;
    context.miso = 9;
    #endif

    hal_gpio_init(context.nss, GPIO_OUT, 1);
    hal_gpio_init(context.reset, GPIO_OUT, 0);

    #ifdef RP2040
    context.spi = 0;
    int br = spi_init(context.spi ? spi1 : spi0, 16 * 1000 * 1000);
    printf("[Lora] SX1262 baudrate: %d\n", br);

    spi_set_format(context.spi ? spi1 : spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(context.sck, GPIO_FUNC_SPI);
    gpio_set_function(context.mosi, GPIO_FUNC_SPI);
    gpio_set_function(context.miso, GPIO_FUNC_SPI);
    #endif

    hal_gpio_init(context.busy, GPIO_IN, 0);
    hal_gpio_init(context.irq, GPIO_IN, 0);

    sx126x_reset(&context);
    sx126x_wakeup(&context);

    sx126x_set_reg_mode(&context, SX126X_REG_MODE_DCDC);
    sx126x_set_standby(&context, SX126X_STANDBY_CFG_RC);
    sx126x_set_pkt_type(&context, SX126X_PKT_TYPE_LORA);
    // sx126x_set_dio3_as_tcxo_ctrl(&context, SX126X_TCXO_CTRL_1_8V, 32);

    sx126x_cal(&context, SX126X_CAL_ALL);
    sx126x_set_standby(&context, SX126X_STANDBY_CFG_XOSC);

    sx126x_pa_cfg_params_t pa_params = {
		.pa_duty_cycle = 0x04,
		.hp_max = 0x07,
		.device_sel = 0x00,
		.pa_lut = 0x01
	};
    sx126x_set_pa_cfg(&context, &pa_params);
    sx126x_set_ocp_value(&context, SX126X_OCP_PARAM_VALUE_140_MA);
    sx126x_cfg_tx_clamp(&context);
    sx126x_set_tx_params(&context, 20, SX126X_RAMP_200_US);
    sx126x_set_rf_freq(&context, 868 * 1000 * 1000);
    sx126x_cal_img(&context, 0xD7, 0xDB);

    sx126x_set_buffer_base_address(&context, 0x00, 0x00);
    sx126x_set_pkt_type(&context, SX126X_PKT_TYPE_LORA);
    sx126x_set_rx_tx_fallback_mode(&context, SX126X_FALLBACK_FS);

    sx126x_mod_params_lora_t lora_mod_params = {
		.sf = SX126X_LORA_SF12,
		.bw = SX126X_LORA_BW_062,
		.cr = SX126X_LORA_CR_4_5,
		.ldro = 1
	};
    sx126x_set_lora_mod_params(&context, &lora_mod_params);

    sx126x_pkt_params_lora_t lora_pkt_params = {
		.preamble_len_in_symb = 64,
		.header_type = SX126X_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = 32,
		.crc_is_on = 1,
		.invert_iq_is_on = 0
	};
	sx126x_set_lora_pkt_params(&context, &lora_pkt_params);

  static const uint16_t lora_irq_mask_dio1 = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERROR;
	sx126x_set_dio_irq_params(&context, lora_irq_mask_dio1, lora_irq_mask_dio1, 0, 0);

	sx126x_cfg_rx_boosted(&context, true);
	sx126x_set_tx_infinite_preamble(&context);
  sx126x_set_rx(&context, SX126X_MAX_TIMEOUT_IN_MS);

	sx126x_clear_irq_status(&context, SX126X_IRQ_ALL);

  LogPrintf("[Lora] Done\n");
}

Lora::~Lora() {
    
}

void Lora::SendData(char* data, uint8_t length) {
	sx126x_pkt_params_lora_t lora_pkt_params = {
		.preamble_len_in_symb = 16,
		.header_type = SX126X_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = length,
		.crc_is_on = 0,
		.invert_iq_is_on = 0
	};

	// printf("[Lora] Sending packet: len=%zu\n", length);

	sx126x_set_lora_pkt_params(&context, &lora_pkt_params);
	sx126x_write_buffer(&context, 0, (uint8_t*)data, length);
	sx126x_set_tx(&context, SX126X_MAX_TIMEOUT_IN_MS);
    // printf("[Lora] Done Sending\n");
}

void Lora::ProcessIrq() {
	sx126x_irq_mask_t irq_mask;
	sx126x_get_irq_status(&context, &irq_mask);
	sx126x_clear_irq_status(&context, irq_mask);

	if (irq_mask & SX126X_IRQ_TX_DONE) {
		LogPrintf("[Lora] IRQ TX_DONE\n");
	}

	if (irq_mask & SX126X_IRQ_RX_DONE) {
		LogPrintf("[Lora] IRQ RX_DONE\n");
	}

	if (irq_mask & SX126X_IRQ_PREAMBLE_DETECTED) {
		LogPrintf("[Lora] IRQ PREAMBLE_DETECTED\n");
	}

	if (irq_mask & SX126X_IRQ_SYNC_WORD_VALID) {
		LogPrintf("[Lora] IRQ SYNC_WORD_VALID\n");
	}

	if (irq_mask & SX126X_IRQ_HEADER_VALID) {
		LogPrintf("[Lora] IRQ HEADER_VALID\n");
	}

	if (irq_mask & SX126X_IRQ_HEADER_ERROR) {
		LogPrintf("[Lora] IRQ HEADER_ERROR\n");
	}

	if (irq_mask & SX126X_IRQ_CRC_ERROR) {
		LogPrintf("[Lora] IRQ CRC_ERROR\n");
	}

	if (irq_mask & SX126X_IRQ_CAD_DONE) {
		LogPrintf("[Lora] IRQ CAD_DONE\n");
	}

	if (irq_mask & SX126X_IRQ_CAD_DETECTED) {
		LogPrintf("[Lora] IRQ CAD_DETECTED\n");
	}

	if (irq_mask & SX126X_IRQ_TIMEOUT) {
		LogPrintf("[Lora] IRQ TIMEOUT\n");
	}
}

void Lora::RecieveData() {
  sx126x_pkt_status_lora_t last_rcvd_pkt_status;

	sx126x_get_lora_pkt_status(&context, &last_rcvd_pkt_status);
  LogPrintf("[Lora] Process_RX: RF status: RSSI: %d, SNR: %d, RSCP: %d\n", last_rcvd_pkt_status.rssi_pkt_in_dbm, last_rcvd_pkt_status.snr_pkt_in_db, last_rcvd_pkt_status.signal_rssi_pkt_in_dbm);

	sx126x_rx_buffer_status_t rx_buf_stat;
	uint8_t last_pld_len = 255;

	do {
		sx126x_get_rx_buffer_status(&context, &rx_buf_stat);
		if (rx_buf_stat.pld_len_in_bytes == last_pld_len) {
			break;
		} else {
			last_pld_len = rx_buf_stat.pld_len_in_bytes;
		}
	} while (1);

//		sx126x_get_rx_buffer_status(NULL, &rx_buf_stat);

	uint8_t buf[256];
	sx126x_read_buffer(&context, 0, buf, rx_buf_stat.pld_len_in_bytes);

}
