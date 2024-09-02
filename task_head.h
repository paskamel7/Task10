#ifndef XC_HEADER_TEMPLATE_H
#define XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <avr/io.h>
void set_bit(volatile uint8_t *REG, uint8_t POS);
void clear_bit(volatile uint8_t *REG, uint8_t POS);
void inter_in(uint8_t PCMS, uint8_t POS);
void adc_init();
uint16_t adc_read();
uint8_t map(uint16_t x, uint16_t input_min, uint16_t input_max, uint8_t output_min, uint8_t output_max);
void pwm_init();
void pwm_set(uint8_t speed);
void encoder();
void speed();
void spi_init();
void spi_send(uint8_t my_data);
void send_data_by_spi(uint16_t rpm);
#endif /* XC_HEADER_TEMPLATE_H */