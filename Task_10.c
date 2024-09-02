#include <xc.h>
#include "task_head.h"
#include <avr/interrupt.h>
#include <util/delay.h>
uint32_t rev = 0;
uint32_t rpm = 0;
uint8_t encoder_flag = 0;
void set_bit(volatile uint8_t *REG,uint8_t POS)
{
    *REG |= (1<<POS);
}
void clear_bit(volatile uint8_t *REG,uint8_t POS)
{
     *REG &= ~(1<<POS);

}
void adc_init()
{
    set_bit(&ADMUX, REFS0);
    set_bit(&ADCSRA, ADEN);
    set_bit(&ADCSRA, ADPS0);
    set_bit(&ADCSRA, ADPS1);
    set_bit(&ADCSRA, ADPS2);
}

uint16_t adc_read()
{
    set_bit(&ADMUX,MUX1);
    set_bit(&ADCSRA, ADSC);
    while(ADCSRA & (1 << ADSC));
    return ADC;
}

uint8_t map(uint16_t x, uint16_t input_min, uint16_t input_max, uint8_t output_min, uint8_t output_max)
{
    uint32_t scale = (uint32_t)(x - input_min) * (output_max - output_min);
    scale /= (input_max - input_min);
    scale += output_min;
    
    return (uint8_t)scale;
}

void pwm_init()
{
    set_bit(&DDRD,DDD6);
    set_bit(&TCCR0A, COM0A1);
    set_bit(&TCCR0A, WGM00);
    set_bit(&TCCR0A, WGM01);
    set_bit(&TCCR0B, CS00); 
}
void pwm_set(uint8_t speed)
{
    OCR0A = speed;
    
}
void encoder()
{
    clear_bit(&DDRD,DDD2);
    clear_bit(&DDRD,DDD3);
    
    set_bit(&PORTD, PORTD2);
    set_bit(&PORTD, PORTD3);
    
    set_bit(&EICRA, ISC10);
    set_bit(&EICRA, ISC11);
    
    set_bit(&EICRA, ISC00);
    set_bit(&EICRA, ISC01);
    
    set_bit(&EIMSK, INT0);
    set_bit(&EIMSK, INT1);

}
void speed()
{
    _delay_ms(100);
    uint8_t pulses_per_rev = 24;
    rpm = (rev * 600) / pulses_per_rev ;
    rev = 0;
}
void spi_init()
{
    DDRB |= (1 << DDB5) | (1 << DDB3) | (1 << DDB2);
    clear_bit(&DDRB, DDB4);
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void spi_send(uint8_t my_data)
{
    SPDR = my_data;
    while(!(SPSR & (1 << SPIF)));
}

void send_data_by_spi(uint16_t rpm)
{
    cli();
    uint8_t rpm_high = (rpm >> 8) & 0xFF;
    uint8_t rpm_low = rpm & 0xFF;
    spi_send(rpm_high);
    spi_send(rpm_low);
    sei();

}
int main(void) {
    clear_bit(&DDRC,DDC0);
    set_bit(&PORTC,PORTC0);
    set_bit(&DDRB,DDB0);
    clear_bit(&DDRC,DDC1);
    set_bit(&PORTC,PORTC1);
    set_bit(&DDRB,DDB1);
    
    
    cli();
    set_bit(&PCICR, PCIE1);
    set_bit(&PCMSK1, PCINT8);
    set_bit(&PCMSK1, PCINT9);
    sei();
    
 
    adc_init();
    pwm_init();
    spi_init();
    uint16_t adc_value;
    uint8_t map_value;
    
    
   
    while(1)
    {
        

        while(!(PINC & (1<<PINC0)));
        clear_bit(&PORTB, PORTB0);
        while(!(PINC & (1<<PINC1)));
        clear_bit(&PORTB, PORTB1);
        
        
        adc_value = adc_read();
        map_value = map(adc_value,0, 1023, 0, 255);
        pwm_set(map_value);
        
        encoder();
        
        speed();
        send_data_by_spi(rpm);
       
    }
    
}


ISR(PCINT1_vect)
{
    if (!(PINC & (1 << PINC0)))
    {
        set_bit(&PORTB, PORTB0); 
    }
    else if (!(PINC & (1 << PINC1)))
    {
        set_bit(&PORTB, PORTB1); 
    }
}
ISR(INT0_vect)
{
    if(PIND & (1<<PIND3))
    {
        rev++;
    }
    else
    {
        rev--;
    }
    encoder_flag = 1;
}
ISR(INT1_vect)
{
    
}