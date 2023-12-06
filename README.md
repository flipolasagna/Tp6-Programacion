# ˗ˏˋTp6- Programacion´ˎ˗
# ˗ˏˋInforme´ˎ˗
 
 ##  ⿻Codigo 
#### » lo primero a realizar fue declarar y definir las macros y funciones.
```c
#include <Arduino.h>



void config_EntradaSalida(void);
void config_EntradaSalida(void)
{
 DDRB |= (1 << PB0); // Defino g
 DDRD |= (1 << PD2); // Defino f
 DDRD |= (1 << PD3); // Defino e
 DDRD |= (1 << PD4); // Defino d
 DDRD |= (1 << PD5); // Defino c
 DDRD |= (1 << PD6); // Defino b
 DDRD |= (1 << PD7); // Defino a

  DDRC |= (1 << PC1); // En esta linea configuro como salida
  DDRC |= (1 << PC2); // En esta linea configuro como salida
  DDRC |= (1 << PC3); // En esta linea configuro como salida
  DDRC |= (1 << PC4); // En esta linea configuro como salida
  DDRC |= (1<< PC0);


}

#define set_bit(sfr, bit) sfr |= (1<<bit)
#define clear_bit(sfr, bit) sfr &=~ (1<<bit)

void config_TIMER0(void);



int8_t numbers[] = {0b00000001, 0b11100101, 0b10010000, 0b11000000, 0b01100100, 0b01001000, 0b00001000, 0b11100001, 0b0000000, 0b01100000, 0b00100000, 0b00001100,0b00011001, 0b10000100, 0b00011000,0b00111000};// Defino el vector

```

#### » Lo segundo a realizar fue el codigo dentro del main y lo demas.
```c
#include <Arduino.h>
#include "macros.h"
#include "util/delay.h"

void mulx(uint16_t data);
int main()
{
  config_EntradaSalida(); // configuro las entradas y salidas
  config_TIMER0();        // configuro el Timer1
  sei();                  // habilita las interrupciones

  while (1)
  {
  }
}
// Interrupcion: in ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR(TIMER0_COMPA_vect) // cual de las interrucion quiero
{
    _delay_ms(5);
    mulx(4321);
  }

//------------------------------------------------------------------------------------------
void mulx(uint16_t data)
{
  uint8_t DIS1, DIS2, DIS3, DIS4;
  static uint8_t mulxi = 0;

  DIS1 = (data / 1000);
  DIS2 = (data / 100) % 10;
  DIS3 = (data / 10) % 10;
  DIS4 = data % 10;

  switch (mulxi) // el switch se activa dependiendo el valor de la variable "mulxi"
  {

  case 0: // Determino la secuencia en el Caso 1, haciendo la cara del dado correspondiente
    clear_bit(PORTC, PC4);
    PORTD = (PORTD & 0b00000011) | (numbers[DIS4] & 0b11111100); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTD)
    PORTB = (PORTB & 0b11111110) | (numbers[DIS4] & 0b00000001); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTB)
    set_bit(PORTC, PC1);
    mulxi = 1;
    break;

  case 1: // Determino la secuencia en el Caso 2, haciendo la cara del dado correspondiente
    clear_bit(PORTC, PC1);
    PORTD = (PORTD & 0b00000011) | (numbers[DIS3] & 0b11111100); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTD)
    PORTB = (PORTB & 0b11111110) | (numbers[DIS3] & 0b00000001); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTB)
    set_bit(PORTC, PC2);
    mulxi = 2;
    break;

  case 2: // Determino la secuencia en el Caso 3, haciendo la cara del dado correspondiente
    clear_bit(PORTC, PC2);
    PORTD = (PORTD & 0b00000011) | (numbers[DIS2] & 0b11111100); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTD)
    PORTB = (PORTB & 0b11111110) | (numbers[DIS2] & 0b00000001); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTB)
    set_bit(PORTC, PC3);
    mulxi = 3;
    break;

  case 3: // Determino la secuencia en el Caso 4, haciendo la cara del dado correspondiente
    clear_bit(PORTC, PC3);
    PORTD = (PORTD & 0b00000011) | (numbers[DIS1] & 0b11111100); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTD)
    PORTB = (PORTB & 0b11111110) | (numbers[DIS1] & 0b00000001); // |--> ENMASCARAMIENTO: El enmascaramiento sirve para utilizar distintos pines de distintos puertos(PORTB)
    set_bit(PORTC, PC4);
    mulxi = 0;
    break;
  }
}


```


###### ↳ Asignacion del timer
```c
// asignacion de timer
void config_TIMER0(void)
{

  // TCCR0A = 0xFF;
  TCCR0A = 0b00000000; // CTC
  TCCR0B = 0b00001011; // clk/64.
  OCR0A = 249;
  TIMSK0 = (1 << OCIE1A);
}
´´´

