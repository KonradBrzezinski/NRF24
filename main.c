/**
 * Copyright (C) 2021, A. Ridyard.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as
 * published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GNU General Public License for more details.
 * 
 * @file primary_receiver.c
 * 
 * @brief example of NRF24L01 setup as a primary receiver using the 
 * NRF24L01 driver. Different payload widths are set for the data pipes,
 * in order to receive different data structures from the transmitter.
 */

#include <stdio.h>
#include <stdlib.h>

#include "nrf24_driver.h"
#include "pico/stdlib.h"
#include "adxl345.h"
#include "pico/time.h"
#include "savgolFilter.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#define START_ADXL 0xF0
#define START_GUN 0x0F
#define TIME_SYNCH 0b01010101

#define INTERVAL_US 1000
#define BUZZER_TIME_ON 200

#define ALPHA 0.7

#define GPS_PIN 15
#define BUZZER_PIN 14

#define RED_PIN 18
#define GREEN_PIN 19
#define BLUE_PIN 20

#define BUFF_SIZE 500
#define DATA_SIZE 500 + 1

typedef struct{
  int16_t data[BUFF_SIZE];
  uint16_t head;
  uint16_t count;
}circ_buffer_t;

uint64_t synch_time_1 =     0;
uint64_t synch_time_2 =     0;
uint64_t time_start   =     0;
uint64_t time_end     =     0;
int16_t  reaction     =   999;
bool synch            = false;
bool start_adxl       = false;
bool start_GUN        = false;
bool started          = false;
bool set_receiver     = false;
bool receive          =  true;
bool send_data        = false;

// GPIO pin numbers
pin_manager_t my_pins = { 
  .sck = 2,
  .copi = 3, 
  .cipo = 4, 
  .csn = 5, 
  .ce = 6 
};
/**
 * nrf_manager_t can be passed to the nrf_client_t
 * initialise function, to specify the NRF24L01 
 * configuration. If NULL is passed to the initialise 
 * function, then the default configuration will be used.
 */
nrf_manager_t my_config = {
  // RF Channel 
  .channel = 120,
  // AW_3_BYTES, AW_4_BYTES, AW_5_BYTES
  .address_width = AW_5_BYTES,
  // dynamic payloads: DYNPD_ENABLE, DYNPD_DISABLE
  .dyn_payloads = DYNPD_ENABLE,
  // data rate: RF_DR_250KBPS, RF_DR_1MBPS, RF_DR_2MBPS
  .data_rate = RF_DR_1MBPS,
  // RF_PWR_NEG_18DBM, RF_PWR_NEG_12DBM, RF_PWR_NEG_6DBM, RF_PWR_0DBM
  .power = RF_PWR_NEG_12DBM,
  // retransmission count: ARC_NONE...ARC_15RT
  .retr_count = ARC_10RT,
  // retransmission delay: ARD_250US, ARD_500US, ARD_750US, ARD_1000US
  .retr_delay = ARD_500US 
};
// SPI baudrate
uint32_t my_baudrate = 5000000;

int16_t raw_data[DATA_SIZE];
int16_t raw_data_together[DATA_SIZE + BUFF_SIZE];
MqsRawDataPoint_t rawDataSavgol[BUFF_SIZE + DATA_SIZE];
MqsRawDataPoint_t filteredData[BUFF_SIZE + DATA_SIZE];
uint16_t counter = 0;

struct repeating_timer timer;
nrf_client_t my_nrf;

circ_buffer_t circ_buffer;
uint16_t *buffer = NULL;

void buffer_init(circ_buffer_t *buff){
  buff->head = 0;
  buff->count = 0;
  for(int i = 0; i < BUFF_SIZE; i++) { buff->data[i] = 0; }
}

void buffer_put(circ_buffer_t *buff, int16_t value){
  buff->data[buff->head] = value;
  buff->head = (buff->head + 1) % BUFF_SIZE;

  if(buff->count < BUFF_SIZE){
    buff->count++;
  }
}

void buffer_clear(circ_buffer_t *buff){
  buff->head = 0;
  buff->count = 0;
  for(int i = 0; i < BUFF_SIZE; i++) { buff->data[i] = 0; }
}

int64_t turn_off_buzzer_callback(alarm_id_t id, void *user_data){
  gpio_put(BUZZER_PIN, 0);
  return 0;
}

int64_t turn_off_leds_callback(alarm_id_t id, void *user_data){
  gpio_put(RED_PIN, 1);
  gpio_put(BLUE_PIN, 1);
  gpio_put(GREEN_PIN, 0);
  return 0;
}

void gps_callback(uint gpio, uint32_t events){
  if(gpio == GPS_PIN){
    synch_time_2 = synch_time_1;
    synch_time_1 = to_us_since_boot(get_absolute_time());
    printf("Synch via GPS: %lldus\n", synch_time_1 - synch_time_2);
  }
}

void falseStart(){
  gpio_put(BUZZER_PIN, 1);
  add_alarm_in_ms(1000, turn_off_buzzer_callback, NULL, false);
}

int16_t check_reaction(MqsRawDataPoint_t *data){
  int16_t reaction = 0;
  uint8_t N = 4;
  int16_t slope;

  for(int i = N; i < BUFF_SIZE + DATA_SIZE; i++){
    slope = abs((int16_t)((data[i].phaseAngle - data[i-N].phaseAngle) / N));
    if(slope == 3){
      return i - BUFF_SIZE;
    }
  }

  return reaction;
}

void filter_data(){
  uint8_t halfWindowSize = 16;
  uint8_t polynomialOrder = 3;
  uint8_t targetPoint = 0;
  uint8_t derivativeOrder = 0;

  for(size_t i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
    rawDataSavgol[i].phaseAngle = raw_data_together[i];
    filteredData[i].phaseAngle = 0.0f;
  }

  mes_savgolFilter(rawDataSavgol, DATA_SIZE + BUFF_SIZE, halfWindowSize, filteredData, polynomialOrder, targetPoint, derivativeOrder);
}

void measure(){
  if(start_adxl && !start_GUN){
    buffer_put(&circ_buffer, ADXL345_read_X_g());
  }
  // printf("ADXL: %d, GUN: %d\n", start_adxl, start_GUN);
  if(start_adxl && start_GUN){
    raw_data[counter++] = ADXL345_read_X_g();
    if(counter == DATA_SIZE){
      counter = 0;
      cancel_repeating_timer(&timer);
      start_adxl = false;
      start_GUN = false;
      send_data = true;

      for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
        if(i < BUFF_SIZE){
          raw_data_together[i] = circ_buffer.data[i];
        }else{
          raw_data_together[i] = raw_data[i - BUFF_SIZE];
        }
      }

      filter_data();

      reaction = check_reaction(filteredData);
      printf("Reaction %d\n", reaction);
    }
  }
}

bool timer_callback(struct repeating_timer *t){
  measure();
  // if(start_adxl && !start_GUN){
  //   buffer_put(&circ_buffer, ADXL345_read_X_g());
  // }

  // if(start_adxl && start_GUN){
  //   raw_data[counter++] = ADXL345_read_X_g();
  //   if(counter == DATA_SIZE){
  //     counter = 0;
  //     cancel_repeating_timer(&timer);
  //     start_adxl = false;
  //     start_GUN = false;
  //     send_data = true;
      
  //     for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
  //       if(i < BUFF_SIZE){
  //         raw_data_together[i] = circ_buffer.data[i];
  //       }else{
  //         raw_data_together[i] = raw_data[i - BUFF_SIZE];
  //       }
  //     }

  //     filter_data();

  //     reaction = check_reaction(filteredData);
  //     printf("Reaction %d\n", reaction);
  //   }
  // }

  return true;
}

void init_led_pins(){
  gpio_init(RED_PIN);
  gpio_init(GREEN_PIN);
  gpio_init(BLUE_PIN);

  gpio_set_dir(RED_PIN, GPIO_OUT);
  gpio_set_dir(GREEN_PIN, GPIO_OUT);
  gpio_set_dir(BLUE_PIN, GPIO_OUT);

  gpio_put(RED_PIN, 1);
  gpio_put(GREEN_PIN, 1);
  gpio_put(GREEN_PIN, 1);
}

int main(void)
{
  stdio_init_all();

  buffer_init(&circ_buffer);

  ADXL345_init();

  nrf_driver_create_client(&my_nrf);
  my_nrf.configure(&my_pins, my_baudrate);
  my_nrf.initialise(&my_config);
  my_nrf.rx_destination(DATA_PIPE_0, (uint8_t[]){0x37,0x37,0x37,0x37,0x37});
  my_nrf.receiver_mode();

  uint8_t pipe_number = 0;
  uint8_t payload_zero = 0;

  bool send_reaction = false;
  bool falseStartFired = false;

  gpio_init(GPS_PIN);
  gpio_set_dir(GPS_PIN, GPIO_IN);

  gpio_init(BUZZER_PIN);
  gpio_set_dir(BUZZER_PIN, GPIO_OUT);

  // init_led_pins();

  gpio_set_irq_enabled_with_callback(GPS_PIN, GPIO_IRQ_EDGE_RISE, true, &gps_callback);

  while (1)
  {
    if (my_nrf.is_packet(NULL))
    {
      printf("JEST PAKET\n");
      if(my_nrf.read_packet(&payload_zero, sizeof(payload_zero))){
        printf("PACKET: %x\n", payload_zero);
      }

      if(payload_zero == START_ADXL){
        // gpio_put(GREEN_PIN, 0);
        start_adxl = true;
        start_GUN = false;
        falseStartFired = false;
        reaction = 9999;
        add_repeating_timer_us(INTERVAL_US, timer_callback, NULL, &timer);
        buffer_clear(&circ_buffer);
      }
      if(payload_zero == START_GUN){
        start_GUN = true;
        
        gpio_put(BUZZER_PIN, 1);
        add_alarm_in_ms(BUZZER_TIME_ON, turn_off_buzzer_callback, NULL, false);
      }
      if(payload_zero == TIME_SYNCH){
        // gpio_put(BLUE_PIN, 0);
        cancel_repeating_timer(&timer);
        add_repeating_timer_us(INTERVAL_US, timer_callback, NULL, &timer);
        // add_alarm_in_ms(1000, turn_off_leds_callback, NULL, false);
      }
    }

    if(reaction < 100 && !falseStartFired){
      falseStartFired = true;
      falseStart();
    }

    if(send_data){
      // gpio_put(BLUE_PIN, 0);
      send_data = false;
      my_nrf.standby_mode();
      my_nrf.tx_destination((uint8_t[]){0xC3,0xC3,0xC3,0xC3,0xC3});
      char msg[32];
      int counter = (-1) * BUFF_SIZE;
      // sprintf(msg, "R%d", reaction);
      // my_nrf.send_packet(&msg, sizeof(msg));
      for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
        sprintf(msg, "%d ", (counter++));
        my_nrf.send_packet(&msg, sizeof(msg));
        // sprintf(msg, "%d\n", raw_data_together[i]);
        sprintf(msg, "%d\n", abs((int16_t)filteredData[i].phaseAngle));
        my_nrf.send_packet(&msg, sizeof(msg));
      }

      sprintf(msg, "R%d\n", reaction);
      my_nrf.send_packet(&msg, sizeof(msg));
      
      my_nrf.receiver_mode();

      buffer_clear(&circ_buffer);
      reaction = 9999;
      // gpio_put(BLUE_PIN, 1);
    }

    __asm volatile ("dmb");
    tight_loop_contents();
  }
  return 0;
}