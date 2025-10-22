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

#include "nrf24_driver.h"
#include "pico/stdlib.h"
#include "adxl345.h"
#include "pico/time.h"
#include <stdlib.h>

#define START_ADXL 0xF0
#define START_GUN 0x0F
#define TIME_SYNCH 0b01010101

#define ALPHA 0.7

#define GPS_PIN 15
#define BUZZER_PIN 14

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
bool synch            = false;
bool start_adxl       = false;
bool start_GUN        = false;
bool started          = false;

int16_t raw_data[DATA_SIZE];
uint16_t counter = 0;

circ_buffer_t circ_buffer;
struct repeating_timer timer;

extern uint16_t buffer[BUFF_SIZE];
// int16_t reaction = 0;

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

bool buff_get_last(circ_buffer_t *buff, int16_t *value){
  if(buff->count == 0){
    return false;
  }

  uint16_t index = (buff->head == 0) ? (BUFF_SIZE - 1) : (buff->head - 1);
  *value = buff->data[index];
  return true;
}

void buffer_process_all(circ_buffer_t *buff, void (*callback)(int16_t value)){
  if(buff->count == 0){
    return;
  }

  uint16_t start_index;

  if(buff->count < BUFF_SIZE){
    start_index = 0;
  }else{
    start_index = buff->head;
  }

  for(uint16_t i = 0; i < buff->count; i++){
    uint16_t current_index = (start_index + i) % BUFF_SIZE;
    callback(buff->data[current_index]);
  }
}

void buffer_clear(circ_buffer_t *buff){
  buff->head = 0;
  buff->count = 0;
  for(int i = 0; i < BUFF_SIZE; i++) { buff->data[i] = 0; }
}

void gps_callback(uint gpio, uint32_t events){
  if(gpio == GPS_PIN){
    synch_time_2 = synch_time_1;
    synch_time_1 = to_us_since_boot(get_absolute_time());
    printf("Synch via GPS: %lldus\n", synch_time_1 - synch_time_2);
  }
}

int16_t low_pass_filter(int16_t input, int16_t *previous_output){
  int16_t output = ALPHA * input + (1.0 - ALPHA) * (*previous_output);
  *previous_output = output;
  return output;
}

int16_t *filter_data(int16_t *data, size_t size){
  if(size == 0) return data;

  int16_t filter_state = data[0];
  for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
    data[i] = low_pass_filter(data[i], &filter_state);
  }

  return data;
}

int16_t check_reaction(int16_t data[]){
  // FILTERING, DECISION, SIGNAL

  // int16_t filtered_data[DATA_SIZE];

  // filtered_data[0] = 0;

  // for(int i = 1; i < DATA_SIZE+BUFF_SIZE; i++){
  //   filtered_data[i] = 0.1f * raw_data[i] + (1.0f - 0.1f)*filtered_data[i-1];
  //   printf("%d %d\n", i, filtered_data[i]);
  // }

  int16_t reaction = 0;

  for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
    if(abs(data[i]) > 30){
      reaction = i - BUFF_SIZE;
      return reaction;
    }
  }

  return reaction;
}

bool timer_callback(struct repeating_timer *t){

  if(start_adxl && !start_GUN){
    buffer_put(&circ_buffer, ADXL345_read_X_g());
  }

  if(start_adxl && start_GUN){
    // if(started){
    //   started = false;
    //   // time_start = to_us_since_boot(get_absolute_time());
    // }
    raw_data[counter++] = ADXL345_read_X_g();
    if(counter == DATA_SIZE){
      // time_end = to_us_since_boot(get_absolute_time());
      cancel_repeating_timer(&timer);
      start_adxl = false;
      start_GUN = false;
      
      int16_t raw_data_together[BUFF_SIZE + DATA_SIZE];
      // printf("Czas pomiaru: %lld\n", time_end - time_start);

      for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
        if(i < BUFF_SIZE){
          raw_data_together[i] = circ_buffer.data[i];
        }else{
          raw_data_together[i] = raw_data[i - BUFF_SIZE];
        }
      }

      int16_t reaction = check_reaction(raw_data_together);

      buffer_clear(&circ_buffer);
      counter = 0;

      for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
        printf("%d %d\n", i - BUFF_SIZE, raw_data_together[i]);
      }
      printf("REACTION: %d\n", reaction);
    }
  }

  return true;
}

int64_t turn_off_buzzer_callback(alarm_id_t id, void *user_data){
  gpio_put(BUZZER_PIN, 0);
  return 0;
}

int main(void)
{
  // initialize all present standard stdio types
  stdio_init_all();

  buffer_init(&circ_buffer);

  ADXL345_init();

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

  // provides access to driver functions
  nrf_client_t my_nrf;

  // initialise my_nrf
  nrf_driver_create_client(&my_nrf);

  // configure GPIO pins and SPI
  my_nrf.configure(&my_pins, my_baudrate);

  // not using default configuration (my_nrf.initialise(NULL)) 
  my_nrf.initialise(&my_config);

  /**
   * set addresses for DATA_PIPE_0 - DATA_PIPE_3.
   * These are addresses the transmitter will send its packets to.
   */
  my_nrf.rx_destination(DATA_PIPE_0, (uint8_t[]){0x37,0x37,0x37,0x37,0x37});
  my_nrf.rx_destination(DATA_PIPE_1, (uint8_t[]){0xC7,0xC7,0xC7,0xC7,0xC7});
  my_nrf.rx_destination(DATA_PIPE_2, (uint8_t[]){0xC8,0xC7,0xC7,0xC7,0xC7});
  my_nrf.rx_destination(DATA_PIPE_3, (uint8_t[]){0xC9,0xC7,0xC7,0xC7,0xC7});

  // set to RX Mode
  my_nrf.receiver_mode();

  // data pipe number a packet was received on
  uint8_t pipe_number = 0;

  // holds payload_zero sent by the transmitter
  uint8_t payload_zero = 0;

  // holds payload_one sent by the transmitter
  uint8_t payload_one[5];

  // two byte struct sent by transmitter
  typedef struct payload_two_s { uint8_t one; uint8_t two; } payload_two_t;

  // holds payload_two struct sent by the transmitter
  payload_two_t payload_two;

  // uint64_t start_time = to_us_since_boot(get_absolute_time());

  gpio_init(GPS_PIN);
  gpio_set_dir(GPS_PIN, GPIO_IN);

  gpio_init(BUZZER_PIN);
  gpio_set_dir(BUZZER_PIN, GPIO_OUT);

  gpio_set_irq_enabled_with_callback(GPS_PIN, GPIO_IRQ_EDGE_RISE, true, &gps_callback);

  while (1)
  {
    if (my_nrf.is_packet(&pipe_number))
    {
      // switch (pipe_number)
      // {
      //   case DATA_PIPE_0:
      //     // read payload
      //     my_nrf.read_packet(&payload_zero, sizeof(payload_zero));

      //     // receiving a one byte uint8_t payload on DATA_PIPE_0
      //     // printf("\nPacket received:- Payload (%d) on data pipe (%d)\n", payload_zero, pipe_number);
      //     if(payload_zero & 0x80){
      //       synch_time_1 = to_us_since_boot(get_absolute_time());
      //       printf("Synch time 1: %lld\n", synch_time_1);
      //     }
      //     if(payload_zero & 0x01){
      //       synch_time_2 = to_us_since_boot(get_absolute_time());
      //       printf("Synch time 1: %lld\n", synch_time_2);
      //       printf("Synchronized time: %llu\n", synch_time_2 - synch_time_1);
      //     }
      //   break;
      // }
      my_nrf.read_packet(&payload_zero, sizeof(payload_zero));

          // receiving a one byte uint8_t payload on DATA_PIPE_0
          // printf("\nPacket received:- Payload (%d) on data pipe (%d)\n", payload_zero, pipe_number);
      // if(payload_zero & 0x80){
      //   synch_time_1 = to_us_since_boot(get_absolute_time());
      //   printf("Synch time 1: %lld\n", synch_time_1);
      // }
      if(payload_zero == TIME_SYNCH){
        synch_time_2 = to_us_since_boot(get_absolute_time());
        printf("Synch time 1: %lld\n", synch_time_2);
        printf("Synchronized time: %llu\n", synch_time_2 - synch_time_1);
      }

      if(payload_zero == START_ADXL){
        start_adxl = true;
        // gpio_put(BUZZER_PIN, 1);
        buffer_clear(&circ_buffer);
        add_repeating_timer_ms(-1, timer_callback, NULL, &timer);
      }
      if(payload_zero == START_GUN){
        start_GUN = true;
        started = true;
        gpio_put(BUZZER_PIN, 1);
        add_alarm_in_ms(200, turn_off_buzzer_callback, NULL, false);
      }
    }

    // if(start_adxl && !start_GUN){
    //   if(if_data_diff(&last_val, x_val.x)){
    //     buffer_put(&circ_buffer, x_val.x);
    //   }
    // }

    // if(start_adxl && start_GUN){
    //   // if(started){
    //   //   started = false;
        
    //   // }
    //   time_start = to_us_since_boot(get_absolute_time());
    //   if(if_data_diff(&last_val, x_val.x) && counter < DATA_SIZE){
    //     raw_data[counter] = x_val.x;
    //     counter++;
    //   }
    //   time_end = to_us_since_boot(get_absolute_time());
    //   printf("Czas pomiaru od wystrzału: %lld\n", time_end - time_start);
    //   if(counter == DATA_SIZE){
        
    //     cancel_repeating_timer(&timer);
    //     start_adxl = false;
    //     start_GUN = false;
    //     counter = 0;

        

    //     // int16_t raw_data_together[BUFF_SIZE + DATA_SIZE];

    //     // for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
    //     //   if(i < BUFF_SIZE){
    //     //     raw_data_together[i] = circ_buffer.data[i];
    //     //   }else{
    //     //     raw_data_together[i] = raw_data[i - BUFF_SIZE];
    //     //   }
    //     // }

    //     // for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
    //     //   // printf("%d %d\n", i - BUFF_SIZE, raw_data_together[i]);
    //     // }
    //     // printf("Całkowity czas wykonania: %lld\n", time_end - time_start);
    //     buffer_clear(&circ_buffer);
    //   }
    // }
  }
}