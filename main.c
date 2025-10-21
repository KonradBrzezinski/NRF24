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

uint64_t synch_time_1 = 0;
uint64_t synch_time_2 = 0;

#define START_ADXL 0xF0
#define START_GUN 0x0F
#define TIME_SYNCH 0b01010101

#define GPS_PIN 15
#define BUZZER_PIN 14

#define BUFF_SIZE 500
#define DATA_SIZE 500
extern uint16_t buffer[BUFF_SIZE];

typedef struct{
  int16_t data[BUFF_SIZE];
  uint16_t head;
  uint16_t count;
}circ_buffer_t;

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

bool synch = false;
bool start_adxl = false;
bool start_GUN = false;

void gps_callback(uint gpio, uint32_t events){
  if(gpio == GPS_PIN){
    synch_time_2 = synch_time_1;
    synch_time_1 = to_us_since_boot(get_absolute_time());
    printf("Synch via GPS: %lldus\n", synch_time_1 - synch_time_2);
  }
}

bool check_if_falsestart(int16_t *raw_data){
  // FILTERING, DECISION, SIGNAL

  int16_t filtered_data[DATA_SIZE];

  filtered_data[0] = 0;

  for(int i = 1; i < DATA_SIZE+BUFF_SIZE; i++){
    filtered_data[i] = 0.1f * raw_data[i] + (1.0f - 0.1f)*filtered_data[i-1];
    printf("%d %d\n", i, filtered_data[i]);
  }

  return true;
}

int main(void)
{
  // initialize all present standard stdio types
  stdio_init_all();

  circ_buffer_t circ_buffer;

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

  int16_t raw_data[DATA_SIZE];
  uint16_t counter = 0;
  gpio_init(GPS_PIN);
  gpio_set_dir(GPS_PIN, GPIO_IN);

  gpio_set_irq_enabled_with_callback(GPS_PIN, GPIO_IRQ_EDGE_RISE, true, &gps_callback);

  start_adxl = false;
  start_GUN = false;

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
        // printf("ADXL STARTED\n");
      }
      if(payload_zero == START_GUN){
        start_GUN = true;
        // printf("GUN SHOT\n");
      }
    }
    // if(start_adxl){
    //   if(start_GUN && counter < DATA_SIZE){
    //     raw_data[counter++] = ADXL345_read_X_g();
    //   }else if(counter == DATA_SIZE){
    //     for(int i = 0; i < DATA_SIZE; i++){
    //       printf("%d %d\n", i, raw_data[i]);
    //     }
    //   }
    // }

    if(start_adxl && !start_GUN){
      int16_t x = ADXL345_read_X_g();
      buffer_put(&circ_buffer, x);
    }
    if(start_adxl && start_GUN){
      raw_data[counter++] = ADXL345_read_X_g();
      if(counter == DATA_SIZE){
        start_adxl = false;
        start_GUN = false;
        counter = 0;

        for(int i = 0; i < BUFF_SIZE + DATA_SIZE; i++){
          if(i < BUFF_SIZE){
            printf("%d %d\n", i - BUFF_SIZE, circ_buffer.data[i]);
          }else{
            printf("%d %d\n", i - BUFF_SIZE, raw_data[i - BUFF_SIZE]);
          }
        }
        buffer_clear(&circ_buffer);
      }
    }

    // if(start_adxl){
    //   if(!start_GUN){
    //     int16_t x_value = ADXL345_read_X_g();
    //     buffer_put(&circ_buffer, x_value);
    //   }else{
    //     // int16_t x_value = ADXL345_read_X_g();
    //     if(counter < DATA_SIZE)
    //       raw_data[counter++] = ADXL345_read_X_g();
    //     else{
    //       int16_t raw_data_together[BUFF_SIZE + DATA_SIZE];
    //       for(int i = 0; i < BUFF_SIZE+DATA_SIZE; i++){
    //         if(i < BUFF_SIZE)
    //           raw_data_together[i] = circ_buffer.data[i];
    //         else
    //           raw_data_together[i] = circ_buffer.data[i-BUFF_SIZE];
    //       }
    //       check_if_falsestart(raw_data_together);
    //     }
    //   }
    // }
  }
}