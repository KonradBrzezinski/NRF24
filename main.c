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
 * @file primary_transmitter.c
 * 
 * @brief example of NRF24L01 setup as a primary transmitter using the 
 * NRF24L01 driver. Different data types and sizes are sent to different 
 * receiver data pipes as an example, utilizing dynamic payload size.
 */

#include <stdio.h>

#include "nrf24_driver.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define START_ADXL 0xF0
#define START_GUN 0x0F
#define TIME_SYNCH 0b01010101
#define DATA_SIZE 500
#define BUFF_SIZE 500

bool send = false;
bool receive = false;
bool time_synch_2_check = false;
bool time_synch_1_check = false;

// payload sent to receiver data pipe 0
uint8_t payload_zero = 0b00000001;

volatile uint64_t synch_time_1 = 0;
volatile uint64_t synch_time_2 = 0;

void gpio_callback(uint gpio, uint32_t events) {
    if(gpio == 16) {
        // synch_time_1 = to_us_since_boot(get_absolute_time());
        payload_zero = START_ADXL;
        send = true;        
        // time_synch_1_check = true;
        // printf("STARTED ADXL\n");
        // printf("FALLING EDGE on GPIO 16\n");
    }
    if(gpio == 17) {
        // synch_time_2 = to_us_since_boot(get_absolute_time());
        payload_zero = TIME_SYNCH;
        send = true;
        time_synch_2_check = true;
        // printf("Synch time 2: %lld us\n", synch_time_2);
        // printf("Time difference: %llu us\n", synch_time_2 - synch_time_1);
        // printf("FALLING EDGE on GPIO 17\n");
    }
    if(gpio == 18){
      payload_zero = START_GUN;
      send = true;
      // printf("STARTED GUN\n");
    }
    if(gpio == 15 && (events & GPIO_IRQ_EDGE_RISE)) {
        synch_time_2 = synch_time_1;
        synch_time_1 = to_us_since_boot(get_absolute_time());
        printf("Synch via GPS: %lldus\n", synch_time_1 - synch_time_2);
    }
}

// void gpio_callback(uint gpio, uint32_t events) {
//     if(gpio == 17) {
//         payload_zero = 0x01;
//         synch_time_2 = to_us_since_boot(get_absolute_time());
//         send = true;
//         printf("Synch time 1: %lld us\n", synch_time_2);
//         printf("Time difference: %llu us\n", synch_time_2 - synch_time_1);
//     }
// }

int main(void)
{
  // initialize all present standard stdio types
  stdio_init_all();


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

  nrf_client_t my_nrf;

  // initialise my_nrf
  nrf_driver_create_client(&my_nrf);

  // configure GPIO pins and SPI
  my_nrf.configure(&my_pins, my_baudrate);

  // not using default configuration (my_nrf.initialise(NULL)) 
  my_nrf.initialise(&my_config);

  // set to Standby-I Mode
  my_nrf.standby_mode();

  // result of packet transmission
  fn_status_t success = 0;

  uint64_t time_sent = 0; // time packet was sent
  uint64_t time_reply = 0; // response time after packet sent

  gpio_init(16);
  gpio_set_dir(16, GPIO_IN);
  gpio_pull_up(16);

  gpio_init(17);
  gpio_set_dir(17, GPIO_IN);
  gpio_pull_up(17);

  gpio_init(18);
  gpio_set_dir(18, GPIO_IN);
  gpio_pull_up(18);
  gpio_init(15);
  gpio_set_dir(15, GPIO_IN);

  gpio_set_irq_enabled_with_callback(16, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(17, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(18, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(15, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

  my_nrf.rx_destination(DATA_PIPE_1, (uint8_t[]){0xC7,0xC7,0xC7,0xC7,0xC7});
  char msg[32];
  uint16_t counter = 0;

  while (1) {
    // send to receiver's DATA_PIPE_0 address
    if(send){
      send = false;
      my_nrf.tx_destination((uint8_t[]){0x37,0x37,0x37,0x37,0x37});
      success = my_nrf.send_packet(&payload_zero, sizeof(payload_zero));
    
      if(success){
        printf("SENT PAYLOAD\n");
      }

      if(payload_zero == START_GUN){
        my_nrf.receiver_mode();
        // sleep_ms(1);
        receive = true;
        payload_zero = 0;
      }

      if(time_synch_1_check){ 
        time_synch_1_check = false;
        synch_time_1 = to_us_since_boot(get_absolute_time());
      }
      
      if(time_synch_2_check){ 
        time_synch_2_check = false;
        synch_time_2 = to_us_since_boot(get_absolute_time());
        printf("Synch time 1: %lld us\n", synch_time_1);
        printf("Synch time 2: %lld us\n", synch_time_2);
        printf("Time difference: %llu us\n", synch_time_2 - synch_time_1);
      }
    }

    if(receive){
      if(my_nrf.is_packet(NULL)){
        my_nrf.read_packet(msg, sizeof(msg));
        printf("%s", msg);
        if(msg[0] == 'r'){
          receive = false;
          my_nrf.standby_mode();
        }
      }
    }

    // if(success && (payload_zero == START_GUN)){
    //   uint8_t pipe_number = 0;
    //   my_nrf.receiver_mode();
    //   char msg[32];
    //   if(my_nrf.is_packet(&pipe_number)){
    //     if(pipe_number == DATA_PIPE_1){
    //       my_nrf.read_packet(msg, sizeof(msg));
    //       printf("%s", msg);
    //     }
    //   }
    // }

    __asm volatile ("dmb");

    tight_loop_contents();
  }
  return 0;
}