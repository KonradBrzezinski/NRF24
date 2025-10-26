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
#define BUFF_SIZE 500 + 1

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
        payload_zero = START_ADXL;
        send = true;        
    }
    if(gpio == 18){
      payload_zero = START_GUN;
      send = true;
      // printf("STARTED GUN\n");
    }
}

int main(void)
{
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
    .channel = 120,
    .address_width = AW_5_BYTES,
    .dyn_payloads = DYNPD_ENABLE,
    .data_rate = RF_DR_1MBPS,
    .power = RF_PWR_NEG_12DBM,
    .retr_count = ARC_10RT,
    .retr_delay = ARD_500US 
  };

  uint32_t my_baudrate = 5000000;
  nrf_client_t my_nrf;
  nrf_driver_create_client(&my_nrf);
  my_nrf.configure(&my_pins, my_baudrate);
  my_nrf.initialise(&my_config);
  my_nrf.standby_mode();
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
  
  my_nrf.rx_destination(DATA_PIPE_0, (uint8_t[]){0xC3,0xC3,0xC3,0xC3,0xC3});
  char msg[32];
  uint16_t counter = 0;

  uint8_t data_pipe = 0;

  while (1) {
    if(send){
      send = false;
      my_nrf.tx_destination((uint8_t[]){0x37,0x37,0x37,0x37,0x37});
      success = my_nrf.send_packet(&payload_zero, sizeof(payload_zero));

      if(payload_zero == START_ADXL){
        payload_zero = 0;
      }
      if(payload_zero == START_GUN){
        payload_zero = 0;
        if(my_nrf.receiver_mode()){
          receive = true;
        }
      }
    }

    if(my_nrf.is_packet(&data_pipe)){
      my_nrf.read_packet(&msg, sizeof(msg));
      printf("%s", msg);
    }

    // if(receive){
    //     if(my_nrf.is_packet(&data_pipe)){
    //       my_nrf.read_packet(&msg, sizeof(msg));
    //       printf("%s\n", msg);
    //       if(msg[0] == 'R'){
    //         printf("RECEIVED SYNCHRONIZATION PACKET\n");
    //         my_nrf.standby_mode();
    //         receive = false;  
    //       }
    //     }
    // }

    __asm volatile ("dmb");

    tight_loop_contents();
  }
  return 0;
}