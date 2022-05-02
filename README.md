# The Program for the SA Detector

## Basic Info

This program is meant for the nRF5_SDK Version 15, and the SoC used is nRF52832 used in a custom design.

For the usage, download the .c and .h code in the ~\nRF5_SDK_15.0.0_a53641a\nRF5_SDK_15.0.0_a53641a\examples\ble_peripheral\ble_app_template, and replace the config file in ~nRF5_SDK_15.0.0_a53641a\nRF5_SDK_15.0.0_a53641a\examples\ble_peripheral\ble_app_template\pca10040\s132\config

For sure that you can directly clone this project.

The entire program is written in SEGGER Embeded studio. To invoke SEGGER, double click the .emProject file ~\nRF5_SDK_15.0.0_a53641a\nRF5_SDK_15.0.0_a53641a\examples\ble_peripheral\ble_app_template\pca10040\s132\ses 

There might be some complilation issues with some of the libraries used, just exclude them from the project if they are not included in the main.c
ht
The sa_cus library is the older version, so you can ignore it

## Program Breakdown

The entire program can be break down into three part, the custom BLE service, the pheripheral configurarion, and boilerplate that come with the example code

### ble_sas (sleep anpea service)

This is a custom service for sa detector. The service is exptected to send 16 byte of information to the cellphone. 14 bytes are used for the adc value, and 2 bytes are used for the sa_message, which indicate the presence of sa event.

### pherpherals

An ADC is used to sample the respitory signal comes from the output of the analog front end circuit at 4 Hz. There are some configurations for gpio and software timers.

### boilerplate

The example comes with some code for the device to be operate as BLE pheripheral device, like the connection interval etc

## Current Issues

The ready flag is set no matter what if the absolute value between ADC value and ADV AVG is less that 1000.
