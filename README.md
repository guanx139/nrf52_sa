# The Program for the SA Detector

## Basic Info

This program is meant for the nRF5_SDK Version 15, and the SoC used is nRF52832 used in a custom design.

For the usage, download the .c and .h code in the ~\nRF5_SDK_15.0.0_a53641a\nRF5_SDK_15.0.0_a53641a\examples\ble_peripheral\ble_app_template, and replace the config file in ~nRF5_SDK_15.0.0_a53641a\nRF5_SDK_15.0.0_a53641a\examples\ble_peripheral\ble_app_template\pca10040\s132\config

For sure that you can directly clone this project.

The entire program is written in SEGGER Embeded studio. To invoke SEGGER, double click the .emProject file ~\nRF5_SDK_15.0.0_a53641a\nRF5_SDK_15.0.0_a53641a\examples\ble_peripheral\ble_app_template\pca10040\s132\ses 

There might be some complilation issues with some of the libraries used, just exclude them from the project if they are not included in the main.c
