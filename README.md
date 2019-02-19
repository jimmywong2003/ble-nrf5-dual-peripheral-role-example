# Dual_ble_peripheral_example

## Descriptions

Nordic SDK has supported the multiple ble peripheral role since SDK 14.x or later (S132v4.x or later).

It has the simple sample by using Blinky application to show how one Peripheral can connect to several Centrals. 

In this example, it would add the NUS (Nordic UART service) with LBS together as duel peripheral role and do the data forwarding from one central to another central.


## Requirements

* nRF5_SDK_15.0.0_a53641a
* nRF52 DK Board
* Segger Embedded Studio

To compile it, clone the repository in the \nRF5_SDK_15.0.0_a53641a\example\ folder. If you download the zip, place each of the project folders of this repository into the \nRF5_SDK_15.0.0_a53641a\example\ folder.
