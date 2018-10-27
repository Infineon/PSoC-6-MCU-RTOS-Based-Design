## Table of Contents

* [Introduction](#introduction)
* [Navigating the Repository](#navigating-the-repository)
* [Required Tools](#required-tools)
* [Code Examples List](#code-examples-list)
* [References](#references)

# Introduction
This repository contains examples and demos for PSoC 6 MCU family of devices, a single chip solution for the emerging IoT devices. PSoC 6 MCU bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power, dual-core architecture of PSoC 6 MCU offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance.

Cypress provides a wealth of data at [www.cypress.com](http://www.cypress.com/) to help you select the right PSoC device and effectively integrate it into your design. Visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage to explore more about PSoC 6 MCU family of device.
Feel free to explore through the code example source files and let us innovate together!

# Navigating the Repository

This repository contains reference for developing PSoC 6 MCU designs using RTOS. The projects use FreeRTOS platform for developing the design. You can visit the [FreeRTOS](https://www.freertos.org/) webpage to learn the concepts of RTOS. 

If you are new to developing projects with PSoC 6 MCU, we recommend you to refer the [PSoC 6 Getting Started GitHub](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Getting-Started) page which can help you familiarize with device features and guides you to create a simple PSoC 6 design with PSoC Creator IDE. For other block specific design please visit the following GitHub Pages:
#### 1. [Analog Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Analog-Designs)
#### 2. [Digital Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Digital-Designs)
#### 3. [BLE Connectivity Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-BLE-Connectivity-Designs)
#### 4. [Audio Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Audio-Designs)
#### 5. [Device Related Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Device-Related-Design)
#### 6. [System-Level Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-System-Level-Designs)
#### 7. [PSoC 6 Pioneer Kit Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Pioneer-Kits)

You can use these block level examples to guide you through the development of a system-level design using PSoC 6 MCU. All the code examples in this repository comes with well documented design guidelines to help you understand the design and how to develop it. The code examples and their associated documentation are in the Code Example folder in the repository.

# Required Tools

## Software
### Integrated Development Environment (IDE)
To use the code examples in this repository, please download and install
[PSoC Creator](http://www.cypress.com/products/psoc-creator)

## Hardware
### PSoC 6 MCU Development Kits
* [CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-ble-pioneer-kit).

* [CY8CKIT-062-WiFi-BT PSoC 6 WiFi-BT Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-wifi-bt-pioneer-kit). 

**Note** Please refer to the code example documentation for selecting the appropriate kit for testing the project

## Code Examples List
#### 1. CE218136 – PSoC 6 MCU E-INK Display with CapSense (RTOS)
This code example demonstrates how to create a user-interface solution using an E-INK display with a CapSense slider and
buttons. E-INK displays consume no power for image retention. Together with PSoC 6 MCU’s CapSense touch sensing, an EINK
display can be used to create user interfaces that have “always-on” functionality.
#### 2. CE218137 – PSoC 6 MCU with BLE Connectivity: BLE with Proximity (RTOS)
This code example demonstrates connectivity between the PSoC 6 MCU with Bluetooth Low Energy (BLE) and CySmart
BLE host emulation tool or mobile device running the CySmart mobile application, to transfer CapSense® proximity sensing
information.
#### 3. CE218138 – PSoC 6 MCU with BLE Connectivity: BLE Thermometer (RTOS)
This code example demonstrates interfacing PSoC 6 MCU with BLE Connectivity (PSoC 6 MCU) with a thermistor circuit to read
temperature information and sending the temperature data as BLE HTS indications to a mobile device running CySmart mobile
application. In addition, PSoC 6 MCU’s real time clock (RTC) generates alarms (interrupts) at every minute to show temperature
information on the E-INK display when BLE is not connected.
#### 4. CE218139 – PSoC 6 MCU with BLE Connectivity: Eddystone Beacon (RTOS)
This code example demonstrates the ability of PSoC 6 MCU with BLE Connectivity (PSoC 6 MCU) to function as a BLE beacon
using the Broadcaster role, which transmits Eddystone fames. Eddystone is an open-source BLE beacon profile released by
Google. This project broadcasts core Eddystone frame types—Eddystone UID, Eddystone URL, and Eddystone TLM. 
#### 5. CE220331 – PSoC 6 MCU with BLE Connectivity: BLE with User Interface (RTOS)
This code example demonstrates interfacing PSoC 6 MCU with an RGB LED with color and intensity control and touch buttons
based on mutual capacitance (CSX), and touch-slider-based on self-capacitance (CSD). This code example also shows
connectivity between the PSoC 6 BLE (acting as a Peripheral and GATT Server) and a PC running the CySmart BLE Host
Emulation tool or a mobile device running the CySmart mobile application (acting as a Central and GATT Client). Custom BLE
services are used for CapSense touch sensing and LED control.
#### 6. CE222604 – PSoC 6 MCU with BLE Connectivity: RTC with Current Time Service (RTOS)
This code example demonstrates accurate time keeping with the RTC of PSoC 6 MCU with BLE Connectivity (PSoC 6 MCU),
which also generates alarms (interrupts) at every one minute to show time information on an E-INK display. In addition, a BLE
CTS is used to synchronize time and date with a current time server such as an iPhone.
#### 7. CE222793 – PSoC 6 MCU: Motion Sensor (RTOS)
This example configures and reads data from a BMI160 motion sensor using PSoC 6 MCU. The example uses the BMI160
motion sensor to detect and count steps from activities such as walking or running, emulating the functionality of a pedometer.
The motion sensor’s accelerometer data is also read and converted to indicate the orientation of the sensor with respect to the
ground. The step count and orientation information is displayed on the E-INK display.


## References
#### 1. PSoC 6 MCU
PSoC 6 bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power PSoC 6 MCU architecture offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance. The PSoC 6 MCU contains a dual‑core architecture, with both cores on a single chip. It has an Arm® Cortex®‑M4 for high‑performance tasks, and an Arm® Cortex®‑M0+ for low-power tasks, and with security built-in, your IoT system is protected.
To learn more on the device, please visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage.

####  2. PSoC 6 MCU Learning resource list
##### 2.1 PSoC 6 MCU Datasheets
Device datasheets list the features and electrical specifications of PSoC 6 families of devices: [PSoC 6 MCU Datasheets](http://www.cypress.com/search/all?f%5B0%5D=meta_type%3Atechnical_documents&f%5B1%5D=resource_meta_type%3A575&f%5B2%5D=field_related_products%3A114026)
##### 2.2 PSoC 6 MCU Application Notes
Application notes are available on the Cypress website to assist you with designing your PSoC application: [A list of PSoC 6 MCU ANs](http://www.cypress.com/psoc6an)
##### 2.3 PSoC 6 MCU Component Datasheets
PSoC Creator utilizes "components" as interfaces to functional Hardware (HW). Each component in PSoC Creator has an associated datasheet that describes the functionality, APIs, and electrical specifications for the HW. You can access component datasheets in PSoC Creator by right-clicking a component on the schematic page or by going through the component library listing. You can also access component datasheets from the Cypress website: [PSoC 6 Component Datasheets](http://www.cypress.com/documentation/component-datasheets)
##### 2.4 PSoC 6 MCU Technical Reference Manuals (TRM)
The TRM provides detailed descriptions of the internal architecture of PSoC 6 devices:[PSoC 6 MCU TRMs](http://www.cypress.com/psoc6trm)

## FAQ

### Technical Support
Need support for your design and development questions? Check out the [Cypress Developer Community 3.0](https://community.cypress.com/welcome).  

Interact with technical experts in the embedded design community and receive answers verified by Cypress' very best applications engineers. You'll also have access to robust technical documentation, active conversation threads, and rich multimedia content. 

You can also use the following support resources if you need quick assistance:
##### Self-help: [Technical Support](http://www.cypress.com/support)
##### Local Sales office locations: [Sales Office](http://www.cypress.com/about-us/sales-offices)
