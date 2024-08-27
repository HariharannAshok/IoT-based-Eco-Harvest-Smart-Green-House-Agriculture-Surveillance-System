# IoT-based-Eco-Harvest-Smart-Green-House-Agriculture-Surveillance-System


## Project Overview

This project explores the creation of a specialized ecosystem adapted for potato fields using Internet of Things (IoT) technology. It focuses on establishing a reliable connection between various sensors and microcontrollers, understanding pin configurations, and programming the ESP32 board to act as a data relay from the XIAO board to the IoT cloud. The system monitors key environmental parameters crucial for optimal potato production, providing valuable insights for enhancing agricultural productivity.

## Group Members
- **Badaseshi Rishab Ajeyakumar**: Responsible for the DHT sensor and programming three distinct nodes.
- **Hariharan Ashok**: Tasked with the implementation of the TSL2591 light sensor along with the integration of the UM Feather S3 microcontroller.
- **Bian Qian**: Handled the integration of the air quality sensor.
- **Cao Zhuohao**: Focused on the soil moisture sensor.

## Abstract

In the emerging environment of IoT and agricultural integration, this project explores the creation of a specialized ecosystem adapted for potato fields. We focus on connecting sensors and microcontrollers, understanding pin configurations, and programming the ESP32 board to relay data from the XIAO board to the IoT cloud. The network of sensors, including air quality, DHT, soil moisture, and light sensors, monitors key environmental parameters crucial for optimal potato production. The project aims to integrate cutting-edge IoT technology with agricultural practices to improve productivity and sustainability in the agricultural sector.

## Objectives

- Develop and connect XIAO ESP32 boards with various sensors to transfer data to the FeatherS3 board.
- Transmit collected data to the cloud via Arduino cloud for IoT-based Eco-Harvest Smart Green House Agriculture Surveillance.
- Understand the working principles of each sensor and the XIAO ESP32 & FeatherS3 boards.
- Calibrate sensors and implement Cyclic Redundancy Check (CRC) and Time Division Multiple Access (TDMA).
- Develop software code for sensor interfacing and ensure the integration of the IoT-based system using Arduino cloud.

## System Design and Implementation

### Sensors Used
- **Air Quality Sensor**: Provides real-time data on the concentration of particulate matter and gases.
- **DHT Sensor**: Monitors temperature and humidity, crucial for assessing plant health.
- **Soil Moisture Sensor**: Detects soil moisture content, essential for managing irrigation systems.
- **TSL2591 Light Sensor**: Measures luminous intensity accurately, vital for monitoring light conditions in the potato field.

### Communication Protocols
- **Cyclic Redundancy Check (CRC)**: Ensures data integrity during transmission from sensors to the ESP32 board.
- **Time Division Multiple Access (TDMA)**: Optimizes communication efficiency between ESP32 slave senders and the master receiver.

### ESP32 as an IoT Gateway
The ESP32 serves as the IoT gateway for wireless sensor networks (WSNs), collecting and transmitting data from various sensors to the cloud. It plays a central role in managing data transmission and ensuring accurate, real-time monitoring of the potato field environment.

## Data Management & Visualization

### Data Management
The ESP32, functioning as an IoT gateway, organizes and transfers collected sensor data to the IoT cloud via the Arduino Cloud platform. Data integrity is maintained using CRC techniques, and each data point is time-stamped for historical tracking.

### Data Visualization
The Arduino Cloud platform provides graphical representations of the collected data, including temperature, humidity, air quality, soil moisture, and light conditions. Correlation analyses are also conducted to examine relationships between various environmental factors.

## Results and Discussion

### Sensor Accuracy and Reliability
The sensors provided highly accurate data, with the TSL2591 light sensor showing a 98% correlation with professional-grade light meters, and the DHT sensor delivering reliable temperature and humidity readings.

### Data Transmission Efficiency
The ESP-NOW protocol ensured a 99% successful transmission rate, with the TDMA method effectively preventing data collisions.

### Power Consumption Analysis
The system operated efficiently with low power consumption, supporting extended use in remote or off-grid locations.

## Conclusion

The IoT-based Eco-Harvest Smart Green House Agriculture Surveillance System successfully integrates various sensors to monitor critical environmental factors in potato fields. This system provides real-time data to farmers, enabling informed decisions to optimize crop production. The project highlights the potential of IoT technologies in advancing agricultural practices.

## Future Work

- Expand system functionality by adding automated control features.
- Enhance security in communication and data storage to protect against unauthorized access.
- Implement advanced data analysis techniques for better prediction and optimization of crop growth.
- Increase system adaptability by supporting multiple IoT platforms.

## References
1. M. A. Elashiri and A. T. Shawky (2018). "Fuzzy Smart Greenhouses Using IoT," IEEE International Conference on Computational Intelligence and Computing Research (ICCIC).
2. Gamal, A. S., & Mohamed, H. K. (2023). Performance Modelling of IoT in Smart Agriculture.
3. Gour, S., et al. (2023). "Low Cost IoT Based Smart Irrigation System for Potato Cultivation," IEEE Guwahati Subsection Conference (GCON).
4. Heble, S., et al. (2018). "A low power IoT network for smart agriculture," IEEE 4th World Forum on Internet of Things (WF-IoT).
5. Kassim, M. R. M. (2020). "IoT Applications in Smart Agriculture: Issues and Challenges," IEEE Conference on Open Systems (ICOS).
6. Kassim, M. R. M. (2022). "Applications of IoT and Blockchain in Smart Agriculture: Architectures and Challenges," IEEE International Conference on Computing (ICOCO).
7. Khaleefah, Raad. M., et al. (2023). "Optimizing IoT Data Transmission in Smart Agriculture: A Comparative Study of Reduction Techniques," International Congress on Human-Computer Interaction, Optimization and Robotic Applications (HORA).
8. Lakshmi, A. J., et al. (2023). "IoT Based Smart Greenhouse Using Raspberry Pi," International Conference on Computer, Electronics & Electrical Engineering & Their Applications (IC2E3).
9. Moreno, M. C., et al. (2021). "IoT-based Automated Greenhouse for Deep Water Culture Hydroponic System," Sustainable Cities Latin America Conference (SCLA).
10. Singh, C., et al. (2023). "Integrated Project for Data Communication in Wireless Sensor Network," International Conference on Advances in Computing, Communication and Applied Informatics (ACCAI).
11. Yuliandoko, H., et al. (2022). "Monitoring System of Greenhouse Based on WSN and Auto Flushing Sensor Mechanism," International Conference on Information Technology, Information Systems and Electrical Engineering(ICITISEE).
12. Zabasta, A., et al. (2021). "Development of IoT based Monitoring and Control System for Small Industrial Greenhouses," Mediterranean Conference on Embedded Computing (MECO).

