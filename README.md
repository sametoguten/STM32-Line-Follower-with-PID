# PID Controller Optimization for Low-cost Line Follower Robots

[![license](https://img.shields.io/badge/license-MIT-%23F65314?style=flat-square)](LICENSE)
[![DOI](https://img.shields.io/badge/DOI-10.13140/RG.2.2.22102.98886-%239aed00?style=flat-square)](https://www.researchgate.net/publication/349641393_PID_Controller_Optimization_for_Low-cost_Line_Follower_Robots)

This repository contains implementation of an optimized PID controller for a line follower robot using STM32F103C8 microcontroller and QTR-8RC reflectance sensor array. For more information about the research, please refer to the [paper](https://www.researchgate.net/publication/349641393_PID_Controller_Optimization_for_Low-cost_Line_Follower_Robots).

> ***Abstract*** - In this paper, modification of the classical PID controller and development of open-loop control mechanisms to improve stability and robustness of a differential wheeled robot are discussed. To deploy the algorithm, a test platform has been constructed using low-cost and off-the-shelf components including a microcontroller, reflectance sensor, and motor driver. This paper describes the heuristic approach used in the identification of the system specifications as well as the optimization of the controller. The PID controller is analyzed in detail and the effect of each term is explained in the context of stability. Lastly, the challenges encountered during the development of controller and robot are discussed.

# Getting Started

<img align="right" width="440" src="https://user-images.githubusercontent.com/53112883/109350542-b6a3b700-7888-11eb-841b-921b91522507.png">

### Components
- STM32F103C8
- L298N motor driver
- QTR-8RC reflectance sensor array
- 6V DC motors x2
- 12V battery
- Wheels x2
- Chasis

### Required Software
- [Keil µVision](https://www.keil.com/demo/eval/arm.htm)

### Build the code
Go to `/LineFollower/MDK-ARM` directory and open `LineFollower.uvprojx` project file. You should first build the code and then upload it to the STM32 board. 

## Authors

- [Samet Öğüten](https://github.com/sametoguten)
- [Bilal Kabaş](https://github.com/bilalkabas)

## License

This project is licensed under the [MIT License](LICENSE).
