# Kalman-Filter

## Description
This repository contains the implementation of a Kalman Filter designed by Avi Komarlingam (akomar@umd.ed) and Abhi Senthilkumar of the Terrapin Rocket Team. The Kalman Filter is a mathematical method that uses noisy measurements observed over time to estimate an object's state.

## Installation
Clone the repository to your local machine using:

`git clone https://github.com/Terrapin-Rocket-Team/Kalman-Filter.git`

If you are planning to incorporate it into your Arduino code, move the Kalman-Filter folder into your Arduino libraries folder located in:

`Documents/Arduino/libraries`

## Usage
To use the Kalman Filter, include the "Kalman-Filter.h" header file at the top of your C++ or Arduino code like below:

`include "Kalman-Filter.h"`

## Folder Structure
### Filters
Contains the abstract implementation of a Linear Kalman Filter, **do not modify**

### MatlabDataTester
Contains a MATLAB script that cleans data outputted by the TERP and TADPOL from the payload and runs a MATLAB version of the Linear Kalman Filter on the data to validate filter tuning. To use, upload your own dataset into the data folder and run DataTester.m

### MatrixMult
Contains a Matrix class which encapsulates basic matrix operations necessary for the implementation of Kalman Filters

### UnitTests
Contains unit tests for the Matrix class

## Acknowledgements
Thanks to all contributors and members of the Terrapin Rocket Team for their ongoing support and contributions to this project.