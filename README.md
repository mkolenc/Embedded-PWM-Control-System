# Embedded System for PWM Control and Frequency Measurement

## Overview  

This project implements an embedded system to control and monitor a PWM signal generated by a 555 timer (NE555 IC). The STM32F0 Discovery board manages PWM signal control, frequency measurement, and real-time display updates. The system uses a potentiometer to adjust the PWM frequency and can measure signals from either the 555 timer or a function generator. Frequency and potentiometer resistance are displayed on an SSD1306 LED screen.  

The main code is in `src/main.c`.  

## System Components  

- **STM32F0 Discovery Board (STM32F051R8T6)**: Handles PWM control, frequency measurement, and display updates.  
- **555 Timer (NE555)**: Generates PWM signals with adjustable frequency and duty cycle.  
- **Optocoupler (4N35)**: Provides isolation between the 555 timer and microcontroller.  
- **Potentiometer**: Adjusts the 555 timer's PWM frequency.  
- **Waveform Generator**: Alternate frequency source.  
- **SSD1306 LED Display**: Displays signal frequency and potentiometer resistance.  

![System Overview](assets/main_diagram.png)  

## System Functionality  

### 1. Analog Input and Resistance Calculation  
- **ADC** reads the potentiometer's voltage on pin PA5 in single conversion mode.  
- The potentiometer resistance is calculated from the ADC value.  

### 2. Frequency Measurement  
- **Timer Input Capture (TIM2)** measures the frequency of incoming signals by recording the interval between rising edges.  
- Interrupts process input signals, and software calculates the frequency.  
- Signal sources include the 555 timer and function generator.  

### 3. Signal Source Switching  
- The **USER button** (PA0) toggles signal sources via an interrupt service routine (ISR).  

### 4. Display Updates via SPI  
- The **SSD1306 display** communicates via SPI, showing real-time frequency and resistance.  
- Display updates clear old data and refresh values for clear visualization.  

## Limitations  

### Measurement Errors  
At high frequencies, interrupt overhead introduces delays, leading to errors:  
- Accurate below **100 Hz**.  
- Errors of **±3 Hz** at **500 Hz** and up to **±9 Hz** at **1 MHz**.  

### Mitigation  
Averaging measurements across multiple edges could improve accuracy but exceeds the project's scope.  

## Demonstration  

**Waveform Generator Input:**  
The frequency remains constant while resistance changes.  
![Waveform Generator Demo](assets/waveform_gen_demo.gif)  

**555 Timer Input:**  
Resistance adjusts the frequency within the 800–1300 Hz range.  
![User Button and Timer Demo](assets/timer_button_demo.gif)  
