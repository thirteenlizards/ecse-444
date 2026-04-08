### PWM

### Use TIM4CH3 PWM
61 PD14 GPIO_EXTI14 ARD.D2-INT0_EXTI14

#### Use the PSC to make 1tick = 1ms

Scale -> 1 tick = 1ms -> desired frequency is 1/1ms = 1Mhz

PSC = (internal clock f / desired clock f) - 1
    = 120Mhz/1Mhz - 1
    = 119

#### Use the ARR to make the update frequency 50Hz

Choose 50Hz as slowest of bluerobotics ESCs can do

ARR = (scaled clock f / desired esc f) - 1
    = 1Mhz/50Hz -1
    = 19 999

#### Source:
https://medium.com/@pqshedy33/how-to-use-pwm-on-stm32-8852be201a79