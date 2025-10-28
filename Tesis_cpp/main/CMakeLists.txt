idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES VariacRS485 SyringePumpRS232 Potenciometro pid_control filters motor_stepper uart_config LaserSensor ServoSG90)

