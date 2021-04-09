# I2C_FPGA_STM32_PWM_Servo
Implementación de protocolo de comunicación serie entre FPGA y microcontrolador STM.
El proyecto centrará su funcionamiento en un mecanismo de balanceo, es decir, según 
una determinada inclinación de la MPU (gobernada por el micro) poder corregir esa inclinación moviendo un 
servo (gobernado por la FPGA).
Por lo tanto, los objetivos de este proyecto serán: 
—> Comunicación de la placa Nexus 4DDR y STM32F407G mediante protocolo de 
comunicación I2C en la que la STM32 actuará como maestro iniciando y 
terminando toda comunicación y la Nexus como esclavo. 
—> STM32: Lectura de los registros de giro del dispositivo MPU6050, se trata de un 
chip con un acelerómetro y un giróscopo integrado. La STM32 deberá ser capaz 
de mediante tecnología I2C, comunicarse con el dispositivo, inicializarlo y leer 
sus valores. 
—> STM32: Lectura de los botones marcados por el usuario sobre la inclinación del 
dispositivo, se realizará mediante interrupciones. 
—> FPGA: Obtención de los valores de inclinación de la STM32 en un numero de 1 
byte. 
—> FPGA: Interpretación de estos valores para generar un pulso PWM que sea 
capaz de mover un servo hacia el ángulo de estabilización. 
—> FPGA: Una vez leídos los valores y generado el PWM, mandar un código a la 
placa con el ángulo al que me mueve el servo y su estado (Espera, Movimiento 
o Parado). 
—> STM32: Obtención de los valores de vuelta y presentación en una pantalla LCD 
comunicado por tecnología I2C. 
