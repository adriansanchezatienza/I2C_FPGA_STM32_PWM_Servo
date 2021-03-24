library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity servo_pwm is
  PORT(
     CLK   :  in std_logic;                      -- 100MHz internal CLK
     RESET :  in std_logic;                      -- Asyncronous reset
     LOAD  :  in std_logic;                      -- Syncronous load of duty cycle
     DUTY  :  in std_logic_vector(7 downto 0);   -- Duty cycle from 1ms(0) to 2ms(255)
     PWM   : out std_logic                       -- PWM output signal
  );
end servo_pwm;
--------------------------COMPONENTS----------------------------------------------------------------
architecture Behavioral of servo_pwm is
  COMPONENT clk_kHz
  PORT(
     CLK     :  in std_logic;                    -- 100Mhz
     RESET   :  in std_logic;                    -- Asyncronous reset
     CLK_OUT : out std_logic                     -- 133.69 kHz 50% duty signal
  );
  END COMPONENT;
    
  COMPONENT pwmgen
  PORT(
     CLK       :  in std_logic;                      -- 100MHz internal CLK
     clk_kHz   :  in std_logic;                      -- 133.69 kHz from freq_div CLK
     RESET     :  in std_logic;                      -- Asyncronous reset
     LOAD      :  in std_logic;                      -- Syncronous load of duty cycle
     DUTY      :  in std_logic_vector(7 downto 0);   -- Duty cycle from 1ms(0) to 2ms(255)
     PWM       : out std_logic                       -- PWM output signal
  );
  END COMPONENT;
    
-----------------------------MAPPING----------------------------------------------------------------
    -- Linking signals
    signal CLK_IN : STD_LOGIC; -- 133.69 kHz clock
    
    
begin
    Inst_freqDIV: clk_kHz PORT MAP(
        CLK     => CLK,
        RESET   => RESET,
        CLK_OUT => CLK_IN
    );
    
    Inst_PWMGEN: pwmgen PORT MAP(
        CLK       => CLK,
        clk_kHz   => CLK_IN,
        RESET     => RESET,
        LOAD      => LOAD,
        DUTY      => DUTY,
        PWM       => PWM
    );
end Behavioral;