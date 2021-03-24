library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity servo_pwm_tb is
end entity;

architecture behavioral of servo_pwm_tb is
-- Inputs
   signal CLK, RESET, LOAD : std_logic;
   signal DUTY : std_logic_vector(7 downto 0);
-- Outputs
   signal PWM  : std_logic;

component servo_pwm is
PORT(
   CLK   :  in std_logic;                      -- 100MHz clock
   RESET :  in std_logic;                      -- Asyncronous reset
   LOAD  :  in std_logic;                      -- Syncronous load of duty cycle
   DUTY  :  in std_logic_vector(7 downto 0);   -- Duty cycle from 1ms(0) to 2ms(255)
   PWM   : out std_logic                       -- PWM output signal
  );
end component;

-- CLKs PERIOD
constant CLK_PERIOD : time := 10 ns;   -- 100MHz internal clk
constant CLK_134kHz : time := 7.48 us; -- 133.69kHz from FREQ_DIV -> LOAD

begin
-- Instance of the Unit Under Test
  uut : servo_pwm
    port map(
      CLK   => CLK,
      RESET => RESET,
      LOAD  => LOAD,
      DUTY  => DUTY,
      PWM   => PWM
    );
    
-- Clock generation process
clkgen : process  -- 100MHz
begin
    CLK <= '0';
    wait for 0.5 * CLK_PERIOD;
    CLK <= '1';
    wait for 0.5 * CLK_PERIOD;
end process;

-- Tester process
tester : process
begin
  -- Reset testing --
  wait for 0.25 * CLK_134kHz;
  RESET <= '1';
  wait for 2*CLK_134kHz;
  RESET <= '0';
  ---------------------------
  
  wait for 2*CLK_134kHz;
  
  ---- Testing MIN VALUE ---- 180º -- OFFSET (1.05 ms)
  LOAD <= '1';
  DUTY <= "00000000";
  wait for 2*CLK_134kHz;
  LOAD <= '0';
  
  wait until PWM = '1';
  
  ---- Testing MID VALUE ---- 90º -- 256x1.5 - 258 - (1.5 ms)
  LOAD <= '1';
  DUTY <= "10010011";
  wait for 2*CLK_134kHz;
  LOAD <= '0';
  
  wait until PWM = '1';
  
  ---- Testing MAX VALUE ---- 0º -- (1.95 ms)
  LOAD <= '1';
  DUTY <= "11111111";
  wait for 2*CLK_134kHz;
  LOAD <= '0';
  
  wait until PWM = '1';
  
    
  assert false
    report "[SUCCESS] : Simulation Finished O.K"
    severity failure;
  
end process;


end architecture;