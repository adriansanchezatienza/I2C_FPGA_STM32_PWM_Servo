library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity pwmgen is 
  port(
    CLK     :  in std_logic;                      -- 100MHz internal CLK
    clk_kHz :  in std_logic;                      -- 133.69kHz from freq_div CLK
    RESET   :  in std_logic;                      -- Asyncronous reset
    LOAD    :  in std_logic;                      -- Asyncronous load of duty cycle
    DUTY    :  in std_logic_vector(7 downto 0);   -- Duty cycle from 1ms(0) to 2ms(255)
    PWM     : out std_logic                       -- PWM output signal
  );
end entity;

architecture behavioral of pwmgen is
    -- internal signals
    signal duty_i : unsigned( 7 downto 0);      -- 8 switches control / 8 bits resolution
    signal cmp_i  : unsigned( 8 downto 0);      -- Compare value = offset + duty
    -- counter signal
    signal cntr_i : unsigned (11 downto 0);
begin

    -- Counter from 0 to 2669.
    cntr : process(RESET, clk_kHz)
    begin
      if RESET = '1' then
          cntr_i <= (others => '0');
      elsif rising_edge(clk_kHz) then     
          if (cntr_i = 2669) then
            cntr_i <= (others => '0');
          else 
            cntr_i <= cntr_i + 1;
          end if;
      end if;
    end process;
    
    -- Duty register. when LOAD -> new duty cycle works
    duty_reg : process(RESET, CLK)
    begin
      if RESET = '1' then
        duty_i <= (others => '0');
      elsif rising_edge(CLK) then
        if LOAD = '1' then
            duty_i <= unsigned(DUTY);
        end if;
      end if;    
    end process;
    
    -- PWM offset: 74/133.69kHz = 0.55ms 
    compare_value : cmp_i <= ("0" & duty_i) + 74;
               
    -- OUTPUT PWM signal
    COMPARATOR : PWM <= '1' when cntr_i < cmp_i else
                        '0';
end architecture;