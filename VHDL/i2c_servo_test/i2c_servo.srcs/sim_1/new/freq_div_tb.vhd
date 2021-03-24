library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity freq_div_tb is
end entity;

architecture behavioral of freq_div_tb is
  -- Input signals
  signal CLK, RESET : std_logic;
  -- Output signals
  signal CLK_OUT    : std_logic;
  
  component clk_kHz is
  port(
    CLK     :  in std_logic;  -- 100Mhz
    RESET   :  in std_logic;  -- Asyncronous reset
    CLK_OUT : out std_logic   -- 256kHz 50% duty signal
  );
  end component;
  
  -- 100MHz internal clock
constant CLK_FREQ   : positive := 100000000;
constant CLK_PERIOD : time := 10ns;

begin
-- Instance of the Unit Under Test
  uut : clk_kHz
    port map(
      CLK       => CLK,
      RESET     => RESET,
      CLK_OUT   => CLK_OUT
    );
    
-- Clock generation process
clkgen : process
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
  wait for 0.25 * CLK_PERIOD;
  RESET <= '1';
  wait for 0.25 * CLK_PERIOD;
  RESET <= '0';
  ---------------------------
  
  
    -- Expand sim x cycles -- 
  for i in 1 to 5 loop
    wait until CLK_OUT = '1';
  end loop;
  --------------------------  
  wait for 0.25 * CLK_PERIOD;
  
  assert false
    report "[SUCCESS] : Simulation Finished O.K"
    severity failure;
end process;
end architecture;