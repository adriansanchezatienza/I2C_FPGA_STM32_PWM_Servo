library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity clk_kHz is
  port(
    CLK     :  in std_logic;  -- 100Mhz
    RESET   :  in std_logic;  -- Asyncronous reset
    CLK_OUT : out std_logic   -- 256kHz 50% duty signal
  );
end entity;

-- duty 50% -> 100MHz / 2*256kHz -> count from 0 to 194

architecture behavioral of clk_kHz is
    signal pulse : std_logic;
    signal cnt   : unsigned(8 downto 0);
begin
    cntr : process(RESET, CLK)
    begin
        if RESET = '1' then
            pulse <= '0';
            cnt   <= (others => '0');  
        elsif rising_edge(CLK) then
            -- Toggle signal pulse each 373 clock cycles
            -- Restart the counter
            if(cnt = 373) then
                pulse <= NOT(pulse);
                cnt   <= (others => '0');
            else
                cnt <= cnt + 1;
            end if;
        end if;
    end process;
    CLK_OUT <= pulse;
end architecture;