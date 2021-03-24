library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;
   use std.textio.all;
   --use work.txt_util.all;

entity I2C_servo_top is
   port (
      scl        : inout std_logic;                     -- SCL line from I2C bus
      sda        : inout std_logic;                     -- SDA line from I2C bus
      clk        : in    std_logic;                     -- 100Mhz internal clk
      --
      rst        : in    std_logic;                     -- Asyncronous reset
      load       : in    std_logic;                     -- Syncronous load of duty cycle
      --
      pwm_o      : out   std_logic;                     -- PWM output signal
      led_o      : out   std_logic_vector(7 downto 0)   -- LEDs lighting data_from_master
      );
end I2C_servo_top;

architecture RTL of I2C_servo_top is
--------------------------SIGNALS----------------------------------------------------------------
signal read_req         : std_logic                      := '0';
signal data_to_master   : std_logic_vector (7 downto 0)  := "01010101";
signal data_valid       : std_logic                      := '0';
signal data_from_master : std_logic_vector (7 downto 0)  := (others => '0');
-- Registro para el manejo del flujo de datos.
signal data_reg         : std_logic_vector (7 downto 0);

--------------------------COMPONENTS----------------------------------------------------------------
component I2C_slave is
  generic (
    SLAVE_ADDR          : std_logic_vector(6 downto 0)   := "0000011"); 
  port (
    scl                 : inout std_logic;
    sda                 : inout std_logic;
    clk                 : in    std_logic;
    rst                 : in    std_logic;
    -- User interface
    read_req            : out   std_logic;
    data_to_master      : in    std_logic_vector(7 downto 0);
    data_valid          : out   std_logic;
    data_from_master    : out   std_logic_vector(7 downto 0));
end component I2C_slave;

component servo_pwm is
  port(
     CLK   :  in std_logic;                      -- 100MHz internal CLK
     RESET :  in std_logic;                      -- Asyncronous reset
     LOAD  :  in std_logic;                      -- Syncronous load of duty cycle
     DUTY  :  in std_logic_vector(7 downto 0);   -- Duty cycle from 1ms(0) to 2ms(255)
     PWM   : out std_logic                       -- PWM output signal
  );
end component servo_pwm;


begin
------------------------LINK COMPONENTS-------------------------------------------------------------
I2C: I2C_slave 
   generic map (
      SLAVE_ADDR => "0000011"
      )
   port map(
      scl               => scl,
      sda               => sda,
      clk               => clk,
      rst               => rst,
      -- User interface
      read_req          => read_req,
      data_to_master    => data_to_master,
      data_valid        => data_valid,
      data_from_master  => data_from_master
   );
      
PWM: servo_pwm
   port map(
      CLK               =>  clk,                  
      RESET             =>  rst,                     
      LOAD              =>  load,                      
      DUTY              =>  data_reg,  
      PWM               =>  pwm_o                     
   );
    
 ------------------------USER CODE BEGIN------------------------------------------------------------ 
 -- LEDS muestran lo que viene del master 
 led_o <= data_reg;
 data_to_master <= data_reg;

process (clk) 
begin
   
if rising_edge(clk) then
   if data_valid = '1' then
      data_reg <= data_from_master;	
      --data_to_master <= std_logic_vector(unsigned(data_from_master) + 1);
      end if;
   end if;
      
end process;
    
end architecture rtl;