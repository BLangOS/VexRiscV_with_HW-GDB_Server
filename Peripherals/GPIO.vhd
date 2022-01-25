--------------------------------------------------------------------------------
-- Dateiname: GPIO.vhd
--
-- GPIO Komponente
--
-- Erstellt: 19.03.2014, Rainer Hoeckmann
--
-- Aenderungen:
--------------------------------------------------------------------------------
----------------------------------------------------------------------------------------
-- (c) Bernhard Lang, Rainer Hoeckmann, Hochschule Osnabrueck
----------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity GPIO is
  generic(
    NUM_PORTS: positive
  );
  port(
    clk_i : in    std_logic;
    rst_i : in    std_logic;
    stb_i : in    std_logic;
    we_i  : in    std_logic;
    adr_i : in    std_logic_vector(7 downto 0);
    dat_i : in    std_logic_vector(31 downto 0);
    ack_o : out   std_logic;
    dat_o : out   std_logic_vector(31 downto 0);
    pins  : inout std_logic_vector(NUM_PORTS - 1 downto 0)
  );
end entity;

architecture rtl of GPIO is
  -- direction register
  signal dir_r: std_logic_vector(NUM_PORTS - 1 downto 0) := (others=>'0');
  -- data register
  signal data_r: std_logic_vector(NUM_PORTS - 1 downto 0) := (others=>'0');
  -- synchronized input pins
  signal pins_sync: std_logic_vector(pins'range) := (others=>'0');
begin
  -- generate ack_o each time this component is accessed
  ack_o <= stb_i;
  
  -- dat_o multiplexer for read accesses
  rd_mux: process(adr_i, dir_r, data_r, pins_sync)
  begin
    dat_o <= (others=>'0');
    case adr_i is
      when x"00"  => dat_o(dir_r'range)     <= dir_r;
      when x"10"  => dat_o(data_r'range)    <= data_r;
      when x"20"  => dat_o(pins_sync'range) <= pins_sync;
      when others => null;
    end case;
  end process;

  -- instantiate needed number of iobufs 
  generate_iobuf: for i in 0 to NUM_PORTS - 1 generate 
    pins(i) <= 
      data_r(i) when dir_r(i) = '1' else 
      'Z'       when dir_r(i) = '0' else
      '-';
  end generate;

  
  reg_proc: process
  begin
    wait until rising_edge(clk_i);
    -- synchronize pin inputs
    pins_sync <= pins;
    -- dir- and data-registers
    if rst_i='1' then
      dir_r <= (others=>'0');
      data_r <= (others=>'0');
    else
      -- register write accesses
      if stb_i = '1' and we_i = '1' then
        case adr_i is
          when x"00"  => dir_r  <= dat_i(dir_r'range);                   -- DIR
          when x"04"  => dir_r  <= dir_r or dat_i(dir_r'range);          -- DIR_SET            
          when x"08"  => dir_r  <= dir_r and (not dat_i(dir_r'range));   -- DIR_DELETE            
          when x"0C"  => dir_r  <= dir_r xor dat_i(dir_r'range);         -- DIR_TOGGLE            
          when x"10"  => data_r <= dat_i(data_r'range);                  -- DATA            
          when x"14"  => data_r <= data_r or dat_i(data_r'range);        -- DATA_SET            
          when x"18"  => data_r <= data_r and (not dat_i(data_r'range)); -- DATA_DELETE            
          when x"1C"  => data_r <= data_r xor dat_i(data_r'range);       -- DATA_TOGGLE            
          when others => null;
        end case;
      end if;
    end if;
  end process;
end architecture;
