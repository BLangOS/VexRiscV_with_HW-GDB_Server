----------------------------------------------------------------------------------------
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
entity Memory is
    generic (
        ADR_I_WIDTH   : natural := 14;
        BASE_ADDR     : natural := 16#10000#
    );
    port (
        CLK_I  : in  std_logic;
        RST_I  : in  std_logic;
        STB_I  : in  std_logic;
        WE_I   : in  std_logic;
        SEL_I  : in  std_logic_vector(3 downto 0);
        ADR_I  : in  std_logic_vector(ADR_I_WIDTH - 1 downto 0);
        DAT_I  : in  std_logic_vector(31 downto 0);
        DAT_O  : out std_logic_vector(31 downto 0);
        ACK_O  : out std_logic
    );
end entity;

library ieee;
use ieee.numeric_std.all;

architecture rtl of Memory is
    type   mem_type is array (natural range<>) of std_logic_vector(31 downto 0);
    signal mem       : mem_type(0 to 2**(ADR_I_WIDTH-2)- 1) := (others => x"00000013"); -- Memory filled with NOPs
    signal read_ack  : std_logic;
    signal write_ack : std_logic;
begin

    write_ack <= STB_I and WE_I;
    
    ACK_O <= read_ack or write_ack;
  
    process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            if WE_I='1' and STB_I='1' then
                for i in 0 to 3 loop
                    if SEL_I(i) = '1' then 
                        mem(to_integer(unsigned(ADR_I(ADR_I'length-1 downto 2))))(i*8+7 downto i*8) <= DAT_I(i*8+7 downto i*8); 
                    end if;
                end loop;
            end if;
            if RST_I = '1' then DAT_O <= x"00000000";
            else                DAT_O <= mem(to_integer(unsigned(ADR_I(ADR_I'length-1 downto 2))));
            end if;
        end if;
    end process;

    gen_read_ack: process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            if WE_I='0' and read_ack='0' and STB_I='1' and RST_I='0' then read_ack <= '1';
            else                                                          read_ack <= '0';
            end if;
        end if;                          
    end process;
  
end architecture;
