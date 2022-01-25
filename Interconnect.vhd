---------------------------------------------------------------------------------
-- based on https://github.com/olofk/serv/blob/main/serving/serving_arbiter.v
-- B.Lang, HS-Osnabrueck
---------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity ibus_dbus_mux is
  port (
    --
    dBusWishbone_STB      : in  std_logic;
    dBusWishbone_ACK      : out std_logic;
    dBusWishbone_WE       : in  std_logic;
    dBusWishbone_ADR      : in  unsigned(29 downto 0);
    dBusWishbone_DAT_MISO : out std_logic_vector(31 downto 0);
    dBusWishbone_DAT_MOSI : in  std_logic_vector(31 downto 0);
    dBusWishbone_SEL      : in  std_logic_vector(3 downto 0);
    --
    iBusWishbone_STB      : in  std_logic;
    iBusWishbone_ACK      : out std_logic;
    iBusWishbone_ADR      : in  unsigned(29 downto 0);
    iBusWishbone_DAT_MISO : out std_logic_vector(31 downto 0); 
    --
    Wishbone_STB          : out std_logic;
    Wishbone_ACK          : in  std_logic;
    Wishbone_WE           : out std_logic;
    Wishbone_ADR          : out unsigned(29 downto 0); 
    Wishbone_DAT_MISO     : in  std_logic_vector(31 downto 0);
    Wishbone_DAT_MOSI     : out std_logic_vector(31 downto 0);
    Wishbone_SEL          : out std_logic_vector(3 downto 0)
  );
end ibus_dbus_mux;

architecture arch of ibus_dbus_mux is
begin
   --
   dBusWishbone_DAT_MISO <= Wishbone_DAT_MISO;
   dBusWishbone_ACK      <= '1' when Wishbone_ACK='1' and iBusWishbone_STB='0' else '0';
   --
   iBusWishbone_DAT_MISO <= Wishbone_DAT_MISO;
   iBusWishbone_ACK      <= Wishbone_ACK and iBusWishbone_STB;
   --
   Wishbone_ADR      <= iBusWishbone_ADR when iBusWishbone_STB='1' else
                        dBusWishbone_ADR when dBusWishbone_STB='1' else
                        (others=>'-');
   Wishbone_DAT_MOSI <= dBusWishbone_DAT_MOSI;
   Wishbone_SEL      <= dBusWishbone_SEL;
   Wishbone_WE       <= dBusWishbone_WE when iBusWishbone_STB='0' else '0';
   Wishbone_STB      <= iBusWishbone_STB or dBusWishbone_STB;
   --
end arch;
