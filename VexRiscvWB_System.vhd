----------------------------------------------------------------------------------------
-- GDB_RSP_Debug for VexRiscV Project
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity VexRiscvWB_System is
  port (
    clk                        : in    std_logic;
    reset                      : in    std_logic;
    -- VexRiscV Debug Interface
    dbgbus_cmd_valid           : in    std_logic;
    dbgbus_cmd_payload_wr      : in    std_logic;
    dbgbus_cmd_payload_address : in    unsigned(7 downto 0);
    dbgbus_cmd_payload_data    : in    std_logic_vector(31 downto 0);
    dbgbus_cmd_ready           : out   std_logic;
    dbgbus_rsp_data            : out   std_logic_vector(31 downto 0);
    CPU_Halted                 : in    std_logic;
    -- GPIO
    GPIO0                      : inout std_logic_vector(31 downto 0);
    GPIO1                      : inout std_logic_vector(11 downto 0)
  );
end VexRiscvWB_System;

architecture arch of VexRiscvWB_System is
  signal dBusWishbone_STB           : std_logic;
  signal dBusWishbone_ACK           : std_logic;
  signal dBusWishbone_WE            : std_logic;
  signal dBusWishbone_ADR           : unsigned(29 downto 0);
  signal dBusWishbone_DAT_MISO      : std_logic_vector(31 downto 0);
  signal dBusWishbone_DAT_MOSI      : std_logic_vector(31 downto 0);
  signal dBusWishbone_SEL           : std_logic_vector(3 downto 0);
  --                                
  signal iBusWishbone_STB           : std_logic;
  signal iBusWishbone_ACK           : std_logic;
  signal iBusWishbone_ADR           : unsigned(29 downto 0);
  signal iBusWishbone_DAT_MISO      : std_logic_vector(31 downto 0); 
  --                                
  signal SYS_STB                    : std_logic;
  signal SYS_ACK                    : std_logic;
  signal SYS_WE                     : std_logic;
  signal SYS_ADR_unsigned           : unsigned(29 downto 0);
  signal SYS_ADR                    : std_logic_vector(31 downto 0);
  signal SYS_DAT_I                  : std_logic_vector(31 downto 0);
  signal SYS_DAT_O                  : std_logic_vector(31 downto 0);
  signal SYS_SEL                    : std_logic_vector(3 downto 0);
  --
  signal SYS_RESET                  : std_logic;
  signal DBG_RESET                  : std_logic;
  -- Interrupt Lines
  signal Timer0_Interrupt           : std_logic;
  signal Timer1_Interrupt           : std_logic;
begin
  --
  SYS_RESET <= '1' when reset='1' else
               '1' when DBG_RESET='1' else
               '0';
  CPU: entity work.VexRiscv
    port map (
      clk                           => clk,                        -- : in std_logic;
      reset                         => SYS_RESET,                  -- : in std_logic;
      debugReset                    => reset,                      -- : in std_logic
      debug_resetOut                => DBG_RESET,                  -- : out std_logic;
      --                            
      LocalInt0                     => Timer1_Interrupt,           -- : in std_logic;
      LocalInt1                     => '0',                        -- : in std_logic;
      LocalInt2                     => '0',                        -- : in std_logic;
      LocalInt3                     => '0',                        -- : in std_logic;
      timerInterrupt                => Timer0_Interrupt,           -- : in std_logic;
      externalInterrupt             => '0',                        -- : in std_logic;
      softwareInterrupt             => '0',                        -- : in std_logic;
      --
      debug_bus_cmd_valid           => dbgbus_cmd_valid,           -- : in std_logic;
      debug_bus_cmd_ready           => dbgbus_cmd_ready,           -- : out std_logic;
      debug_bus_cmd_payload_wr      => dbgbus_cmd_payload_wr,      -- : in std_logic;
      debug_bus_cmd_payload_address => dbgbus_cmd_payload_address, -- : in unsigned(7 downto 0);
      debug_bus_cmd_payload_data    => dbgbus_cmd_payload_data,    -- : in std_logic_vector(31 downto 0);
      debug_bus_rsp_data            => dbgbus_rsp_data,            -- : out std_logic_vector(31 downto 0);
      --
      iBusWishbone_CYC              => open,                       -- : out std_logic;
      iBusWishbone_STB              => iBusWishbone_STB,           -- : out std_logic;
      iBusWishbone_ACK              => iBusWishbone_ACK,           -- : in std_logic;
      iBusWishbone_WE               => open,                       -- : out std_logic;
      iBusWishbone_ADR              => iBusWishbone_ADR,           -- : out unsigned(29 downto 0);
      iBusWishbone_DAT_MISO         => iBusWishbone_DAT_MISO,      -- : in std_logic_vector(31 downto 0);
      iBusWishbone_DAT_MOSI         => open,                       -- : out std_logic_vector(31 downto 0);
      iBusWishbone_SEL              => open,                       -- : out std_logic_vector(3 downto 0);
      iBusWishbone_ERR              => '0',                        -- : in std_logic;
      iBusWishbone_CTI              => open,                       -- : out std_logic_vector(2 downto 0);
      iBusWishbone_BTE              => open,                       -- : out std_logic_vector(1 downto 0);
      --                                                           
      dBusWishbone_CYC              => open,                       -- : out std_logic;
      dBusWishbone_STB              => dBusWishbone_STB,           -- : out std_logic;
      dBusWishbone_ACK              => dBusWishbone_ACK,           -- : in std_logic;
      dBusWishbone_WE               => dBusWishbone_WE,            -- : out std_logic;
      dBusWishbone_ADR              => dBusWishbone_ADR,           -- : out unsigned(29 downto 0);
      dBusWishbone_DAT_MISO         => dBusWishbone_DAT_MISO,      -- : in std_logic_vector(31 downto 0);
      dBusWishbone_DAT_MOSI         => dBusWishbone_DAT_MOSI,      -- : out std_logic_vector(31 downto 0);
      dBusWishbone_SEL              => dBusWishbone_SEL,           -- : out std_logic_vector(3 downto 0);
      dBusWishbone_ERR              => '0',                        -- : in std_logic;
      dBusWishbone_CTI              => open,                       -- : out std_logic_vector(2 downto 0);
      dBusWishbone_BTE              => open                        -- : out std_logic_vector(1 downto 0);
    );
    --
  BUS_MUX: entity work.ibus_dbus_mux
    port map (
      --
      dBusWishbone_STB      => dBusWishbone_STB,
      dBusWishbone_ACK      => dBusWishbone_ACK,
      dBusWishbone_WE       => dBusWishbone_WE,
      dBusWishbone_ADR      => dBusWishbone_ADR,
      dBusWishbone_DAT_MISO => dBusWishbone_DAT_MISO,
      dBusWishbone_DAT_MOSI => dBusWishbone_DAT_MOSI,
      dBusWishbone_SEL      => dBusWishbone_SEL,
      --
      iBusWishbone_STB      => iBusWishbone_STB,
      iBusWishbone_ACK      => iBusWishbone_ACK,
      iBusWishbone_ADR      => iBusWishbone_ADR,
      iBusWishbone_DAT_MISO => iBusWishbone_DAT_MISO,
      --
      Wishbone_STB          => SYS_STB,
      Wishbone_ACK          => SYS_ACK,
      Wishbone_WE           => SYS_WE,
      Wishbone_ADR          => SYS_ADR_unsigned,
      Wishbone_DAT_MISO     => SYS_DAT_I,
      Wishbone_DAT_MOSI     => SYS_DAT_O,
      Wishbone_SEL          => SYS_SEL
    );
  SYS_ADR <= std_logic_vector(SYS_ADR_unsigned)&"00";
  ------------------------------------------------------------
  -- Wishbone Interconnect
  ------------------------------------------------------------
  intercon_block: block is
    signal ROM_STB            : std_logic;
    signal ROM_WE             : std_logic;
    signal ROM_ACK            : std_logic;
    signal ROM_DAT_O          : std_logic_vector(31 downto 0);

    signal RAM_STB            : std_logic;
    signal RAM_ACK            : std_logic;
    signal RAM_DAT_O          : std_logic_vector(31 downto 0);

    signal GPIO0_STB          : std_logic;
    signal GPIO0_ACK          : std_logic;
    signal GPIO0_DAT_O        : std_logic_vector(31 downto 0);

    signal GPIO1_STB          : std_logic;
    signal GPIO1_ACK          : std_logic;
    signal GPIO1_DAT_O        : std_logic_vector(31 downto 0);

    signal Timer0_STB         : std_logic;
    signal Timer0_ACK         : std_logic;
    signal Timer0_DAT_O       : std_logic_vector(31 downto 0);

    signal Timer1_STB         : std_logic;
    signal Timer1_ACK         : std_logic;
    signal Timer1_DAT_O       : std_logic_vector(31 downto 0);
  begin
    ------------------------------------------------------------
    -- Bus Decoding
    ------------------------------------------------------------
    -- Address Decoder
    ROM_STB     <= SYS_STB when unsigned(SYS_ADR) >= 16#00010000# and unsigned(SYS_ADR) <= 16#00013FFF# else '0';
    RAM_STB     <= SYS_STB when unsigned(SYS_ADR) >= 16#00000000# and unsigned(SYS_ADR) <= 16#00003FFF# else '0';
    GPIO0_STB   <= SYS_STB when unsigned(SYS_ADR) >= 16#00008000# and unsigned(SYS_ADR) <= 16#00008023# else '0';
    GPIO1_STB   <= SYS_STB when unsigned(SYS_ADR) >= 16#00008100# and unsigned(SYS_ADR) <= 16#00008123# else '0';
    Timer0_STB  <= SYS_STB when unsigned(SYS_ADR) >= 16#00008200# and unsigned(SYS_ADR) <= 16#0000820F# else '0';
    Timer1_STB  <= SYS_STB when unsigned(SYS_ADR) >= 16#00008300# and unsigned(SYS_ADR) <= 16#0000830F# else '0';
    -- Read Data MUX
    SYS_DAT_I   <= ROM_DAT_O     when ROM_STB     = '1' else
                   RAM_DAT_O     when RAM_STB     = '1' else
                   GPIO0_DAT_O   when GPIO0_STB   = '1' else
                   GPIO1_DAT_O   when GPIO1_STB   = '1' else
                   Timer0_DAT_O  when Timer0_STB  = '1' else
                   Timer1_DAT_O  when Timer1_STB  = '1' else
                   (others=>'1');
    -- Acknowledge MUX
    SYS_ACK     <= ROM_ACK       when ROM_STB     = '1' else
                   RAM_ACK       when RAM_STB     = '1' else
                   GPIO0_ACK     when GPIO0_STB   = '1' else
                   GPIO1_ACK     when GPIO1_STB   = '1' else
                   Timer0_ACK    when Timer0_STB  = '1' else
                   Timer1_ACK    when Timer1_STB  = '1' else
                   '1';

    ------------------------------------------------------------
    -- ROM
    ------------------------------------------------------------
    ROM_WE <= '1' when SYS_WE='1' and CPU_Halted='1' else '0';
    ROM_Inst: entity work.Memory
      generic map (
        ADR_I_WIDTH   => 14,
        BASE_ADDR     => 16#00010000#
      )
      port map(
        CLK_I         => CLK,
        RST_I         => SYS_RESET,
        STB_I         => ROM_STB,
        WE_I          => ROM_WE,
        SEL_I         => SYS_SEL,
        ADR_I         => SYS_ADR(13 downto 0),
        DAT_I         => SYS_DAT_O,
        DAT_O         => ROM_DAT_O,
        ACK_O         => ROM_ACK
      );

    ------------------------------------------------------------
    -- RAM
    ------------------------------------------------------------
    RAM_Inst: entity work.Memory
      generic map (
        ADR_I_WIDTH  => 14,
        BASE_ADDR    => 16#00000000#
      )
      port map(
        CLK_I => CLK,
        RST_I => SYS_RESET,
        STB_I => RAM_STB,
        WE_I  => SYS_WE,
        SEL_I => SYS_SEL,
        ADR_I => SYS_ADR(13 downto 0),
        DAT_I => SYS_DAT_O,
        DAT_O => RAM_DAT_O,
        ACK_O => RAM_ACK
      );

    ------------------------------------------------------------
    -- GPIO0
    ------------------------------------------------------------
    GPIO0_Inst: entity work.GPIO
      generic map (
        NUM_PORTS => GPIO0'length
      ) port map (
        CLK_I => CLK,
        RST_I => SYS_RESET,
        STB_I => GPIO0_STB,
        WE_I  => SYS_WE,
        ADR_I => SYS_ADR(7 downto 0),
        DAT_I => SYS_DAT_O,
        DAT_O => GPIO0_DAT_O,
        ACK_O => GPIO0_ACK,
        Pins  => GPIO0
      );

    ------------------------------------------------------------
    -- GPIO1
    ------------------------------------------------------------
    GPIO1_Inst: entity work.GPIO
      generic map (
        NUM_PORTS => GPIO1'length
      ) port map (
        CLK_I => CLK,
        RST_I => SYS_RESET,
        STB_I => GPIO1_STB,
        WE_I  => SYS_WE,
        ADR_I => SYS_ADR(7 downto 0),
        DAT_I => SYS_DAT_O,
        DAT_O => GPIO1_DAT_O,
        ACK_O => GPIO1_ACK,
        Pins  => GPIO1
      );

    ------------------------------------------------------------
    -- Timer0
    ------------------------------------------------------------
    Timer0_Inst: entity work.Timer
      port map(
        CLK_I     => CLK,
        RST_I     => SYS_RESET,
        STB_I     => Timer0_STB,
        WE_I      => SYS_WE,
        ADR_I     => SYS_ADR(3 downto 0),
        DAT_I     => SYS_DAT_O,
        ACK_O     => Timer0_ACK,
        DAT_O     => Timer0_DAT_O,
        Timer_IRQ => Timer0_Interrupt
      );

    ------------------------------------------------------------
    -- Timer1
    ------------------------------------------------------------
    Timer1_Inst: entity work.Timer
      port map(
        CLK_I     => CLK,
        RST_I     => SYS_RESET,
        STB_I     => Timer1_STB,
        WE_I      => SYS_WE,
        ADR_I     => SYS_ADR(3 downto 0),
        DAT_I     => SYS_DAT_O,
        ACK_O     => Timer1_ACK,
        DAT_O     => Timer1_DAT_O,
        Timer_IRQ => Timer1_Interrupt
      );
      
  end block;

end arch;