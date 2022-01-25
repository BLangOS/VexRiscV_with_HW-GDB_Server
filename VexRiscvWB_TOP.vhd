----------------------------------------------------------------------------------------
-- GDB_RSP_Debug for VexRiscV Project
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------
library ieee;
use ieee.math_real.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity VexRiscvWB_TOP is
  -- Generic Values are for Synthesizing. Testbench may override these values to speed up simulation
  generic (
    DBG_BW   : integer := 434;
    ExtFreq  : real    := 100_000_000.0;             -- External frequency in MHz
    Mult     : real    := 8.000;                     -- Multiply value (2.000-64.000), VCO Frequency must be in range 600-1200 MHz
    Divide   : real    := 16.000;                    -- Divide value (1.000-128.000)
    DBG_Baud : integer := 115200                     -- Baudrate for GDB serial line debug port
  );
  port (
    ExtClk     : in    std_logic;
    -- GDB serial line debug port 
    DBG_RXD    : in    std_logic;
    DBG_TXD    : out   std_logic;
    -- GPIO
    LED        : inout std_logic_vector(15 downto 0);
    sw         : inout std_logic_vector(15 downto 0);
    -- SevenSeg Display
    seg        : inout std_logic_vector( 6 downto 0);
    dp         : inout std_logic;
    an         : inout std_logic_vector( 3 downto 0)
  );
  -- Specify PLL Parameters (BASYS3)
  constant SysFreq       : real    := (ExtFreq*Mult)/Divide;
end VexRiscvWB_TOP;

architecture arch of VexRiscvWB_TOP is
  signal RESET :  std_logic;
  --
  signal SysClk    : std_logic;
  signal SysReset  : std_logic;
--  signal SysResetn : std_logic;
  -- VexRiscV Debug Interface
  signal dbgbus_cmd_valid           : std_logic;
  signal dbgbus_cmd_payload_wr      : std_logic;
  signal dbgbus_cmd_payload_address : unsigned(7 downto 0);
  signal dbgbus_cmd_payload_data    : std_logic_vector(31 downto 0);
  signal dbgbus_cmd_ready           : std_logic;
  signal dbgbus_rsp_data            : std_logic_vector(31 downto 0);
  signal CPU_Halted                 : std_logic;
begin

  -- Power on Reset Signal
  PowerOn_Reset: process(ExtClk) is
    variable cnt : unsigned(4 downto 0) := (others=>'1');
  begin
    if rising_edge(ExtClk) then
      RESET <= '1'; -- assert Reset
      if cnt>0 then cnt:=cnt-1;
      else          RESET <= '0';
      end if;
    end if;
  end process;

  -- Generate system clock from external clock input ExtClk
  clkgen: entity work.ScaleClock
    Generic map (
      ExtFreq => ExtFreq/1_000_000.0,
      Mult    => Mult,
      Divide  => Divide
    )
    Port map (
      ExtClk   => ExtClk,
      ExtReset => RESET,
      SysClk   => SysClk,
      SysReset => SysReset
    );
--SysReset<='0';
--  SysResetn <= not SysReset;
  
  -- The VexRicV Wishbone system
  MCU: entity work.VexRiscvWB_System
    port map (
      clk                        => SysClk,
      reset                      => SysReset,
      -- VexRiscV Debug Interface
      dbgbus_cmd_valid           => dbgbus_cmd_valid,
      dbgbus_cmd_payload_wr      => dbgbus_cmd_payload_wr,
      dbgbus_cmd_payload_address => dbgbus_cmd_payload_address,
      dbgbus_cmd_payload_data    => dbgbus_cmd_payload_data,
      dbgbus_cmd_ready           => dbgbus_cmd_ready,
      dbgbus_rsp_data            => dbgbus_rsp_data,
      CPU_Halted                 => CPU_Halted,
      -- GPIO for switches and LEDs
      GPIO0(15 downto  0)        => LED,
      GPIO0(31 downto 16)        => sw,
      -- GPIO for SevenSegment display
      GPIO1( 6 downto  0)        => seg,
      GPIO1(7)                   => dp,
      GPIO1(11 downto  8)        => an
    );
  --
  Debug_IF: entity work.GDB_RSP_Debug
    generic map (
      DBG_BW => DBG_BW
    )
    port map (
      Clk                        => SysClk,
      DebugReset                 => SysReset,
      CPU_Halted                 => CPU_Halted,
      -- GDB serial line debug port 
      DBG_RXD                    => DBG_RXD,
      DBG_TXD                    => DBG_TXD,
      -- VexRiscV Debug Interface
      dbgbus_cmd_valid           => dbgbus_cmd_valid,
      dbgbus_cmd_payload_wr      => dbgbus_cmd_payload_wr,
      dbgbus_cmd_payload_address => dbgbus_cmd_payload_address,
      dbgbus_cmd_payload_data    => dbgbus_cmd_payload_data,
      dbgbus_cmd_ready           => dbgbus_cmd_ready,
      dbgbus_rsp_data            => dbgbus_rsp_data
    );
  
end arch;