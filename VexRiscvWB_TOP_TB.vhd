----------------------------------------------------------------------------------------
-- GDB_RSP_Debug for VexRiscV Project
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------

entity VexRiscvWB_TOP_TB is
  constant ExtFreq      : real := 100_000_000.0;
  constant ExtResetTime : time := 50 ns;
end VexRiscvWB_TOP_TB;

library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

use work.string_stream.all;

architecture test of VexRiscvWB_TOP_TB is
  signal ExtClk   : std_logic := '0';
  signal ExtRESET : std_logic := '0';
  --
  signal s_valid  : std_logic;
  signal s_last   : std_logic:='0';
  signal s_data   : std_logic_vector(7 downto 0);
  signal s_ready  : std_logic;
  --
  signal DBG_RXD  : std_logic := '1';
  signal DBG_TXD  : std_logic := '1';
  --
  signal response : character;
begin
  
  ExtClk   <= not ExtClk after 0.5 sec / ExtFreq;
  ExtRESET <= '1' after 0 ns, '0' after ExtResetTime;

  DUT: entity work.VexRiscvWB_TOP
    generic map (
      DBG_BW => 3
    )
    port map (
      ExtClk     => ExtClk, 
      -- GDB serial link debug port 
      DBG_RXD    => DBG_RXD,
      DBG_TXD    => DBG_TXD,
      -- GPIO
      LED        => open,
      sw         => open,
      -- SevenSeg Display
      seg        => open,
      dp         => open,
      an         => open
    );

  DBG_Sender: entity work.UART_Sender_Simulation
    generic map (
      Baudrate      => real(12500000),
      Bitanzahl     => 8
    )
    port map (
      Takt    => ExtClk,
      Reset   => ExtRESET,
      -- Eingang
      s_valid => s_valid,
      s_data  => s_data,
      s_ready => s_ready,
      -- Ausgang
      TxD     => DBG_RXD
    );
    
  DBG_Receiver: process
    constant Bitbreite : time := 1 sec / real(12500000);
    variable rec_val : std_logic_vector(7 downto 0);
  begin
    wait until falling_edge(DBG_TXD) and DBG_TXD='0';
    wait for Bitbreite*1.5;
    response <= ' ';
    for i in 0 to 7 loop
      rec_val(i) := DBG_TXD;
      wait for Bitbreite;
    end loop;
    response <= character'val(to_integer(unsigned(rec_val)));
    wait for Bitbreite;
  end process;
  
  Stimulate: process
    variable message : string(1 to MsgLen);
  begin
    s_valid <= '0';
    wait for 2 us; -- PLL must be locked, check simultion wave
    wait until rising_edge(ExtClk);
    --
    -- extended remote
    Send_String("$!#00",                        Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 0.4 us;
    wait until rising_edge(ExtClk) and response='#';
    -- monitor r
    Send_String("$qRcmd,72#00",                 Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    --
    --Send_String("$g#00",                        Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    --wait until rising_edge(ExtClk) and response='#';
    --
    -- Load test program
    Send_String("$M10000,10:6F000009000000000000000000000000#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10010,10:6F00600B6F00200B6F00E00A6F00A00A#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10020,10:6F00600A6F00200A6F00E0096F00A009#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10030,10:6F0060096F0020096F00E0086F00A008#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10040,10:6F0060086F0020086F00E0076F00A007#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10050,10:6F0060076F0020076F00E0066F00A006#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10060,10:6F0060066F0020066F00E0056F00A005#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10070,10:6F0060056F0020056F00E0046F00A004#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10080,10:6F0060046F0020046F00E0036F00A003#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M10090,10:73F005301711FFFF1301C1F693010000#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M100A0,10:93811100890193813100938141009381#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M100B0,10:510099019D0193818100938191009381#00",Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$M100C0,0C:A1006FF0BFFD6F0000000000#00"        ,Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    -- resume
    wait until rising_edge(ExtClk) and response='#';
    Send_String("$c#00",                        Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 2 us;
    wait until rising_edge(ExtClk) and response='+';
    -- ctrl-C
    Send_String(""&ETX,                         Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 1 us;
    wait until rising_edge(ExtClk) and response='#';
    -- Step
    Send_String("$s#00",                        Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 2 us;
    wait until rising_edge(ExtClk) and response='#';
    -- Step
    Send_String("$s#00",                        Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 2 us;
    wait until rising_edge(ExtClk) and response='#';
    -- resume
    Send_String("$c#00",                        Message, ExtClk, s_valid, s_last, s_data, s_ready); wait for 2 us;
    wait until rising_edge(ExtClk) and response='+';

    report "Fertig" severity note;
    wait;
  end process;
  
 
end test;