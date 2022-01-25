----------------------------------------------------------------------------------------
-- GDB_RSP_Debug for VexRiscV Project
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
package string_stream is
  constant MsgLen: integer := 300;
  function SetMessage(constant message: string) return string;
  procedure Send_String(
    constant command  : in    string;
    variable message  : inout string;
    signal Clk        : in    std_logic;
    signal out_valid  : out   std_logic;
    signal out_last   : out   std_logic;
    signal out_data   : out   std_logic_vector(7 downto 0);
    signal out_ready  : in    std_logic
  );
  procedure Receive_String(
    variable response : out string;
    signal Clk        : in  std_logic;
    signal in_valid   : in  std_logic;
    signal in_last    : in  std_logic;
    signal in_data    : in  std_logic_vector(7 downto 0);
    signal in_ready   : out std_logic
  );
end package;

library ieee;
use ieee.numeric_std.all;
package body string_stream is
  ----------------------------------------------------------------------------
  function SetMessage(constant message: string) return string is
    variable m: string(1 to MsgLen) := (others=>' ');
  begin
    m(message'range) := message;
    return m;
  end;
  ----------------------------------------------------------------------------
  procedure Send_String(
    constant command  : in    string;
    variable message  : inout string;
    signal Clk        : in    std_logic;
    signal out_valid  : out   std_logic;
    signal out_last   : out   std_logic;
    signal out_data   : out   std_logic_vector(7 downto 0);
    signal out_ready  : in    std_logic
  ) is
  begin
    message := (message'range =>' ');
    out_valid <= '0';
    out_last  <= '0';
    out_data  <= x"00";
    wait until rising_edge(Clk);
    for i in command'range loop
      out_valid <= '1';
      if i=command'right then out_last<='1'; else out_last<='0'; end if;
      out_data  <= std_logic_vector(to_unsigned(character'pos(command(i)),8));
      wait until rising_edge(Clk) and out_ready='1';
    end loop;
    message := SetMessage(command);
    out_valid <= '0';
    out_last  <= '0';
    out_data  <= x"00";
    wait until rising_edge(Clk);
    message := (message'range =>' ');
  end procedure;
  ----------------------------------------------------------------------------
  procedure Receive_String(
    variable response : out string;
    signal Clk        : in  std_logic;
    signal in_valid   : in  std_logic;
    signal in_last    : in  std_logic;
    signal in_data    : in  std_logic_vector(7 downto 0);
    signal in_ready   : out std_logic
  ) is
    variable index: integer;
    variable char:  character;
    variable resp:  string(response'range);
  begin
    response := (response'range =>' ');
    index := 1;
    in_ready <= '1';
    loop
      wait until rising_edge(Clk) and in_valid='1';
      char := character'val(to_integer(unsigned(in_data)));
      resp(index) := char;
      exit when in_last='1'; 
      if index<MsgLen then index := index+1;
      else report "Receive_String: response to short" severity error;
      end if;
    end loop;
    response(1 to index) := resp(1 to index);
    in_ready <= '0';
    wait until rising_edge(Clk);
    response := (response'range =>' ');
  end procedure;
  ----------------------------------------------------------------------------
end package body;

