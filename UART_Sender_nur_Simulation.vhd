----------------------------------------------------------------------------------------
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------
-- Serieller Sender
-------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
entity UART_Sender_Simulation is
  generic (
    Baudrate      : real;
    Bitanzahl     : integer  -- Anzahl der Bits pro Datenwort
  );
  port (
    Takt    : in  std_logic;
    Reset   : in  std_logic;
    -- Eingang
    s_valid : in  std_logic;
    s_data  : in  std_logic_vector(Bitanzahl-1 downto 0);
    s_ready : out std_logic;
    -- Ausgang
    TxD     : out std_logic
  );
end UART_Sender_Simulation;

library ieee;
use ieee.numeric_std.all;
architecture Nur_Simulation of UART_Sender_Simulation is
  constant SIMULATION: boolean := true;
  constant Bitbreite : time := 1 sec / Baudrate;
  signal marker: integer:=0;
begin

  SimModel: process
    variable SendeWert: std_logic_vector(Bitanzahl-1 downto 0);
  begin
    Outer_Loop: loop
      -- Initialisierung
      s_ready  <= '0';
      TxD <= '1';
      wait until rising_edge(Takt);
      exit Outer_Loop when Reset='1'; 
      assert s_valid='1' or s_valid='0' report "Falscher Wert für Signal 's_valid' (1)" severity Failure;
      
      loop -- Endlosschleife zum Senden von Zeichen
        marker <= 0;
      
        -- Auf neuen Wert zum Senden warten
        s_ready <= '1';
        wait until rising_edge(Takt);
        exit Outer_Loop when Reset='1'; 
        while s_valid='0' loop
          wait until rising_edge(Takt);
          exit Outer_Loop when Reset='1'; 
        end loop;
        assert s_valid='1' report "Falscher Wert für Signal 's_valid' (2)" severity Failure;
        SendeWert := s_data;
        
        -- Startbit senden
        marker <= marker+1;
        s_ready  <= '0';
        TxD <= '0';
        wait for Bitbreite;
        exit Outer_Loop when Reset='1'; 
    
        -- Wert senden
        for b in 0 to Bitanzahl-1 loop
          marker <= marker+1;
          TxD <= SendeWert(b);
          wait for Bitbreite;
          exit Outer_Loop when Reset='1'; 
        end loop;
        
        -- 2 Stoppbits senden
        marker <= marker+1;
        TxD <= '1';
        wait for Bitbreite;
        exit Outer_Loop when Reset='1'; 
        marker <= marker+1;
        wait for Bitbreite;
        exit Outer_Loop when Reset='1'; 

        wait until rising_edge(Takt);
        exit Outer_Loop when Reset='1'; 
      
      end loop;
    end loop;
    
  end process;
  
end Nur_Simulation;


