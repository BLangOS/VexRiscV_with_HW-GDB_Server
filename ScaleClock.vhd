----------------------------------------------------------------------------------------
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VComponents.all;
entity ScaleClock is
  Generic (
    ExtFreq : real := 12.0;   -- External frequency in MHz
    Mult    : real := 60.000; -- Multiply value (2.000-64.000)
    Divide  : real := 12.0    -- Divide value (1.000-128.000)
  );
  Port (
    ExtClk   : in  std_logic;
    ExtReset : in  std_logic;
    SysClk   : out std_logic;
    SysReset : out std_logic
  );
end ScaleClock;

architecture arch of ScaleClock is
    -- signal ExtClk_buffered   : std_logic := '0';
    signal clkfb    : std_logic := '0';
    signal SysClk_i : std_logic;
    signal locked   : std_logic := '0';
begin

  -- MMCME2_BASE: Base Mixed Mode Clock Manager, 7 Series
  MMCME2_BASE_inst : MMCME2_BASE
    generic map (
      BANDWIDTH          => "OPTIMIZED",     -- Jitter programming (OPTIMIZED, HIGH, LOW)
      CLKFBOUT_MULT_F    => Mult,            -- Multiply value for all CLKOUT (2.000-64.000).
      CLKFBOUT_PHASE     => 0.000,           -- Phase offset in degrees of CLKFB (-360.000-360.000).
      CLKIN1_PERIOD      => 1000.0/ExtFreq,  -- Input clock period in ns to ps resolution (i.e. 83.333 is 12 MHz).
      -- CLKOUT0_DIVIDE: Divide amount for CLKOUT0 (1.000-128.000).
      CLKOUT0_DIVIDE_F   => Divide, -- 18.0, 
      -- CLKOUT1_DIVIDE - CLKOUT6_DIVIDE: Divide amount for each CLKOUT (1-128)
      CLKOUT1_DIVIDE     => 100,
      CLKOUT2_DIVIDE     => 100,
      CLKOUT3_DIVIDE     => 100,
      CLKOUT4_DIVIDE     => 100,
      CLKOUT5_DIVIDE     => 100,
      CLKOUT6_DIVIDE     => 100,
      -- CLKOUT0_DUTY_CYCLE - CLKOUT6_DUTY_CYCLE: Duty cycle for each CLKOUT (0.01-0.99).
      CLKOUT0_DUTY_CYCLE => 0.5,
      CLKOUT1_DUTY_CYCLE => 0.5,
      CLKOUT2_DUTY_CYCLE => 0.5,
      CLKOUT3_DUTY_CYCLE => 0.5,
      CLKOUT4_DUTY_CYCLE => 0.5,
      CLKOUT5_DUTY_CYCLE => 0.5,
      CLKOUT6_DUTY_CYCLE => 0.5,
      -- CLKOUT0_PHASE - CLKOUT6_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
      CLKOUT0_PHASE      => 0.0,
      CLKOUT1_PHASE      => 0.0,
      CLKOUT2_PHASE      => 0.0,
      CLKOUT3_PHASE      => 0.0,
      CLKOUT4_PHASE      => 0.0,
      CLKOUT5_PHASE      => 0.0,
      CLKOUT6_PHASE      => 0.0,
      CLKOUT4_CASCADE    => FALSE, -- Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
      DIVCLK_DIVIDE      => 1,     -- Master division value (1-106)
      REF_JITTER1        => 0.0,   -- Reference input jitter in UI (0.000-0.999).
      STARTUP_WAIT       => FALSE  -- Delays DONE until MMCM is locked (FALSE, TRUE)
    )
    port map (
      -- Clock Outputs: 1-bit (each) output: User configurable clock outputs
      CLKOUT0   => SysClk_i, 
      CLKOUT0B  => open,
      CLKOUT1   => open,
      CLKOUT1B  => open,
      CLKOUT2   => open,
      CLKOUT2B  => open,
      CLKOUT3   => open,
      CLKOUT3B  => open,
      CLKOUT4   => open,
      CLKOUT5   => open,
      CLKOUT6   => open,
      -- Feedback Clocks: 1-bit (each) output: Clock feedback ports
      CLKFBOUT  => clkfb, 
      CLKFBOUTB => open,
      -- Status Ports: 1-bit (each) output: MMCM status ports
      LOCKED    => locked,
      -- Clock Inputs: 1-bit (each) input: Clock input
      CLKIN1    => ExtClk, -- ExtClk_buffered,
      -- Control Ports: 1-bit (each) input: MMCM control ports
      PWRDWN    => '0',
      RST       => ExtReset,
      -- Feedback Clocks: 1-bit (each) input: Clock feedback ports
      CLKFBIN   => clkfb
    );
    
  SysClk <= SysClk_i;

  process(SysClk_i)
  begin
    if rising_edge(SysClk_i) then
      SysReset <= not locked;
    end if;
  end process;
  
end arch;