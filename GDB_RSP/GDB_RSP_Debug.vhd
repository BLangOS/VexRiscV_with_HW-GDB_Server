----------------------------------------------------------------------------------------
-- GDB_RSP_Debug for VexRiscV Project
-- (c) Bernhard Lang, Hochschule Osnabrueck
----------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
entity GDB_RSP_Debug is
  generic (
    constant DBG_BW : integer := 434
  );
  port (
    Clk        : in  std_logic := '0'; 
    DebugReset : in  std_logic := '1';
    CPU_Halted : out std_logic;
    -- Serial Port
    DBG_RXD    : in  std_logic;
    DBG_TXD    : out std_logic;
    -- VexRiscV Debug Interface
    dbgbus_cmd_valid           : out std_logic;
    dbgbus_cmd_payload_wr      : out std_logic;
    dbgbus_cmd_payload_address : out unsigned(7 downto 0);
    dbgbus_cmd_payload_data    : out std_logic_vector(31 downto 0);
    dbgbus_cmd_ready           : in  std_logic;
    dbgbus_rsp_data            : in  std_logic_vector(31 downto 0)
  );
end GDB_RSP_Debug;
architecture arch of GDB_RSP_Debug is
  signal in_valid      : std_logic := '0';
  signal in_data       : std_logic_vector(7 downto 0);
  signal in_ready      : std_logic;
  signal out_valid     : std_logic;
  signal out_last      : std_logic;
  signal out_data      : std_logic_vector(7 downto 0);
  signal out_ready     : std_logic;
  type debug_action is (Halt,Resume,Step,ReadReg,WriteReg,ReadPC,WritePC,
                        ReadByte,WriteByte,DoReset,CheckHalted,AdjustPC,
                        SetHwBp,ClrHwBp,none);
  signal A_valid       : std_logic;
  signal A_Action      : debug_action;
  signal A_Addr        : std_logic_vector(31 downto 0);
  signal A_Value       : std_logic_vector(31 downto 0);
  signal R_OK          : std_logic;
  signal R_Data        : std_logic_vector(31 downto 0);
  signal A_ready       : std_logic;
  signal rspC_valid    : std_logic;
  signal GRRF_last     : std_logic;
  signal GRRF_data     : std_logic_vector(7 downto 0);
  signal GRRF_ready    : std_logic;
  signal GRRF_valid    : std_logic;
  signal rspC_last     : std_logic;
  signal rspC_data     : std_logic_vector(7 downto 0);
  signal rspC_ready    : std_logic;
  signal rspR_valid    : std_logic;
  signal rspR_data     : std_logic_vector(7 downto 0);
  signal rspR_ready    : std_logic;
  signal DebugReset_n  : std_logic;
begin
  DebugReset_n <= not DebugReset;
  DBG_RTXD_block: block
    constant DBG_BW_Bits  : natural := 9;
    constant DBG_BW_Vector: unsigned(DBG_BW_Bits-1 downto 0) := to_unsigned(DBG_BW,DBG_BW_Bits);
    signal serial_in_r0    : std_logic := '1';
    signal serial_in_r1    : std_logic := '1';
    signal cntval          : unsigned(DBG_BW_Vector'range) := (others=>'0');
    signal load            : std_logic;
    signal enablesr        : std_logic;
    signal m_axis_tdata_i  : std_logic_vector (7 downto 0) := (others=>'-');
    signal TC     : std_logic := '1';
    signal Zero   : std_logic := '1';
    signal Laden   : std_logic := '1';
    signal EnBCnt : std_logic := '1';
    type Zustaende is (Warte, Aktiv, Start, Bit_i, Parity, Stopp, Err );
    signal Zustand : Zustaende := Warte;
    signal FolgeZ  : Zustaende;
  begin
    FFs: process(Clk)
    begin
      if rising_edge(Clk) then
        if DebugReset_n = '0' then
          serial_in_r0 <= '1';
          serial_in_r1 <= '1';
        else
          serial_in_r0 <= DBG_RXD;
          serial_in_r1 <= serial_in_r0;
        end if;
      end if;
    end process;
    Modulo_counter1: process(Clk)
    begin
      if rising_edge(Clk) then
        if DebugReset_n='0' then
          cntval <= DBG_BW_Vector;
        else
          if (cntval=0) or (serial_in_r0 /= serial_in_r1) then
            cntval <= DBG_BW_Vector;
          else
            cntval <= cntval - 1;
          end if;
        end if;
      end if;
    end process;
    EnableSR <= '1' when cntval = ('0' & DBG_BW_Vector(DBG_BW_Vector'high downto 1)) else '0';
    Shift_Register1: process (Clk)
      variable value: std_logic_vector((8+2)-1 downto 0):=(others=>'1');
    begin
      if rising_edge(Clk) then
        if DebugReset_n='0' then
          value      := (others => '1');
          Load       <= '0';
          m_axis_tdata_i  <= (others=>'-');
        else
          if Load='1' then
            value := (others => '1');
          elsif EnableSR='1' then
            value := serial_in_r1 & value(value'high downto 1);
          end if;
          Load           <= not value(0);
          m_axis_tdata_i <= value(8 downto 1);
        end if;
        if DebugReset = '1' then
          in_valid <= '0';
          in_data  <= (others=>'-');
        else
          if (in_ready or (not in_valid)) = '1' then
            in_valid <= Load;
            in_data  <= m_axis_tdata_i;
          end if;
        end if;
      end if;
    end process;
    FZ_Mealy: process(Zustand, out_valid, TC, Zero, DebugReset_n)
    begin
      FolgeZ <= Err;
      Laden  <= '0';
      EnBCnt <= '0';
      if DebugReset_n='0' then FolgeZ <= Warte;
      elsif DebugReset_n='1' then
        case Zustand is
          when Warte  =>
            if    out_valid='0' then FolgeZ <= Warte;
            elsif out_valid='1' then FolgeZ <= Aktiv; Laden <= '1';
            end if;
          when Aktiv  =>
            if    TC='0' then FolgeZ <= Aktiv;
            elsif TC='1' then FolgeZ <= Start;
            end if;
          when Start  =>
            if    TC='0' then FolgeZ <= Start;
            elsif TC='1' then FolgeZ <= Bit_i;
            end if;
          when Bit_i  =>
            if    TC='0' then FolgeZ <= Bit_i;
            elsif TC='1' and Zero='0' then FolgeZ <= Bit_i;  EnBCnt <= '1';
            elsif TC='1' and Zero='1' then FolgeZ <= Parity;
            end if;
          when Parity =>
            if    TC='0' then FolgeZ <= Parity;
            elsif TC='1' then FolgeZ <= Stopp;
            end if;
          when Stopp  =>
            if    TC='0' then FolgeZ <= Stopp;
            elsif TC='1' then FolgeZ <= Warte;
            end if;
          when Err    =>
            null;
        end case;
      end if;
    end process;
    Z_Moore: process (Clk)
    begin
      if rising_edge(Clk) then
        Zustand <= FolgeZ;
        case FolgeZ is
          when Warte  => out_ready <= '1';
          when Aktiv  => out_ready <= '0';
          when Start  => out_ready <= '0';
          when Bit_i  => out_ready <= '0';
          when Parity => out_ready <= '0';
          when Stopp  => out_ready <= '0';
          when Err    => out_ready <= 'X';
        end case;
      end if;
    end process;
    modulo_counter2: process(Clk)
      variable value : integer range 0 to 2 ** DBG_BW_Bits - 1 := 0;
    begin
      if rising_edge(Clk) then
        if DebugReset_n = '0' then
          value := to_integer(DBG_BW_Vector);
         else
          if value > 0 then
            value := value - 1;
          else
            value := to_integer(DBG_BW_Vector);
          end if;
        end if;
        if value = 0 then TC <= '1'; else TC <= '0'; end if;
      end if;
    end process;
    shift_register2: process (Clk)
      variable value : std_logic_vector((8 + 3) - 1 downto 0) := (others=>'1');
    begin
      if rising_edge(Clk) then
        if DebugReset_n = '0' then
          value        := (others => '1');
          DBG_TXD <= value(0);
        else
          if Laden = '1' then
            value := '1' & out_data(7 downto 0) & "01";
          elsif TC = '1' then
            value := '1' & value(value'high downto 1);
          end if;
          DBG_TXD <= value(0);
        end if;
      end if;
    end process;
    BitCounter:  process(Clk)
      variable value : integer range 0 to 7 := 0;
    begin
      if rising_edge(Clk) then
        if DebugReset_n = '0' then
          value := 0;
          Zero  <= '1';
        else
          if Laden='1' then
            value := 7;
            Zero  <= '0';
          elsif value > 1 and EnBCnt = '1' then
            value := value-1;
            Zero  <= '0';
          elsif value=1 and EnBCnt = '1' then
            value := value - 1;
            Zero  <= '1';
          end if;
        end if;
      end if;
    end process;
  end block;
  GRRF: block
    signal IsCtrlC  : std_logic;
    signal IsDollar : std_logic;
    signal IsHash   : std_logic;
    signal EnReg    : std_logic;
    type   TheStates is (WaitS, CtrlC, Dollar, Cmd, Error);
    signal state      : TheStates := Error;
    signal next_state : TheStates;
  begin
    process(Clk)
    begin
      if rising_edge(Clk) then
        if EnReg='1' then
          GRRF_data <= in_data;
        end if;
      end if;
    end process;
    IsCtrlC  <= '1' when in_data=std_logic_vector(to_unsigned(character'pos(etx),8)) else
                '0';
    IsDollar <= '1' when in_data=std_logic_vector(to_unsigned(character'pos('$'),8)) else
                '0';
    IsHash   <= '1' when in_data=std_logic_vector(to_unsigned(character'pos('#'),8)) else
                '0';
    NextState_and_Mealy: process(state, DebugReset, in_valid, IsCtrlC, IsDollar, IsHash, GRRF_ready)
    begin
      in_ready   <= '0';
      EnReg      <= '0';
      GRRF_last  <= '0';
      GRRF_valid <= '0';
      next_state <= Error;
      if    DebugReset='1' then next_state <= WaitS;
      elsif DebugReset='0' then
        case state is
          when WaitS  => if     in_valid='0'                                   then next_state <= WaitS;  in_ready <= '1';
                         elsif (in_valid='1')and(IsDollar='0')and(IsCtrlC='0') then next_state <= WaitS;  in_ready <= '1';
                         elsif (in_valid='1')and(IsCtrlC='1')                  then next_state <= CtrlC;  in_ready <= '1'; EnReg <= '1';
                         elsif (in_valid='1')and(IsDollar='1')                 then next_state <= Dollar; in_ready <= '1';
                         end if;
          when CtrlC  => if    GRRF_ready='0' then next_state <= CtrlC; GRRF_valid <= '1'; GRRF_last <= '1';
                         elsif GRRF_ready='1' then next_state <= WaitS; GRRF_valid <= '1'; GRRF_last <= '1';
                         end if;
          when Dollar => if     in_valid='0'                 then next_state <= Dollar; in_ready <= '1';
                         elsif (in_valid='1')and(IsHash='1') then next_state <= WaitS;  in_ready <= '1';
                         elsif (in_valid='1')and(IsHash='0') then next_state <= Cmd;    in_ready <= '1'; EnReg <= '1';
                         end if;
          when Cmd    => if     in_valid='0'                                    then next_state <= Cmd;   in_ready   <= '1';
                         elsif (in_valid='1')and(GRRF_ready='0')and(IsHash='0') then next_state <= Cmd;   GRRF_valid <= '1';
                         elsif (in_valid='1')and(GRRF_ready='0')and(IsHash='1') then next_state <= Cmd;
                                                                                     GRRF_valid <= '1';   GRRF_last  <= '1';
                         elsif (in_valid='1')and(GRRF_ready='1')and(IsHash='0') then next_state <= Cmd;   in_ready   <= '1';
                                                                                     GRRF_valid <= '1';   EnReg      <= '1';
                         elsif (in_valid='1')and(GRRF_ready='1')and(IsHash='1') then next_state <= WaitS; in_ready   <= '1';
                                                                                     GRRF_valid <= '1';   GRRF_last  <= '1';
                         end if;
          when others => null;
        end case;
      end if;
    end process;
    State_Reg: process(Clk)
    begin
      if rising_edge(Clk) then
        state <= next_state;
      end if;
    end process;
    process (Clk)
    begin
      if rising_edge(Clk) then
        if DebugReset = '1' then
          rspC_valid <= '0';
          rspC_last  <= '0';
          rspC_data  <= (others=>'-');
        else
          if GRRF_ready = '1' then
            rspC_valid <= GRRF_valid;
            rspC_last  <= GRRF_last;       
            rspC_data  <= GRRF_data;
          end if;
        end if;
      end if;
    end process;
    GRRF_ready <= rspC_ready or (not rspC_valid);
  end block;
  GRAC: block
    signal IsDollar    : std_logic;
    signal IsHash      : std_logic;
    signal Add         : std_logic;
    signal Clr         : std_logic;
    signal Nibble      : std_logic;
    signal OMux        : integer range 0 to 2;
    signal Sum         : unsigned(7 downto 0);
    signal ASCII_digit : std_logic_vector(7 downto 0);
    type   TheStates is (WaitS, Bypass, SumH, SumL, Error);
    signal state      : TheStates := Error;
    signal next_state : TheStates;
  begin
    IsDollar <= '1' when rspR_data=std_logic_vector(to_unsigned(character'pos('$'),8)) else
                '0';
    IsHash   <= '1' when rspR_data=std_logic_vector(to_unsigned(character'pos('#'),8)) else
                '0';
    process(Clk)
      variable accu: unsigned(7 downto 0);
    begin
      if rising_edge(Clk) then
        if Clr='1'                then accu := x"00";
        elsif Clr='0' and Add='1' then accu := accu + unsigned(rspR_data);
        elsif Clr='0' and Add='0' then null;
        else                           accu := (others=>'X');
        end if;
        sum <= accu;
      end if;
    end process;
    MUX_HEX2ASCII: process(sum,Nibble)
      variable sum4: unsigned(3 downto 0);
    begin
      if    Nibble='0' then sum4 := sum(3 downto 0);
      elsif Nibble='1' then sum4 := sum(7 downto 4);
      else                  sum4 := "XXXX";
      end if;
      case sum4 is
        when x"0"|x"1"|x"2"|x"3"|x"4"|x"5"|x"6"|x"7"|x"8"|x"9" => ASCII_digit <= std_logic_vector(x"3"&sum4);
        when x"a"|x"b"|x"c"|x"d"|x"e"|x"f"                     => ASCII_digit <= std_logic_vector(("0000"&sum4)+(65-10));
        when others                                            => ASCII_digit <= x"66";
      end case;
    end process;
    out_data <= ASCII_digit   when OMux=1 else
                rspR_data     when OMux=2 else
                (others=>'X');
    NextState_and_Mealy: process(state, DebugReset, rspR_valid, IsDollar, IsHash, out_ready)
    begin
      rspR_ready <= '0';
      out_valid  <= '0';
      Clr        <= '0';
      Add        <= '0';
      next_state <= Error;
      if    DebugReset='1' then next_state <= WaitS;
      elsif DebugReset='0' then
        case state is
          when WaitS   => if     rspR_valid='0'                                     then next_state <= WaitS;  rspR_ready <= '1';
                          elsif (rspR_valid='1')and(IsDollar='1')and(out_ready='0') then next_state <= WaitS;  out_valid  <= '1';
                          elsif (rspR_valid='1')and(IsDollar='1')and(out_ready='1') then next_state <= Bypass; out_valid  <= '1';
                                                                                         rspR_ready <= '1';    Clr        <= '1';
                          elsif (rspR_valid='1')and(IsDollar='0')and(out_ready='1') then next_state <= WaitS;  out_valid  <= '1';
                                                                                         rspR_ready <= '1';
                          elsif (rspR_valid='1')and(IsDollar='0')and(out_ready='0') then next_state <= WaitS;  out_valid  <= '1';
                          end if;
          when Bypass  => if     rspR_valid='0'                                   then next_state <= Bypass; rspR_ready <= '1';
                          elsif (rspR_valid='1')and(out_ready='0')                then next_state <= Bypass; out_valid  <= '1';
                          elsif (rspR_valid='1')and(out_ready='1')and(IsHash='1') then next_state <= SumH;   out_valid  <= '1';
                                                                                       rspR_ready <= '1';
                          elsif (rspR_valid='1')and(out_ready='1')and(IsHash='0') then next_state <= Bypass; out_valid  <= '1';
                                                                                       rspR_ready <= '1';    Add        <= '1';
                          end if;
          when SumH    => if    out_ready='0' then next_state <= SumH; out_valid <= '1';
                          elsif out_ready='1' then next_state <= SumL; out_valid <= '1';
                          end if;
          when SumL    => if    out_ready='0' then next_state <= SumL;  out_valid <= '1';
                          elsif out_ready='1' then next_state <= WaitS; out_valid <= '1';
                          end if;
          when others  => null;
        end case;
      end if;
    end process;
    State_Reg: process(Clk)
    begin
      if rising_edge(Clk) then
        state <= next_state;
        case next_state is
          when WaitS   => OMux <= 2; Nibble <= '0'; out_last <= '0';
          when Bypass  => OMux <= 2; Nibble <= '0'; out_last <= '0';
          when SumH    => OMux <= 1; Nibble <= '1'; out_last <= '0';
          when SumL    => OMux <= 1; Nibble <= '0'; out_last <= '1';
          when others  => OMux <= 0; Nibble <= 'X'; out_last <= 'X';
        end case;
      end if;
    end process;
  end block;
  GRI: block
    type   GDB_RSP is (Cmd_c_lower, Cmd_d_lower, Cmd_D_upper, Cmd_g_lower, Cmd_m_lower, Cmd_M_upper, Cmd_P_upper, Cmd_q_lower,
                       Cmd_R_upper, Cmd_s_lower, Cmd_1, Cmd_2, Cmd_7,
                       Cmd_questionmark, Cmd_z_lower, Cmd_Z_upper, Cmd_ctrl_c, Cmd_Equal, Cmd_Comma, Cmd_Colon, Cmd_Exclm, Cmd_none );
    signal Cmd : GDB_RSP;
    type   rspR_MuxVals is (Digit, S, E, Plus, nul, eins, fuenf, O, K, Dollar, Hash, none);
    signal SelResp  : rspR_MuxVals;
    signal IsHex    : std_logic;
    signal ShlAddr  : std_logic;
    signal ClrAddr  : std_logic;
    signal IncAddr  : std_logic;
    signal AddrIs32 : std_logic;
    signal BAddr    : unsigned(1 downto 0);
    signal ClrLen   : std_logic;
    signal DecLen   : std_logic;
    signal ShlLen   : std_logic;
    signal LenIs0   : std_logic;
    signal ClrVal   : std_logic;
    signal ShrBVal  : std_logic;
    signal ShrWVal  : std_logic;
    signal LdResp   : std_logic;
    signal ShrResp  : std_logic;
    signal ClrCnt   : std_logic;
    signal EnCnt    : std_logic;
    signal CntIs1   : std_logic;
    signal CntIs7   : std_logic;
    signal hex_data    : std_logic_vector(3 downto 0);
    signal ASCII_digit : std_logic_vector(7 downto 0);
    signal R_nibble    : std_logic_vector(3 downto 0);
    type TheStates is (Halt, Running, ChkHalt, AdjustPC, HaltIt, GoHalt, QM_P,
                       SOKD_P, SOKD_0, SOKD_1, SOKD_2, SOKD_3,
                       RFER_F, RFER_P, RFER_0, RFER_1,
                       IfH_F, IfH_P, IfH_0, IfH_1,
                       SHW_1, SHW_2, SHW_3, SHW_4, SHW_5,
                       SES_F,
                       CHW_1, CHW_2, CHW_3, CHW_4, CHW_5,
                       SOK_P, SOK_0, SOK_1, SOK_2, SOK_3,
                       SResp_0, SResp_1, SResp_2, SResp_3, SResp_4,
                       ErrEx_F, ErrEx_P, ErrEx_0, ErrEx_1, ErrEx_2, ErrEx_3, ErrEx_4,
                       M_q, M_R, M_c, M_m, M_d, M_Com, M_7, M_2,
                       c_Resum, c_P,
                       s_Step, s_P,
                       g_P, g_Dollar, g_RdReg, g_Resp, g_RdPC, g_ResPC, g_Hash,
                       P_GetA, P_GetV, P_WrReg, P_WrPC,
                       mR_GetA, mR_GetL, mR_P, mR_Dollar, mR_RdB, mR_RspB, mR_Hash,
                       MW_GetA, MW_GetL, MW_GB, MW_WB,
                       Error
                      );
    signal state      : TheStates := Halt;
    signal next_state : TheStates;
  begin
    ASCII2HEX: process(rspC_data)
    begin
      case rspC_data is
        when x"30"|x"31"|x"32"|x"33"|x"34"|x"35"|x"36"|x"37"|x"38"|x"39" => hex_data<=rspC_data(3 downto 0); IsHex<='1';
        when x"41"|x"61" => hex_data<=x"a"; IsHex<='1';
        when x"42"|x"62" => hex_data<=x"b"; IsHex<='1';
        when x"43"|x"63" => hex_data<=x"c"; IsHex<='1';
        when x"44"|x"64" => hex_data<=x"d"; IsHex<='1';
        when x"45"|x"65" => hex_data<=x"e"; IsHex<='1';
        when x"46"|x"66" => hex_data<=x"f"; IsHex<='1';
        when others => hex_data<="0000"; IsHex<='0';
      end case;
    end process;
    Decode: process(rspC_data)
    begin
      case character'val(to_integer(unsigned(rspC_data))) is
        when 'c' =>    Cmd <= Cmd_c_lower;
        when 'd' =>    Cmd <= Cmd_d_lower;
        when 'D' =>    Cmd <= Cmd_D_upper;
        when 'g' =>    Cmd <= Cmd_g_lower;
        when 'm' =>    Cmd <= Cmd_m_lower;
        when 'M' =>    Cmd <= Cmd_M_upper;
        when 'P' =>    Cmd <= Cmd_P_upper;
        when 's' =>    Cmd <= Cmd_s_lower;
        when 'q' =>    Cmd <= Cmd_q_lower;
        when 'R' =>    Cmd <= Cmd_R_upper;
        when 'z' =>    Cmd <= Cmd_z_lower;
        when 'Z' =>    Cmd <= Cmd_Z_upper;
        when '1' =>    Cmd <= Cmd_1;
        when '2' =>    Cmd <= Cmd_2;
        when '7' =>    Cmd <= Cmd_7;
        when '=' =>    Cmd <= Cmd_Equal;
        when ',' =>    Cmd <= Cmd_Comma;
        when ':' =>    Cmd <= Cmd_Colon;
        when '!' =>    Cmd <= Cmd_Exclm;
        when '?' =>    Cmd <= Cmd_questionmark;
        when ETX =>    Cmd <= Cmd_ctrl_c;
        when others => Cmd <= Cmd_none;
      end case;
    end process;
    HEX2ASCII: process(R_nibble)
    begin
      case R_nibble is
        when x"0"|x"1"|x"2"|x"3"|x"4"|x"5"|x"6"|x"7"|x"8"|x"9" => ASCII_digit <= x"3"&R_nibble;
        when x"a"|x"b"|x"c"|x"d"|x"e"|x"f" => ASCII_digit <= std_logic_vector(unsigned("0000"&R_nibble)+(65-10));
        when others => ASCII_digit <= x"66";
      end case;
    end process;
    rspR_data_Mux: process(SelResp,ASCII_digit)
    begin
      case SelResp is
        when Digit  => rspR_data <= ASCII_digit;
        when S      => rspR_data <= std_logic_vector(to_unsigned(character'pos('S'),8));
        when E      => rspR_data <= std_logic_vector(to_unsigned(character'pos('E'),8));
        when Plus   => rspR_data <= std_logic_vector(to_unsigned(character'pos('+'),8));
        when nul    => rspR_data <= std_logic_vector(to_unsigned(character'pos('0'),8));
        when eins   => rspR_data <= std_logic_vector(to_unsigned(character'pos('1'),8));
        when fuenf  => rspR_data <= std_logic_vector(to_unsigned(character'pos('5'),8));
        when O      => rspR_data <= std_logic_vector(to_unsigned(character'pos('O'),8));
        when K      => rspR_data <= std_logic_vector(to_unsigned(character'pos('K'),8));
        when Dollar => rspR_data <= std_logic_vector(to_unsigned(character'pos('$'),8));
        when Hash   => rspR_data <= std_logic_vector(to_unsigned(character'pos('#'),8));
        when others => rspR_data <= std_logic_vector(to_unsigned(character'pos('X'),8));
      end case;
    end process;
    Addr: process(Clk)
      variable AddR_Data_XX : unsigned(31 downto 0); -- AddR_Data
    begin
      if rising_edge(Clk) then
        if ClrAddr='1' then
          AddR_Data_XX := x"00000000";
        elsif IncAddr='1' then
          AddR_Data_XX := AddR_Data_XX+1;
        elsif ShlAddr='1' then
          AddR_Data_XX := AddR_Data_XX(27 downto 0) & unsigned(hex_data);
        end if;
        if AddR_Data_XX=32 then AddrIs32 <= '1'; else AddrIs32 <= '0'; end if;
        A_Addr <= std_logic_vector(AddR_Data_XX);
        BAddr     <= AddR_Data_XX(1 downto 0);
      end if;
    end process;
    Len: process(Clk)
      variable Len_value : unsigned(31 downto 0);
    begin
      if rising_edge(Clk) then
        if    ClrLen ='1' then
          Len_value := x"00000000";
        elsif DecLen ='1' then
          Len_value := Len_value -1;
        elsif ShlLen ='1' then
          Len_value := Len_value(27 downto 0) & unsigned(hex_data);
        end if;
        if Len_value=0 then LenIs0<='1'; else LenIs0<='0'; end if;
      end if;
    end process;
    Val: process(Clk)
      variable XValue : std_logic_vector(31 downto 0);
    begin
      if rising_edge(Clk) then
        if ClrVal ='1' then
          XValue := x"00000000";
        elsif ShrBVal='1' then
          XValue := x"000000" & hex_data & XValue(7 downto 4);
        elsif ShrWVal='1' then
          XValue := hex_data & XValue(31 downto 4);
        end if;
        A_Value <= XValue(27 downto 24)&XValue(31 downto 28) &
                   XValue(19 downto 16)&XValue(23 downto 20) &
                   XValue(11 downto  8)&XValue(15 downto 12) &
                   XValue( 3 downto  0)&XValue( 7 downto  4);
      end if;
    end process;
    Resp: process(Clk)
      variable XValue : std_logic_vector(31 downto 0);
    begin
      if rising_edge(Clk) then
        if DebugReset='1' then
          XValue := x"00000000";
        elsif LdResp ='1' then
          XValue := R_Data(27 downto 24)&R_Data(31 downto 28) &
                    R_Data(19 downto 16)&R_Data(23 downto 20) &
                    R_Data(11 downto  8)&R_Data(15 downto 12) &
                    R_Data( 3 downto  0)&R_Data( 7 downto  4);
        elsif ShrResp='1' then
          XValue := "0000" & XValue(31 downto 4);
        end if;
        R_nibble <= XValue( 3 downto  0);
      end if;
    end process;
    Cnt: process(Clk)
      variable value : unsigned(2 downto 0);
    begin
      if rising_edge(Clk) then
        if    ClrCnt='1' then value := "000";
        elsif EnCnt ='1' then value := value+1;
        end if;
        if value=1 then CntIs1<='1'; else CntIs1<='0'; end if;
        if value=7 then CntIs7<='1'; else CntIs7<='0'; end if;
      end if;
    end process;
    NextState_and_Mealy: process(state, DebugReset, rspC_valid, rspC_last, Cmd, IsHex,
                                 rspR_ready, R_OK, A_ready, AddrIs32, LenIs0,
                                 CntIs1, CntIs7, BAddr)
    begin
      rspC_ready <= '0';
      ClrAddr    <= '0';
      IncAddr    <= '0';
      ShlAddr    <= '0';
      ShlLen     <= '0';
      ClrLen     <= '0';
      DecLen     <= '0';
      ClrVal     <= '0';
      ShrWVal    <= '0';
      ShrBVal    <= '0';
      ClrCnt     <= '0';
      EnCnt      <= '0';
      LdResp     <= '0';
      ShrResp    <= '0';
      next_state <= Error;
      if    DebugReset='1' then next_state <= Running;
      elsif DebugReset='0' then
        case state is
          when Halt => if    rspC_valid='0' then next_state <= Halt;
                       elsif rspC_valid='1' then
                         case Cmd is
                           when Cmd_c_lower      => if rspC_last='1' then next_state<=c_Resum;               rspC_ready <= '1'; end if;
                           when Cmd_D_upper      => if rspC_last='1' then next_state<=SOKD_P;                rspC_ready <= '1'; end if;
                           when Cmd_g_lower      => if rspC_last='1' then next_state<=g_P;                   rspC_ready <= '1'; end if;
                           when Cmd_m_lower      => if rspC_last='0' then next_state<=mR_GetA; ClrAddr<='1'; rspC_ready <= '1'; end if;
                           when Cmd_M_upper      => if rspC_last='0' then next_state<=MW_GetA; ClrAddr<='1'; rspC_ready <= '1'; end if;
                           when Cmd_P_upper      => if rspC_last='0' then next_state<=P_GetA;  ClrAddr<='1'; rspC_ready <= '1'; end if;
                           when Cmd_q_lower      => if rspC_last='0' then next_state<=M_q;                   rspC_ready <= '1'; end if;
                           when Cmd_s_lower      => if rspC_last='1' then next_state<=s_Step;                rspC_ready <= '1'; end if;
                           when Cmd_questionmark => if rspC_last='1' then next_state<=QM_P;                  rspC_ready <= '1'; end if;
                           when Cmd_Exclm        => if rspC_last='1' then next_state<=SOK_P;                 rspC_ready <= '1'; end if;
                           when Cmd_Z_upper      => if rspC_last='0' then next_state<=SHW_1;                 rspC_ready <= '1'; end if;
                           when Cmd_z_lower      => if rspC_last='0' then next_state<=CHW_1;                 rspC_ready <= '1'; end if;
                           when others           =>                       next_state<=IfH_F;                 rspC_ready <= '0';
                         end case;
                       end if;
          when Running => if    rspC_valid='0'                                       then next_state <= ChkHalt;
                          elsif rspC_valid='1' and Cmd=Cmd_ctrl_c                    then next_state <= HaltIt; rspC_ready<='1';
                          elsif rspC_valid='1' and Cmd=Cmd_Exclm and rspC_last='1'   then next_state <= GoHalt;
                          elsif rspC_valid='1'                                       then next_state <= RFER_F;
                          end if;
          when SOKD_P  => if rspR_ready='0' then next_state<=SOKD_P; elsif rspR_ready='1' then next_state<=SOKD_0;  end if;
          when SOKD_0  => if rspR_ready='0' then next_state<=SOKD_0; elsif rspR_ready='1' then next_state<=SOKD_1;  end if;
          when SOKD_1  => if rspR_ready='0' then next_state<=SOKD_1; elsif rspR_ready='1' then next_state<=SOKD_2;  end if;
          when SOKD_2  => if rspR_ready='0' then next_state<=SOKD_2; elsif rspR_ready='1' then next_state<=SOKD_3;  end if;
          when SOKD_3  => if rspR_ready='0' then next_state<=SOKD_3; elsif rspR_ready='1' then next_state<=c_Resum; end if;
          when ChkHalt => if    A_ready='0'                  then next_state <= ChkHalt;
                          elsif A_ready='1' and R_OK='0' then next_state <= Running;
                          elsif A_ready='1' and R_OK='1' then next_state <= AdjustPC;
                          end if;
          when AdjustPC=> if    A_ready='0'              then next_state <= AdjustPC;
                          elsif A_ready='1' and R_OK='1' then next_state <= SResp_0;
                          elsif A_ready='1' and R_OK='0' then next_state <= ErrEx_0;
                          end if;
          when HaltIt  => if    A_ready='0' then next_state <= HaltIt;
                          elsif A_ready='1' then next_state <= SResp_0;
                          end if;
          when GoHalt  => if A_ready='0'    then next_state<=GoHalt; elsif A_ready='1'    then next_state<=Halt;     end if;
          when QM_P    => if rspR_ready='0' then next_state<=QM_P;   elsif rspR_ready='1' then next_state<=SResp_0;  end if;
          when RFER_F  => if    rspC_valid='0'                   then next_state<=RFER_F;
                          elsif rspC_valid='1' and rspC_last='0' then next_state<=RFER_F; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='1' then next_state<=RFER_P; rspC_ready<='1';
                          end if;
          when RFER_P  => if rspR_ready='0' then next_state<=RFER_P; elsif rspR_ready='1' then next_state<=RFER_0;   end if;
          when RFER_0  => if rspR_ready='0' then next_state<=RFER_0; elsif rspR_ready='1' then next_state<=RFER_1;   end if;
          when RFER_1  => if rspR_ready='0' then next_state<=RFER_1; elsif rspR_ready='1' then next_state<=Running; end if;
          when IfH_F   => if    rspC_valid='0'                   then next_state<=IfH_F;
                          elsif rspC_valid='1' and rspC_last='0' then next_state<=IfH_F; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='1' then next_state<=IfH_P; rspC_ready<='1';
                          end if;
          when IfH_P   => if rspR_ready='0' then next_state<=IfH_P;   elsif rspR_ready='1' then next_state<=IfH_0;   end if;
          when IfH_0   => if rspR_ready='0' then next_state<=IfH_0;   elsif rspR_ready='1' then next_state<=IfH_1;   end if;
          when IfH_1   => if rspR_ready='0' then next_state<=IfH_1;   elsif rspR_ready='1' then next_state<=Halt;    end if;
          when SHW_1   => if    rspC_valid='0'                                   then next_state<=SHW_1;
                          elsif rspC_valid='1' and (rspC_last='0' and Cmd=Cmd_1) then next_state<=SHW_2; rspC_ready<='1';
                          elsif rspC_valid='1' and (rspC_last='1' or Cmd/=Cmd_1) then next_state<=SES_F; ClrVal<='1';
                          end if;
          when SHW_2   => if    rspC_valid='0'                                       then next_state<=SHW_2;
                          elsif rspC_valid='1' and (rspC_last='0' and Cmd=Cmd_Comma) then next_state<=SHW_3; rspC_ready<='1'; ClrAddr<='1';
                          elsif rspC_valid='1'                                       then next_state<=ErrEx_F;
                          end if;
          when SHW_3   => if    rspC_valid='0'                                       then next_state<=SHW_3;
                          elsif rspC_valid='1' and (rspC_last='0' and IsHex='1')     then next_state<=SHW_3; rspC_ready<='1'; ShlAddr<='1';
                          elsif rspC_valid='1' and (rspC_last='0' and Cmd=Cmd_Comma) then next_state<=SHW_4; rspC_ready<='1'; ClrVal<='1';
                          elsif rspC_valid='1'                                       then next_state<=ErrEx_F;
                          end if;
          when SHW_4   => if    rspC_valid='0'                   then next_state<=SHW_4;
                          elsif rspC_valid='1' and rspC_last='1' then next_state<=SHW_5; rspC_ready<='1'; ShrBVal<='1';
                          elsif rspC_valid='1' and rspC_last='0' then next_state<=ErrEx_F;
                          end if;
          when SHW_5   => if    A_ready='0' then next_state<=SHW_5;
                          elsif A_ready='1' and R_OK='1' then next_state<=SOK_P;
                          elsif A_ready='1' and R_OK='0' then next_state<=ErrEx_P;
                          end if;
          when SES_F   => if    rspC_valid='0'                   then next_state<=SES_F;
                          elsif rspC_valid='1' and rspC_last='0' then next_state<=SES_F;  rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='1' then next_state<=IfH_P;  rspC_ready<='1'; ShrBVal<='1';
                          end if;
          when CHW_1   => if    rspC_valid='0'                                   then next_state<=CHW_1;
                          elsif rspC_valid='1' and (rspC_last='0' and Cmd=Cmd_1) then next_state<=CHW_2; rspC_ready<='1';
                          elsif rspC_valid='1' and (rspC_last='1' or Cmd/=Cmd_1) then next_state<=IfH_P; ClrVal<='1';
                          end if;
          when CHW_2   => if    rspC_valid='0'                                       then next_state<=CHW_2;
                          elsif rspC_valid='1' and (rspC_last='0' and Cmd=Cmd_Comma) then next_state<=CHW_3; rspC_ready<='1'; ClrAddr<='1';
                          elsif rspC_valid='1'                                       then next_state<=ErrEx_F;
                          end if;
          when CHW_3   => if    rspC_valid='0'                                       then next_state<=CHW_3;
                          elsif rspC_valid='1' and (rspC_last='0' and IsHex='1')     then next_state<=CHW_3; rspC_ready<='1'; ShlAddr<='1';
                          elsif rspC_valid='1' and (rspC_last='0' and Cmd=Cmd_Comma) then next_state<=CHW_4; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=ErrEx_F;
                          end if;
          when CHW_4   => if    rspC_valid='0'                   then next_state<=CHW_4;
                          elsif rspC_valid='1' and rspC_last='1' then next_state<=CHW_5; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='0' then next_state<=ErrEx_F;
                          end if;
          when CHW_5   => if A_ready='0' then next_state<=CHW_5; elsif A_ready='1' then next_state<=SOK_P; end if;
          when SOK_P   => if rspR_ready='0' then next_state<=SOK_P;   elsif rspR_ready='1' then next_state<=SOK_0;   end if;
          when SOK_0   => if rspR_ready='0' then next_state<=SOK_0;   elsif rspR_ready='1' then next_state<=SOK_1;   end if;
          when SOK_1   => if rspR_ready='0' then next_state<=SOK_1;   elsif rspR_ready='1' then next_state<=SOK_2;   end if;
          when SOK_2   => if rspR_ready='0' then next_state<=SOK_2;   elsif rspR_ready='1' then next_state<=SOK_3;   end if;
          when SOK_3   => if rspR_ready='0' then next_state<=SOK_3;   elsif rspR_ready='1' then next_state<=Halt;    end if;
          when SResp_0 => if rspR_ready='0' then next_state<=SResp_0; elsif rspR_ready='1' then next_state<=SResp_1; end if;
          when SResp_1 => if rspR_ready='0' then next_state<=SResp_1; elsif rspR_ready='1' then next_state<=SResp_2; end if;
          when SResp_2 => if rspR_ready='0' then next_state<=SResp_2; elsif rspR_ready='1' then next_state<=SResp_3; end if;
          when SResp_3 => if rspR_ready='0' then next_state<=SResp_3; elsif rspR_ready='1' then next_state<=SResp_4; end if;
          when SResp_4 => if rspR_ready='0' then next_state<=SResp_4; elsif rspR_ready='1' then next_state<=Halt;    end if;
          when ErrEx_F => if    rspC_valid='0'                   then next_state<=ErrEx_F;
                          elsif rspC_valid='1' and rspC_last='0' then next_state<=ErrEx_F; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='1' then next_state<=ErrEx_P; rspC_ready<='1';
                          end if;
          when ErrEx_P => if rspR_ready='0' then next_state<=ErrEx_P; elsif rspR_ready='1' then next_state<=ErrEx_0; end if;
          when ErrEx_0 => if rspR_ready='0' then next_state<=ErrEx_0; elsif rspR_ready='1' then next_state<=ErrEx_1; end if;
          when ErrEx_1 => if rspR_ready='0' then next_state<=ErrEx_1; elsif rspR_ready='1' then next_state<=ErrEx_2; end if;
          when ErrEx_2 => if rspR_ready='0' then next_state<=ErrEx_2; elsif rspR_ready='1' then next_state<=ErrEx_3; end if;
          when ErrEx_3 => if rspR_ready='0' then next_state<=ErrEx_3; elsif rspR_ready='1' then next_state<=ErrEx_4; end if;
          when ErrEx_4 => if rspR_ready='0' then next_state<=ErrEx_4; elsif rspR_ready='1' then next_state<=Halt;    end if;
          when M_q     => if    rspC_valid='0'                                       then next_state<=M_q;
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_R_upper then next_state<=M_R; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=IfH_F;
                          end if;
          when M_R     => if    rspC_valid='0'                                       then next_state<=M_R;
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_c_lower then next_state<=M_c; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=IfH_F;
                          end if;
          when M_c     => if    rspC_valid='0'                                       then next_state<=M_c;
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_m_lower then next_state<=M_m; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=IfH_F;
                          end if;
          when M_m     => if    rspC_valid='0'                                       then next_state<=M_m;
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_d_lower then next_state<=M_d; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=IfH_F;
                          end if;
          when M_d     => if    rspC_valid='0'                                       then next_state<=M_d;
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_Comma   then next_state<=M_Com; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=IfH_F;
                          end if;
          when M_Com   => if    rspC_valid='0'                                       then next_state<=M_Com;
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_7       then next_state<=M_7; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=IfH_F;
                          end if;
          when M_7     => if    rspC_valid='0'                                       then next_state<=M_7;
                          elsif rspC_valid='1' and rspC_last='1' and Cmd=Cmd_2       then next_state<=M_2; rspC_ready<='1';
                          elsif rspC_valid='1'                                       then next_state<=IfH_F;
                          end if;
          when M_2     => if A_ready='0' then next_state<=M_2; elsif A_ready='1' then next_state<=SOK_P; end if;
          when c_Resum => if A_ready='0'    then next_state<=c_Resum; elsif A_ready='1'    then next_state<=c_P;     end if;
          when c_P     => if rspR_ready='0' then next_state<=c_P;     elsif rspR_ready='1' then next_state<=Running; end if;
          when s_Step  => if A_ready='0'    then next_state<=s_Step;  elsif A_ready='1'    then next_state<=s_P;     end if;
          when s_P     => if rspR_ready='0' then next_state<=s_P;     elsif rspR_ready='1' then next_state<=SResp_0; end if;
          when g_P      => if rspR_ready='0' then next_state<=g_P;      elsif rspR_ready='1' then next_state<=g_Dollar;
                                                                                                  ClrAddr<='1';         end if;
          when g_Dollar => if rspR_ready='0' then next_state<=g_Dollar; elsif rspR_ready='1' then next_state<=g_RdReg;  end if;
          when g_RdReg  => if    A_ready='0' then next_state<=g_RdReg;
                           elsif A_ready='1' then next_state<=g_Resp; LdResp<='1'; ClrCnt<='1'; IncAddr<='1';
                           end if;
          when g_Resp   => if    rspR_ready='0'                                 then next_state<=g_Resp;
                           elsif rspR_ready='1' and CntIs7='0'                  then next_state<=g_Resp; EnCnt<='1'; ShrResp<='1';
                           elsif rspR_ready='1' and CntIs7='1' and AddrIs32='0' then next_state<=g_RdReg;
                           elsif rspR_ready='1' and CntIs7='1' and AddrIs32='1' then next_state<=g_RdPC;
                           end if;
          when g_RdPC   => if    A_ready='0' then next_state<=g_RdPC;
                           elsif A_ready='1' then next_state<=g_ResPC; LdResp<='1'; ClrCnt<='1';
                           end if;
          when g_ResPC  => if    rspR_ready='0'                then next_state<=g_ResPC;
                           elsif rspR_ready='1' and CntIs7='0' then next_state<=g_ResPC; EnCnt<='1'; ShrResp<='1';
                           elsif rspR_ready='1' and CntIs7='1' then next_state<=g_Hash;
                           end if;
          when g_Hash   => if    rspR_ready='0' then next_state<=g_Hash;
                           elsif rspR_ready='1' then next_state<=Halt;
                           end if;
          when P_GetA  => if    rspC_valid='0'                                     then next_state<=P_GetA;
                          elsif rspC_valid='1' and rspC_last='0' and IsHex='1'     then next_state<=P_GetA; ShlAddr<='1'; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_Equal then next_state<=P_GetV; ClrVal<='1';  rspC_ready<='1';
                          elsif rspC_valid='1'                                     then next_state<=ErrEx_F;
                          end if;
          when P_GetV  => if    rspC_valid='0'                                 then next_state<=P_GetV;
                          elsif rspC_valid='1' and rspC_last='0' and IsHex='1' then next_state<=P_GetV;  ShrWVal<='1'; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='1' and IsHex='1' then
                            if    AddrIs32='0' then next_state<=P_WrReg; ShrWVal<='1'; rspC_ready<='1';
                            elsif AddrIs32='1' then next_state<=P_WrPC;  ShrWVal<='1'; rspC_ready<='1';
                            else                    next_state<=ErrEx_F;
                            end if;
                          elsif rspC_valid='1' then next_state<=ErrEx_F;
                          end if;
          when P_WrReg => if    A_ready='0' then next_state<=P_WrReg;
                          elsif A_ready='1' then next_state<=SOK_P;
                          end if;
          when P_WrPC  => if    A_ready='0' then next_state<=P_WrPC;
                          elsif A_ready='1' then next_state<=SOK_P;
                          end if;
          when mR_GetA   => if    rspC_valid='0'                                     then next_state<=mR_GetA;
                            elsif rspC_valid='1' and rspC_last='0' and IsHex='1'     then next_state<=mR_GetA; ShlAddr<='1'; rspC_ready<='1';
                            elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_Comma then next_state<=mR_GetL; ClrLen<='1';  rspC_ready<='1';
                            elsif rspC_valid='1'                                     then next_state<=ErrEx_F;
                            end if;
          when mR_GetL   => if    rspC_valid='0'                                 then next_state<=mR_GetL;
                            elsif rspC_valid='1' and rspC_last='0' and IsHex='1' then next_state<=mR_GetL; ShlLen<='1'; rspC_ready<='1';
                            elsif rspC_valid='1' and rspC_last='1' and IsHex='1' then next_state<=mR_P;    ShlLen<='1'; rspC_ready<='1';
                            elsif rspC_valid='1'                                 then next_state<=ErrEx_F;
                            end if;
          when mR_P      => if    rspR_ready='0' then next_state<=mR_P;     elsif rspR_ready='1' then next_state<=mR_Dollar;  end if;
          when mR_Dollar => if    rspR_ready='0' then next_state<=mR_Dollar;
                            elsif rspR_ready='1' then
                              if    LenIs0='0' then next_state<=mR_RdB;
                              elsif LenIs0='1' then next_state<=mR_Hash;
                              end if;
                            end if;
          when mR_RdB    => if    A_ready='0' then next_state<=mR_RdB;
                            elsif A_ready='1' then next_state<=mR_RspB; IncAddr<='1';  DecLen<='1'; LdResp<='1'; ClrCnt<='1';
                            end if;
          when mR_RspB   => if    rspR_ready='0'                then next_state<=mR_RspB;
                            elsif rspR_ready='1' and CntIs1='0' then next_state<=mR_RspB; EnCnt<='1';ShrResp <= '1';
                            elsif rspR_ready='1' and CntIs1='1' then
                              if    LenIs0='0' then next_state<=mR_RdB;
                              elsif LenIs0='1' then next_state<=mR_Hash;
                              end if;
                            end if;
          when mR_Hash   => if    rspR_ready='0' then next_state<=mR_Hash;
                            elsif rspR_ready='1' then next_state<=Halt;
                            end if;
          when MW_GetA => if    rspC_valid='0'                                     then next_state<=MW_GetA;
                          elsif rspC_valid='1' and rspC_last='0' and IsHex='1'     then next_state<=MW_GetA; ShlAddr<='1'; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_Comma then next_state<=MW_GetL; ClrLen<='1';  rspC_ready<='1';
                          elsif rspC_valid='1'                                     then next_state<=ErrEx_F;
                          end if;
          when MW_GetL => if    rspC_valid='0'                                     then next_state<=MW_GetL;
                          elsif rspC_valid='1' and rspC_last='0' and IsHex='1'     then next_state<=MW_GetL;  ShlLen<='1'; rspC_ready<='1';
                          elsif rspC_valid='1' and rspC_last='0' and Cmd=Cmd_Colon then
                            rspC_ready<='1';
                              if    LenIs0='0' then next_state<=MW_GB; ClrCnt<='1';
                              elsif LenIs0='1' then next_state<=SOK_P;
                              end if;
                          elsif rspC_valid='1'                                     then next_state<=ErrEx_F;
                          end if;
          when MW_GB   => if    rspC_valid='0'                              then next_state<=MW_GB;
                          elsif rspC_valid='1' and CntIs1='0' and IsHex='1' then next_state<=MW_GB; rspC_ready<='1'; ShrBVal<='1'; EnCnt<='1';
                          elsif rspC_valid='1' and CntIs1='1' and IsHex='1' then next_state<=MW_WB; rspC_ready<='1'; ShrBVal<='1'; DecLen<='1';
                          elsif IsHex='0'                                   then next_state<=ErrEx_F;
                          end if;
          when MW_WB   => if    A_ready='0' then next_state<=MW_WB;
                          elsif A_ready='1' then
                            IncAddr<='1';
                            if    LenIs0='0' then next_state<=MW_GB; ClrCnt<='1';
                            elsif LenIs0='1' then next_state<=SOK_P;
                            end if;
                          end if;
          when Error   => null;
          when others  => null;
        end case;
      end if;
    end process;
    State_and_Moore: process(Clk)
    begin
      if rising_edge(Clk) then
        state <= next_state;
        rspR_valid <= '0';
        SelResp    <= none;
        A_valid    <= '0';
        A_Action   <= none;
        CPU_Halted <= '0';
        case next_state is
          when Running   => CPU_Halted <= '0';
          when SOKD_P    => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when SOKD_0    => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Dollar;
          when SOKD_1    => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=O;
          when SOKD_2    => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=K;
          when SOKD_3    => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Hash;
          when ChkHalt   => CPU_Halted <= '0'; A_valid<='1';    A_Action<=CheckHalted;
          when AdjustPC  => CPU_Halted <= '0'; A_valid<='1';    A_Action<=AdjustPC;
          when HaltIt    => CPU_Halted <= '0'; A_valid<='1';    A_Action<= Halt;
          when GoHalt    => CPU_Halted <= '0'; A_valid<='1';    A_Action<= Halt;
          when Halt      => CPU_Halted <= '1';
          when QM_P      => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when RFER_F    => CPU_Halted <= '0';
          when RFER_P    => CPU_Halted <= '0'; rspR_valid<='1'; SelResp<=Plus;
          when RFER_0    => CPU_Halted <= '0'; rspR_valid<='1'; SelResp<=Dollar;
          when RFER_1    => CPU_Halted <= '0'; rspR_valid<='1'; SelResp<=Hash;
          when IfH_F     => CPU_Halted <= '1';
          when IfH_P     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when IfH_0     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Dollar;
          when IfH_1     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Hash;
          when SHW_1     => CPU_Halted <= '1';
          when SHW_2     => CPU_Halted <= '1';
          when SHW_3     => CPU_Halted <= '1';
          when SHW_4     => CPU_Halted <= '1';
          when SHW_5     => CPU_Halted <= '1'; A_valid<='1';    A_Action<= SetHwBp;
          when SES_F     => CPU_Halted <= '1';
          when CHW_1     => CPU_Halted <= '1';
          when CHW_2     => CPU_Halted <= '1';
          when CHW_3     => CPU_Halted <= '1';
          when CHW_4     => CPU_Halted <= '1';
          when CHW_5     => CPU_Halted <= '1'; A_valid<='1';    A_Action<= ClrHwBp;
          when SOK_P     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when SOK_0     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Dollar;
          when SOK_1     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=O;
          when SOK_2     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=K;
          when SOK_3     => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Hash;
          when SResp_0   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Dollar;
          when SResp_1   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=S;
          when SResp_2   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=nul;
          when SResp_3   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=fuenf;
          when SResp_4   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Hash;
          when ErrEx_F   => CPU_Halted <= '1';
          when ErrEx_P   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when ErrEx_0   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Dollar;
          when ErrEx_1   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=E;
          when ErrEx_2   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=nul;
          when ErrEx_3   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=eins;
          when ErrEx_4   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Hash;
          when M_q       => CPU_Halted <= '1';
          when M_R       => CPU_Halted <= '1';
          when M_c       => CPU_Halted <= '1';
          when M_m       => CPU_Halted <= '1';
          when M_d       => CPU_Halted <= '1';
          when M_Com     => CPU_Halted <= '1';
          when M_7       => CPU_Halted <= '1';
          when M_2       => CPU_Halted <= '1'; A_valid<='1';    A_Action<=DoReset;
          when c_Resum   => CPU_Halted <= '1'; A_valid<='1';    A_Action<=Resume;
          when c_P       => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when s_Step    => CPU_Halted <= '1'; A_valid<='1';    A_Action<=Step;
          when s_P       => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when g_P       => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when g_Dollar  => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Dollar;
          when g_RdReg   => CPU_Halted <= '1'; A_valid<='1';    A_Action<=ReadReg;
          when g_Resp    => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Digit;
          when g_RdPC    => CPU_Halted <= '1'; A_valid<='1';    A_Action<=ReadPC;
          when g_ResPC   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Digit;
          when g_Hash    => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Hash;
          when P_GetA    => CPU_Halted <= '1';
          when P_GetV    => CPU_Halted <= '1';
          when P_WrReg   => CPU_Halted <= '1'; A_valid<='1';    A_Action<=WriteReg;
          when P_WrPC    => CPU_Halted <= '1'; A_valid<='1';    A_Action<=WritePC;
          when mR_GetA   => CPU_Halted <= '1';
          when mR_GetL   => CPU_Halted <= '1';
          when mR_P      => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Plus;
          when mR_Dollar => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Dollar;
          when mR_RdB    => CPU_Halted <= '1'; A_valid<='1';    A_Action<=ReadByte;
          when mR_RspB   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Digit;
          when mR_Hash   => CPU_Halted <= '1'; rspR_valid<='1'; SelResp<=Hash;
          when MW_GetA   => CPU_Halted <= '1';
          when MW_GetL   => CPU_Halted <= '1';
          when MW_GB     => CPU_Halted <= '1';
          when MW_WB     => CPU_Halted <= '1'; A_valid<='1';    A_Action<=WriteByte;
          when Error     => null;
        end case;
      end if;
    end process;
  end block;
  VDI: block
    type SelVal_Type  is (A,V,rsp,X1,X2);
    type SelReg_Type  is (X1,X2,A);
    type SelAddr_Type is (Inject,Cmd,HwBk);
    type SelCmd_Type  is (HALT,Resume,Step0,Step1,HwBkpt,ClrBkpt,ReadReg,WriteRegH,WriteRegL,ReadPC,WritePC,
                          WriteByte,ReadByte,clrRESET,setRESET,PC2X1,DecX1_by2,IncX1_by2,RdHalf);
    signal SelVal    : SelVal_Type;
    signal SelReg    : SelReg_Type;
    signal SelAddr   : SelAddr_Type;
    signal SelCmd    : SelCmd_Type;
    signal EnData    : std_logic;
    signal EnX1      : std_logic;
    signal EnX2      : std_logic;
    signal SelCnt    : std_logic;
    signal LdCnt     : std_logic;
    signal EnCnt     : std_logic;
    signal CntTC     : std_logic;
    signal BkptFound : std_logic;
    signal EmptyBkpt : std_logic;
    signal HData     : std_logic_vector(15 downto 0);
    signal Value       : std_logic_vector(31 downto 0);
    signal X1_Reg      : std_logic_vector(31 downto 0);
    signal X2_Reg      : std_logic_vector(31 downto 0);
    signal R_Data_reg  : std_logic_vector(31 downto 0);
    signal regno       : std_logic_vector( 4 downto 0);
    signal Bkpt_Addr   : unsigned(7 downto 0);
    constant isPipBusy     : integer := 2;
    constant resetIt       : integer := 0;
    constant HaltIt        : integer := 1;
    constant haltedByBreak : integer := 3;
    type TheStates is (WaitAction,
                       Halt_req, Halt_Chk,
                       Res_req, Res_Chk,
                       Step_req, Step_Chk, Step_PC, Step_done,
                       RR_ij, RR_get, RR_done,
                       WR_ij0, WR_ij1,
                       RPC_ij, RPC_get, RPC_done,
                       WPC_Sx2a, WPC_Sx2b, WPC_ij0, WPC_ij1, WPC_ij2, WPC_Rx2a, WPC_Rx2b,
                       RB_Sx1a, RB_Sx1b, RB_Ax1a, RB_Ax1b, RB_ij, RB_get, RB_Rx1a, RB_Rx1b,
                       WB_Sx1a, WB_Sx1b, WB_Ax1a, WB_Ax1b, WB_Sx2a, WB_Sx2b, WB_Vx2a, WB_Vx2b,
                       WB_do, WB_Rx2a, WB_Rx2b, WB_Rx1a, WB_Rx1b,
                       ChkHalt,
                       RS_set, RS_s_done, RS_clr, RS_c_done,
                       Adj1_Sx1a,Adj1_Sx1b,Adj2_PCx1,Adj2_W,Adj3_D2x1,Adj3_W,Adj3_SB0,Adj3_SB1,
                       Adj4_D2x1,Adj4_W1,Adj4_SB0,Adj4_SB1,Adj4_I2x1,Adj4_W2,Adj5_RH1,Adj5_RH2,Adj5_D2x1,Adj5_W,
                       Adj6_RH1,Adj6_RH2,Adj7_Rx1a,Adj7_Rx1b,Adj7_x1PC,Adj8_Rx1a,Adj8_Rx1b,
                       SBP_sb0, SBP_sb1, SBP_fe0, SBP_fe1, SBP_set,
                       CBP_sb0, CBP_sb1, CBP_clr,
                       Error);
    signal state      : TheStates := WaitAction;
    signal next_state : TheStates;
  begin
    R_Data <= R_Data_reg;
    HData <= dbgbus_rsp_data(15 downto 0);
    SelVal_Mux:
    with SelVal select
      Value <= A_Addr     when A,
               A_Value    when V,
               R_Data_reg when rsp,
               X1_Reg     when X1,
               X2_Reg     when X2;
    SelReg_Mux:
    with SelReg select
      regno <= A_Addr(4 downto 0) when A,
               "00001"            when X1,
               "00010"            when X2;
    SelAddr_Mux:
    with SelAddr select
      dbgbus_cmd_payload_address <= x"00"     when Cmd,
                                    x"04"     when Inject,
                                    Bkpt_Addr when HwBk;
    Data_X1_X2_reg: process (Clk)
    begin
      if rising_edge(Clk) then
        if    EnData='1'  then R_Data_reg <= dbgbus_rsp_data;
        elsif EnData/='0' then R_Data_reg <= (others=>'X');
        end if;
        if    EnX1='1'  then X1_Reg <= dbgbus_rsp_data;
        elsif EnX1/='0' then X1_Reg <= (others=>'X');
        end if;
        if    EnX2='1'  then X2_Reg <= dbgbus_rsp_data;
        elsif EnX1/='0' then X2_Reg <= (others=>'X');
        end if;
      end if;
    end process;
    Cnt: process(Clk)
      function max(a,b: natural) return natural is
      begin
        if a>=b then return a;
        else         return b;
        end if;
      end;
      constant BN          : natural := 16;
      constant cntval_size : natural := 4;
      variable cntval      : unsigned(cntval_size-1 downto 0):=to_unsigned(0,cntval_size);
    begin
      if rising_edge(Clk) then
        if    LdCnt='1' and SelCnt='0' then cntval := to_unsigned(3,cntval_size);
        elsif LdCnt='1' and SelCnt='1' then cntval := to_unsigned(BN-1,cntval_size);
        elsif cntval>0  and EnCnt='1'  then cntval := cntval-1;
        end if;
        if    cntval=0 then CntTC<='1';
        elsif cntval>0 then CntTC<='0';
        else                CntTC<='X';
        end if;
        Bkpt_Addr <= x"40" + (cntval&"00");
      end if;
    end process;
    BkptFound <= '1' when Value(31 downto 1)=dbgbus_rsp_data(31 downto 1) else '0';
    EmptyBkpt <= '1' when dbgbus_rsp_data(0)='0' else '0';
    SelCmd_Mux: block
      constant Step1_inject     : std_logic_vector(31 downto 0) := x"0000006F";
      signal   ReadReg_inject   : std_logic_vector(31 downto 0) := x"00006013";
      signal   WriteRegH_inject : std_logic_vector(31 downto 0) := x"00000037";
      signal   WriteRegL_inject : std_logic_vector(31 downto 0) := x"00000013";
      constant ReadPC_inject    : std_logic_vector(31 downto 0) := x"00000017";
      constant WritePC_inject   : std_logic_vector(31 downto 0) := x"00008067";
      constant ReadByte_inject  : std_logic_vector(31 downto 0) := x"0000C003";
      constant WriteByte_inject : std_logic_vector(31 downto 0) := x"00208023";
      constant PC_2_X1_inject   : std_logic_vector(31 downto 0) := x"00000097";
      constant DecX1_by2_inject : std_logic_vector(31 downto 0) := x"ffe08093";
      constant IncX1_by2_inject : std_logic_vector(31 downto 0) := x"00208093";
      constant RdHalf_inject    : std_logic_vector(31 downto 0) := x"0000d003";
    begin
      ReadReg_inject  (19 downto 15) <= regno;
      ReadReg_inject  (11 downto  7) <= regno;
      WriteRegH_inject(31 downto 12) <= std_logic_vector(unsigned(Value(31 downto 12))+unsigned(Value(11 downto 11)));
      WriteRegH_inject(11 downto  7) <= regno;
      WriteRegL_inject(31 downto 20) <= Value(11 downto 0);
      WriteRegL_inject(19 downto 15) <= regno;
      WriteRegL_inject(11 downto  7) <= regno;
      with SelCmd select
        dbgbus_cmd_payload_data <= x"00020000"            when HALT,
                                   x"02000000"            when Resume,
                                   x"02000010"            when Step0,
                                   Step1_inject           when Step1,
                                   Value(31 downto 1)&'1' when HwBkpt,
                                   x"00000000"            when ClrBkpt,
                                   ReadReg_inject         when ReadReg,
                                   WriteRegH_inject       when WriteRegH,
                                   WriteRegL_inject       when WriteRegL,
                                   ReadPC_inject          when ReadPC,
                                   WritePC_inject         when WritePC,
                                   ReadByte_inject        when ReadByte,
                                   WriteByte_inject       when WriteByte,
                                   x"00010000"            when setRESET,
                                   x"01000000"            when clrRESET,
                                   PC_2_X1_inject         when PC2X1,
                                   DecX1_by2_inject       when DecX1_by2,
                                   IncX1_by2_inject       when IncX1_by2,
                                   RdHalf_inject          when RdHalf;
    end block;
    NextState_and_Mealy: process(state, DebugReset, A_valid, A_Action, CntTC, dbgbus_cmd_ready,
                                 dbgbus_rsp_data(isPipBusy), dbgbus_rsp_data(resetIt), dbgbus_rsp_data(HaltIt), dbgbus_rsp_data(haltedByBreak),
                                 EmptyBkpt, BkptFound, HData)
    begin
      A_ready  <= '0';
      R_OK     <= '0';
      EnData   <= '0';
      EnX1     <= '0';
      EnX2     <= '0';
      SelCnt   <= '0';
      LdCnt    <= '0';
      EnCnt    <= '0';
      next_state <= Error;
      if    DebugReset='1' then next_state <= WaitAction;
      elsif DebugReset='0' then
        case state is
          when WaitAction => if    A_valid='0' then next_state <= WaitAction;
                             elsif A_valid='1' then
                               case A_Action is
                                 when Halt          => next_state <= Halt_req;
                                 when Resume        => next_state <= Res_req;
                                 when Step          => next_state <= Step_req;
                                 when ReadReg       => next_state <= RR_ij;
                                 when WriteReg      => next_state <= WR_ij0;
                                 when ReadPC        => next_state <= RPC_ij;
                                 when WritePC       => next_state <= WPC_Sx2a;
                                 when ReadByte      => next_state <= RB_Sx1a;
                                 when WriteByte     => next_state <= WB_Sx1a;
                                 when DoReset       => next_state <= RS_set;
                                 when CheckHalted   => next_state <= ChkHalt;
                                 when AdjustPC      => next_state <= Adj1_Sx1a;
                                 when SetHwBp       => next_state <= SBP_sb0; SelCnt <= '1'; LdCnt <= '1';
                                 when ClrHwBp       => next_state <= CBP_sb0; SelCnt <= '1'; LdCnt <= '1';
                                 when none          => next_state <= WaitAction;
                               end case;
                             end if;
          when Halt_req   => if    dbgbus_cmd_ready='0' then next_state <= Halt_req;
                             elsif dbgbus_cmd_ready='1' then next_state <= Halt_Chk;
                             end if;
          when Halt_Chk   => if dbgbus_cmd_ready='1' and dbgbus_rsp_data(isPipBusy)='0' and dbgbus_rsp_data(HaltIt)='1' then
                                                      next_state <= WaitAction; A_ready <= '1';
                             else                     next_state <= Halt_Chk;
                             end if;
          when Res_req    => if    dbgbus_cmd_ready='0' then next_state <= Res_req;
                             elsif dbgbus_cmd_ready='1' then next_state <= Res_Chk;
                             end if;
          when Res_Chk    => if dbgbus_cmd_ready='1' and dbgbus_rsp_data(isPipBusy)='1' and dbgbus_rsp_data(HaltIt)='0' then
                                                      next_state <= WaitAction; A_ready <= '1';
                             else                     next_state <= Res_Chk;
                             end if;
          when Step_req   => if    dbgbus_cmd_ready='0' then next_state <= Step_req;
                             elsif dbgbus_cmd_ready='1' then next_state <= Step_Chk;
                             end if;
          when Step_Chk   => if dbgbus_cmd_ready='1' and dbgbus_rsp_data(isPipBusy)='0' and dbgbus_rsp_data(HaltIt)='1' then
                                                      next_state <= Step_PC;
                             else                     next_state <= Step_Chk;
                             end if;
          when Step_PC    => if    dbgbus_cmd_ready='0' then next_state <= Step_PC;
                             elsif dbgbus_cmd_ready='1' then next_state <= Step_done;
                             end if;
          when Step_done  => next_state <= WaitAction; A_ready <= '1';
          when RR_ij      => if    dbgbus_cmd_ready='0' then next_state <= RR_ij;
                             elsif dbgbus_cmd_ready='1' then next_state <= RR_get; LdCnt <= '1';
                             end if;
          when RR_get     => if    CntTC='0' then next_state <= RR_get;  EnCnt  <= '1';
                             elsif CntTC='1' then next_state <= RR_done; EnData <= '1';
                             end if;
          when RR_done    => next_state <= WaitAction; A_ready <= '1';
          when WR_ij0     => if    dbgbus_cmd_ready='0' then next_state <= WR_ij0;
                             elsif dbgbus_cmd_ready='1' then next_state <= WR_ij1;
                             end if;
          when WR_ij1     => if    dbgbus_cmd_ready='0' then next_state <= WR_ij1;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when RPC_ij     => if    dbgbus_cmd_ready='0' then next_state <= RPC_ij;
                             elsif dbgbus_cmd_ready='1' then next_state <= RPC_get; LdCnt <= '1';
                             end if;
          when RPC_get    => if    CntTC='0' then next_state <= RPC_get;  EnCnt  <= '1';
                             elsif CntTC='1' then next_state <= RPC_done; EnData <= '1';
                             end if;
          when RPC_done   => next_state <= WaitAction; A_ready <= '1';
          when WPC_Sx2a   => if    dbgbus_cmd_ready='0' then next_state <= WPC_Sx2a;
                             elsif dbgbus_cmd_ready='1' then next_state <= WPC_Sx2b; LdCnt <= '1';
                             end if;
          when WPC_Sx2b   => if    CntTC='0'     then next_state <= WPC_Sx2b; EnCnt <= '1';
                             elsif CntTC='1'     then next_state <= WPC_ij0;  EnX1  <= '1';
                             end if;
          when WPC_ij0    => if    dbgbus_cmd_ready='0' then next_state <= WPC_ij0;
                             elsif dbgbus_cmd_ready='1' then next_state <= WPC_ij1;
                             end if;
          when WPC_ij1    => if    dbgbus_cmd_ready='0' then next_state <= WPC_ij1;
                             elsif dbgbus_cmd_ready='1' then next_state <= WPC_ij2;
                             end if;
          when WPC_ij2    => if    dbgbus_cmd_ready='0' then next_state <= WPC_ij2;
                             elsif dbgbus_cmd_ready='1' then next_state <= WPC_Rx2a;
                             end if;
          when WPC_Rx2a   => if    dbgbus_cmd_ready='0' then next_state <= WPC_Rx2a;
                             elsif dbgbus_cmd_ready='1' then next_state <= WPC_Rx2b;
                             end if;
          when WPC_Rx2b   => if    dbgbus_cmd_ready='0' then next_state <= WPC_Rx2b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when RB_Sx1a    => if    dbgbus_cmd_ready='0' then next_state <= RB_Sx1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= RB_Sx1b; LdCnt <= '1';
                             end if;
          when RB_Sx1b    => if    CntTC='0'     then next_state <= RB_Sx1b; EnCnt  <= '1';
                             elsif CntTC='1'     then next_state <= RB_Ax1a; EnX1 <= '1';
                             end if;
          when RB_Ax1a    => if    dbgbus_cmd_ready='0' then next_state <= RB_Ax1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= RB_Ax1b;
                             end if;
          when RB_Ax1b    => if    dbgbus_cmd_ready='0' then next_state <= RB_Ax1b;
                             elsif dbgbus_cmd_ready='1' then next_state <= RB_ij;
                             end if;
          when RB_ij      => if    dbgbus_cmd_ready='0' then next_state <= RB_ij;
                             elsif dbgbus_cmd_ready='1' then next_state <= RB_get; LdCnt <= '1';
                             end if;
          when RB_get     => if    CntTC='0'     then next_state <= RB_get;  EnCnt  <= '1';
                             elsif CntTC='1'     then next_state <= RB_Rx1a; EnData <= '1';
                             end if;
          when RB_Rx1a    => if    dbgbus_cmd_ready='0' then next_state <= RB_Rx1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= RB_Rx1b;
                             end if;
          when RB_Rx1b    => if    dbgbus_cmd_ready='0' then next_state <= RB_Rx1b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when WB_Sx1a    => if    dbgbus_cmd_ready='0' then next_state <= WB_Sx1a; LdCnt <= '1';
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Sx1b;
                             end if;
          when WB_Sx1b    => if    CntTC='0'     then next_state <= WB_Sx1b; EnCnt <= '1';
                             elsif CntTC='1'     then next_state <= WB_Ax1a; EnX1  <= '1';
                             end if;
          when WB_Ax1a    => if    dbgbus_cmd_ready='0' then next_state <= WB_Ax1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Ax1b;
                             end if;
          when WB_Ax1b    => if    dbgbus_cmd_ready='0' then next_state <= WB_Ax1b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Sx2a;
                             end if;
          when WB_Sx2a    => if    dbgbus_cmd_ready='0' then next_state <= WB_Sx2a; LdCnt <= '1';
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Sx2b;
                             end if;
          when WB_Sx2b    => if    CntTC='0'     then next_state <= WB_Sx2b; EnCnt  <= '1';
                             elsif CntTC='1'     then next_state <= WB_Vx2a; EnX2 <= '1';
                             end if;
          when WB_Vx2a    => if    dbgbus_cmd_ready='0' then next_state <= WB_Vx2a;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Vx2b;
                             end if;
          when WB_Vx2b    => if    dbgbus_cmd_ready='0' then next_state <= WB_Vx2b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_do;
                             end if;
          when WB_do      => if    dbgbus_cmd_ready='0' then next_state <= WB_do;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Rx2a;
                             end if;
          when WB_Rx2a    => if    dbgbus_cmd_ready='0' then next_state <= WB_Rx2a;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Rx2b;
                             end if;
          when WB_Rx2b    => if    dbgbus_cmd_ready='0' then next_state <= WB_Rx2b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Rx1a;
                             end if;
          when WB_Rx1a    => if    dbgbus_cmd_ready='0' then next_state <= WB_Rx1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= WB_Rx1b;
                             end if;
          when WB_Rx1b    => if    dbgbus_cmd_ready='0' then next_state <= WB_Rx1b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when ChkHalt    => if    dbgbus_cmd_ready='0' then next_state <= ChkHalt;
                             elsif dbgbus_cmd_ready='1' and dbgbus_rsp_data(isPipBusy)='0' and (dbgbus_rsp_data(HaltIt)='1' or dbgbus_rsp_data(haltedByBreak)='1') then
                                                      next_state <= WaitAction; A_ready <= '1'; R_OK <= '1';
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when RS_set     => if    dbgbus_cmd_ready='0' then next_state <= RS_set;
                             elsif dbgbus_cmd_ready='1' then next_state <= RS_s_done;
                             end if;
          when RS_s_done  => if    dbgbus_cmd_ready='0' or  dbgbus_rsp_data(resetIt)='0' then next_state <= RS_s_done;
                             elsif dbgbus_cmd_ready='1' and dbgbus_rsp_data(resetIt)='1' then next_state <= RS_clr;
                             end if;
          when RS_clr     => if    dbgbus_cmd_ready='0' then next_state <= RS_clr;
                             elsif dbgbus_cmd_ready='1' then next_state <= RS_c_done;
                             end if;
          when RS_c_done  => if    dbgbus_cmd_ready='0' or  dbgbus_rsp_data(resetIt)='1' then next_state <= RS_c_done;
                             elsif dbgbus_cmd_ready='1' and dbgbus_rsp_data(resetIt)='0' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when Adj1_Sx1a  => if    dbgbus_cmd_ready='0' then next_state <= Adj1_Sx1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj1_Sx1b; LdCnt<='1';
                             end if;
          when Adj1_Sx1b  => if    CntTC='0'     then next_state <= Adj1_Sx1b; EnCnt <= '1';
                             elsif CntTC='1'     then next_state <= Adj2_PCx1; EnX1  <= '1';
                             end if;
          when Adj2_PCx1  => if    dbgbus_cmd_ready='0' then next_state <= Adj2_PCx1;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj2_W; LdCnt<='1';
                             end if;
          when Adj2_W     => if    CntTC='0'     then next_state <= Adj2_W; EnCnt <= '1';
                             elsif CntTC='1'     then next_state <= Adj3_D2x1;
                             end if;
          when Adj3_D2x1  => if    dbgbus_cmd_ready='0' then next_state <= Adj3_D2x1;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj3_W; LdCnt<='1';
                             end if;
          when Adj3_W     => if    CntTC='0'     then next_state <= Adj3_W;   EnCnt <= '1';
                             elsif CntTC='1'     then next_state <= Adj3_SB0; EnData <= '1'; LdCnt <= '1'; SelCnt<='1';
                             end if;
          when Adj3_SB0   => if    dbgbus_cmd_ready='0' then next_state <= Adj3_SB0;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj3_SB1;
                             end if;
          when Adj3_SB1   => if    CntTC='0' and (EmptyBkpt='1' or BkptFound='0') then next_state <= Adj3_SB0; EnCnt  <= '1';
                             elsif CntTC='1' and (EmptyBkpt='1' or BkptFound='0') then next_state <= Adj4_D2x1;
                             elsif EmptyBkpt='0' and BkptFound='1'                then next_state <= Adj7_x1PC;
                             end if;
          when Adj4_D2x1  => if    dbgbus_cmd_ready='0' then next_state <= Adj4_D2x1;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj4_W1; LdCnt<='1';
                             end if;
          when Adj4_W1    => if    CntTC='0'     then next_state <= Adj4_W1;  EnData <= '1'; EnCnt <= '1';
                             elsif CntTC='1'     then next_state <= Adj4_SB0; LdCnt <= '1';  SelCnt<='1';
                             end if;
          when Adj4_SB0   => if    dbgbus_cmd_ready='0' then next_state <= Adj4_SB0;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj4_SB1;
                             end if;
          when Adj4_SB1   => if    CntTC='0' and (EmptyBkpt='1' or BkptFound='0') then next_state <= Adj4_SB0; EnCnt  <= '1';
                             elsif CntTC='1' and (EmptyBkpt='1' or BkptFound='0') then next_state <= Adj4_I2x1;
                             elsif EmptyBkpt='0' and BkptFound='1'                then next_state <= Adj7_x1PC;
                             end if;
          when Adj4_I2x1  => if    dbgbus_cmd_ready='0' then next_state <= Adj4_I2x1;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj4_W2; LdCnt <= '1';
                             end if;
          when Adj4_W2    => if    CntTC='0'     then next_state <= Adj4_W2;  EnCnt <= '1';
                             elsif CntTC='1'     then next_state <= Adj5_RH1; EnData <= '1';
                             end if;
          when Adj5_RH1   => if    dbgbus_cmd_ready='0' then next_state <= Adj5_RH1;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj5_RH2; LdCnt <= '1';
                             end if;
          when Adj5_RH2   => if    CntTC='0'     then next_state <= Adj5_RH2; EnCnt <= '1';
                             elsif CntTC='1'     then
                                if    HData=x"9002" then next_state <= Adj7_x1PC;
                                elsif HData=x"0010" then next_state <= Adj5_D2x1;
                                else                     next_state <= Adj8_Rx1a;
                                end if;
                             end if;
          when Adj5_D2x1  => if    dbgbus_cmd_ready='0' then next_state <= Adj5_D2x1;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj5_W; LdCnt <= '1';
                             end if;
          when Adj5_W     => if    CntTC='0'     then next_state <= Adj5_W;   EnCnt  <= '1';
                             elsif CntTC='1'     then next_state <= Adj6_RH1; EnData <= '1';
                             end if;
          when Adj6_RH1   => if    dbgbus_cmd_ready='0' then next_state <= Adj6_RH1;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj6_RH2; LdCnt<='1';
                             end if;
          when Adj6_RH2   => if    CntTC='0'     then next_state <= Adj6_RH2; EnCnt <= '1';
                             elsif CntTC='1'     then
                                if    HData=x"0073" then next_state <= Adj7_x1PC;
                                else                     next_state <= Adj8_Rx1a;
                                end if;
                             end if;
          when Adj7_x1PC  => if    dbgbus_cmd_ready='0' then next_state <= Adj7_x1PC;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj7_Rx1a;
                             end if;
          when Adj7_Rx1a  => if    dbgbus_cmd_ready='0' then next_state <= Adj7_Rx1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj7_Rx1b;
                             end if;
          when Adj7_Rx1b  => if    dbgbus_cmd_ready='0' then next_state <= Adj7_Rx1b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1'; R_OK <= '1';
                             end if;
          when Adj8_Rx1a  => if    dbgbus_cmd_ready='0' then next_state <= Adj8_Rx1a;
                             elsif dbgbus_cmd_ready='1' then next_state <= Adj8_Rx1b;
                             end if;
          when Adj8_Rx1b  => if    dbgbus_cmd_ready='0' then next_state <= Adj8_Rx1b;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when SBP_sb0    => if    dbgbus_cmd_ready='0' then next_state <= SBP_sb0;
                             elsif dbgbus_cmd_ready='1' then next_state <= SBP_sb1;
                             end if;
          when SBP_sb1    => if    CntTC='0' and (EmptyBkpt='1' or BkptFound='0') then next_state <= SBP_sb0; EnCnt  <= '1';
                             elsif CntTC='1' and (EmptyBkpt='1' or BkptFound='0') then next_state <= SBP_fe0; SelCnt <= '1'; LdCnt <= '1';
                             elsif EmptyBkpt='0' and BkptFound='1'                then next_state <= WaitAction; A_ready <= '1'; R_OK <= '1';
                             end if;
          when SBP_fe0    => if    dbgbus_cmd_ready='0' then next_state <= SBP_fe0;
                             elsif dbgbus_cmd_ready='1' then next_state <= SBP_fe1;
                             end if;
          when SBP_fe1    => if    CntTC='0' and EmptyBkpt='0' then next_state <= SBP_fe0; EnCnt  <= '1';
                             elsif EmptyBkpt='1'               then next_state <= SBP_set;
                             elsif CntTC='1' and EmptyBkpt='0' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when SBP_set    => if    dbgbus_cmd_ready='0' then next_state <= SBP_set;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1'; R_OK <= '1';
                             end if;
          when CBP_sb0    => if    dbgbus_cmd_ready='0' then next_state <= CBP_sb0;
                             elsif dbgbus_cmd_ready='1' then next_state <= CBP_sb1;
                             end if;
          when CBP_sb1    => if    CntTC='0' and (EmptyBkpt='1' or BkptFound='0') then next_state <= CBP_sb0; EnCnt  <= '1';
                             elsif CntTC='1' and (EmptyBkpt='1' or BkptFound='0') then next_state <= WaitAction; A_ready <= '1';
                             elsif EmptyBkpt='0' and BkptFound='1'                then next_state <= CBP_clr;
                             end if;
          when CBP_clr    => if    dbgbus_cmd_ready='0' then next_state <= CBP_clr;
                             elsif dbgbus_cmd_ready='1' then next_state <= WaitAction; A_ready <= '1';
                             end if;
          when Error      => null;
        end case;
      end if;
    end process;
    State_and_Moore: process(Clk)
    begin
      if rising_edge(Clk) then
        state <= next_state;
        SelVal     <= V;
        SelReg     <= A;
        SelAddr    <= Cmd;
        SelCmd     <= HALT;
        dbgbus_cmd_valid  <= '0';
        dbgbus_cmd_payload_wr     <= '0';
        case next_state is
          when WaitAction =>
          when Halt_req   => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Cmd; SelCmd <= HALT;
          when Halt_Chk   => dbgbus_cmd_valid <= '1'; SelAddr <= Cmd;
          when Res_req    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Cmd; SelCmd <= Resume;
          when Res_Chk    => dbgbus_cmd_valid <= '1'; SelAddr <= Cmd;
          when Step_req   => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Cmd; SelCmd <= Step0;
          when Step_Chk   => dbgbus_cmd_valid <= '1'; SelAddr <= Cmd;
          when Step_PC    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= Step1;
          when Step_done  =>
          when RR_ij      => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadReg; SelReg <= A;
          when RR_get     => SelAddr <= Inject;
          when RR_done    =>
          when WR_ij0     => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= A; SelVal <= V;
          when WR_ij1     => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= A; SelVal <= V;
          when RPC_ij     => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadPC;
          when RPC_get    => SelAddr <= Inject;
          when RPC_done   =>
          when WPC_Sx2a   => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadReg; SelReg <= X1;
          when WPC_Sx2b   => SelAddr   <= Inject;
          when WPC_ij0    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= V;
          when WPC_ij1    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= V;
          when WPC_ij2    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WritePC;
          when WPC_Rx2a   => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= X1;
          when WPC_Rx2b   => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= X1;
          when RB_Sx1a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadReg; SelReg <= X1;
          when RB_Sx1b    => SelAddr <= Inject;
          when RB_Ax1a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= A;
          when RB_Ax1b    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= A;
          when RB_ij      => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadByte;
          when RB_get     => SelAddr <= Inject;
          when RB_Rx1a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= X1;
          when RB_Rx1b    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= X1;
          when WB_Sx1a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadReg; SelReg <= X1;
          when WB_Sx1b    => SelAddr <= Inject;
          when WB_Ax1a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= A;
          when WB_Ax1b    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= A;
          when WB_Sx2a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadReg; SelReg <= X2;
          when WB_Sx2b    => SelAddr <= Inject;
          when WB_Vx2a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X2; SelVal <= V;
          when WB_Vx2b    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X2; SelVal <= V;
          when WB_do      => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteByte;
          when WB_Rx2a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X2; SelVal <= X2;
          when WB_Rx2b    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X2; SelVal <= X2;
          when WB_Rx1a    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= X1;
          when WB_Rx1b    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= X1;
          when ChkHalt    => dbgbus_cmd_valid <= '1'; SelAddr <= Cmd;
          when RS_set     => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Cmd; SelCmd <= setRESET;
          when RS_s_done  => dbgbus_cmd_valid <= '1'; SelAddr <= Cmd;
          when RS_clr     => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Cmd; SelCmd <= clrRESET;
          when RS_c_done  => dbgbus_cmd_valid <= '1'; SelAddr <= Cmd;
          when Adj1_Sx1a  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= ReadReg; SelReg <= X1;
          when Adj1_Sx1b  => SelAddr <= Inject;
          when Adj2_PCx1  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= PC2X1;
          when Adj2_W     => SelAddr <= Inject;
          when Adj3_D2x1  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= DecX1_by2;
          when Adj3_W     => SelAddr <= Inject;
          when Adj3_SB0   => dbgbus_cmd_valid <= '1'; SelAddr <= HwBk; SelVal <= rsp;
          when Adj3_SB1   => SelAddr <= HwBk; SelVal <= rsp;
          when Adj4_D2x1  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= DecX1_by2;
          when Adj4_W1    => SelAddr <= Inject;
          when Adj4_SB0   => dbgbus_cmd_valid <= '1'; SelAddr <= HwBk; SelVal <= rsp;
          when Adj4_SB1   => SelAddr <= HwBk; SelVal <= rsp;
          when Adj4_I2x1  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= IncX1_by2;
          when Adj4_W2    => SelAddr <= Inject;
          when Adj5_RH1   => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= RdHalf;
          when Adj5_RH2   => SelAddr <= Inject;
          when Adj5_D2x1  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= DecX1_by2;
          when Adj5_W     => SelAddr <= Inject;
          when Adj6_RH1   => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= RdHalf;
          when Adj6_RH2   => SelAddr <= Inject;
          when Adj7_x1PC  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WritePC;
          when Adj7_Rx1a  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= X1;
          when Adj7_Rx1b  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= X1;
          when Adj8_Rx1a  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegH; SelReg <= X1; SelVal <= X1;
          when Adj8_Rx1b  => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= Inject; SelCmd <= WriteRegL; SelReg <= X1; SelVal <= X1;
          when SBP_sb0    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '0'; SelAddr <= HwBk; SelVal <= A;
          when SBP_sb1    => SelAddr <= HwBk;  SelVal <= A;
          when SBP_fe0    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '0'; SelAddr <= HwBk; SelVal <= A;
          when SBP_fe1    => SelAddr <= HwBk;  SelVal <= A;
          when SBP_set    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= HwBk; SelVal <= A; SelCmd <= HwBkpt;
          when CBP_sb0    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '0'; SelAddr <= HwBk; SelVal <= A;
          when CBP_sb1    => SelAddr <= HwBk;  SelVal <= A;
          when CBP_clr    => dbgbus_cmd_valid <= '1'; dbgbus_cmd_payload_wr <= '1'; SelAddr <= HwBk; SelCmd <= ClrBkpt;
          when Error      =>
        end case;
      end if;
    end process;
  end block;
end arch;
