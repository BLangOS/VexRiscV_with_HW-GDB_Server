-- Generator : SpinalHDL v1.6.1    git head : 4d4778cda740ce2648add90ec8b07c8482d85132
-- Component : VexRiscv
-- Git hash  : 4d4778cda740ce2648add90ec8b07c8482d85132

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.all;

package pkg_enum is
  type BranchCtrlEnum is (INC,B,JAL,JALR);
  type ShiftCtrlEnum is (DISABLE_1,SLL_1,SRL_1,SRA_1);
  type AluBitwiseCtrlEnum is (XOR_1,OR_1,AND_1);
  type EnvCtrlEnum is (NONE,XRET,ECALL);
  type AluCtrlEnum is (ADD_SUB,SLT_SLTU,BITWISE);
  type Src2CtrlEnum is (RS,IMI,IMS,PC);
  type Src1CtrlEnum is (RS,IMU,PC_INCREMENT,URS1);

  function pkg_mux (sel : std_logic; one : BranchCtrlEnum; zero : BranchCtrlEnum) return BranchCtrlEnum;
  subtype BranchCtrlEnum_seq_type is std_logic_vector(1 downto 0);
  constant BranchCtrlEnum_seq_INC : BranchCtrlEnum_seq_type := "00";
  constant BranchCtrlEnum_seq_B : BranchCtrlEnum_seq_type := "01";
  constant BranchCtrlEnum_seq_JAL : BranchCtrlEnum_seq_type := "10";
  constant BranchCtrlEnum_seq_JALR : BranchCtrlEnum_seq_type := "11";

  function pkg_mux (sel : std_logic; one : ShiftCtrlEnum; zero : ShiftCtrlEnum) return ShiftCtrlEnum;
  subtype ShiftCtrlEnum_seq_type is std_logic_vector(1 downto 0);
  constant ShiftCtrlEnum_seq_DISABLE_1 : ShiftCtrlEnum_seq_type := "00";
  constant ShiftCtrlEnum_seq_SLL_1 : ShiftCtrlEnum_seq_type := "01";
  constant ShiftCtrlEnum_seq_SRL_1 : ShiftCtrlEnum_seq_type := "10";
  constant ShiftCtrlEnum_seq_SRA_1 : ShiftCtrlEnum_seq_type := "11";

  function pkg_mux (sel : std_logic; one : AluBitwiseCtrlEnum; zero : AluBitwiseCtrlEnum) return AluBitwiseCtrlEnum;
  subtype AluBitwiseCtrlEnum_seq_type is std_logic_vector(1 downto 0);
  constant AluBitwiseCtrlEnum_seq_XOR_1 : AluBitwiseCtrlEnum_seq_type := "00";
  constant AluBitwiseCtrlEnum_seq_OR_1 : AluBitwiseCtrlEnum_seq_type := "01";
  constant AluBitwiseCtrlEnum_seq_AND_1 : AluBitwiseCtrlEnum_seq_type := "10";

  function pkg_mux (sel : std_logic; one : EnvCtrlEnum; zero : EnvCtrlEnum) return EnvCtrlEnum;
  subtype EnvCtrlEnum_seq_type is std_logic_vector(1 downto 0);
  constant EnvCtrlEnum_seq_NONE : EnvCtrlEnum_seq_type := "00";
  constant EnvCtrlEnum_seq_XRET : EnvCtrlEnum_seq_type := "01";
  constant EnvCtrlEnum_seq_ECALL : EnvCtrlEnum_seq_type := "10";

  function pkg_mux (sel : std_logic; one : AluCtrlEnum; zero : AluCtrlEnum) return AluCtrlEnum;
  subtype AluCtrlEnum_seq_type is std_logic_vector(1 downto 0);
  constant AluCtrlEnum_seq_ADD_SUB : AluCtrlEnum_seq_type := "00";
  constant AluCtrlEnum_seq_SLT_SLTU : AluCtrlEnum_seq_type := "01";
  constant AluCtrlEnum_seq_BITWISE : AluCtrlEnum_seq_type := "10";

  function pkg_mux (sel : std_logic; one : Src2CtrlEnum; zero : Src2CtrlEnum) return Src2CtrlEnum;
  subtype Src2CtrlEnum_seq_type is std_logic_vector(1 downto 0);
  constant Src2CtrlEnum_seq_RS : Src2CtrlEnum_seq_type := "00";
  constant Src2CtrlEnum_seq_IMI : Src2CtrlEnum_seq_type := "01";
  constant Src2CtrlEnum_seq_IMS : Src2CtrlEnum_seq_type := "10";
  constant Src2CtrlEnum_seq_PC : Src2CtrlEnum_seq_type := "11";

  function pkg_mux (sel : std_logic; one : Src1CtrlEnum; zero : Src1CtrlEnum) return Src1CtrlEnum;
  subtype Src1CtrlEnum_seq_type is std_logic_vector(1 downto 0);
  constant Src1CtrlEnum_seq_RS : Src1CtrlEnum_seq_type := "00";
  constant Src1CtrlEnum_seq_IMU : Src1CtrlEnum_seq_type := "01";
  constant Src1CtrlEnum_seq_PC_INCREMENT : Src1CtrlEnum_seq_type := "10";
  constant Src1CtrlEnum_seq_URS1 : Src1CtrlEnum_seq_type := "11";

end pkg_enum;

package body pkg_enum is
  function pkg_mux (sel : std_logic; one : BranchCtrlEnum; zero : BranchCtrlEnum) return BranchCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : ShiftCtrlEnum; zero : ShiftCtrlEnum) return ShiftCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : AluBitwiseCtrlEnum; zero : AluBitwiseCtrlEnum) return AluBitwiseCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : EnvCtrlEnum; zero : EnvCtrlEnum) return EnvCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : AluCtrlEnum; zero : AluCtrlEnum) return AluCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : Src2CtrlEnum; zero : Src2CtrlEnum) return Src2CtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : Src1CtrlEnum; zero : Src1CtrlEnum) return Src1CtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

end pkg_enum;


library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

package pkg_scala2hdl is
  function pkg_extract (that : std_logic_vector; bitId : integer) return std_logic;
  function pkg_extract (that : std_logic_vector; base : unsigned; size : integer) return std_logic_vector;
  function pkg_cat (a : std_logic_vector; b : std_logic_vector) return std_logic_vector;
  function pkg_not (value : std_logic_vector) return std_logic_vector;
  function pkg_extract (that : unsigned; bitId : integer) return std_logic;
  function pkg_extract (that : unsigned; base : unsigned; size : integer) return unsigned;
  function pkg_cat (a : unsigned; b : unsigned) return unsigned;
  function pkg_not (value : unsigned) return unsigned;
  function pkg_extract (that : signed; bitId : integer) return std_logic;
  function pkg_extract (that : signed; base : unsigned; size : integer) return signed;
  function pkg_cat (a : signed; b : signed) return signed;
  function pkg_not (value : signed) return signed;

  function pkg_mux (sel : std_logic; one : std_logic; zero : std_logic) return std_logic;
  function pkg_mux (sel : std_logic; one : std_logic_vector; zero : std_logic_vector) return std_logic_vector;
  function pkg_mux (sel : std_logic; one : unsigned; zero : unsigned) return unsigned;
  function pkg_mux (sel : std_logic; one : signed; zero : signed) return signed;

  function pkg_toStdLogic (value : boolean) return std_logic;
  function pkg_toStdLogicVector (value : std_logic) return std_logic_vector;
  function pkg_toUnsigned (value : std_logic) return unsigned;
  function pkg_toSigned (value : std_logic) return signed;
  function pkg_stdLogicVector (lit : std_logic_vector) return std_logic_vector;
  function pkg_unsigned (lit : unsigned) return unsigned;
  function pkg_signed (lit : signed) return signed;

  function pkg_resize (that : std_logic_vector; width : integer) return std_logic_vector;
  function pkg_resize (that : unsigned; width : integer) return unsigned;
  function pkg_resize (that : signed; width : integer) return signed;

  function pkg_extract (that : std_logic_vector; high : integer; low : integer) return std_logic_vector;
  function pkg_extract (that : unsigned; high : integer; low : integer) return unsigned;
  function pkg_extract (that : signed; high : integer; low : integer) return signed;

  function pkg_shiftRight (that : std_logic_vector; size : natural) return std_logic_vector;
  function pkg_shiftRight (that : std_logic_vector; size : unsigned) return std_logic_vector;
  function pkg_shiftLeft (that : std_logic_vector; size : natural) return std_logic_vector;
  function pkg_shiftLeft (that : std_logic_vector; size : unsigned) return std_logic_vector;

  function pkg_shiftRight (that : unsigned; size : natural) return unsigned;
  function pkg_shiftRight (that : unsigned; size : unsigned) return unsigned;
  function pkg_shiftLeft (that : unsigned; size : natural) return unsigned;
  function pkg_shiftLeft (that : unsigned; size : unsigned) return unsigned;

  function pkg_shiftRight (that : signed; size : natural) return signed;
  function pkg_shiftRight (that : signed; size : unsigned) return signed;
  function pkg_shiftLeft (that : signed; size : natural) return signed;
  function pkg_shiftLeft (that : signed; size : unsigned; w : integer) return signed;

  function pkg_rotateLeft (that : std_logic_vector; size : unsigned) return std_logic_vector;
end  pkg_scala2hdl;

package body pkg_scala2hdl is
  function pkg_extract (that : std_logic_vector; bitId : integer) return std_logic is
    alias temp : std_logic_vector(that'length-1 downto 0) is that;
  begin
    return temp(bitId);
  end pkg_extract;

  function pkg_extract (that : std_logic_vector; base : unsigned; size : integer) return std_logic_vector is
    alias temp : std_logic_vector(that'length-1 downto 0) is that;    constant elementCount : integer := temp'length - size + 1;
    type tableType is array (0 to elementCount-1) of std_logic_vector(size-1 downto 0);
    variable table : tableType;
  begin
    for i in 0 to elementCount-1 loop
      table(i) := temp(i + size - 1 downto i);
    end loop;
    return table(to_integer(base));
  end pkg_extract;

  function pkg_cat (a : std_logic_vector; b : std_logic_vector) return std_logic_vector is
    variable cat : std_logic_vector(a'length + b'length-1 downto 0);
  begin
    cat := a & b;
    return cat;
  end pkg_cat;

  function pkg_not (value : std_logic_vector) return std_logic_vector is
    variable ret : std_logic_vector(value'length-1 downto 0);
  begin
    ret := not value;
    return ret;
  end pkg_not;

  function pkg_extract (that : unsigned; bitId : integer) return std_logic is
    alias temp : unsigned(that'length-1 downto 0) is that;
  begin
    return temp(bitId);
  end pkg_extract;

  function pkg_extract (that : unsigned; base : unsigned; size : integer) return unsigned is
    alias temp : unsigned(that'length-1 downto 0) is that;    constant elementCount : integer := temp'length - size + 1;
    type tableType is array (0 to elementCount-1) of unsigned(size-1 downto 0);
    variable table : tableType;
  begin
    for i in 0 to elementCount-1 loop
      table(i) := temp(i + size - 1 downto i);
    end loop;
    return table(to_integer(base));
  end pkg_extract;

  function pkg_cat (a : unsigned; b : unsigned) return unsigned is
    variable cat : unsigned(a'length + b'length-1 downto 0);
  begin
    cat := a & b;
    return cat;
  end pkg_cat;

  function pkg_not (value : unsigned) return unsigned is
    variable ret : unsigned(value'length-1 downto 0);
  begin
    ret := not value;
    return ret;
  end pkg_not;

  function pkg_extract (that : signed; bitId : integer) return std_logic is
    alias temp : signed(that'length-1 downto 0) is that;
  begin
    return temp(bitId);
  end pkg_extract;

  function pkg_extract (that : signed; base : unsigned; size : integer) return signed is
    alias temp : signed(that'length-1 downto 0) is that;    constant elementCount : integer := temp'length - size + 1;
    type tableType is array (0 to elementCount-1) of signed(size-1 downto 0);
    variable table : tableType;
  begin
    for i in 0 to elementCount-1 loop
      table(i) := temp(i + size - 1 downto i);
    end loop;
    return table(to_integer(base));
  end pkg_extract;

  function pkg_cat (a : signed; b : signed) return signed is
    variable cat : signed(a'length + b'length-1 downto 0);
  begin
    cat := a & b;
    return cat;
  end pkg_cat;

  function pkg_not (value : signed) return signed is
    variable ret : signed(value'length-1 downto 0);
  begin
    ret := not value;
    return ret;
  end pkg_not;


  -- unsigned shifts
  function pkg_shiftRight (that : unsigned; size : natural) return unsigned is
    variable ret : unsigned(that'length-1 downto 0);
  begin
    if size >= that'length then
      return "";
    else
      ret := shift_right(that,size);
      return ret(that'length-1-size downto 0);
    end if;
  end pkg_shiftRight;

  function pkg_shiftRight (that : unsigned; size : unsigned) return unsigned is
    variable ret : unsigned(that'length-1 downto 0);
  begin
    ret := shift_right(that,to_integer(size));
    return ret;
  end pkg_shiftRight;

  function pkg_shiftLeft (that : unsigned; size : natural) return unsigned is
  begin
    return shift_left(resize(that,that'length + size),size);
  end pkg_shiftLeft;

  function pkg_shiftLeft (that : unsigned; size : unsigned) return unsigned is
  begin
    return shift_left(resize(that,that'length + 2**size'length - 1),to_integer(size));
  end pkg_shiftLeft;

  -- std_logic_vector shifts
  function pkg_shiftRight (that : std_logic_vector; size : natural) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftRight(unsigned(that),size));
  end pkg_shiftRight;

  function pkg_shiftRight (that : std_logic_vector; size : unsigned) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftRight(unsigned(that),size));
  end pkg_shiftRight;

  function pkg_shiftLeft (that : std_logic_vector; size : natural) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftLeft(unsigned(that),size));
  end pkg_shiftLeft;

  function pkg_shiftLeft (that : std_logic_vector; size : unsigned) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftLeft(unsigned(that),size));
  end pkg_shiftLeft;

  -- signed shifts
  function pkg_shiftRight (that : signed; size : natural) return signed is
  begin
    return signed(pkg_shiftRight(unsigned(that),size));
  end pkg_shiftRight;

  function pkg_shiftRight (that : signed; size : unsigned) return signed is
  begin
    return shift_right(that,to_integer(size));
  end pkg_shiftRight;

  function pkg_shiftLeft (that : signed; size : natural) return signed is
  begin
    return signed(pkg_shiftLeft(unsigned(that),size));
  end pkg_shiftLeft;

  function pkg_shiftLeft (that : signed; size : unsigned; w : integer) return signed is
  begin
    return shift_left(resize(that,w),to_integer(size));
  end pkg_shiftLeft;

  function pkg_rotateLeft (that : std_logic_vector; size : unsigned) return std_logic_vector is
  begin
    return std_logic_vector(rotate_left(unsigned(that),to_integer(size)));
  end pkg_rotateLeft;

  function pkg_extract (that : std_logic_vector; high : integer; low : integer) return std_logic_vector is
    alias temp : std_logic_vector(that'length-1 downto 0) is that;
  begin
    return temp(high downto low);
  end pkg_extract;

  function pkg_extract (that : unsigned; high : integer; low : integer) return unsigned is
    alias temp : unsigned(that'length-1 downto 0) is that;
  begin
    return temp(high downto low);
  end pkg_extract;

  function pkg_extract (that : signed; high : integer; low : integer) return signed is
    alias temp : signed(that'length-1 downto 0) is that;
  begin
    return temp(high downto low);
  end pkg_extract;

  function pkg_mux (sel : std_logic; one : std_logic; zero : std_logic) return std_logic is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : std_logic_vector; zero : std_logic_vector) return std_logic_vector is
    variable ret : std_logic_vector(zero'range);
  begin
    if sel = '1' then
      ret := one;
    else
      ret := zero;
    end if;
    return ret;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : unsigned; zero : unsigned) return unsigned is
    variable ret : unsigned(zero'range);
  begin
    if sel = '1' then
      ret := one;
    else
      ret := zero;
    end if;
    return ret;
  end pkg_mux;

  function pkg_mux (sel : std_logic; one : signed; zero : signed) return signed is
    variable ret : signed(zero'range);
  begin
    if sel = '1' then
      ret := one;
    else
      ret := zero;
    end if;
    return ret;
  end pkg_mux;

  function pkg_toStdLogic (value : boolean) return std_logic is
  begin
    if value = true then
      return '1';
    else
      return '0';
    end if;
  end pkg_toStdLogic;

  function pkg_toStdLogicVector (value : std_logic) return std_logic_vector is
    variable ret : std_logic_vector(0 downto 0);
  begin
    ret(0) := value;
    return ret;
  end pkg_toStdLogicVector;

  function pkg_toUnsigned (value : std_logic) return unsigned is
    variable ret : unsigned(0 downto 0);
  begin
    ret(0) := value;
    return ret;
  end pkg_toUnsigned;

  function pkg_toSigned (value : std_logic) return signed is
    variable ret : signed(0 downto 0);
  begin
    ret(0) := value;
    return ret;
  end pkg_toSigned;

  function pkg_stdLogicVector (lit : std_logic_vector) return std_logic_vector is
    alias ret : std_logic_vector(lit'length-1 downto 0) is lit;
  begin
    return ret;
  end pkg_stdLogicVector;

  function pkg_unsigned (lit : unsigned) return unsigned is
    alias ret : unsigned(lit'length-1 downto 0) is lit;
  begin
    return ret;
  end pkg_unsigned;

  function pkg_signed (lit : signed) return signed is
    alias ret : signed(lit'length-1 downto 0) is lit;
  begin
    return ret;
  end pkg_signed;

  function pkg_resize (that : std_logic_vector; width : integer) return std_logic_vector is
  begin
    return std_logic_vector(resize(unsigned(that),width));
  end pkg_resize;

  function pkg_resize (that : unsigned; width : integer) return unsigned is
    variable ret : unsigned(width-1 downto 0);
  begin
    if that'length = 0 then
       ret := (others => '0');
    else
       ret := resize(that,width);
    end if;
    return ret;
  end pkg_resize;
  function pkg_resize (that : signed; width : integer) return signed is
    alias temp : signed(that'length-1 downto 0) is that;
    variable ret : signed(width-1 downto 0);
  begin
    if temp'length = 0 then
       ret := (others => '0');
    elsif temp'length >= width then
       ret := temp(width-1 downto 0);
    else
       ret := resize(temp,width);
    end if;
    return ret;
  end pkg_resize;
end pkg_scala2hdl;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.pkg_scala2hdl.all;
use work.all;
use work.pkg_enum.all;


entity StreamFifoLowLatency is
  port(
    io_push_valid : in std_logic;
    io_push_ready : out std_logic;
    io_push_payload_error : in std_logic;
    io_push_payload_inst : in std_logic_vector(31 downto 0);
    io_pop_valid : out std_logic;
    io_pop_ready : in std_logic;
    io_pop_payload_error : out std_logic;
    io_pop_payload_inst : out std_logic_vector(31 downto 0);
    io_flush : in std_logic;
    io_occupancy : out unsigned(0 downto 0);
    clk : in std_logic;
    reset : in std_logic
  );
end StreamFifoLowLatency;

architecture arch of StreamFifoLowLatency is
  signal io_push_ready_read_buffer : std_logic;
  signal io_pop_valid_read_buffer : std_logic;

  signal when_Phase_l623 : std_logic;
  signal pushPtr_willIncrement : std_logic;
  signal pushPtr_willClear : std_logic;
  signal pushPtr_willOverflowIfInc : std_logic;
  signal pushPtr_willOverflow : std_logic;
  signal popPtr_willIncrement : std_logic;
  signal popPtr_willClear : std_logic;
  signal popPtr_willOverflowIfInc : std_logic;
  signal popPtr_willOverflow : std_logic;
  signal ptrMatch : std_logic;
  signal risingOccupancy : std_logic;
  signal empty : std_logic;
  signal full : std_logic;
  signal pushing : std_logic;
  signal popping : std_logic;
  signal readed_error : std_logic;
  signal readed_inst : std_logic_vector(31 downto 0);
  signal zz_readed_error : std_logic_vector(32 downto 0);
  signal when_Stream_l1000 : std_logic;
  signal when_Stream_l1013 : std_logic;
  signal zz_readed_error_1 : std_logic_vector(32 downto 0);
begin
  io_push_ready <= io_push_ready_read_buffer;
  io_pop_valid <= io_pop_valid_read_buffer;
  process(pushing)
  begin
    when_Phase_l623 <= pkg_toStdLogic(false);
    if pushing = '1' then
      when_Phase_l623 <= pkg_toStdLogic(true);
    end if;
  end process;

  process(pushing)
  begin
    pushPtr_willIncrement <= pkg_toStdLogic(false);
    if pushing = '1' then
      pushPtr_willIncrement <= pkg_toStdLogic(true);
    end if;
  end process;

  process(io_flush)
  begin
    pushPtr_willClear <= pkg_toStdLogic(false);
    if io_flush = '1' then
      pushPtr_willClear <= pkg_toStdLogic(true);
    end if;
  end process;

  pushPtr_willOverflowIfInc <= pkg_toStdLogic(true);
  pushPtr_willOverflow <= (pushPtr_willOverflowIfInc and pushPtr_willIncrement);
  process(popping)
  begin
    popPtr_willIncrement <= pkg_toStdLogic(false);
    if popping = '1' then
      popPtr_willIncrement <= pkg_toStdLogic(true);
    end if;
  end process;

  process(io_flush)
  begin
    popPtr_willClear <= pkg_toStdLogic(false);
    if io_flush = '1' then
      popPtr_willClear <= pkg_toStdLogic(true);
    end if;
  end process;

  popPtr_willOverflowIfInc <= pkg_toStdLogic(true);
  popPtr_willOverflow <= (popPtr_willOverflowIfInc and popPtr_willIncrement);
  ptrMatch <= pkg_toStdLogic(true);
  empty <= (ptrMatch and (not risingOccupancy));
  full <= (ptrMatch and risingOccupancy);
  pushing <= (io_push_valid and io_push_ready_read_buffer);
  popping <= (io_pop_valid_read_buffer and io_pop_ready);
  io_push_ready_read_buffer <= (not full);
  zz_readed_error <= zz_readed_error_1;
  readed_error <= pkg_extract(zz_readed_error,0);
  readed_inst <= pkg_extract(zz_readed_error,32,1);
  when_Stream_l1000 <= (not empty);
  process(when_Stream_l1000,io_push_valid)
  begin
    if when_Stream_l1000 = '1' then
      io_pop_valid_read_buffer <= pkg_toStdLogic(true);
    else
      io_pop_valid_read_buffer <= io_push_valid;
    end if;
  end process;

  process(when_Stream_l1000,readed_error,io_push_payload_error)
  begin
    if when_Stream_l1000 = '1' then
      io_pop_payload_error <= readed_error;
    else
      io_pop_payload_error <= io_push_payload_error;
    end if;
  end process;

  process(when_Stream_l1000,readed_inst,io_push_payload_inst)
  begin
    if when_Stream_l1000 = '1' then
      io_pop_payload_inst <= readed_inst;
    else
      io_pop_payload_inst <= io_push_payload_inst;
    end if;
  end process;

  when_Stream_l1013 <= pkg_toStdLogic(pushing /= popping);
  io_occupancy <= unsigned(pkg_toStdLogicVector((risingOccupancy and ptrMatch)));
  process(clk, reset)
  begin
    if reset = '1' then
      risingOccupancy <= pkg_toStdLogic(false);
    elsif rising_edge(clk) then
      if when_Stream_l1013 = '1' then
        risingOccupancy <= pushing;
      end if;
      if io_flush = '1' then
        risingOccupancy <= pkg_toStdLogic(false);
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if when_Phase_l623 = '1' then
        zz_readed_error_1 <= pkg_cat(io_push_payload_inst,pkg_toStdLogicVector(io_push_payload_error));
      end if;
    end if;
  end process;

end arch;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.pkg_scala2hdl.all;
use work.all;
use work.pkg_enum.all;


entity VexRiscv is
  port(
    timerInterrupt : in std_logic;
    externalInterrupt : in std_logic;
    softwareInterrupt : in std_logic;
    debug_bus_cmd_valid : in std_logic;
    debug_bus_cmd_ready : out std_logic;
    debug_bus_cmd_payload_wr : in std_logic;
    debug_bus_cmd_payload_address : in unsigned(7 downto 0);
    debug_bus_cmd_payload_data : in std_logic_vector(31 downto 0);
    debug_bus_rsp_data : out std_logic_vector(31 downto 0);
    debug_resetOut : out std_logic;
    LocalInt0 : in std_logic;
    LocalInt1 : in std_logic;
    LocalInt2 : in std_logic;
    LocalInt3 : in std_logic;
    iBusWishbone_CYC : out std_logic;
    iBusWishbone_STB : out std_logic;
    iBusWishbone_ACK : in std_logic;
    iBusWishbone_WE : out std_logic;
    iBusWishbone_ADR : out unsigned(29 downto 0);
    iBusWishbone_DAT_MISO : in std_logic_vector(31 downto 0);
    iBusWishbone_DAT_MOSI : out std_logic_vector(31 downto 0);
    iBusWishbone_SEL : out std_logic_vector(3 downto 0);
    iBusWishbone_ERR : in std_logic;
    iBusWishbone_CTI : out std_logic_vector(2 downto 0);
    iBusWishbone_BTE : out std_logic_vector(1 downto 0);
    dBusWishbone_CYC : out std_logic;
    dBusWishbone_STB : out std_logic;
    dBusWishbone_ACK : in std_logic;
    dBusWishbone_WE : out std_logic;
    dBusWishbone_ADR : out unsigned(29 downto 0);
    dBusWishbone_DAT_MISO : in std_logic_vector(31 downto 0);
    dBusWishbone_DAT_MOSI : out std_logic_vector(31 downto 0);
    dBusWishbone_SEL : out std_logic_vector(3 downto 0);
    dBusWishbone_ERR : in std_logic;
    dBusWishbone_CTI : out std_logic_vector(2 downto 0);
    dBusWishbone_BTE : out std_logic_vector(1 downto 0);
    clk : in std_logic;
    reset : in std_logic;
    debugReset : in std_logic
  );
end VexRiscv;

architecture arch of VexRiscv is
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_ready : std_logic;
  signal zz_RegFilePlugin_regFile_port0 : std_logic_vector(31 downto 0);
  signal zz_RegFilePlugin_regFile_port0_1 : std_logic_vector(31 downto 0);
  signal debug_bus_cmd_ready_read_buffer : std_logic;
  signal iBusWishbone_CYC_read_buffer : std_logic;
  signal dBusWishbone_WE_read_buffer : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy : unsigned(0 downto 0);
  signal zz_decode_DO_EBREAK : std_logic;
  signal zz_decode_DO_EBREAK_1 : std_logic;
  signal zz_decode_DO_EBREAK_2 : std_logic;
  signal zz_decode_DO_EBREAK_3 : unsigned(30 downto 0);
  signal zz_decode_DO_EBREAK_4 : std_logic;
  signal zz_decode_DO_EBREAK_5 : std_logic;
  signal zz_decode_DO_EBREAK_6 : unsigned(30 downto 0);
  signal zz_decode_DO_EBREAK_7 : std_logic;
  signal zz_decode_DO_EBREAK_8 : std_logic;
  signal zz_decode_DO_EBREAK_9 : unsigned(30 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_1 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_2 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_3 : std_logic;
  signal zz_decode_LEGAL_INSTRUCTION_4 : std_logic_vector(0 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_5 : std_logic_vector(12 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_6 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_7 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_8 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_9 : std_logic;
  signal zz_decode_LEGAL_INSTRUCTION_10 : std_logic_vector(0 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_11 : std_logic_vector(6 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_12 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_13 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_14 : std_logic_vector(31 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_15 : std_logic;
  signal zz_decode_LEGAL_INSTRUCTION_16 : std_logic_vector(0 downto 0);
  signal zz_decode_LEGAL_INSTRUCTION_17 : std_logic_vector(0 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_27 : std_logic_vector(6 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_28 : std_logic_vector(0 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_29 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_30 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_31 : std_logic_vector(6 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_32 : std_logic_vector(4 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_33 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_34 : std_logic_vector(4 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_1 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_2 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_3 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_4 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_5 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_6 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_7 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_8 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_9 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_10 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_11 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_12 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_13 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_14 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_15 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_16 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_17 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_18 : std_logic_vector(24 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_19 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_20 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_21 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_22 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_23 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_24 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_25 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_26 : std_logic_vector(21 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_27 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_28 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_29 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_30 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_31 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_32 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_33 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_34 : std_logic_vector(18 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_35 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_36 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_37 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_38 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_39 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_40 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_41 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_42 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_43 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_44 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_45 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_46 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_47 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_48 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_49 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_50 : std_logic_vector(14 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_51 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_52 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_53 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_54 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_55 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_56 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_57 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_58 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_59 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_60 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_61 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_62 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_63 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_64 : std_logic_vector(11 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_65 : std_logic_vector(4 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_66 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_67 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_68 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_69 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_70 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_71 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_72 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_73 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_74 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_75 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_76 : std_logic_vector(4 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_77 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_78 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_79 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_80 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_81 : std_logic_vector(2 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_82 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_83 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_84 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_85 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_86 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_87 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_88 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_89 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_90 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_91 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_92 : std_logic_vector(4 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_93 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_94 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_95 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_96 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_97 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_98 : std_logic_vector(2 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_99 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_100 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_101 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_102 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_103 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_104 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_105 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_106 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_107 : std_logic_vector(8 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_108 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_109 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_110 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_111 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_112 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_113 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_114 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_115 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_116 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_117 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_118 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_119 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_120 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_121 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_122 : std_logic_vector(5 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_123 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_124 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_125 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_126 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_127 : std_logic_vector(3 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_128 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_129 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_130 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_131 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_132 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_133 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_134 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_135 : std_logic_vector(3 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_136 : std_logic_vector(3 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_137 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_138 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_139 : std_logic_vector(0 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_140 : std_logic_vector(2 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_141 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_142 : std_logic_vector(31 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_143 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_144 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_145 : std_logic_vector(2 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_146 : std_logic_vector(1 downto 0);
  signal zz_zz_decode_BRANCH_CTRL_2_147 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_148 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_149 : std_logic;
  signal zz_zz_decode_BRANCH_CTRL_2_150 : std_logic;
  signal zz_RegFilePlugin_regFile_port : std_logic;
  signal zz_decode_RegFilePlugin_rs1Data : std_logic;
  signal zz_RegFilePlugin_regFile_port_1 : std_logic;
  signal zz_decode_RegFilePlugin_rs2Data : std_logic;
  attribute keep : boolean;
  attribute syn_keep : boolean;

  signal memory_MUL_LOW : signed(51 downto 0);
  signal memory_MEMORY_READ_DATA : std_logic_vector(31 downto 0);
  signal execute_BRANCH_CALC : unsigned(31 downto 0);
  signal execute_BRANCH_DO : std_logic;
  signal execute_SHIFT_RIGHT : std_logic_vector(31 downto 0);
  signal memory_MUL_HH : signed(33 downto 0);
  signal execute_MUL_HH : signed(33 downto 0);
  signal execute_MUL_HL : signed(33 downto 0);
  signal execute_MUL_LH : signed(33 downto 0);
  signal execute_MUL_LL : unsigned(31 downto 0);
  signal writeBack_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal execute_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal memory_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal execute_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal decode_DO_EBREAK : std_logic;
  signal decode_SRC2 : std_logic_vector(31 downto 0);
  signal decode_SRC1 : std_logic_vector(31 downto 0);
  signal decode_SRC2_FORCE_ZERO : std_logic;
  signal decode_BRANCH_CTRL : BranchCtrlEnum_seq_type;
  signal zz_decode_BRANCH_CTRL : BranchCtrlEnum_seq_type;
  signal zz_decode_to_execute_BRANCH_CTRL : BranchCtrlEnum_seq_type;
  signal zz_decode_to_execute_BRANCH_CTRL_1 : BranchCtrlEnum_seq_type;
  signal zz_execute_to_memory_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal zz_execute_to_memory_SHIFT_CTRL_1 : ShiftCtrlEnum_seq_type;
  signal decode_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal zz_decode_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal zz_decode_to_execute_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal zz_decode_to_execute_SHIFT_CTRL_1 : ShiftCtrlEnum_seq_type;
  signal decode_IS_RS2_SIGNED : std_logic;
  signal decode_IS_RS1_SIGNED : std_logic;
  signal decode_IS_DIV : std_logic;
  signal memory_IS_MUL : std_logic;
  signal execute_IS_MUL : std_logic;
  signal decode_IS_MUL : std_logic;
  signal decode_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_seq_type;
  signal zz_decode_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_seq_type;
  signal zz_decode_to_execute_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_seq_type;
  signal zz_decode_to_execute_ALU_BITWISE_CTRL_1 : AluBitwiseCtrlEnum_seq_type;
  signal decode_SRC_LESS_UNSIGNED : std_logic;
  signal zz_memory_to_writeBack_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_memory_to_writeBack_ENV_CTRL_1 : EnvCtrlEnum_seq_type;
  signal zz_execute_to_memory_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_execute_to_memory_ENV_CTRL_1 : EnvCtrlEnum_seq_type;
  signal decode_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_decode_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_decode_to_execute_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_decode_to_execute_ENV_CTRL_1 : EnvCtrlEnum_seq_type;
  signal decode_IS_CSR : std_logic;
  signal decode_MEMORY_STORE : std_logic;
  signal execute_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal decode_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal decode_BYPASSABLE_EXECUTE_STAGE : std_logic;
  signal decode_ALU_CTRL : AluCtrlEnum_seq_type;
  signal zz_decode_ALU_CTRL : AluCtrlEnum_seq_type;
  signal zz_decode_to_execute_ALU_CTRL : AluCtrlEnum_seq_type;
  signal zz_decode_to_execute_ALU_CTRL_1 : AluCtrlEnum_seq_type;
  signal decode_MEMORY_ENABLE : std_logic;
  signal decode_CSR_READ_OPCODE : std_logic;
  signal decode_CSR_WRITE_OPCODE : std_logic;
  signal writeBack_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal memory_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal execute_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal decode_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal memory_PC : unsigned(31 downto 0);
  signal memory_BRANCH_CALC : unsigned(31 downto 0);
  signal memory_BRANCH_DO : std_logic;
  signal execute_BRANCH_CTRL : BranchCtrlEnum_seq_type;
  signal zz_execute_BRANCH_CTRL : BranchCtrlEnum_seq_type;
  signal execute_PC : unsigned(31 downto 0);
  signal execute_DO_EBREAK : std_logic;
  signal decode_IS_EBREAK : std_logic;
  signal decode_RS2_USE : std_logic;
  signal decode_RS1_USE : std_logic;
  signal execute_REGFILE_WRITE_VALID : std_logic;
  signal execute_BYPASSABLE_EXECUTE_STAGE : std_logic;
  signal memory_REGFILE_WRITE_VALID : std_logic;
  signal memory_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal writeBack_REGFILE_WRITE_VALID : std_logic;
  signal decode_RS2 : std_logic_vector(31 downto 0);
  signal decode_RS1 : std_logic_vector(31 downto 0);
  signal memory_SHIFT_RIGHT : std_logic_vector(31 downto 0);
  signal memory_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal zz_memory_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal execute_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal zz_execute_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal execute_SRC_LESS_UNSIGNED : std_logic;
  signal execute_SRC2_FORCE_ZERO : std_logic;
  signal execute_SRC_USE_SUB_LESS : std_logic;
  signal zz_decode_SRC2 : unsigned(31 downto 0);
  signal zz_decode_SRC2_1 : std_logic_vector(31 downto 0);
  signal decode_SRC2_CTRL : Src2CtrlEnum_seq_type;
  signal zz_decode_SRC2_CTRL : Src2CtrlEnum_seq_type;
  signal zz_decode_SRC1 : std_logic_vector(31 downto 0);
  signal decode_SRC1_CTRL : Src1CtrlEnum_seq_type;
  signal zz_decode_SRC1_CTRL : Src1CtrlEnum_seq_type;
  signal decode_SRC_USE_SUB_LESS : std_logic;
  signal decode_SRC_ADD_ZERO : std_logic;
  signal execute_IS_RS1_SIGNED : std_logic;
  signal execute_IS_DIV : std_logic;
  signal execute_IS_RS2_SIGNED : std_logic;
  signal zz_decode_RS2 : std_logic_vector(31 downto 0);
  signal memory_INSTRUCTION : std_logic_vector(31 downto 0);
  signal memory_IS_DIV : std_logic;
  signal writeBack_IS_MUL : std_logic;
  signal writeBack_MUL_HH : signed(33 downto 0);
  signal writeBack_MUL_LOW : signed(51 downto 0);
  signal memory_MUL_HL : signed(33 downto 0);
  signal memory_MUL_LH : signed(33 downto 0);
  signal memory_MUL_LL : unsigned(31 downto 0);
  signal execute_RS1 : std_logic_vector(31 downto 0);
  attribute keep of execute_RS1 : signal is true;
  attribute syn_keep of execute_RS1 : signal is true;
  signal execute_SRC_ADD_SUB : std_logic_vector(31 downto 0);
  signal execute_SRC_LESS : std_logic;
  signal execute_ALU_CTRL : AluCtrlEnum_seq_type;
  signal zz_execute_ALU_CTRL : AluCtrlEnum_seq_type;
  signal execute_SRC2 : std_logic_vector(31 downto 0);
  signal execute_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_seq_type;
  signal zz_execute_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_seq_type;
  signal zz_lastStageRegFileWrite_payload_address : std_logic_vector(31 downto 0);
  signal zz_lastStageRegFileWrite_valid : std_logic;
  signal zz_1 : std_logic;
  signal decode_INSTRUCTION_ANTICIPATED : std_logic_vector(31 downto 0);
  signal decode_REGFILE_WRITE_VALID : std_logic;
  signal decode_LEGAL_INSTRUCTION : std_logic;
  signal zz_decode_BRANCH_CTRL_1 : BranchCtrlEnum_seq_type;
  signal zz_decode_SHIFT_CTRL_1 : ShiftCtrlEnum_seq_type;
  signal zz_decode_ALU_BITWISE_CTRL_1 : AluBitwiseCtrlEnum_seq_type;
  signal zz_decode_ENV_CTRL_1 : EnvCtrlEnum_seq_type;
  signal zz_decode_SRC2_CTRL_1 : Src2CtrlEnum_seq_type;
  signal zz_decode_ALU_CTRL_1 : AluCtrlEnum_seq_type;
  signal zz_decode_SRC1_CTRL_1 : Src1CtrlEnum_seq_type;
  signal zz_decode_RS2_1 : std_logic_vector(31 downto 0);
  signal execute_SRC1 : std_logic_vector(31 downto 0);
  signal execute_CSR_READ_OPCODE : std_logic;
  signal execute_CSR_WRITE_OPCODE : std_logic;
  signal execute_IS_CSR : std_logic;
  signal memory_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_memory_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal execute_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_execute_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal writeBack_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal zz_writeBack_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal writeBack_MEMORY_STORE : std_logic;
  signal zz_decode_RS2_2 : std_logic_vector(31 downto 0);
  signal writeBack_MEMORY_ENABLE : std_logic;
  signal writeBack_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal writeBack_MEMORY_READ_DATA : std_logic_vector(31 downto 0);
  signal memory_ALIGNEMENT_FAULT : std_logic;
  signal memory_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal memory_MEMORY_STORE : std_logic;
  signal memory_MEMORY_ENABLE : std_logic;
  signal execute_SRC_ADD : std_logic_vector(31 downto 0);
  signal execute_RS2 : std_logic_vector(31 downto 0);
  attribute keep of execute_RS2 : signal is true;
  attribute syn_keep of execute_RS2 : signal is true;
  signal execute_INSTRUCTION : std_logic_vector(31 downto 0);
  signal execute_MEMORY_STORE : std_logic;
  signal execute_MEMORY_ENABLE : std_logic;
  signal execute_ALIGNEMENT_FAULT : std_logic;
  signal zz_memory_to_writeBack_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal decode_PC : unsigned(31 downto 0);
  signal decode_INSTRUCTION : std_logic_vector(31 downto 0);
  signal decode_IS_RVC : std_logic;
  signal writeBack_PC : unsigned(31 downto 0);
  signal writeBack_INSTRUCTION : std_logic_vector(31 downto 0);
  signal decode_arbitration_haltItself : std_logic;
  signal decode_arbitration_haltByOther : std_logic;
  signal decode_arbitration_removeIt : std_logic;
  signal decode_arbitration_flushIt : std_logic;
  signal decode_arbitration_flushNext : std_logic;
  signal decode_arbitration_isValid : std_logic;
  signal decode_arbitration_isStuck : std_logic;
  signal decode_arbitration_isStuckByOthers : std_logic;
  signal decode_arbitration_isFlushed : std_logic;
  signal decode_arbitration_isMoving : std_logic;
  signal decode_arbitration_isFiring : std_logic;
  signal execute_arbitration_haltItself : std_logic;
  signal execute_arbitration_haltByOther : std_logic;
  signal execute_arbitration_removeIt : std_logic;
  signal execute_arbitration_flushIt : std_logic;
  signal execute_arbitration_flushNext : std_logic;
  signal execute_arbitration_isValid : std_logic;
  signal execute_arbitration_isStuck : std_logic;
  signal execute_arbitration_isStuckByOthers : std_logic;
  signal execute_arbitration_isFlushed : std_logic;
  signal execute_arbitration_isMoving : std_logic;
  signal execute_arbitration_isFiring : std_logic;
  signal memory_arbitration_haltItself : std_logic;
  signal memory_arbitration_haltByOther : std_logic;
  signal memory_arbitration_removeIt : std_logic;
  signal memory_arbitration_flushIt : std_logic;
  signal memory_arbitration_flushNext : std_logic;
  signal memory_arbitration_isValid : std_logic;
  signal memory_arbitration_isStuck : std_logic;
  signal memory_arbitration_isStuckByOthers : std_logic;
  signal memory_arbitration_isFlushed : std_logic;
  signal memory_arbitration_isMoving : std_logic;
  signal memory_arbitration_isFiring : std_logic;
  signal writeBack_arbitration_haltItself : std_logic;
  signal writeBack_arbitration_haltByOther : std_logic;
  signal writeBack_arbitration_removeIt : std_logic;
  signal writeBack_arbitration_flushIt : std_logic;
  signal writeBack_arbitration_flushNext : std_logic;
  signal writeBack_arbitration_isValid : std_logic;
  signal writeBack_arbitration_isStuck : std_logic;
  signal writeBack_arbitration_isStuckByOthers : std_logic;
  signal writeBack_arbitration_isFlushed : std_logic;
  signal writeBack_arbitration_isMoving : std_logic;
  signal writeBack_arbitration_isFiring : std_logic;
  signal lastStageInstruction : std_logic_vector(31 downto 0);
  signal lastStagePc : unsigned(31 downto 0);
  signal lastStageIsValid : std_logic;
  signal lastStageIsFiring : std_logic;
  signal IBusSimplePlugin_fetcherHalt : std_logic;
  signal IBusSimplePlugin_incomingInstruction : std_logic;
  signal IBusSimplePlugin_pcValids_0 : std_logic;
  signal IBusSimplePlugin_pcValids_1 : std_logic;
  signal IBusSimplePlugin_pcValids_2 : std_logic;
  signal IBusSimplePlugin_pcValids_3 : std_logic;
  signal iBus_cmd_valid : std_logic;
  signal iBus_cmd_ready : std_logic;
  signal iBus_cmd_payload_pc : unsigned(31 downto 0);
  signal iBus_rsp_valid : std_logic;
  signal iBus_rsp_payload_error : std_logic;
  signal iBus_rsp_payload_inst : std_logic_vector(31 downto 0);
  signal DBusSimplePlugin_memoryExceptionPort_valid : std_logic;
  signal DBusSimplePlugin_memoryExceptionPort_payload_code : unsigned(3 downto 0);
  signal DBusSimplePlugin_memoryExceptionPort_payload_badAddr : unsigned(31 downto 0);
  signal CsrPlugin_csrMapping_readDataSignal : std_logic_vector(31 downto 0);
  signal CsrPlugin_csrMapping_readDataInit : std_logic_vector(31 downto 0);
  signal CsrPlugin_csrMapping_writeDataSignal : std_logic_vector(31 downto 0);
  signal CsrPlugin_csrMapping_allowCsrSignal : std_logic;
  signal CsrPlugin_csrMapping_hazardFree : std_logic;
  signal CsrPlugin_inWfi : std_logic;
  signal CsrPlugin_thirdPartyWake : std_logic;
  signal CsrPlugin_jumpInterface_valid : std_logic;
  signal CsrPlugin_jumpInterface_payload : unsigned(31 downto 0);
  signal CsrPlugin_exceptionPendings_0 : std_logic;
  signal CsrPlugin_exceptionPendings_1 : std_logic;
  signal CsrPlugin_exceptionPendings_2 : std_logic;
  signal CsrPlugin_exceptionPendings_3 : std_logic;
  signal contextSwitching : std_logic;
  signal CsrPlugin_privilege : unsigned(1 downto 0);
  signal CsrPlugin_forceMachineWire : std_logic;
  signal CsrPlugin_selfException_valid : std_logic;
  signal CsrPlugin_selfException_payload_code : unsigned(3 downto 0);
  signal CsrPlugin_selfException_payload_badAddr : unsigned(31 downto 0);
  signal CsrPlugin_allowInterrupts : std_logic;
  signal CsrPlugin_allowException : std_logic;
  signal CsrPlugin_allowEbreakException : std_logic;
  signal decodeExceptionPort_valid : std_logic;
  signal decodeExceptionPort_payload_code : unsigned(3 downto 0);
  signal decodeExceptionPort_payload_badAddr : unsigned(31 downto 0);
  signal IBusSimplePlugin_injectionPort_valid : std_logic;
  signal IBusSimplePlugin_injectionPort_ready : std_logic;
  signal IBusSimplePlugin_injectionPort_payload : std_logic_vector(31 downto 0);
  signal BranchPlugin_jumpInterface_valid : std_logic;
  signal BranchPlugin_jumpInterface_payload : unsigned(31 downto 0);
  signal LocalInt0_regNext : std_logic;
  signal LocalInt0_enable : std_logic;
  signal zz_when_CsrPlugin_l952 : std_logic;
  signal LocalInt1_regNext : std_logic;
  signal LocalInt1_enable : std_logic;
  signal zz_when_CsrPlugin_l952_1 : std_logic;
  signal LocalInt2_regNext : std_logic;
  signal LocalInt2_enable : std_logic;
  signal zz_when_CsrPlugin_l952_2 : std_logic;
  signal LocalInt3_regNext : std_logic;
  signal LocalInt3_enable : std_logic;
  signal zz_when_CsrPlugin_l952_3 : std_logic;
  signal IBusSimplePlugin_externalFlush : std_logic;
  signal IBusSimplePlugin_jump_pcLoad_valid : std_logic;
  signal IBusSimplePlugin_jump_pcLoad_payload : unsigned(31 downto 0);
  signal zz_IBusSimplePlugin_jump_pcLoad_payload : unsigned(1 downto 0);
  signal IBusSimplePlugin_fetchPc_output_valid : std_logic;
  signal IBusSimplePlugin_fetchPc_output_ready : std_logic;
  signal IBusSimplePlugin_fetchPc_output_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_fetchPc_pcReg : unsigned(31 downto 0);
  signal IBusSimplePlugin_fetchPc_correction : std_logic;
  signal IBusSimplePlugin_fetchPc_correctionReg : std_logic;
  signal IBusSimplePlugin_fetchPc_output_fire : std_logic;
  signal IBusSimplePlugin_fetchPc_corrected : std_logic;
  signal IBusSimplePlugin_fetchPc_pcRegPropagate : std_logic;
  signal IBusSimplePlugin_fetchPc_booted : std_logic;
  signal IBusSimplePlugin_fetchPc_inc : std_logic;
  signal when_Fetcher_l131 : std_logic;
  signal IBusSimplePlugin_fetchPc_output_fire_1 : std_logic;
  signal when_Fetcher_l131_1 : std_logic;
  signal IBusSimplePlugin_fetchPc_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_fetchPc_flushed : std_logic;
  signal when_Fetcher_l158 : std_logic;
  signal IBusSimplePlugin_decodePc_flushed : std_logic;
  signal IBusSimplePlugin_decodePc_pcReg : unsigned(31 downto 0);
  signal IBusSimplePlugin_decodePc_pcPlus : unsigned(31 downto 0);
  signal IBusSimplePlugin_decodePc_injectedDecode : std_logic;
  signal when_Fetcher_l180 : std_logic;
  signal when_Fetcher_l192 : std_logic;
  signal IBusSimplePlugin_iBusRsp_redoFetch : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_input_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_input_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_input_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_0_output_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_output_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_output_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_0_halt : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_input_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_input_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_input_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_1_output_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_output_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_output_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_1_halt : std_logic;
  signal zz_IBusSimplePlugin_iBusRsp_stages_0_input_ready : std_logic;
  signal zz_IBusSimplePlugin_iBusRsp_stages_1_input_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_flush : std_logic;
  signal zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready : std_logic;
  signal zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_1 : std_logic;
  signal zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_2 : std_logic;
  signal IBusSimplePlugin_iBusRsp_readyForError : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_output_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_output_payload_isRvc : std_logic;
  signal IBusSimplePlugin_decompressor_input_valid : std_logic;
  signal IBusSimplePlugin_decompressor_input_ready : std_logic;
  signal IBusSimplePlugin_decompressor_input_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_decompressor_input_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_decompressor_input_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_decompressor_input_payload_isRvc : std_logic;
  signal IBusSimplePlugin_decompressor_output_valid : std_logic;
  signal IBusSimplePlugin_decompressor_output_ready : std_logic;
  signal IBusSimplePlugin_decompressor_output_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_decompressor_output_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_decompressor_output_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_decompressor_output_payload_isRvc : std_logic;
  signal IBusSimplePlugin_decompressor_flushNext : std_logic;
  signal IBusSimplePlugin_decompressor_consumeCurrent : std_logic;
  signal IBusSimplePlugin_decompressor_bufferValid : std_logic;
  signal IBusSimplePlugin_decompressor_bufferData : std_logic_vector(15 downto 0);
  signal IBusSimplePlugin_decompressor_isInputLowRvc : std_logic;
  signal IBusSimplePlugin_decompressor_isInputHighRvc : std_logic;
  signal IBusSimplePlugin_decompressor_throw2BytesReg : std_logic;
  signal IBusSimplePlugin_decompressor_throw2Bytes : std_logic;
  signal IBusSimplePlugin_decompressor_unaligned : std_logic;
  signal IBusSimplePlugin_decompressor_bufferValidLatch : std_logic;
  signal IBusSimplePlugin_decompressor_throw2BytesLatch : std_logic;
  signal IBusSimplePlugin_decompressor_bufferValidPatched : std_logic;
  signal IBusSimplePlugin_decompressor_throw2BytesPatched : std_logic;
  signal IBusSimplePlugin_decompressor_raw : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_decompressor_isRvc : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed : std_logic_vector(15 downto 0);
  signal IBusSimplePlugin_decompressor_decompressed : std_logic_vector(31 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_1 : std_logic_vector(4 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_2 : std_logic_vector(4 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_3 : std_logic_vector(11 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_4 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_5 : std_logic_vector(11 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_6 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_7 : std_logic_vector(9 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_8 : std_logic_vector(20 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_9 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_10 : std_logic_vector(14 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_11 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_12 : std_logic_vector(2 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_13 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_14 : std_logic_vector(9 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_15 : std_logic_vector(20 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_16 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_17 : std_logic_vector(4 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_18 : std_logic_vector(12 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_19 : std_logic_vector(4 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_20 : std_logic_vector(4 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_21 : std_logic_vector(4 downto 0);
  signal switch_Misc_l44 : std_logic_vector(4 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_22 : std_logic;
  signal switch_Misc_l204 : std_logic_vector(1 downto 0);
  signal switch_Misc_l204_1 : std_logic_vector(1 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_23 : std_logic_vector(2 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_24 : std_logic_vector(2 downto 0);
  signal zz_IBusSimplePlugin_decompressor_decompressed_25 : std_logic;
  signal zz_IBusSimplePlugin_decompressor_decompressed_26 : std_logic_vector(6 downto 0);
  signal IBusSimplePlugin_decompressor_output_fire : std_logic;
  signal IBusSimplePlugin_decompressor_bufferFill : std_logic;
  signal when_Fetcher_l283 : std_logic;
  signal when_Fetcher_l286 : std_logic;
  signal when_Fetcher_l291 : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_valid : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_ready : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_injector_decodeInput_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_injector_decodeInput_payload_isRvc : std_logic;
  signal zz_IBusSimplePlugin_injector_decodeInput_valid : std_logic;
  signal zz_IBusSimplePlugin_injector_decodeInput_payload_pc : unsigned(31 downto 0);
  signal zz_IBusSimplePlugin_injector_decodeInput_payload_rsp_error : std_logic;
  signal zz_IBusSimplePlugin_injector_decodeInput_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal zz_IBusSimplePlugin_injector_decodeInput_payload_isRvc : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_0 : std_logic;
  signal when_Fetcher_l329 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_1 : std_logic;
  signal when_Fetcher_l329_1 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_2 : std_logic;
  signal when_Fetcher_l329_2 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_3 : std_logic;
  signal when_Fetcher_l329_3 : std_logic;
  signal IBusSimplePlugin_injector_formal_rawInDecode : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_cmd_valid : std_logic;
  signal IBusSimplePlugin_cmd_ready : std_logic;
  signal IBusSimplePlugin_cmd_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_pending_inc : std_logic;
  signal IBusSimplePlugin_pending_dec : std_logic;
  signal IBusSimplePlugin_pending_value : unsigned(2 downto 0);
  signal IBusSimplePlugin_pending_next : unsigned(2 downto 0);
  signal IBusSimplePlugin_cmdFork_canEmit : std_logic;
  signal when_IBusSimplePlugin_l305 : std_logic;
  signal IBusSimplePlugin_cmd_fire : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_output_valid : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_output_ready : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_rspBuffer_discardCounter : unsigned(2 downto 0);
  signal IBusSimplePlugin_rspJoin_rspBuffer_flush : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_fire : std_logic;
  signal IBusSimplePlugin_rspJoin_fetchRsp_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_rspJoin_fetchRsp_rsp_error : std_logic;
  signal IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_fetchRsp_isRvc : std_logic;
  signal when_IBusSimplePlugin_l376 : std_logic;
  signal IBusSimplePlugin_rspJoin_join_valid : std_logic;
  signal IBusSimplePlugin_rspJoin_join_ready : std_logic;
  signal IBusSimplePlugin_rspJoin_join_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_rspJoin_join_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_rspJoin_join_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_join_payload_isRvc : std_logic;
  signal IBusSimplePlugin_rspJoin_exceptionDetected : std_logic;
  signal IBusSimplePlugin_rspJoin_join_fire : std_logic;
  signal IBusSimplePlugin_rspJoin_join_fire_1 : std_logic;
  signal zz_IBusSimplePlugin_iBusRsp_output_valid : std_logic;
  signal dBus_cmd_valid : std_logic;
  signal dBus_cmd_ready : std_logic;
  signal dBus_cmd_payload_wr : std_logic;
  signal dBus_cmd_payload_address : unsigned(31 downto 0);
  signal dBus_cmd_payload_data : std_logic_vector(31 downto 0);
  signal dBus_cmd_payload_size : unsigned(1 downto 0);
  signal dBus_rsp_ready : std_logic;
  signal dBus_rsp_error : std_logic;
  signal dBus_rsp_data : std_logic_vector(31 downto 0);
  signal zz_dBus_cmd_valid : std_logic;
  signal execute_DBusSimplePlugin_skipCmd : std_logic;
  signal zz_dBus_cmd_payload_data : std_logic_vector(31 downto 0);
  signal when_DBusSimplePlugin_l428 : std_logic;
  signal zz_execute_DBusSimplePlugin_formalMask : std_logic_vector(3 downto 0);
  signal execute_DBusSimplePlugin_formalMask : std_logic_vector(3 downto 0);
  signal when_DBusSimplePlugin_l481 : std_logic;
  signal when_DBusSimplePlugin_l514 : std_logic;
  signal writeBack_DBusSimplePlugin_rspShifted : std_logic_vector(31 downto 0);
  signal switch_Misc_l204_2 : std_logic_vector(1 downto 0);
  signal zz_writeBack_DBusSimplePlugin_rspFormated : std_logic;
  signal zz_writeBack_DBusSimplePlugin_rspFormated_1 : std_logic_vector(31 downto 0);
  signal zz_writeBack_DBusSimplePlugin_rspFormated_2 : std_logic;
  signal zz_writeBack_DBusSimplePlugin_rspFormated_3 : std_logic_vector(31 downto 0);
  signal writeBack_DBusSimplePlugin_rspFormated : std_logic_vector(31 downto 0);
  signal when_DBusSimplePlugin_l560 : std_logic;
  signal CsrPlugin_misa_base : unsigned(1 downto 0);
  signal CsrPlugin_misa_extensions : std_logic_vector(25 downto 0);
  signal CsrPlugin_mtvec_mode : std_logic_vector(1 downto 0);
  signal CsrPlugin_mtvec_base : unsigned(29 downto 0);
  signal CsrPlugin_mepc : unsigned(31 downto 0);
  signal CsrPlugin_mstatus_MIE : std_logic;
  signal CsrPlugin_mstatus_MPIE : std_logic;
  signal CsrPlugin_mstatus_MPP : unsigned(1 downto 0);
  signal CsrPlugin_mip_MEIP : std_logic;
  signal CsrPlugin_mip_MTIP : std_logic;
  signal CsrPlugin_mip_MSIP : std_logic;
  signal CsrPlugin_mie_MEIE : std_logic;
  signal CsrPlugin_mie_MTIE : std_logic;
  signal CsrPlugin_mie_MSIE : std_logic;
  signal CsrPlugin_mcause_interrupt : std_logic;
  signal CsrPlugin_mcause_exceptionCode : unsigned(4 downto 0);
  signal CsrPlugin_mtval : unsigned(31 downto 0);
  signal CsrPlugin_mcycle : unsigned(63 downto 0) := "0000000000000000000000000000000000000000000000000000000000000000";
  signal CsrPlugin_minstret : unsigned(63 downto 0) := "0000000000000000000000000000000000000000000000000000000000000000";
  signal zz_when_CsrPlugin_l952_4 : std_logic;
  signal zz_when_CsrPlugin_l952_5 : std_logic;
  signal zz_when_CsrPlugin_l952_6 : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValids_decode : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValids_execute : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValids_memory : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack : std_logic;
  signal CsrPlugin_exceptionPortCtrl_exceptionContext_code : unsigned(3 downto 0);
  signal CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr : unsigned(31 downto 0);
  signal CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped : unsigned(1 downto 0);
  signal CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege : unsigned(1 downto 0);
  signal when_CsrPlugin_l909 : std_logic;
  signal when_CsrPlugin_l909_1 : std_logic;
  signal when_CsrPlugin_l909_2 : std_logic;
  signal when_CsrPlugin_l909_3 : std_logic;
  signal when_CsrPlugin_l922 : std_logic;
  signal CsrPlugin_interrupt_valid : std_logic;
  signal CsrPlugin_interrupt_code : unsigned(4 downto 0);
  signal CsrPlugin_interrupt_targetPrivilege : unsigned(1 downto 0);
  signal when_CsrPlugin_l946 : std_logic;
  signal when_CsrPlugin_l952 : std_logic;
  signal when_CsrPlugin_l952_1 : std_logic;
  signal when_CsrPlugin_l952_2 : std_logic;
  signal when_CsrPlugin_l952_3 : std_logic;
  signal when_CsrPlugin_l952_4 : std_logic;
  signal when_CsrPlugin_l952_5 : std_logic;
  signal when_CsrPlugin_l952_6 : std_logic;
  signal CsrPlugin_exception : std_logic;
  signal CsrPlugin_lastStageWasWfi : std_logic;
  signal CsrPlugin_pipelineLiberator_pcValids_0 : std_logic;
  signal CsrPlugin_pipelineLiberator_pcValids_1 : std_logic;
  signal CsrPlugin_pipelineLiberator_pcValids_2 : std_logic;
  signal CsrPlugin_pipelineLiberator_active : std_logic;
  signal when_CsrPlugin_l980 : std_logic;
  signal when_CsrPlugin_l980_1 : std_logic;
  signal when_CsrPlugin_l980_2 : std_logic;
  signal when_CsrPlugin_l985 : std_logic;
  signal CsrPlugin_pipelineLiberator_done : std_logic;
  signal when_CsrPlugin_l991 : std_logic;
  signal CsrPlugin_interruptJump : std_logic;
  signal CsrPlugin_hadException : std_logic;
  signal CsrPlugin_targetPrivilege : unsigned(1 downto 0);
  signal CsrPlugin_trapCause : unsigned(4 downto 0);
  signal CsrPlugin_xtvec_mode : std_logic_vector(1 downto 0);
  signal CsrPlugin_xtvec_base : unsigned(29 downto 0);
  signal when_CsrPlugin_l1019 : std_logic;
  signal when_CsrPlugin_l1064 : std_logic;
  signal switch_CsrPlugin_l1068 : std_logic_vector(1 downto 0);
  signal execute_CsrPlugin_wfiWake : std_logic;
  signal when_CsrPlugin_l1116 : std_logic;
  signal execute_CsrPlugin_blockedBySideEffects : std_logic;
  signal execute_CsrPlugin_illegalAccess : std_logic;
  signal execute_CsrPlugin_illegalInstruction : std_logic;
  signal when_CsrPlugin_l1136 : std_logic;
  signal when_CsrPlugin_l1137 : std_logic;
  signal when_CsrPlugin_l1144 : std_logic;
  signal execute_CsrPlugin_writeInstruction : std_logic;
  signal execute_CsrPlugin_readInstruction : std_logic;
  signal execute_CsrPlugin_writeEnable : std_logic;
  signal execute_CsrPlugin_readEnable : std_logic;
  signal execute_CsrPlugin_readToWriteData : std_logic_vector(31 downto 0);
  signal switch_Misc_l204_3 : std_logic;
  signal zz_CsrPlugin_csrMapping_writeDataSignal : std_logic_vector(31 downto 0);
  signal when_CsrPlugin_l1176 : std_logic;
  signal when_CsrPlugin_l1180 : std_logic;
  signal execute_CsrPlugin_csrAddress : std_logic_vector(11 downto 0);
  signal zz_decode_BRANCH_CTRL_2 : std_logic_vector(30 downto 0);
  signal zz_decode_BRANCH_CTRL_3 : std_logic;
  signal zz_decode_BRANCH_CTRL_4 : std_logic;
  signal zz_decode_BRANCH_CTRL_5 : std_logic;
  signal zz_decode_BRANCH_CTRL_6 : std_logic;
  signal zz_decode_BRANCH_CTRL_7 : std_logic;
  signal zz_decode_SRC1_CTRL_2 : Src1CtrlEnum_seq_type;
  signal zz_decode_ALU_CTRL_2 : AluCtrlEnum_seq_type;
  signal zz_decode_SRC2_CTRL_2 : Src2CtrlEnum_seq_type;
  signal zz_decode_ENV_CTRL_2 : EnvCtrlEnum_seq_type;
  signal zz_decode_ALU_BITWISE_CTRL_2 : AluBitwiseCtrlEnum_seq_type;
  signal zz_decode_SHIFT_CTRL_2 : ShiftCtrlEnum_seq_type;
  signal zz_decode_BRANCH_CTRL_8 : BranchCtrlEnum_seq_type;
  signal when_RegFilePlugin_l63 : std_logic;
  signal decode_RegFilePlugin_regFileReadAddress1 : unsigned(4 downto 0);
  signal decode_RegFilePlugin_regFileReadAddress2 : unsigned(4 downto 0);
  signal decode_RegFilePlugin_rs1Data : std_logic_vector(31 downto 0);
  signal decode_RegFilePlugin_rs2Data : std_logic_vector(31 downto 0);
  signal lastStageRegFileWrite_valid : std_logic;
  signal lastStageRegFileWrite_payload_address : unsigned(4 downto 0);
  signal lastStageRegFileWrite_payload_data : std_logic_vector(31 downto 0);
  signal zz_2 : std_logic;
  signal execute_IntAluPlugin_bitwise : std_logic_vector(31 downto 0);
  signal zz_execute_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal execute_MulPlugin_aSigned : std_logic;
  signal execute_MulPlugin_bSigned : std_logic;
  signal execute_MulPlugin_a : std_logic_vector(31 downto 0);
  signal execute_MulPlugin_b : std_logic_vector(31 downto 0);
  signal switch_MulPlugin_l87 : std_logic_vector(1 downto 0);
  signal execute_MulPlugin_aULow : unsigned(15 downto 0);
  signal execute_MulPlugin_bULow : unsigned(15 downto 0);
  signal execute_MulPlugin_aSLow : signed(16 downto 0);
  signal execute_MulPlugin_bSLow : signed(16 downto 0);
  signal execute_MulPlugin_aHigh : signed(16 downto 0);
  signal execute_MulPlugin_bHigh : signed(16 downto 0);
  signal writeBack_MulPlugin_result : signed(65 downto 0);
  signal when_MulPlugin_l147 : std_logic;
  signal switch_MulPlugin_l148 : std_logic_vector(1 downto 0);
  signal memory_DivPlugin_rs1 : unsigned(32 downto 0);
  signal memory_DivPlugin_rs2 : unsigned(31 downto 0);
  signal memory_DivPlugin_accumulator : unsigned(64 downto 0);
  signal memory_DivPlugin_frontendOk : std_logic;
  signal memory_DivPlugin_div_needRevert : std_logic;
  signal memory_DivPlugin_div_counter_willIncrement : std_logic;
  signal memory_DivPlugin_div_counter_willClear : std_logic;
  signal memory_DivPlugin_div_counter_valueNext : unsigned(5 downto 0);
  signal memory_DivPlugin_div_counter_value : unsigned(5 downto 0);
  signal memory_DivPlugin_div_counter_willOverflowIfInc : std_logic;
  signal memory_DivPlugin_div_counter_willOverflow : std_logic;
  signal memory_DivPlugin_div_done : std_logic;
  signal when_MulDivIterativePlugin_l126 : std_logic;
  signal when_MulDivIterativePlugin_l126_1 : std_logic;
  signal memory_DivPlugin_div_result : std_logic_vector(31 downto 0);
  signal when_MulDivIterativePlugin_l128 : std_logic;
  signal when_MulDivIterativePlugin_l129 : std_logic;
  signal when_MulDivIterativePlugin_l132 : std_logic;
  signal zz_memory_DivPlugin_div_stage_0_remainderShifted : unsigned(31 downto 0);
  signal memory_DivPlugin_div_stage_0_remainderShifted : unsigned(32 downto 0);
  signal memory_DivPlugin_div_stage_0_remainderMinusDenominator : unsigned(32 downto 0);
  signal memory_DivPlugin_div_stage_0_outRemainder : unsigned(31 downto 0);
  signal memory_DivPlugin_div_stage_0_outNumerator : unsigned(31 downto 0);
  signal when_MulDivIterativePlugin_l151 : std_logic;
  signal zz_memory_DivPlugin_div_result : unsigned(31 downto 0);
  signal when_MulDivIterativePlugin_l162 : std_logic;
  signal zz_memory_DivPlugin_rs2 : std_logic;
  signal zz_memory_DivPlugin_rs1 : std_logic;
  signal zz_memory_DivPlugin_rs1_1 : std_logic_vector(32 downto 0);
  signal zz_decode_SRC1_1 : std_logic_vector(31 downto 0);
  signal zz_decode_SRC2_2 : std_logic;
  signal zz_decode_SRC2_3 : std_logic_vector(19 downto 0);
  signal zz_decode_SRC2_4 : std_logic;
  signal zz_decode_SRC2_5 : std_logic_vector(19 downto 0);
  signal zz_decode_SRC2_6 : std_logic_vector(31 downto 0);
  signal execute_SrcPlugin_addSub : std_logic_vector(31 downto 0);
  signal execute_SrcPlugin_less : std_logic;
  signal execute_FullBarrelShifterPlugin_amplitude : unsigned(4 downto 0);
  signal zz_execute_FullBarrelShifterPlugin_reversed : std_logic_vector(31 downto 0);
  signal execute_FullBarrelShifterPlugin_reversed : std_logic_vector(31 downto 0);
  signal zz_decode_RS2_3 : std_logic_vector(31 downto 0);
  signal HazardSimplePlugin_src0Hazard : std_logic;
  signal HazardSimplePlugin_src1Hazard : std_logic;
  signal HazardSimplePlugin_writeBackWrites_valid : std_logic;
  signal HazardSimplePlugin_writeBackWrites_payload_address : std_logic_vector(4 downto 0);
  signal HazardSimplePlugin_writeBackWrites_payload_data : std_logic_vector(31 downto 0);
  signal HazardSimplePlugin_writeBackBuffer_valid : std_logic;
  signal HazardSimplePlugin_writeBackBuffer_payload_address : std_logic_vector(4 downto 0);
  signal HazardSimplePlugin_writeBackBuffer_payload_data : std_logic_vector(31 downto 0);
  signal HazardSimplePlugin_addr0Match : std_logic;
  signal HazardSimplePlugin_addr1Match : std_logic;
  signal when_HazardSimplePlugin_l47 : std_logic;
  signal when_HazardSimplePlugin_l48 : std_logic;
  signal when_HazardSimplePlugin_l51 : std_logic;
  signal when_HazardSimplePlugin_l45 : std_logic;
  signal when_HazardSimplePlugin_l57 : std_logic;
  signal when_HazardSimplePlugin_l58 : std_logic;
  signal when_HazardSimplePlugin_l48_1 : std_logic;
  signal when_HazardSimplePlugin_l51_1 : std_logic;
  signal when_HazardSimplePlugin_l45_1 : std_logic;
  signal when_HazardSimplePlugin_l57_1 : std_logic;
  signal when_HazardSimplePlugin_l58_1 : std_logic;
  signal when_HazardSimplePlugin_l48_2 : std_logic;
  signal when_HazardSimplePlugin_l51_2 : std_logic;
  signal when_HazardSimplePlugin_l45_2 : std_logic;
  signal when_HazardSimplePlugin_l57_2 : std_logic;
  signal when_HazardSimplePlugin_l58_2 : std_logic;
  signal when_HazardSimplePlugin_l105 : std_logic;
  signal when_HazardSimplePlugin_l108 : std_logic;
  signal when_HazardSimplePlugin_l113 : std_logic;
  signal DebugPlugin_firstCycle : std_logic;
  signal DebugPlugin_secondCycle : std_logic;
  signal DebugPlugin_resetIt : std_logic;
  signal DebugPlugin_haltIt : std_logic;
  signal DebugPlugin_stepIt : std_logic;
  signal DebugPlugin_isPipBusy : std_logic;
  signal DebugPlugin_godmode : std_logic;
  signal when_DebugPlugin_l225 : std_logic;
  signal DebugPlugin_haltedByBreak : std_logic;
  signal DebugPlugin_debugUsed : std_logic;
  signal DebugPlugin_disableEbreak : std_logic;
  signal DebugPlugin_allowEBreak : std_logic;
  signal DebugPlugin_hardwareBreakpoints_0_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_0_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_1_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_1_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_2_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_2_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_3_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_3_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_4_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_4_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_5_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_5_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_6_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_6_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_7_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_7_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_8_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_8_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_9_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_9_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_10_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_10_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_11_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_11_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_12_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_12_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_13_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_13_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_14_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_14_pc : unsigned(30 downto 0);
  signal DebugPlugin_hardwareBreakpoints_15_valid : std_logic;
  signal DebugPlugin_hardwareBreakpoints_15_pc : unsigned(30 downto 0);
  signal DebugPlugin_busReadDataReg : std_logic_vector(31 downto 0);
  signal zz_when_DebugPlugin_l244 : std_logic;
  signal when_DebugPlugin_l244 : std_logic;
  signal zz_3 : unsigned(5 downto 0);
  signal switch_DebugPlugin_l268 : unsigned(5 downto 0);
  signal when_DebugPlugin_l272 : std_logic;
  signal when_DebugPlugin_l272_1 : std_logic;
  signal when_DebugPlugin_l273 : std_logic;
  signal when_DebugPlugin_l273_1 : std_logic;
  signal when_DebugPlugin_l274 : std_logic;
  signal when_DebugPlugin_l275 : std_logic;
  signal when_DebugPlugin_l276 : std_logic;
  signal when_DebugPlugin_l276_1 : std_logic;
  signal when_DebugPlugin_l296 : std_logic;
  signal when_DebugPlugin_l299 : std_logic;
  signal when_DebugPlugin_l312 : std_logic;
  signal zz_4 : std_logic;
  signal DebugPlugin_resetIt_regNext : std_logic;
  signal when_DebugPlugin_l328 : std_logic;
  signal execute_BranchPlugin_eq : std_logic;
  signal switch_Misc_l204_4 : std_logic_vector(2 downto 0);
  signal zz_execute_BRANCH_DO : std_logic;
  signal zz_execute_BRANCH_DO_1 : std_logic;
  signal execute_BranchPlugin_branch_src1 : unsigned(31 downto 0);
  signal zz_execute_BranchPlugin_branch_src2 : std_logic;
  signal zz_execute_BranchPlugin_branch_src2_1 : std_logic_vector(10 downto 0);
  signal zz_execute_BranchPlugin_branch_src2_2 : std_logic;
  signal zz_execute_BranchPlugin_branch_src2_3 : std_logic_vector(19 downto 0);
  signal zz_execute_BranchPlugin_branch_src2_4 : std_logic;
  signal zz_execute_BranchPlugin_branch_src2_5 : std_logic_vector(18 downto 0);
  signal zz_execute_BranchPlugin_branch_src2_6 : std_logic_vector(31 downto 0);
  signal execute_BranchPlugin_branch_src2 : unsigned(31 downto 0);
  signal execute_BranchPlugin_branchAdder : unsigned(31 downto 0);
  signal when_Pipeline_l124 : std_logic;
  signal decode_to_execute_PC : unsigned(31 downto 0);
  signal when_Pipeline_l124_1 : std_logic;
  signal execute_to_memory_PC : unsigned(31 downto 0);
  signal when_Pipeline_l124_2 : std_logic;
  signal memory_to_writeBack_PC : unsigned(31 downto 0);
  signal when_Pipeline_l124_3 : std_logic;
  signal decode_to_execute_INSTRUCTION : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_4 : std_logic;
  signal execute_to_memory_INSTRUCTION : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_5 : std_logic;
  signal memory_to_writeBack_INSTRUCTION : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_6 : std_logic;
  signal decode_to_execute_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal when_Pipeline_l124_7 : std_logic;
  signal execute_to_memory_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal when_Pipeline_l124_8 : std_logic;
  signal memory_to_writeBack_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal when_Pipeline_l124_9 : std_logic;
  signal decode_to_execute_CSR_WRITE_OPCODE : std_logic;
  signal when_Pipeline_l124_10 : std_logic;
  signal decode_to_execute_CSR_READ_OPCODE : std_logic;
  signal when_Pipeline_l124_11 : std_logic;
  signal decode_to_execute_SRC_USE_SUB_LESS : std_logic;
  signal when_Pipeline_l124_12 : std_logic;
  signal decode_to_execute_MEMORY_ENABLE : std_logic;
  signal when_Pipeline_l124_13 : std_logic;
  signal execute_to_memory_MEMORY_ENABLE : std_logic;
  signal when_Pipeline_l124_14 : std_logic;
  signal memory_to_writeBack_MEMORY_ENABLE : std_logic;
  signal when_Pipeline_l124_15 : std_logic;
  signal decode_to_execute_ALU_CTRL : AluCtrlEnum_seq_type;
  signal when_Pipeline_l124_16 : std_logic;
  signal decode_to_execute_REGFILE_WRITE_VALID : std_logic;
  signal when_Pipeline_l124_17 : std_logic;
  signal execute_to_memory_REGFILE_WRITE_VALID : std_logic;
  signal when_Pipeline_l124_18 : std_logic;
  signal memory_to_writeBack_REGFILE_WRITE_VALID : std_logic;
  signal when_Pipeline_l124_19 : std_logic;
  signal decode_to_execute_BYPASSABLE_EXECUTE_STAGE : std_logic;
  signal when_Pipeline_l124_20 : std_logic;
  signal decode_to_execute_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal when_Pipeline_l124_21 : std_logic;
  signal execute_to_memory_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal when_Pipeline_l124_22 : std_logic;
  signal decode_to_execute_MEMORY_STORE : std_logic;
  signal when_Pipeline_l124_23 : std_logic;
  signal execute_to_memory_MEMORY_STORE : std_logic;
  signal when_Pipeline_l124_24 : std_logic;
  signal memory_to_writeBack_MEMORY_STORE : std_logic;
  signal when_Pipeline_l124_25 : std_logic;
  signal decode_to_execute_IS_CSR : std_logic;
  signal when_Pipeline_l124_26 : std_logic;
  signal decode_to_execute_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal when_Pipeline_l124_27 : std_logic;
  signal execute_to_memory_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal when_Pipeline_l124_28 : std_logic;
  signal memory_to_writeBack_ENV_CTRL : EnvCtrlEnum_seq_type;
  signal when_Pipeline_l124_29 : std_logic;
  signal decode_to_execute_SRC_LESS_UNSIGNED : std_logic;
  signal when_Pipeline_l124_30 : std_logic;
  signal decode_to_execute_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_seq_type;
  signal when_Pipeline_l124_31 : std_logic;
  signal decode_to_execute_IS_MUL : std_logic;
  signal when_Pipeline_l124_32 : std_logic;
  signal execute_to_memory_IS_MUL : std_logic;
  signal when_Pipeline_l124_33 : std_logic;
  signal memory_to_writeBack_IS_MUL : std_logic;
  signal when_Pipeline_l124_34 : std_logic;
  signal decode_to_execute_IS_DIV : std_logic;
  signal when_Pipeline_l124_35 : std_logic;
  signal execute_to_memory_IS_DIV : std_logic;
  signal when_Pipeline_l124_36 : std_logic;
  signal decode_to_execute_IS_RS1_SIGNED : std_logic;
  signal when_Pipeline_l124_37 : std_logic;
  signal decode_to_execute_IS_RS2_SIGNED : std_logic;
  signal when_Pipeline_l124_38 : std_logic;
  signal decode_to_execute_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal when_Pipeline_l124_39 : std_logic;
  signal execute_to_memory_SHIFT_CTRL : ShiftCtrlEnum_seq_type;
  signal when_Pipeline_l124_40 : std_logic;
  signal decode_to_execute_BRANCH_CTRL : BranchCtrlEnum_seq_type;
  signal when_Pipeline_l124_41 : std_logic;
  signal decode_to_execute_RS1 : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_42 : std_logic;
  signal decode_to_execute_RS2 : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_43 : std_logic;
  signal decode_to_execute_SRC2_FORCE_ZERO : std_logic;
  signal when_Pipeline_l124_44 : std_logic;
  signal decode_to_execute_SRC1 : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_45 : std_logic;
  signal decode_to_execute_SRC2 : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_46 : std_logic;
  signal decode_to_execute_DO_EBREAK : std_logic;
  signal when_Pipeline_l124_47 : std_logic;
  signal execute_to_memory_ALIGNEMENT_FAULT : std_logic;
  signal when_Pipeline_l124_48 : std_logic;
  signal execute_to_memory_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal when_Pipeline_l124_49 : std_logic;
  signal memory_to_writeBack_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal when_Pipeline_l124_50 : std_logic;
  signal execute_to_memory_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_51 : std_logic;
  signal memory_to_writeBack_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_52 : std_logic;
  signal execute_to_memory_MUL_LL : unsigned(31 downto 0);
  signal when_Pipeline_l124_53 : std_logic;
  signal execute_to_memory_MUL_LH : signed(33 downto 0);
  signal when_Pipeline_l124_54 : std_logic;
  signal execute_to_memory_MUL_HL : signed(33 downto 0);
  signal when_Pipeline_l124_55 : std_logic;
  signal execute_to_memory_MUL_HH : signed(33 downto 0);
  signal when_Pipeline_l124_56 : std_logic;
  signal memory_to_writeBack_MUL_HH : signed(33 downto 0);
  signal when_Pipeline_l124_57 : std_logic;
  signal execute_to_memory_SHIFT_RIGHT : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_58 : std_logic;
  signal execute_to_memory_BRANCH_DO : std_logic;
  signal when_Pipeline_l124_59 : std_logic;
  signal execute_to_memory_BRANCH_CALC : unsigned(31 downto 0);
  signal when_Pipeline_l124_60 : std_logic;
  signal memory_to_writeBack_MEMORY_READ_DATA : std_logic_vector(31 downto 0);
  signal when_Pipeline_l124_61 : std_logic;
  signal memory_to_writeBack_MUL_LOW : signed(51 downto 0);
  signal when_Pipeline_l151 : std_logic;
  signal when_Pipeline_l154 : std_logic;
  signal when_Pipeline_l151_1 : std_logic;
  signal when_Pipeline_l154_1 : std_logic;
  signal when_Pipeline_l151_2 : std_logic;
  signal when_Pipeline_l154_2 : std_logic;
  signal switch_Fetcher_l362 : unsigned(2 downto 0);
  signal when_Fetcher_l360 : std_logic;
  signal when_Fetcher_l378 : std_logic;
  signal when_Fetcher_l398 : std_logic;
  signal when_CsrPlugin_l1264 : std_logic;
  signal execute_CsrPlugin_csr_836 : std_logic;
  signal when_CsrPlugin_l1264_1 : std_logic;
  signal execute_CsrPlugin_csr_772 : std_logic;
  signal when_CsrPlugin_l1264_2 : std_logic;
  signal execute_CsrPlugin_csr_768 : std_logic;
  signal when_CsrPlugin_l1264_3 : std_logic;
  signal execute_CsrPlugin_csr_773 : std_logic;
  signal when_CsrPlugin_l1264_4 : std_logic;
  signal execute_CsrPlugin_csr_833 : std_logic;
  signal when_CsrPlugin_l1264_5 : std_logic;
  signal execute_CsrPlugin_csr_834 : std_logic;
  signal zz_CsrPlugin_csrMapping_readDataInit : std_logic_vector(31 downto 0);
  signal zz_CsrPlugin_csrMapping_readDataInit_1 : std_logic_vector(31 downto 0);
  signal zz_CsrPlugin_csrMapping_readDataInit_2 : std_logic_vector(31 downto 0);
  signal zz_CsrPlugin_csrMapping_readDataInit_3 : std_logic_vector(31 downto 0);
  signal zz_CsrPlugin_csrMapping_readDataInit_4 : std_logic_vector(31 downto 0);
  signal zz_CsrPlugin_csrMapping_readDataInit_5 : std_logic_vector(31 downto 0);
  signal when_CsrPlugin_l1297 : std_logic;
  signal when_CsrPlugin_l1302 : std_logic;
  signal iBus_cmd_m2sPipe_valid : std_logic;
  signal iBus_cmd_m2sPipe_ready : std_logic;
  signal iBus_cmd_m2sPipe_payload_pc : unsigned(31 downto 0);
  signal iBus_cmd_rValid : std_logic;
  signal iBus_cmd_rData_pc : unsigned(31 downto 0);
  signal when_Stream_l342 : std_logic;
  signal dBus_cmd_halfPipe_valid : std_logic;
  signal dBus_cmd_halfPipe_ready : std_logic;
  signal dBus_cmd_halfPipe_payload_wr : std_logic;
  signal dBus_cmd_halfPipe_payload_address : unsigned(31 downto 0);
  signal dBus_cmd_halfPipe_payload_data : std_logic_vector(31 downto 0);
  signal dBus_cmd_halfPipe_payload_size : unsigned(1 downto 0);
  signal dBus_cmd_rValid : std_logic;
  signal dBus_cmd_halfPipe_fire : std_logic;
  signal dBus_cmd_rData_wr : std_logic;
  signal dBus_cmd_rData_address : unsigned(31 downto 0);
  signal dBus_cmd_rData_data : std_logic_vector(31 downto 0);
  signal dBus_cmd_rData_size : unsigned(1 downto 0);
  signal zz_dBusWishbone_SEL : std_logic_vector(3 downto 0);
  signal when_DBusSimplePlugin_l189 : std_logic;
  type RegFilePlugin_regFile_type is array (0 to 31) of std_logic_vector(31 downto 0);
  signal RegFilePlugin_regFile : RegFilePlugin_regFile_type;
begin
  debug_bus_cmd_ready <= debug_bus_cmd_ready_read_buffer;
  iBusWishbone_CYC <= iBusWishbone_CYC_read_buffer;
  dBusWishbone_WE <= dBusWishbone_WE_read_buffer;
  zz_decode_RegFilePlugin_rs1Data <= pkg_toStdLogic(true);
  zz_decode_RegFilePlugin_rs2Data <= pkg_toStdLogic(true);
  zz_decode_DO_EBREAK <= ((((((zz_decode_DO_EBREAK_1 or zz_decode_DO_EBREAK_4) or (DebugPlugin_hardwareBreakpoints_6_valid and zz_decode_DO_EBREAK_5)) or (DebugPlugin_hardwareBreakpoints_7_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_7_pc = zz_decode_DO_EBREAK_6))) or (DebugPlugin_hardwareBreakpoints_8_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_8_pc = pkg_shiftRight(decode_PC,1)))) or (DebugPlugin_hardwareBreakpoints_9_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_9_pc = pkg_shiftRight(decode_PC,1)))) or (DebugPlugin_hardwareBreakpoints_10_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_10_pc = pkg_shiftRight(decode_PC,1))));
  zz_decode_DO_EBREAK_7 <= (DebugPlugin_hardwareBreakpoints_11_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_11_pc = pkg_shiftRight(decode_PC,1)));
  zz_decode_DO_EBREAK_8 <= pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_12_pc = pkg_shiftRight(decode_PC,1));
  zz_decode_DO_EBREAK_9 <= pkg_shiftRight(decode_PC,1);
  zz_decode_DO_EBREAK_1 <= (((((pkg_toStdLogic(false) or (DebugPlugin_hardwareBreakpoints_0_valid and zz_decode_DO_EBREAK_2)) or (DebugPlugin_hardwareBreakpoints_1_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_1_pc = zz_decode_DO_EBREAK_3))) or (DebugPlugin_hardwareBreakpoints_2_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_2_pc = pkg_shiftRight(decode_PC,1)))) or (DebugPlugin_hardwareBreakpoints_3_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_3_pc = pkg_shiftRight(decode_PC,1)))) or (DebugPlugin_hardwareBreakpoints_4_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_4_pc = pkg_shiftRight(decode_PC,1))));
  zz_decode_DO_EBREAK_4 <= (DebugPlugin_hardwareBreakpoints_5_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_5_pc = pkg_shiftRight(decode_PC,1)));
  zz_decode_DO_EBREAK_5 <= pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_6_pc = pkg_shiftRight(decode_PC,1));
  zz_decode_DO_EBREAK_6 <= pkg_shiftRight(decode_PC,1);
  zz_decode_DO_EBREAK_2 <= pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_0_pc = pkg_shiftRight(decode_PC,1));
  zz_decode_DO_EBREAK_3 <= pkg_shiftRight(decode_PC,1);
  zz_decode_LEGAL_INSTRUCTION <= pkg_stdLogicVector("00000000000000000001000001101111");
  zz_decode_LEGAL_INSTRUCTION_1 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000001000001111111"));
  zz_decode_LEGAL_INSTRUCTION_2 <= pkg_stdLogicVector("00000000000000000001000001110011");
  zz_decode_LEGAL_INSTRUCTION_3 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000001111111")) = pkg_stdLogicVector("00000000000000000010000001110011"));
  zz_decode_LEGAL_INSTRUCTION_4 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000100000001111111")) = pkg_stdLogicVector("00000000000000000100000001100011")));
  zz_decode_LEGAL_INSTRUCTION_5 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000001111111")) = pkg_stdLogicVector("00000000000000000010000000010011"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000110000000111111")) = pkg_stdLogicVector("00000000000000000000000000100011"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_decode_LEGAL_INSTRUCTION_6) = pkg_stdLogicVector("00000000000000000000000000000011"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_decode_LEGAL_INSTRUCTION_7 = zz_decode_LEGAL_INSTRUCTION_8)),pkg_cat(pkg_toStdLogicVector(zz_decode_LEGAL_INSTRUCTION_9),pkg_cat(zz_decode_LEGAL_INSTRUCTION_10,zz_decode_LEGAL_INSTRUCTION_11))))));
  zz_decode_LEGAL_INSTRUCTION_6 <= pkg_stdLogicVector("00000000000000000010000001111111");
  zz_decode_LEGAL_INSTRUCTION_7 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000101000001011111"));
  zz_decode_LEGAL_INSTRUCTION_8 <= pkg_stdLogicVector("00000000000000000000000000000011");
  zz_decode_LEGAL_INSTRUCTION_9 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000111000001111011")) = pkg_stdLogicVector("00000000000000000000000001100011"));
  zz_decode_LEGAL_INSTRUCTION_10 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000110000001111111")) = pkg_stdLogicVector("00000000000000000000000000001111")));
  zz_decode_LEGAL_INSTRUCTION_11 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("11111100000000000000000001111111")) = pkg_stdLogicVector("00000000000000000000000000110011"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("10111100000000000111000001111111")) = pkg_stdLogicVector("00000000000000000101000000010011"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_decode_LEGAL_INSTRUCTION_12) = pkg_stdLogicVector("00000000000000000001000000010011"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_decode_LEGAL_INSTRUCTION_13 = zz_decode_LEGAL_INSTRUCTION_14)),pkg_cat(pkg_toStdLogicVector(zz_decode_LEGAL_INSTRUCTION_15),pkg_cat(zz_decode_LEGAL_INSTRUCTION_16,zz_decode_LEGAL_INSTRUCTION_17))))));
  zz_decode_LEGAL_INSTRUCTION_12 <= pkg_stdLogicVector("11111100000000000011000001111111");
  zz_decode_LEGAL_INSTRUCTION_13 <= (decode_INSTRUCTION and pkg_stdLogicVector("10111110000000000111000001111111"));
  zz_decode_LEGAL_INSTRUCTION_14 <= pkg_stdLogicVector("00000000000000000101000000110011");
  zz_decode_LEGAL_INSTRUCTION_15 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("10111110000000000111000001111111")) = pkg_stdLogicVector("00000000000000000000000000110011"));
  zz_decode_LEGAL_INSTRUCTION_16 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("11011111111111111111111111111111")) = pkg_stdLogicVector("00010000001000000000000001110011")));
  zz_decode_LEGAL_INSTRUCTION_17 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("11111111111011111111111111111111")) = pkg_stdLogicVector("00000000000000000000000001110011")));
  zz_IBusSimplePlugin_decompressor_decompressed_27 <= pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_12,pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,4,3)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,5))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,2)));
  zz_IBusSimplePlugin_decompressor_decompressed_28 <= pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6));
  zz_IBusSimplePlugin_decompressor_decompressed_29 <= pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,10) = pkg_stdLogicVector("01"));
  zz_IBusSimplePlugin_decompressor_decompressed_30 <= (pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,10) = pkg_stdLogicVector("11")) and pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,5) = pkg_stdLogicVector("00")));
  zz_IBusSimplePlugin_decompressor_decompressed_31 <= pkg_stdLogicVector("0000000");
  zz_IBusSimplePlugin_decompressor_decompressed_32 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,2);
  zz_IBusSimplePlugin_decompressor_decompressed_33 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  zz_IBusSimplePlugin_decompressor_decompressed_34 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7);
  zz_zz_decode_BRANCH_CTRL_2 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000011100"));
  zz_zz_decode_BRANCH_CTRL_2_1 <= pkg_stdLogicVector("00000000000000000000000000000100");
  zz_zz_decode_BRANCH_CTRL_2_2 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001011000"));
  zz_zz_decode_BRANCH_CTRL_2_3 <= pkg_stdLogicVector("00000000000000000000000001000000");
  zz_zz_decode_BRANCH_CTRL_2_4 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000100000011000001010000")) = pkg_stdLogicVector("00000000000100000000000001010000"));
  zz_zz_decode_BRANCH_CTRL_2_5 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_6 = zz_zz_decode_BRANCH_CTRL_2_7)),pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_8 = zz_zz_decode_BRANCH_CTRL_2_9)));
  zz_zz_decode_BRANCH_CTRL_2_10 <= pkg_stdLogicVector("00");
  zz_zz_decode_BRANCH_CTRL_2_11 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_12),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_13,zz_zz_decode_BRANCH_CTRL_2_15)) /= pkg_stdLogicVector("000"));
  zz_zz_decode_BRANCH_CTRL_2_17 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_7) /= pkg_stdLogicVector("0")));
  zz_zz_decode_BRANCH_CTRL_2_18 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_19 /= zz_zz_decode_BRANCH_CTRL_2_20)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_21),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_24,zz_zz_decode_BRANCH_CTRL_2_26)));
  zz_zz_decode_BRANCH_CTRL_2_6 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000111000000110100"));
  zz_zz_decode_BRANCH_CTRL_2_7 <= pkg_stdLogicVector("00000000000000000101000000010000");
  zz_zz_decode_BRANCH_CTRL_2_8 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000010000000000111000001100100"));
  zz_zz_decode_BRANCH_CTRL_2_9 <= pkg_stdLogicVector("00000000000000000101000000100000");
  zz_zz_decode_BRANCH_CTRL_2_12 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("01000000000000000011000001010100")) = pkg_stdLogicVector("01000000000000000001000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_13 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_14) = pkg_stdLogicVector("00000000000000000001000000010000")));
  zz_zz_decode_BRANCH_CTRL_2_15 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_16) = pkg_stdLogicVector("00000000000000000001000000010000")));
  zz_zz_decode_BRANCH_CTRL_2_19 <= pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_7);
  zz_zz_decode_BRANCH_CTRL_2_20 <= pkg_stdLogicVector("0");
  zz_zz_decode_BRANCH_CTRL_2_21 <= pkg_toStdLogic(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_22 = zz_zz_decode_BRANCH_CTRL_2_23)) /= pkg_stdLogicVector("0"));
  zz_zz_decode_BRANCH_CTRL_2_24 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_25) /= pkg_stdLogicVector("0")));
  zz_zz_decode_BRANCH_CTRL_2_26 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_27 /= zz_zz_decode_BRANCH_CTRL_2_28)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_29),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_31,zz_zz_decode_BRANCH_CTRL_2_34)));
  zz_zz_decode_BRANCH_CTRL_2_14 <= pkg_stdLogicVector("00000000000000000111000000110100");
  zz_zz_decode_BRANCH_CTRL_2_16 <= pkg_stdLogicVector("00000010000000000111000001010100");
  zz_zz_decode_BRANCH_CTRL_2_22 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000010000000000100000001100100"));
  zz_zz_decode_BRANCH_CTRL_2_23 <= pkg_stdLogicVector("00000010000000000100000000100000");
  zz_zz_decode_BRANCH_CTRL_2_25 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000010000000000100000001110100")) = pkg_stdLogicVector("00000010000000000000000000110000"));
  zz_zz_decode_BRANCH_CTRL_2_27 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001100100")) = pkg_stdLogicVector("00000000000000000000000000100100")));
  zz_zz_decode_BRANCH_CTRL_2_28 <= pkg_stdLogicVector("0");
  zz_zz_decode_BRANCH_CTRL_2_29 <= pkg_toStdLogic(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_30) = pkg_stdLogicVector("00000000000000000001000000000000"))) /= pkg_stdLogicVector("0"));
  zz_zz_decode_BRANCH_CTRL_2_31 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_32 = zz_zz_decode_BRANCH_CTRL_2_33)) /= pkg_stdLogicVector("0")));
  zz_zz_decode_BRANCH_CTRL_2_34 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(zz_zz_decode_BRANCH_CTRL_2_35,zz_zz_decode_BRANCH_CTRL_2_37) /= pkg_stdLogicVector("00"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_39 /= zz_zz_decode_BRANCH_CTRL_2_41)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_42),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_45,zz_zz_decode_BRANCH_CTRL_2_50))));
  zz_zz_decode_BRANCH_CTRL_2_30 <= pkg_stdLogicVector("00000000000000000001000000000000");
  zz_zz_decode_BRANCH_CTRL_2_32 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000011000000000000"));
  zz_zz_decode_BRANCH_CTRL_2_33 <= pkg_stdLogicVector("00000000000000000010000000000000");
  zz_zz_decode_BRANCH_CTRL_2_35 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_36) = pkg_stdLogicVector("00000000000000000010000000000000")));
  zz_zz_decode_BRANCH_CTRL_2_37 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_38) = pkg_stdLogicVector("00000000000000000001000000000000")));
  zz_zz_decode_BRANCH_CTRL_2_39 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_40) = pkg_stdLogicVector("00000000000000000000000001010000")));
  zz_zz_decode_BRANCH_CTRL_2_41 <= pkg_stdLogicVector("0");
  zz_zz_decode_BRANCH_CTRL_2_42 <= pkg_toStdLogic(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_43 = zz_zz_decode_BRANCH_CTRL_2_44)) /= pkg_stdLogicVector("0"));
  zz_zz_decode_BRANCH_CTRL_2_45 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(zz_zz_decode_BRANCH_CTRL_2_46,zz_zz_decode_BRANCH_CTRL_2_48) /= pkg_stdLogicVector("00")));
  zz_zz_decode_BRANCH_CTRL_2_50 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_51 /= zz_zz_decode_BRANCH_CTRL_2_56)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_57),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_62,zz_zz_decode_BRANCH_CTRL_2_64)));
  zz_zz_decode_BRANCH_CTRL_2_36 <= pkg_stdLogicVector("00000000000000000010000000010000");
  zz_zz_decode_BRANCH_CTRL_2_38 <= pkg_stdLogicVector("00000000000000000101000000000000");
  zz_zz_decode_BRANCH_CTRL_2_40 <= pkg_stdLogicVector("00010000000100000011000001010000");
  zz_zz_decode_BRANCH_CTRL_2_43 <= (decode_INSTRUCTION and pkg_stdLogicVector("00010000000000000011000001010000"));
  zz_zz_decode_BRANCH_CTRL_2_44 <= pkg_stdLogicVector("00010000000000000000000001010000");
  zz_zz_decode_BRANCH_CTRL_2_46 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_47) = pkg_stdLogicVector("00000000000000000001000001010000")));
  zz_zz_decode_BRANCH_CTRL_2_48 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_49) = pkg_stdLogicVector("00000000000000000010000001010000")));
  zz_zz_decode_BRANCH_CTRL_2_51 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_52 = zz_zz_decode_BRANCH_CTRL_2_53)),pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_54 = zz_zz_decode_BRANCH_CTRL_2_55)));
  zz_zz_decode_BRANCH_CTRL_2_56 <= pkg_stdLogicVector("00");
  zz_zz_decode_BRANCH_CTRL_2_57 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_58),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_59,zz_zz_decode_BRANCH_CTRL_2_60)) /= pkg_stdLogicVector("000"));
  zz_zz_decode_BRANCH_CTRL_2_62 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_63) /= pkg_stdLogicVector("0")));
  zz_zz_decode_BRANCH_CTRL_2_64 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_65 /= zz_zz_decode_BRANCH_CTRL_2_76)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_77),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_90,zz_zz_decode_BRANCH_CTRL_2_107)));
  zz_zz_decode_BRANCH_CTRL_2_47 <= pkg_stdLogicVector("00000000000000000001000001010000");
  zz_zz_decode_BRANCH_CTRL_2_49 <= pkg_stdLogicVector("00000000000000000010000001010000");
  zz_zz_decode_BRANCH_CTRL_2_52 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000110100"));
  zz_zz_decode_BRANCH_CTRL_2_53 <= pkg_stdLogicVector("00000000000000000000000000100000");
  zz_zz_decode_BRANCH_CTRL_2_54 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001100100"));
  zz_zz_decode_BRANCH_CTRL_2_55 <= pkg_stdLogicVector("00000000000000000000000000100000");
  zz_zz_decode_BRANCH_CTRL_2_58 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001010000")) = pkg_stdLogicVector("00000000000000000000000001000000"));
  zz_zz_decode_BRANCH_CTRL_2_59 <= pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_4);
  zz_zz_decode_BRANCH_CTRL_2_60 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_61) = pkg_stdLogicVector("00000000000000000000000001000000")));
  zz_zz_decode_BRANCH_CTRL_2_63 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000100000")) = pkg_stdLogicVector("00000000000000000000000000100000"));
  zz_zz_decode_BRANCH_CTRL_2_65 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_66 = zz_zz_decode_BRANCH_CTRL_2_67)),pkg_cat(pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_5),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_68,zz_zz_decode_BRANCH_CTRL_2_71)));
  zz_zz_decode_BRANCH_CTRL_2_76 <= pkg_stdLogicVector("00000");
  zz_zz_decode_BRANCH_CTRL_2_77 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_5),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_78,zz_zz_decode_BRANCH_CTRL_2_81)) /= pkg_stdLogicVector("00000"));
  zz_zz_decode_BRANCH_CTRL_2_90 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(zz_zz_decode_BRANCH_CTRL_2_91,zz_zz_decode_BRANCH_CTRL_2_92) /= pkg_stdLogicVector("000000")));
  zz_zz_decode_BRANCH_CTRL_2_107 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_108 /= zz_zz_decode_BRANCH_CTRL_2_111)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_112),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_117,zz_zz_decode_BRANCH_CTRL_2_122)));
  zz_zz_decode_BRANCH_CTRL_2_61 <= pkg_stdLogicVector("00000000000100000011000001000000");
  zz_zz_decode_BRANCH_CTRL_2_66 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001000000"));
  zz_zz_decode_BRANCH_CTRL_2_67 <= pkg_stdLogicVector("00000000000000000000000001000000");
  zz_zz_decode_BRANCH_CTRL_2_68 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_69 = zz_zz_decode_BRANCH_CTRL_2_70));
  zz_zz_decode_BRANCH_CTRL_2_71 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_72),pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_74));
  zz_zz_decode_BRANCH_CTRL_2_78 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_79 = zz_zz_decode_BRANCH_CTRL_2_80));
  zz_zz_decode_BRANCH_CTRL_2_81 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_82),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_84,zz_zz_decode_BRANCH_CTRL_2_87));
  zz_zz_decode_BRANCH_CTRL_2_91 <= pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_6);
  zz_zz_decode_BRANCH_CTRL_2_92 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_93),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_95,zz_zz_decode_BRANCH_CTRL_2_98));
  zz_zz_decode_BRANCH_CTRL_2_108 <= pkg_cat(pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_5),pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_109));
  zz_zz_decode_BRANCH_CTRL_2_111 <= pkg_stdLogicVector("00");
  zz_zz_decode_BRANCH_CTRL_2_112 <= pkg_toStdLogic(pkg_cat(zz_zz_decode_BRANCH_CTRL_2_113,zz_zz_decode_BRANCH_CTRL_2_114) /= pkg_stdLogicVector("00"));
  zz_zz_decode_BRANCH_CTRL_2_117 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_118 /= zz_zz_decode_BRANCH_CTRL_2_121));
  zz_zz_decode_BRANCH_CTRL_2_122 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_123),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_126,zz_zz_decode_BRANCH_CTRL_2_136));
  zz_zz_decode_BRANCH_CTRL_2_69 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000100000000100000"));
  zz_zz_decode_BRANCH_CTRL_2_70 <= pkg_stdLogicVector("00000000000000000100000000100000");
  zz_zz_decode_BRANCH_CTRL_2_72 <= pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_73) = pkg_stdLogicVector("00000000000000000000000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_74 <= pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_75) = pkg_stdLogicVector("00000000000000000000000000100000"));
  zz_zz_decode_BRANCH_CTRL_2_79 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000000110000"));
  zz_zz_decode_BRANCH_CTRL_2_80 <= pkg_stdLogicVector("00000000000000000010000000010000");
  zz_zz_decode_BRANCH_CTRL_2_82 <= pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_83) = pkg_stdLogicVector("00000000000000000000000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_84 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_85 = zz_zz_decode_BRANCH_CTRL_2_86));
  zz_zz_decode_BRANCH_CTRL_2_87 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_88 = zz_zz_decode_BRANCH_CTRL_2_89));
  zz_zz_decode_BRANCH_CTRL_2_93 <= pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_94) = pkg_stdLogicVector("00000000000000000001000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_95 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_96 = zz_zz_decode_BRANCH_CTRL_2_97));
  zz_zz_decode_BRANCH_CTRL_2_98 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_99),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_101,zz_zz_decode_BRANCH_CTRL_2_104));
  zz_zz_decode_BRANCH_CTRL_2_109 <= pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_110) = pkg_stdLogicVector("00000000000000000000000000100000"));
  zz_zz_decode_BRANCH_CTRL_2_113 <= pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_5);
  zz_zz_decode_BRANCH_CTRL_2_114 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_115 = zz_zz_decode_BRANCH_CTRL_2_116));
  zz_zz_decode_BRANCH_CTRL_2_118 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_119 = zz_zz_decode_BRANCH_CTRL_2_120));
  zz_zz_decode_BRANCH_CTRL_2_121 <= pkg_stdLogicVector("0");
  zz_zz_decode_BRANCH_CTRL_2_123 <= pkg_toStdLogic(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_124) /= pkg_stdLogicVector("0"));
  zz_zz_decode_BRANCH_CTRL_2_126 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_127 /= zz_zz_decode_BRANCH_CTRL_2_135));
  zz_zz_decode_BRANCH_CTRL_2_136 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_137),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_139,zz_zz_decode_BRANCH_CTRL_2_146));
  zz_zz_decode_BRANCH_CTRL_2_73 <= pkg_stdLogicVector("00000000000000000000000000110000");
  zz_zz_decode_BRANCH_CTRL_2_75 <= pkg_stdLogicVector("00000010000000000000000000100000");
  zz_zz_decode_BRANCH_CTRL_2_83 <= pkg_stdLogicVector("00000000000000000001000000110000");
  zz_zz_decode_BRANCH_CTRL_2_85 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000010000000000010000001100000"));
  zz_zz_decode_BRANCH_CTRL_2_86 <= pkg_stdLogicVector("00000000000000000010000000100000");
  zz_zz_decode_BRANCH_CTRL_2_88 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000010000000000011000000100000"));
  zz_zz_decode_BRANCH_CTRL_2_89 <= pkg_stdLogicVector("00000000000000000000000000100000");
  zz_zz_decode_BRANCH_CTRL_2_94 <= pkg_stdLogicVector("00000000000000000001000000010000");
  zz_zz_decode_BRANCH_CTRL_2_96 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_97 <= pkg_stdLogicVector("00000000000000000010000000010000");
  zz_zz_decode_BRANCH_CTRL_2_99 <= pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_100) = pkg_stdLogicVector("00000000000000000000000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_101 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_102 = zz_zz_decode_BRANCH_CTRL_2_103));
  zz_zz_decode_BRANCH_CTRL_2_104 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_105 = zz_zz_decode_BRANCH_CTRL_2_106));
  zz_zz_decode_BRANCH_CTRL_2_110 <= pkg_stdLogicVector("00000000000000000000000001110000");
  zz_zz_decode_BRANCH_CTRL_2_115 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000100000"));
  zz_zz_decode_BRANCH_CTRL_2_116 <= pkg_stdLogicVector("00000000000000000000000000000000");
  zz_zz_decode_BRANCH_CTRL_2_119 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000100000000010100"));
  zz_zz_decode_BRANCH_CTRL_2_120 <= pkg_stdLogicVector("00000000000000000100000000010000");
  zz_zz_decode_BRANCH_CTRL_2_124 <= pkg_toStdLogic((decode_INSTRUCTION and zz_zz_decode_BRANCH_CTRL_2_125) = pkg_stdLogicVector("00000000000000000010000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_127 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_128),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_129,zz_zz_decode_BRANCH_CTRL_2_130));
  zz_zz_decode_BRANCH_CTRL_2_135 <= pkg_stdLogicVector("0000");
  zz_zz_decode_BRANCH_CTRL_2_137 <= pkg_toStdLogic(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_138) /= pkg_stdLogicVector("0"));
  zz_zz_decode_BRANCH_CTRL_2_139 <= pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_140 /= zz_zz_decode_BRANCH_CTRL_2_145));
  zz_zz_decode_BRANCH_CTRL_2_146 <= pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_147),pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_149));
  zz_zz_decode_BRANCH_CTRL_2_100 <= pkg_stdLogicVector("00000000000000000000000001010000");
  zz_zz_decode_BRANCH_CTRL_2_102 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000001100"));
  zz_zz_decode_BRANCH_CTRL_2_103 <= pkg_stdLogicVector("00000000000000000000000000000100");
  zz_zz_decode_BRANCH_CTRL_2_105 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000101000"));
  zz_zz_decode_BRANCH_CTRL_2_106 <= pkg_stdLogicVector("00000000000000000000000000000000");
  zz_zz_decode_BRANCH_CTRL_2_125 <= pkg_stdLogicVector("00000000000000000110000000010100");
  zz_zz_decode_BRANCH_CTRL_2_128 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001000100")) = pkg_stdLogicVector("00000000000000000000000000000000"));
  zz_zz_decode_BRANCH_CTRL_2_129 <= pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_4);
  zz_zz_decode_BRANCH_CTRL_2_130 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_131 = zz_zz_decode_BRANCH_CTRL_2_132)),pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_133 = zz_zz_decode_BRANCH_CTRL_2_134)));
  zz_zz_decode_BRANCH_CTRL_2_138 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001011000")) = pkg_stdLogicVector("00000000000000000000000000000000"));
  zz_zz_decode_BRANCH_CTRL_2_140 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_141 = zz_zz_decode_BRANCH_CTRL_2_142)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_143),pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_144)));
  zz_zz_decode_BRANCH_CTRL_2_145 <= pkg_stdLogicVector("000");
  zz_zz_decode_BRANCH_CTRL_2_147 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_148),pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_3)) /= pkg_stdLogicVector("00"));
  zz_zz_decode_BRANCH_CTRL_2_149 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_150),pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_3)) /= pkg_stdLogicVector("00"));
  zz_zz_decode_BRANCH_CTRL_2_131 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000110000000000100"));
  zz_zz_decode_BRANCH_CTRL_2_132 <= pkg_stdLogicVector("00000000000000000010000000000000");
  zz_zz_decode_BRANCH_CTRL_2_133 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000101000000000100"));
  zz_zz_decode_BRANCH_CTRL_2_134 <= pkg_stdLogicVector("00000000000000000001000000000000");
  zz_zz_decode_BRANCH_CTRL_2_141 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001000100"));
  zz_zz_decode_BRANCH_CTRL_2_142 <= pkg_stdLogicVector("00000000000000000000000001000000");
  zz_zz_decode_BRANCH_CTRL_2_143 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000000010100")) = pkg_stdLogicVector("00000000000000000010000000010000"));
  zz_zz_decode_BRANCH_CTRL_2_144 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("01000000000000000000000000110100")) = pkg_stdLogicVector("01000000000000000000000000110000"));
  zz_zz_decode_BRANCH_CTRL_2_148 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000010100")) = pkg_stdLogicVector("00000000000000000000000000000100"));
  zz_zz_decode_BRANCH_CTRL_2_150 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001000100")) = pkg_stdLogicVector("00000000000000000000000000000100"));
  process(clk)
  begin
    if rising_edge(clk) then
      if zz_decode_RegFilePlugin_rs1Data = '1' then
        zz_RegFilePlugin_regFile_port0 <= RegFilePlugin_regFile(to_integer(decode_RegFilePlugin_regFileReadAddress1));
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if zz_decode_RegFilePlugin_rs2Data = '1' then
        zz_RegFilePlugin_regFile_port0_1 <= RegFilePlugin_regFile(to_integer(decode_RegFilePlugin_regFileReadAddress2));
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if zz_1 = '1' then
        RegFilePlugin_regFile(to_integer(lastStageRegFileWrite_payload_address)) <= lastStageRegFileWrite_payload_data;
      end if;
    end if;
  end process;

  IBusSimplePlugin_rspJoin_rspBuffer_c : entity work.StreamFifoLowLatency
    port map ( 
      io_push_valid => iBus_rsp_valid,
      io_push_ready => IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready,
      io_push_payload_error => iBus_rsp_payload_error,
      io_push_payload_inst => iBus_rsp_payload_inst,
      io_pop_valid => IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid,
      io_pop_ready => IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_ready,
      io_pop_payload_error => IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error,
      io_pop_payload_inst => IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst,
      io_flush => pkg_toStdLogic(false),
      io_occupancy => IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy,
      clk => clk,
      reset => reset 
    );
  memory_MUL_LOW <= (((pkg_signed("0000000000000000000000000000000000000000000000000000") + pkg_resize(signed(pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(false)),std_logic_vector(memory_MUL_LL))),52)) + pkg_resize(pkg_shiftLeft(memory_MUL_LH,16),52)) + pkg_resize(pkg_shiftLeft(memory_MUL_HL,16),52));
  memory_MEMORY_READ_DATA <= dBus_rsp_data;
  execute_BRANCH_CALC <= unsigned(pkg_cat(std_logic_vector(pkg_extract(execute_BranchPlugin_branchAdder,31,1)),std_logic_vector(pkg_unsigned("0"))));
  execute_BRANCH_DO <= zz_execute_BRANCH_DO_1;
  execute_SHIFT_RIGHT <= std_logic_vector(pkg_extract(pkg_shiftRight(signed(pkg_cat(pkg_toStdLogicVector((pkg_toStdLogic(execute_SHIFT_CTRL = ShiftCtrlEnum_seq_SRA_1) and pkg_extract(execute_FullBarrelShifterPlugin_reversed,31))),execute_FullBarrelShifterPlugin_reversed)),execute_FullBarrelShifterPlugin_amplitude),31,0));
  memory_MUL_HH <= execute_to_memory_MUL_HH;
  execute_MUL_HH <= (execute_MulPlugin_aHigh * execute_MulPlugin_bHigh);
  execute_MUL_HL <= (execute_MulPlugin_aHigh * execute_MulPlugin_bSLow);
  execute_MUL_LH <= (execute_MulPlugin_aSLow * execute_MulPlugin_bHigh);
  execute_MUL_LL <= (execute_MulPlugin_aULow * execute_MulPlugin_bULow);
  writeBack_REGFILE_WRITE_DATA <= memory_to_writeBack_REGFILE_WRITE_DATA;
  execute_REGFILE_WRITE_DATA <= zz_execute_REGFILE_WRITE_DATA;
  memory_MEMORY_ADDRESS_LOW <= execute_to_memory_MEMORY_ADDRESS_LOW;
  execute_MEMORY_ADDRESS_LOW <= pkg_extract(dBus_cmd_payload_address,1,0);
  decode_DO_EBREAK <= (((not DebugPlugin_haltIt) and (decode_IS_EBREAK or (((((zz_decode_DO_EBREAK or zz_decode_DO_EBREAK_7) or (DebugPlugin_hardwareBreakpoints_12_valid and zz_decode_DO_EBREAK_8)) or (DebugPlugin_hardwareBreakpoints_13_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_13_pc = zz_decode_DO_EBREAK_9))) or (DebugPlugin_hardwareBreakpoints_14_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_14_pc = pkg_shiftRight(decode_PC,1)))) or (DebugPlugin_hardwareBreakpoints_15_valid and pkg_toStdLogic(DebugPlugin_hardwareBreakpoints_15_pc = pkg_shiftRight(decode_PC,1)))))) and DebugPlugin_allowEBreak);
  decode_SRC2 <= zz_decode_SRC2_6;
  decode_SRC1 <= zz_decode_SRC1_1;
  decode_SRC2_FORCE_ZERO <= (decode_SRC_ADD_ZERO and (not decode_SRC_USE_SUB_LESS));
  decode_BRANCH_CTRL <= zz_decode_BRANCH_CTRL;
  zz_decode_to_execute_BRANCH_CTRL <= zz_decode_to_execute_BRANCH_CTRL_1;
  zz_execute_to_memory_SHIFT_CTRL <= zz_execute_to_memory_SHIFT_CTRL_1;
  decode_SHIFT_CTRL <= zz_decode_SHIFT_CTRL;
  zz_decode_to_execute_SHIFT_CTRL <= zz_decode_to_execute_SHIFT_CTRL_1;
  decode_IS_RS2_SIGNED <= pkg_extract(zz_decode_BRANCH_CTRL_2,25);
  decode_IS_RS1_SIGNED <= pkg_extract(zz_decode_BRANCH_CTRL_2,24);
  decode_IS_DIV <= pkg_extract(zz_decode_BRANCH_CTRL_2,23);
  memory_IS_MUL <= execute_to_memory_IS_MUL;
  execute_IS_MUL <= decode_to_execute_IS_MUL;
  decode_IS_MUL <= pkg_extract(zz_decode_BRANCH_CTRL_2,22);
  decode_ALU_BITWISE_CTRL <= zz_decode_ALU_BITWISE_CTRL;
  zz_decode_to_execute_ALU_BITWISE_CTRL <= zz_decode_to_execute_ALU_BITWISE_CTRL_1;
  decode_SRC_LESS_UNSIGNED <= pkg_extract(zz_decode_BRANCH_CTRL_2,18);
  zz_memory_to_writeBack_ENV_CTRL <= zz_memory_to_writeBack_ENV_CTRL_1;
  zz_execute_to_memory_ENV_CTRL <= zz_execute_to_memory_ENV_CTRL_1;
  decode_ENV_CTRL <= zz_decode_ENV_CTRL;
  zz_decode_to_execute_ENV_CTRL <= zz_decode_to_execute_ENV_CTRL_1;
  decode_IS_CSR <= pkg_extract(zz_decode_BRANCH_CTRL_2,15);
  decode_MEMORY_STORE <= pkg_extract(zz_decode_BRANCH_CTRL_2,12);
  execute_BYPASSABLE_MEMORY_STAGE <= decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  decode_BYPASSABLE_MEMORY_STAGE <= pkg_extract(zz_decode_BRANCH_CTRL_2,11);
  decode_BYPASSABLE_EXECUTE_STAGE <= pkg_extract(zz_decode_BRANCH_CTRL_2,10);
  decode_ALU_CTRL <= zz_decode_ALU_CTRL;
  zz_decode_to_execute_ALU_CTRL <= zz_decode_to_execute_ALU_CTRL_1;
  decode_MEMORY_ENABLE <= pkg_extract(zz_decode_BRANCH_CTRL_2,3);
  decode_CSR_READ_OPCODE <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,13,7) /= pkg_stdLogicVector("0100000"));
  decode_CSR_WRITE_OPCODE <= (not ((pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,14,13) = pkg_stdLogicVector("01")) and pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,19,15) = pkg_stdLogicVector("00000"))) or (pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,14,13) = pkg_stdLogicVector("11")) and pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,19,15) = pkg_stdLogicVector("00000")))));
  writeBack_FORMAL_PC_NEXT <= memory_to_writeBack_FORMAL_PC_NEXT;
  memory_FORMAL_PC_NEXT <= execute_to_memory_FORMAL_PC_NEXT;
  execute_FORMAL_PC_NEXT <= decode_to_execute_FORMAL_PC_NEXT;
  decode_FORMAL_PC_NEXT <= (decode_PC + pkg_resize(pkg_mux(decode_IS_RVC,pkg_unsigned("010"),pkg_unsigned("100")),32));
  memory_PC <= execute_to_memory_PC;
  memory_BRANCH_CALC <= execute_to_memory_BRANCH_CALC;
  memory_BRANCH_DO <= execute_to_memory_BRANCH_DO;
  execute_BRANCH_CTRL <= zz_execute_BRANCH_CTRL;
  execute_PC <= decode_to_execute_PC;
  execute_DO_EBREAK <= decode_to_execute_DO_EBREAK;
  decode_IS_EBREAK <= pkg_extract(zz_decode_BRANCH_CTRL_2,28);
  decode_RS2_USE <= pkg_extract(zz_decode_BRANCH_CTRL_2,14);
  decode_RS1_USE <= pkg_extract(zz_decode_BRANCH_CTRL_2,4);
  execute_REGFILE_WRITE_VALID <= decode_to_execute_REGFILE_WRITE_VALID;
  execute_BYPASSABLE_EXECUTE_STAGE <= decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  memory_REGFILE_WRITE_VALID <= execute_to_memory_REGFILE_WRITE_VALID;
  memory_BYPASSABLE_MEMORY_STAGE <= execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  writeBack_REGFILE_WRITE_VALID <= memory_to_writeBack_REGFILE_WRITE_VALID;
  process(decode_RegFilePlugin_rs2Data,HazardSimplePlugin_writeBackBuffer_valid,HazardSimplePlugin_addr1Match,HazardSimplePlugin_writeBackBuffer_payload_data,when_HazardSimplePlugin_l45,when_HazardSimplePlugin_l47,when_HazardSimplePlugin_l51,zz_decode_RS2_2,when_HazardSimplePlugin_l45_1,memory_BYPASSABLE_MEMORY_STAGE,when_HazardSimplePlugin_l51_1,zz_decode_RS2,when_HazardSimplePlugin_l45_2,execute_BYPASSABLE_EXECUTE_STAGE,when_HazardSimplePlugin_l51_2,zz_decode_RS2_1)
  begin
    decode_RS2 <= decode_RegFilePlugin_rs2Data;
    if HazardSimplePlugin_writeBackBuffer_valid = '1' then
      if HazardSimplePlugin_addr1Match = '1' then
        decode_RS2 <= HazardSimplePlugin_writeBackBuffer_payload_data;
      end if;
    end if;
    if when_HazardSimplePlugin_l45 = '1' then
      if when_HazardSimplePlugin_l47 = '1' then
        if when_HazardSimplePlugin_l51 = '1' then
          decode_RS2 <= zz_decode_RS2_2;
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l45_1 = '1' then
      if memory_BYPASSABLE_MEMORY_STAGE = '1' then
        if when_HazardSimplePlugin_l51_1 = '1' then
          decode_RS2 <= zz_decode_RS2;
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l45_2 = '1' then
      if execute_BYPASSABLE_EXECUTE_STAGE = '1' then
        if when_HazardSimplePlugin_l51_2 = '1' then
          decode_RS2 <= zz_decode_RS2_1;
        end if;
      end if;
    end if;
  end process;

  process(decode_RegFilePlugin_rs1Data,HazardSimplePlugin_writeBackBuffer_valid,HazardSimplePlugin_addr0Match,HazardSimplePlugin_writeBackBuffer_payload_data,when_HazardSimplePlugin_l45,when_HazardSimplePlugin_l47,when_HazardSimplePlugin_l48,zz_decode_RS2_2,when_HazardSimplePlugin_l45_1,memory_BYPASSABLE_MEMORY_STAGE,when_HazardSimplePlugin_l48_1,zz_decode_RS2,when_HazardSimplePlugin_l45_2,execute_BYPASSABLE_EXECUTE_STAGE,when_HazardSimplePlugin_l48_2,zz_decode_RS2_1)
  begin
    decode_RS1 <= decode_RegFilePlugin_rs1Data;
    if HazardSimplePlugin_writeBackBuffer_valid = '1' then
      if HazardSimplePlugin_addr0Match = '1' then
        decode_RS1 <= HazardSimplePlugin_writeBackBuffer_payload_data;
      end if;
    end if;
    if when_HazardSimplePlugin_l45 = '1' then
      if when_HazardSimplePlugin_l47 = '1' then
        if when_HazardSimplePlugin_l48 = '1' then
          decode_RS1 <= zz_decode_RS2_2;
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l45_1 = '1' then
      if memory_BYPASSABLE_MEMORY_STAGE = '1' then
        if when_HazardSimplePlugin_l48_1 = '1' then
          decode_RS1 <= zz_decode_RS2;
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l45_2 = '1' then
      if execute_BYPASSABLE_EXECUTE_STAGE = '1' then
        if when_HazardSimplePlugin_l48_2 = '1' then
          decode_RS1 <= zz_decode_RS2_1;
        end if;
      end if;
    end if;
  end process;

  memory_SHIFT_RIGHT <= execute_to_memory_SHIFT_RIGHT;
  memory_SHIFT_CTRL <= zz_memory_SHIFT_CTRL;
  execute_SHIFT_CTRL <= zz_execute_SHIFT_CTRL;
  execute_SRC_LESS_UNSIGNED <= decode_to_execute_SRC_LESS_UNSIGNED;
  execute_SRC2_FORCE_ZERO <= decode_to_execute_SRC2_FORCE_ZERO;
  execute_SRC_USE_SUB_LESS <= decode_to_execute_SRC_USE_SUB_LESS;
  zz_decode_SRC2 <= decode_PC;
  zz_decode_SRC2_1 <= decode_RS2;
  decode_SRC2_CTRL <= zz_decode_SRC2_CTRL;
  zz_decode_SRC1 <= decode_RS1;
  decode_SRC1_CTRL <= zz_decode_SRC1_CTRL;
  decode_SRC_USE_SUB_LESS <= pkg_extract(zz_decode_BRANCH_CTRL_2,2);
  decode_SRC_ADD_ZERO <= pkg_extract(zz_decode_BRANCH_CTRL_2,21);
  execute_IS_RS1_SIGNED <= decode_to_execute_IS_RS1_SIGNED;
  execute_IS_DIV <= decode_to_execute_IS_DIV;
  execute_IS_RS2_SIGNED <= decode_to_execute_IS_RS2_SIGNED;
  process(memory_REGFILE_WRITE_DATA,when_MulDivIterativePlugin_l128,memory_DivPlugin_div_result,memory_arbitration_isValid,memory_SHIFT_CTRL,zz_decode_RS2_3,memory_SHIFT_RIGHT)
  begin
    zz_decode_RS2 <= memory_REGFILE_WRITE_DATA;
    if when_MulDivIterativePlugin_l128 = '1' then
      zz_decode_RS2 <= memory_DivPlugin_div_result;
    end if;
    if memory_arbitration_isValid = '1' then
      case memory_SHIFT_CTRL is
        when ShiftCtrlEnum_seq_SLL_1 =>
          zz_decode_RS2 <= zz_decode_RS2_3;
        when ShiftCtrlEnum_seq_SRL_1 | ShiftCtrlEnum_seq_SRA_1 =>
          zz_decode_RS2 <= memory_SHIFT_RIGHT;
        when others =>
      end case;
    end if;
  end process;

  memory_INSTRUCTION <= execute_to_memory_INSTRUCTION;
  memory_IS_DIV <= execute_to_memory_IS_DIV;
  writeBack_IS_MUL <= memory_to_writeBack_IS_MUL;
  writeBack_MUL_HH <= memory_to_writeBack_MUL_HH;
  writeBack_MUL_LOW <= memory_to_writeBack_MUL_LOW;
  memory_MUL_HL <= execute_to_memory_MUL_HL;
  memory_MUL_LH <= execute_to_memory_MUL_LH;
  memory_MUL_LL <= execute_to_memory_MUL_LL;
  execute_RS1 <= decode_to_execute_RS1;
  execute_SRC_ADD_SUB <= execute_SrcPlugin_addSub;
  execute_SRC_LESS <= execute_SrcPlugin_less;
  execute_ALU_CTRL <= zz_execute_ALU_CTRL;
  execute_SRC2 <= decode_to_execute_SRC2;
  execute_ALU_BITWISE_CTRL <= zz_execute_ALU_BITWISE_CTRL;
  zz_lastStageRegFileWrite_payload_address <= writeBack_INSTRUCTION;
  zz_lastStageRegFileWrite_valid <= writeBack_REGFILE_WRITE_VALID;
  process(lastStageRegFileWrite_valid)
  begin
    zz_1 <= pkg_toStdLogic(false);
    if lastStageRegFileWrite_valid = '1' then
      zz_1 <= pkg_toStdLogic(true);
    end if;
  end process;

  decode_INSTRUCTION_ANTICIPATED <= pkg_mux(decode_arbitration_isStuck,decode_INSTRUCTION,IBusSimplePlugin_decompressor_output_payload_rsp_inst);
  process(zz_decode_BRANCH_CTRL_2,when_RegFilePlugin_l63)
  begin
    decode_REGFILE_WRITE_VALID <= pkg_extract(zz_decode_BRANCH_CTRL_2,9);
    if when_RegFilePlugin_l63 = '1' then
      decode_REGFILE_WRITE_VALID <= pkg_toStdLogic(false);
    end if;
  end process;

  decode_LEGAL_INSTRUCTION <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001011111")) = pkg_stdLogicVector("00000000000000000000000000010111"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001111111")) = pkg_stdLogicVector("00000000000000000000000001101111"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_decode_LEGAL_INSTRUCTION) = pkg_stdLogicVector("00000000000000000000000000000011"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_decode_LEGAL_INSTRUCTION_1 = zz_decode_LEGAL_INSTRUCTION_2)),pkg_cat(pkg_toStdLogicVector(zz_decode_LEGAL_INSTRUCTION_3),pkg_cat(zz_decode_LEGAL_INSTRUCTION_4,zz_decode_LEGAL_INSTRUCTION_5)))))) /= pkg_stdLogicVector("0000000000000000000"));
  process(execute_REGFILE_WRITE_DATA,when_CsrPlugin_l1176,CsrPlugin_csrMapping_readDataSignal)
  begin
    zz_decode_RS2_1 <= execute_REGFILE_WRITE_DATA;
    if when_CsrPlugin_l1176 = '1' then
      zz_decode_RS2_1 <= CsrPlugin_csrMapping_readDataSignal;
    end if;
  end process;

  execute_SRC1 <= decode_to_execute_SRC1;
  execute_CSR_READ_OPCODE <= decode_to_execute_CSR_READ_OPCODE;
  execute_CSR_WRITE_OPCODE <= decode_to_execute_CSR_WRITE_OPCODE;
  execute_IS_CSR <= decode_to_execute_IS_CSR;
  memory_ENV_CTRL <= zz_memory_ENV_CTRL;
  execute_ENV_CTRL <= zz_execute_ENV_CTRL;
  writeBack_ENV_CTRL <= zz_writeBack_ENV_CTRL;
  writeBack_MEMORY_STORE <= memory_to_writeBack_MEMORY_STORE;
  process(writeBack_REGFILE_WRITE_DATA,when_DBusSimplePlugin_l560,writeBack_DBusSimplePlugin_rspFormated,when_MulPlugin_l147,switch_MulPlugin_l148,writeBack_MUL_LOW,writeBack_MulPlugin_result)
  begin
    zz_decode_RS2_2 <= writeBack_REGFILE_WRITE_DATA;
    if when_DBusSimplePlugin_l560 = '1' then
      zz_decode_RS2_2 <= writeBack_DBusSimplePlugin_rspFormated;
    end if;
    if when_MulPlugin_l147 = '1' then
      case switch_MulPlugin_l148 is
        when "00" =>
          zz_decode_RS2_2 <= std_logic_vector(pkg_extract(writeBack_MUL_LOW,31,0));
        when others =>
          zz_decode_RS2_2 <= std_logic_vector(pkg_extract(writeBack_MulPlugin_result,63,32));
      end case;
    end if;
  end process;

  writeBack_MEMORY_ENABLE <= memory_to_writeBack_MEMORY_ENABLE;
  writeBack_MEMORY_ADDRESS_LOW <= memory_to_writeBack_MEMORY_ADDRESS_LOW;
  writeBack_MEMORY_READ_DATA <= memory_to_writeBack_MEMORY_READ_DATA;
  memory_ALIGNEMENT_FAULT <= execute_to_memory_ALIGNEMENT_FAULT;
  memory_REGFILE_WRITE_DATA <= execute_to_memory_REGFILE_WRITE_DATA;
  memory_MEMORY_STORE <= execute_to_memory_MEMORY_STORE;
  memory_MEMORY_ENABLE <= execute_to_memory_MEMORY_ENABLE;
  execute_SRC_ADD <= execute_SrcPlugin_addSub;
  execute_RS2 <= decode_to_execute_RS2;
  execute_INSTRUCTION <= decode_to_execute_INSTRUCTION;
  execute_MEMORY_STORE <= decode_to_execute_MEMORY_STORE;
  execute_MEMORY_ENABLE <= decode_to_execute_MEMORY_ENABLE;
  execute_ALIGNEMENT_FAULT <= ((pkg_toStdLogic(dBus_cmd_payload_size = pkg_unsigned("10")) and pkg_toStdLogic(pkg_extract(dBus_cmd_payload_address,1,0) /= pkg_unsigned("00"))) or (pkg_toStdLogic(dBus_cmd_payload_size = pkg_unsigned("01")) and pkg_toStdLogic(pkg_extract(dBus_cmd_payload_address,0,0) /= pkg_unsigned("0"))));
  process(memory_FORMAL_PC_NEXT,BranchPlugin_jumpInterface_valid,BranchPlugin_jumpInterface_payload)
  begin
    zz_memory_to_writeBack_FORMAL_PC_NEXT <= memory_FORMAL_PC_NEXT;
    if BranchPlugin_jumpInterface_valid = '1' then
      zz_memory_to_writeBack_FORMAL_PC_NEXT <= BranchPlugin_jumpInterface_payload;
    end if;
  end process;

  decode_PC <= IBusSimplePlugin_decodePc_pcReg;
  decode_INSTRUCTION <= IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  decode_IS_RVC <= IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  writeBack_PC <= memory_to_writeBack_PC;
  writeBack_INSTRUCTION <= memory_to_writeBack_INSTRUCTION;
  process(switch_Fetcher_l362)
  begin
    decode_arbitration_haltItself <= pkg_toStdLogic(false);
    case switch_Fetcher_l362 is
      when "010" =>
        decode_arbitration_haltItself <= pkg_toStdLogic(true);
      when others =>
    end case;
  end process;

  process(CsrPlugin_pipelineLiberator_active,when_CsrPlugin_l1116,when_HazardSimplePlugin_l113)
  begin
    decode_arbitration_haltByOther <= pkg_toStdLogic(false);
    if CsrPlugin_pipelineLiberator_active = '1' then
      decode_arbitration_haltByOther <= pkg_toStdLogic(true);
    end if;
    if when_CsrPlugin_l1116 = '1' then
      decode_arbitration_haltByOther <= pkg_toStdLogic(true);
    end if;
    if when_HazardSimplePlugin_l113 = '1' then
      decode_arbitration_haltByOther <= pkg_toStdLogic(true);
    end if;
  end process;

  process(decodeExceptionPort_valid,decode_arbitration_isFlushed)
  begin
    decode_arbitration_removeIt <= pkg_toStdLogic(false);
    if decodeExceptionPort_valid = '1' then
      decode_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
    if decode_arbitration_isFlushed = '1' then
      decode_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  decode_arbitration_flushIt <= pkg_toStdLogic(false);
  process(decodeExceptionPort_valid)
  begin
    decode_arbitration_flushNext <= pkg_toStdLogic(false);
    if decodeExceptionPort_valid = '1' then
      decode_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
  end process;

  process(when_DBusSimplePlugin_l428,when_CsrPlugin_l1180,execute_CsrPlugin_blockedBySideEffects)
  begin
    execute_arbitration_haltItself <= pkg_toStdLogic(false);
    if when_DBusSimplePlugin_l428 = '1' then
      execute_arbitration_haltItself <= pkg_toStdLogic(true);
    end if;
    if when_CsrPlugin_l1180 = '1' then
      if execute_CsrPlugin_blockedBySideEffects = '1' then
        execute_arbitration_haltItself <= pkg_toStdLogic(true);
      end if;
    end if;
  end process;

  process(when_DebugPlugin_l296)
  begin
    execute_arbitration_haltByOther <= pkg_toStdLogic(false);
    if when_DebugPlugin_l296 = '1' then
      execute_arbitration_haltByOther <= pkg_toStdLogic(true);
    end if;
  end process;

  process(CsrPlugin_selfException_valid,execute_arbitration_isFlushed)
  begin
    execute_arbitration_removeIt <= pkg_toStdLogic(false);
    if CsrPlugin_selfException_valid = '1' then
      execute_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
    if execute_arbitration_isFlushed = '1' then
      execute_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  process(when_DebugPlugin_l296,when_DebugPlugin_l299)
  begin
    execute_arbitration_flushIt <= pkg_toStdLogic(false);
    if when_DebugPlugin_l296 = '1' then
      if when_DebugPlugin_l299 = '1' then
        execute_arbitration_flushIt <= pkg_toStdLogic(true);
      end if;
    end if;
  end process;

  process(CsrPlugin_selfException_valid,when_DebugPlugin_l296,when_DebugPlugin_l299,zz_4)
  begin
    execute_arbitration_flushNext <= pkg_toStdLogic(false);
    if CsrPlugin_selfException_valid = '1' then
      execute_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
    if when_DebugPlugin_l296 = '1' then
      if when_DebugPlugin_l299 = '1' then
        execute_arbitration_flushNext <= pkg_toStdLogic(true);
      end if;
    end if;
    if zz_4 = '1' then
      execute_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
  end process;

  process(when_DBusSimplePlugin_l481,when_MulDivIterativePlugin_l128,when_MulDivIterativePlugin_l129)
  begin
    memory_arbitration_haltItself <= pkg_toStdLogic(false);
    if when_DBusSimplePlugin_l481 = '1' then
      memory_arbitration_haltItself <= pkg_toStdLogic(true);
    end if;
    if when_MulDivIterativePlugin_l128 = '1' then
      if when_MulDivIterativePlugin_l129 = '1' then
        memory_arbitration_haltItself <= pkg_toStdLogic(true);
      end if;
    end if;
  end process;

  memory_arbitration_haltByOther <= pkg_toStdLogic(false);
  process(DBusSimplePlugin_memoryExceptionPort_valid,memory_arbitration_isFlushed)
  begin
    memory_arbitration_removeIt <= pkg_toStdLogic(false);
    if DBusSimplePlugin_memoryExceptionPort_valid = '1' then
      memory_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
    if memory_arbitration_isFlushed = '1' then
      memory_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  memory_arbitration_flushIt <= pkg_toStdLogic(false);
  process(DBusSimplePlugin_memoryExceptionPort_valid,BranchPlugin_jumpInterface_valid)
  begin
    memory_arbitration_flushNext <= pkg_toStdLogic(false);
    if DBusSimplePlugin_memoryExceptionPort_valid = '1' then
      memory_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
    if BranchPlugin_jumpInterface_valid = '1' then
      memory_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
  end process;

  writeBack_arbitration_haltItself <= pkg_toStdLogic(false);
  writeBack_arbitration_haltByOther <= pkg_toStdLogic(false);
  process(writeBack_arbitration_isFlushed)
  begin
    writeBack_arbitration_removeIt <= pkg_toStdLogic(false);
    if writeBack_arbitration_isFlushed = '1' then
      writeBack_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  writeBack_arbitration_flushIt <= pkg_toStdLogic(false);
  process(when_CsrPlugin_l1019,when_CsrPlugin_l1064)
  begin
    writeBack_arbitration_flushNext <= pkg_toStdLogic(false);
    if when_CsrPlugin_l1019 = '1' then
      writeBack_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
    if when_CsrPlugin_l1064 = '1' then
      writeBack_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
  end process;

  lastStageInstruction <= writeBack_INSTRUCTION;
  lastStagePc <= writeBack_PC;
  lastStageIsValid <= writeBack_arbitration_isValid;
  lastStageIsFiring <= writeBack_arbitration_isFiring;
  process(when_CsrPlugin_l922,when_CsrPlugin_l1019,when_CsrPlugin_l1064,when_DebugPlugin_l296,when_DebugPlugin_l299,DebugPlugin_haltIt,when_DebugPlugin_l312)
  begin
    IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(false);
    if when_CsrPlugin_l922 = '1' then
      IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
    end if;
    if when_CsrPlugin_l1019 = '1' then
      IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
    end if;
    if when_CsrPlugin_l1064 = '1' then
      IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
    end if;
    if when_DebugPlugin_l296 = '1' then
      if when_DebugPlugin_l299 = '1' then
        IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
      end if;
    end if;
    if DebugPlugin_haltIt = '1' then
      IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
    end if;
    if when_DebugPlugin_l312 = '1' then
      IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
    end if;
  end process;

  process(IBusSimplePlugin_iBusRsp_stages_1_input_valid,IBusSimplePlugin_injector_decodeInput_valid)
  begin
    IBusSimplePlugin_incomingInstruction <= pkg_toStdLogic(false);
    if IBusSimplePlugin_iBusRsp_stages_1_input_valid = '1' then
      IBusSimplePlugin_incomingInstruction <= pkg_toStdLogic(true);
    end if;
    if IBusSimplePlugin_injector_decodeInput_valid = '1' then
      IBusSimplePlugin_incomingInstruction <= pkg_toStdLogic(true);
    end if;
  end process;

  CsrPlugin_csrMapping_allowCsrSignal <= pkg_toStdLogic(false);
  CsrPlugin_csrMapping_readDataSignal <= CsrPlugin_csrMapping_readDataInit;
  CsrPlugin_inWfi <= pkg_toStdLogic(false);
  process(DebugPlugin_haltIt)
  begin
    CsrPlugin_thirdPartyWake <= pkg_toStdLogic(false);
    if DebugPlugin_haltIt = '1' then
      CsrPlugin_thirdPartyWake <= pkg_toStdLogic(true);
    end if;
  end process;

  process(when_CsrPlugin_l1019,when_CsrPlugin_l1064)
  begin
    CsrPlugin_jumpInterface_valid <= pkg_toStdLogic(false);
    if when_CsrPlugin_l1019 = '1' then
      CsrPlugin_jumpInterface_valid <= pkg_toStdLogic(true);
    end if;
    if when_CsrPlugin_l1064 = '1' then
      CsrPlugin_jumpInterface_valid <= pkg_toStdLogic(true);
    end if;
  end process;

  process(when_CsrPlugin_l1019,CsrPlugin_xtvec_mode,CsrPlugin_hadException,CsrPlugin_xtvec_base,CsrPlugin_trapCause,when_CsrPlugin_l1064,switch_CsrPlugin_l1068,CsrPlugin_mepc)
  begin
    CsrPlugin_jumpInterface_payload <= pkg_unsigned("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    if when_CsrPlugin_l1019 = '1' then
      CsrPlugin_jumpInterface_payload <= pkg_mux((pkg_toStdLogic(CsrPlugin_xtvec_mode = pkg_stdLogicVector("00")) or CsrPlugin_hadException),unsigned(pkg_cat(std_logic_vector(CsrPlugin_xtvec_base),std_logic_vector(pkg_unsigned("00")))),unsigned(pkg_cat(std_logic_vector((CsrPlugin_xtvec_base + pkg_resize(CsrPlugin_trapCause,30))),std_logic_vector(pkg_unsigned("00")))));
    end if;
    if when_CsrPlugin_l1064 = '1' then
      case switch_CsrPlugin_l1068 is
        when "11" =>
          CsrPlugin_jumpInterface_payload <= CsrPlugin_mepc;
        when others =>
      end case;
    end if;
  end process;

  process(DebugPlugin_godmode)
  begin
    CsrPlugin_forceMachineWire <= pkg_toStdLogic(false);
    if DebugPlugin_godmode = '1' then
      CsrPlugin_forceMachineWire <= pkg_toStdLogic(true);
    end if;
  end process;

  process(when_DebugPlugin_l328)
  begin
    CsrPlugin_allowInterrupts <= pkg_toStdLogic(true);
    if when_DebugPlugin_l328 = '1' then
      CsrPlugin_allowInterrupts <= pkg_toStdLogic(false);
    end if;
  end process;

  process(DebugPlugin_godmode)
  begin
    CsrPlugin_allowException <= pkg_toStdLogic(true);
    if DebugPlugin_godmode = '1' then
      CsrPlugin_allowException <= pkg_toStdLogic(false);
    end if;
  end process;

  process(DebugPlugin_allowEBreak)
  begin
    CsrPlugin_allowEbreakException <= pkg_toStdLogic(true);
    if DebugPlugin_allowEBreak = '1' then
      CsrPlugin_allowEbreakException <= pkg_toStdLogic(false);
    end if;
  end process;

  zz_when_CsrPlugin_l952 <= (LocalInt0_regNext and LocalInt0_enable);
  zz_when_CsrPlugin_l952_1 <= (LocalInt1_regNext and LocalInt1_enable);
  zz_when_CsrPlugin_l952_2 <= (LocalInt2_regNext and LocalInt2_enable);
  zz_when_CsrPlugin_l952_3 <= (LocalInt3_regNext and LocalInt3_enable);
  IBusSimplePlugin_externalFlush <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushNext),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushNext),pkg_cat(pkg_toStdLogicVector(execute_arbitration_flushNext),pkg_toStdLogicVector(decode_arbitration_flushNext)))) /= pkg_stdLogicVector("0000"));
  IBusSimplePlugin_jump_pcLoad_valid <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(BranchPlugin_jumpInterface_valid),pkg_toStdLogicVector(CsrPlugin_jumpInterface_valid)) /= pkg_stdLogicVector("00"));
  zz_IBusSimplePlugin_jump_pcLoad_payload <= unsigned(pkg_cat(pkg_toStdLogicVector(BranchPlugin_jumpInterface_valid),pkg_toStdLogicVector(CsrPlugin_jumpInterface_valid)));
  IBusSimplePlugin_jump_pcLoad_payload <= pkg_mux(pkg_extract(std_logic_vector((zz_IBusSimplePlugin_jump_pcLoad_payload and pkg_not((zz_IBusSimplePlugin_jump_pcLoad_payload - pkg_unsigned("01"))))),0),CsrPlugin_jumpInterface_payload,BranchPlugin_jumpInterface_payload);
  process(IBusSimplePlugin_jump_pcLoad_valid)
  begin
    IBusSimplePlugin_fetchPc_correction <= pkg_toStdLogic(false);
    if IBusSimplePlugin_jump_pcLoad_valid = '1' then
      IBusSimplePlugin_fetchPc_correction <= pkg_toStdLogic(true);
    end if;
  end process;

  IBusSimplePlugin_fetchPc_output_fire <= (IBusSimplePlugin_fetchPc_output_valid and IBusSimplePlugin_fetchPc_output_ready);
  IBusSimplePlugin_fetchPc_corrected <= (IBusSimplePlugin_fetchPc_correction or IBusSimplePlugin_fetchPc_correctionReg);
  process(IBusSimplePlugin_iBusRsp_stages_1_input_ready)
  begin
    IBusSimplePlugin_fetchPc_pcRegPropagate <= pkg_toStdLogic(false);
    if IBusSimplePlugin_iBusRsp_stages_1_input_ready = '1' then
      IBusSimplePlugin_fetchPc_pcRegPropagate <= pkg_toStdLogic(true);
    end if;
  end process;

  when_Fetcher_l131 <= (IBusSimplePlugin_fetchPc_correction or IBusSimplePlugin_fetchPc_pcRegPropagate);
  IBusSimplePlugin_fetchPc_output_fire_1 <= (IBusSimplePlugin_fetchPc_output_valid and IBusSimplePlugin_fetchPc_output_ready);
  when_Fetcher_l131_1 <= ((not IBusSimplePlugin_fetchPc_output_valid) and IBusSimplePlugin_fetchPc_output_ready);
  process(IBusSimplePlugin_fetchPc_pcReg,IBusSimplePlugin_fetchPc_inc,IBusSimplePlugin_jump_pcLoad_valid,IBusSimplePlugin_jump_pcLoad_payload)
  begin
    IBusSimplePlugin_fetchPc_pc <= (IBusSimplePlugin_fetchPc_pcReg + pkg_resize(unsigned(pkg_cat(pkg_toStdLogicVector(IBusSimplePlugin_fetchPc_inc),pkg_stdLogicVector("00"))),32));
    if IBusSimplePlugin_fetchPc_inc = '1' then
      IBusSimplePlugin_fetchPc_pc(1) <= pkg_toStdLogic(false);
    end if;
    if IBusSimplePlugin_jump_pcLoad_valid = '1' then
      IBusSimplePlugin_fetchPc_pc <= IBusSimplePlugin_jump_pcLoad_payload;
    end if;
    IBusSimplePlugin_fetchPc_pc(0) <= pkg_toStdLogic(false);
  end process;

  process(IBusSimplePlugin_jump_pcLoad_valid)
  begin
    IBusSimplePlugin_fetchPc_flushed <= pkg_toStdLogic(false);
    if IBusSimplePlugin_jump_pcLoad_valid = '1' then
      IBusSimplePlugin_fetchPc_flushed <= pkg_toStdLogic(true);
    end if;
  end process;

  when_Fetcher_l158 <= (IBusSimplePlugin_fetchPc_booted and ((IBusSimplePlugin_fetchPc_output_ready or IBusSimplePlugin_fetchPc_correction) or IBusSimplePlugin_fetchPc_pcRegPropagate));
  IBusSimplePlugin_fetchPc_output_valid <= ((not IBusSimplePlugin_fetcherHalt) and IBusSimplePlugin_fetchPc_booted);
  IBusSimplePlugin_fetchPc_output_payload <= IBusSimplePlugin_fetchPc_pc;
  process(when_Fetcher_l192)
  begin
    IBusSimplePlugin_decodePc_flushed <= pkg_toStdLogic(false);
    if when_Fetcher_l192 = '1' then
      IBusSimplePlugin_decodePc_flushed <= pkg_toStdLogic(true);
    end if;
  end process;

  IBusSimplePlugin_decodePc_pcPlus <= (IBusSimplePlugin_decodePc_pcReg + pkg_resize(pkg_mux(decode_IS_RVC,pkg_unsigned("010"),pkg_unsigned("100")),32));
  process(when_Fetcher_l360)
  begin
    IBusSimplePlugin_decodePc_injectedDecode <= pkg_toStdLogic(false);
    if when_Fetcher_l360 = '1' then
      IBusSimplePlugin_decodePc_injectedDecode <= pkg_toStdLogic(true);
    end if;
  end process;

  when_Fetcher_l180 <= (decode_arbitration_isFiring and (not IBusSimplePlugin_decodePc_injectedDecode));
  when_Fetcher_l192 <= (IBusSimplePlugin_jump_pcLoad_valid and ((not decode_arbitration_isStuck) or decode_arbitration_removeIt));
  IBusSimplePlugin_iBusRsp_redoFetch <= pkg_toStdLogic(false);
  IBusSimplePlugin_iBusRsp_stages_0_input_valid <= IBusSimplePlugin_fetchPc_output_valid;
  IBusSimplePlugin_fetchPc_output_ready <= IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  IBusSimplePlugin_iBusRsp_stages_0_input_payload <= IBusSimplePlugin_fetchPc_output_payload;
  process(when_IBusSimplePlugin_l305)
  begin
    IBusSimplePlugin_iBusRsp_stages_0_halt <= pkg_toStdLogic(false);
    if when_IBusSimplePlugin_l305 = '1' then
      IBusSimplePlugin_iBusRsp_stages_0_halt <= pkg_toStdLogic(true);
    end if;
  end process;

  zz_IBusSimplePlugin_iBusRsp_stages_0_input_ready <= (not IBusSimplePlugin_iBusRsp_stages_0_halt);
  IBusSimplePlugin_iBusRsp_stages_0_input_ready <= (IBusSimplePlugin_iBusRsp_stages_0_output_ready and zz_IBusSimplePlugin_iBusRsp_stages_0_input_ready);
  IBusSimplePlugin_iBusRsp_stages_0_output_valid <= (IBusSimplePlugin_iBusRsp_stages_0_input_valid and zz_IBusSimplePlugin_iBusRsp_stages_0_input_ready);
  IBusSimplePlugin_iBusRsp_stages_0_output_payload <= IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  IBusSimplePlugin_iBusRsp_stages_1_halt <= pkg_toStdLogic(false);
  zz_IBusSimplePlugin_iBusRsp_stages_1_input_ready <= (not IBusSimplePlugin_iBusRsp_stages_1_halt);
  IBusSimplePlugin_iBusRsp_stages_1_input_ready <= (IBusSimplePlugin_iBusRsp_stages_1_output_ready and zz_IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  IBusSimplePlugin_iBusRsp_stages_1_output_valid <= (IBusSimplePlugin_iBusRsp_stages_1_input_valid and zz_IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  IBusSimplePlugin_iBusRsp_stages_1_output_payload <= IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  IBusSimplePlugin_iBusRsp_flush <= (IBusSimplePlugin_externalFlush or IBusSimplePlugin_iBusRsp_redoFetch);
  IBusSimplePlugin_iBusRsp_stages_0_output_ready <= zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready <= ((pkg_toStdLogic(false) and (not zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_1)) or IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_1 <= zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_2;
  IBusSimplePlugin_iBusRsp_stages_1_input_valid <= zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_1;
  IBusSimplePlugin_iBusRsp_stages_1_input_payload <= IBusSimplePlugin_fetchPc_pcReg;
  process(IBusSimplePlugin_injector_decodeInput_valid)
  begin
    IBusSimplePlugin_iBusRsp_readyForError <= pkg_toStdLogic(true);
    if IBusSimplePlugin_injector_decodeInput_valid = '1' then
      IBusSimplePlugin_iBusRsp_readyForError <= pkg_toStdLogic(false);
    end if;
  end process;

  IBusSimplePlugin_decompressor_input_valid <= (IBusSimplePlugin_iBusRsp_output_valid and (not IBusSimplePlugin_iBusRsp_redoFetch));
  IBusSimplePlugin_decompressor_input_payload_pc <= IBusSimplePlugin_iBusRsp_output_payload_pc;
  IBusSimplePlugin_decompressor_input_payload_rsp_error <= IBusSimplePlugin_iBusRsp_output_payload_rsp_error;
  IBusSimplePlugin_decompressor_input_payload_rsp_inst <= IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
  IBusSimplePlugin_decompressor_input_payload_isRvc <= IBusSimplePlugin_iBusRsp_output_payload_isRvc;
  IBusSimplePlugin_iBusRsp_output_ready <= IBusSimplePlugin_decompressor_input_ready;
  IBusSimplePlugin_decompressor_flushNext <= pkg_toStdLogic(false);
  IBusSimplePlugin_decompressor_consumeCurrent <= pkg_toStdLogic(false);
  IBusSimplePlugin_decompressor_isInputLowRvc <= pkg_toStdLogic(pkg_extract(IBusSimplePlugin_decompressor_input_payload_rsp_inst,1,0) /= pkg_stdLogicVector("11"));
  IBusSimplePlugin_decompressor_isInputHighRvc <= pkg_toStdLogic(pkg_extract(IBusSimplePlugin_decompressor_input_payload_rsp_inst,17,16) /= pkg_stdLogicVector("11"));
  IBusSimplePlugin_decompressor_throw2Bytes <= (IBusSimplePlugin_decompressor_throw2BytesReg or pkg_extract(IBusSimplePlugin_decompressor_input_payload_pc,1));
  IBusSimplePlugin_decompressor_unaligned <= (IBusSimplePlugin_decompressor_throw2Bytes or IBusSimplePlugin_decompressor_bufferValid);
  IBusSimplePlugin_decompressor_bufferValidPatched <= pkg_mux(IBusSimplePlugin_decompressor_input_valid,IBusSimplePlugin_decompressor_bufferValid,IBusSimplePlugin_decompressor_bufferValidLatch);
  IBusSimplePlugin_decompressor_throw2BytesPatched <= pkg_mux(IBusSimplePlugin_decompressor_input_valid,IBusSimplePlugin_decompressor_throw2Bytes,IBusSimplePlugin_decompressor_throw2BytesLatch);
  IBusSimplePlugin_decompressor_raw <= pkg_mux(IBusSimplePlugin_decompressor_bufferValidPatched,pkg_cat(pkg_extract(IBusSimplePlugin_decompressor_input_payload_rsp_inst,15,0),IBusSimplePlugin_decompressor_bufferData),pkg_cat(pkg_extract(IBusSimplePlugin_decompressor_input_payload_rsp_inst,31,16),pkg_mux(IBusSimplePlugin_decompressor_throw2BytesPatched,pkg_extract(IBusSimplePlugin_decompressor_input_payload_rsp_inst,31,16),pkg_extract(IBusSimplePlugin_decompressor_input_payload_rsp_inst,15,0))));
  IBusSimplePlugin_decompressor_isRvc <= pkg_toStdLogic(pkg_extract(IBusSimplePlugin_decompressor_raw,1,0) /= pkg_stdLogicVector("11"));
  zz_IBusSimplePlugin_decompressor_decompressed <= pkg_extract(IBusSimplePlugin_decompressor_raw,15,0);
  process(switch_Misc_l44,zz_IBusSimplePlugin_decompressor_decompressed,zz_IBusSimplePlugin_decompressor_decompressed_2,zz_IBusSimplePlugin_decompressor_decompressed_3,zz_IBusSimplePlugin_decompressor_decompressed_1,zz_IBusSimplePlugin_decompressor_decompressed_5,zz_IBusSimplePlugin_decompressor_decompressed_8,zz_IBusSimplePlugin_decompressor_decompressed_20,zz_IBusSimplePlugin_decompressor_decompressed_27,zz_IBusSimplePlugin_decompressor_decompressed_28,zz_IBusSimplePlugin_decompressor_decompressed_10,zz_IBusSimplePlugin_decompressor_decompressed_26,zz_IBusSimplePlugin_decompressor_decompressed_29,zz_IBusSimplePlugin_decompressor_decompressed_30,zz_IBusSimplePlugin_decompressor_decompressed_22,zz_IBusSimplePlugin_decompressor_decompressed_24,zz_IBusSimplePlugin_decompressor_decompressed_15,zz_IBusSimplePlugin_decompressor_decompressed_19,zz_IBusSimplePlugin_decompressor_decompressed_18,zz_IBusSimplePlugin_decompressor_decompressed_21,zz_IBusSimplePlugin_decompressor_decompressed_31,zz_IBusSimplePlugin_decompressor_decompressed_32,zz_IBusSimplePlugin_decompressor_decompressed_33,zz_IBusSimplePlugin_decompressor_decompressed_34)
  begin
    IBusSimplePlugin_decompressor_decompressed <= pkg_stdLogicVector("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    case switch_Misc_l44 is
      when "00000" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_stdLogicVector("00"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,10,7)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12,11)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,5))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6))),pkg_stdLogicVector("00")),pkg_stdLogicVector("00010")),pkg_stdLogicVector("000")),zz_IBusSimplePlugin_decompressor_decompressed_2),pkg_stdLogicVector("0010011"));
      when "00010" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_3,zz_IBusSimplePlugin_decompressor_decompressed_1),pkg_stdLogicVector("010")),zz_IBusSimplePlugin_decompressor_decompressed_2),pkg_stdLogicVector("0000011"));
      when "00110" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_3,11,5),zz_IBusSimplePlugin_decompressor_decompressed_2),zz_IBusSimplePlugin_decompressor_decompressed_1),pkg_stdLogicVector("010")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_3,4,0)),pkg_stdLogicVector("0100011"));
      when "01000" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_5,pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("000")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("0010011"));
      when "01001" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_8,20)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_8,10,1)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_8,11))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_8,19,12)),zz_IBusSimplePlugin_decompressor_decompressed_20),pkg_stdLogicVector("1101111"));
      when "01010" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_5,pkg_stdLogicVector("00000")),pkg_stdLogicVector("000")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("0010011"));
      when "01011" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_mux(pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7) = pkg_stdLogicVector("00010")),pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_27,zz_IBusSimplePlugin_decompressor_decompressed_28),pkg_stdLogicVector("0000")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("000")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("0010011")),pkg_cat(pkg_cat(pkg_extract(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_10,pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,2)),pkg_stdLogicVector("000000000000")),31,12),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("0110111")));
      when "01100" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_mux(pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,10) = pkg_stdLogicVector("10")),zz_IBusSimplePlugin_decompressor_decompressed_26,pkg_cat(pkg_cat(pkg_stdLogicVector("0"),pkg_toStdLogicVector((zz_IBusSimplePlugin_decompressor_decompressed_29 or zz_IBusSimplePlugin_decompressor_decompressed_30))),pkg_stdLogicVector("00000"))),pkg_mux(((not pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11)) or zz_IBusSimplePlugin_decompressor_decompressed_22),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,2),zz_IBusSimplePlugin_decompressor_decompressed_2)),zz_IBusSimplePlugin_decompressor_decompressed_1),zz_IBusSimplePlugin_decompressor_decompressed_24),zz_IBusSimplePlugin_decompressor_decompressed_1),pkg_mux(zz_IBusSimplePlugin_decompressor_decompressed_22,pkg_stdLogicVector("0010011"),pkg_stdLogicVector("0110011")));
      when "01101" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_15,20)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_15,10,1)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_15,11))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_15,19,12)),zz_IBusSimplePlugin_decompressor_decompressed_19),pkg_stdLogicVector("1101111"));
      when "01110" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,12)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,10,5)),zz_IBusSimplePlugin_decompressor_decompressed_19),zz_IBusSimplePlugin_decompressor_decompressed_1),pkg_stdLogicVector("000")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,4,1)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,11))),pkg_stdLogicVector("1100011"));
      when "01111" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,12)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,10,5)),zz_IBusSimplePlugin_decompressor_decompressed_19),zz_IBusSimplePlugin_decompressor_decompressed_1),pkg_stdLogicVector("001")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,4,1)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed_18,11))),pkg_stdLogicVector("1100011"));
      when "10000" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_stdLogicVector("0000000"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,2)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("001")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("0010011"));
      when "10010" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_stdLogicVector("0000"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,3,2)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,4)),pkg_stdLogicVector("00")),zz_IBusSimplePlugin_decompressor_decompressed_21),pkg_stdLogicVector("010")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("0000011"));
      when "10100" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_mux(pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12,2) = pkg_stdLogicVector("10000000000")),pkg_stdLogicVector("00000000000100000000000001110011"),pkg_mux(pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,2) = pkg_stdLogicVector("00000")),pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_stdLogicVector("000000000000"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("000")),pkg_mux(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12),zz_IBusSimplePlugin_decompressor_decompressed_20,zz_IBusSimplePlugin_decompressor_decompressed_19)),pkg_stdLogicVector("1100111")),pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_31,zz_IBusSimplePlugin_decompressor_decompressed_32),pkg_mux(zz_IBusSimplePlugin_decompressor_decompressed_33,zz_IBusSimplePlugin_decompressor_decompressed_34,zz_IBusSimplePlugin_decompressor_decompressed_19)),pkg_stdLogicVector("000")),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,7)),pkg_stdLogicVector("0110011"))));
      when "10110" =>
        IBusSimplePlugin_decompressor_decompressed <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_extract(pkg_cat(pkg_cat(pkg_cat(pkg_stdLogicVector("0000"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,8,7)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12,9)),pkg_stdLogicVector("00")),11,5),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,2)),zz_IBusSimplePlugin_decompressor_decompressed_21),pkg_stdLogicVector("010")),pkg_extract(pkg_cat(pkg_cat(pkg_cat(pkg_stdLogicVector("0000"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,8,7)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12,9)),pkg_stdLogicVector("00")),4,0)),pkg_stdLogicVector("0100011"));
      when others =>
    end case;
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_1 <= pkg_cat(pkg_stdLogicVector("01"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,9,7));
  zz_IBusSimplePlugin_decompressor_decompressed_2 <= pkg_cat(pkg_stdLogicVector("01"),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,4,2));
  zz_IBusSimplePlugin_decompressor_decompressed_3 <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_stdLogicVector("00000"),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,5))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12,10)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6))),pkg_stdLogicVector("00"));
  zz_IBusSimplePlugin_decompressor_decompressed_4 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  process(zz_IBusSimplePlugin_decompressor_decompressed_4,zz_IBusSimplePlugin_decompressor_decompressed)
  begin
    zz_IBusSimplePlugin_decompressor_decompressed_5(11) <= zz_IBusSimplePlugin_decompressor_decompressed_4;
    zz_IBusSimplePlugin_decompressor_decompressed_5(10) <= zz_IBusSimplePlugin_decompressor_decompressed_4;
    zz_IBusSimplePlugin_decompressor_decompressed_5(9) <= zz_IBusSimplePlugin_decompressor_decompressed_4;
    zz_IBusSimplePlugin_decompressor_decompressed_5(8) <= zz_IBusSimplePlugin_decompressor_decompressed_4;
    zz_IBusSimplePlugin_decompressor_decompressed_5(7) <= zz_IBusSimplePlugin_decompressor_decompressed_4;
    zz_IBusSimplePlugin_decompressor_decompressed_5(6) <= zz_IBusSimplePlugin_decompressor_decompressed_4;
    zz_IBusSimplePlugin_decompressor_decompressed_5(5) <= zz_IBusSimplePlugin_decompressor_decompressed_4;
    zz_IBusSimplePlugin_decompressor_decompressed_5(4 downto 0) <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,2);
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_6 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  process(zz_IBusSimplePlugin_decompressor_decompressed_6)
  begin
    zz_IBusSimplePlugin_decompressor_decompressed_7(9) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(8) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(7) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(6) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(5) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(4) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(3) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(2) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(1) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
    zz_IBusSimplePlugin_decompressor_decompressed_7(0) <= zz_IBusSimplePlugin_decompressor_decompressed_6;
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_8 <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_7,pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,8))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,10,9)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,7))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,2))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,5,3)),pkg_stdLogicVector("0"));
  zz_IBusSimplePlugin_decompressor_decompressed_9 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  process(zz_IBusSimplePlugin_decompressor_decompressed_9)
  begin
    zz_IBusSimplePlugin_decompressor_decompressed_10(14) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(13) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(12) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(11) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(10) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(9) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(8) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(7) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(6) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(5) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(4) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(3) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(2) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(1) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
    zz_IBusSimplePlugin_decompressor_decompressed_10(0) <= zz_IBusSimplePlugin_decompressor_decompressed_9;
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_11 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  process(zz_IBusSimplePlugin_decompressor_decompressed_11)
  begin
    zz_IBusSimplePlugin_decompressor_decompressed_12(2) <= zz_IBusSimplePlugin_decompressor_decompressed_11;
    zz_IBusSimplePlugin_decompressor_decompressed_12(1) <= zz_IBusSimplePlugin_decompressor_decompressed_11;
    zz_IBusSimplePlugin_decompressor_decompressed_12(0) <= zz_IBusSimplePlugin_decompressor_decompressed_11;
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_13 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  process(zz_IBusSimplePlugin_decompressor_decompressed_13)
  begin
    zz_IBusSimplePlugin_decompressor_decompressed_14(9) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(8) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(7) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(6) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(5) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(4) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(3) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(2) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(1) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
    zz_IBusSimplePlugin_decompressor_decompressed_14(0) <= zz_IBusSimplePlugin_decompressor_decompressed_13;
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_15 <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_14,pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,8))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,10,9)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,7))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,2))),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,5,3)),pkg_stdLogicVector("0"));
  zz_IBusSimplePlugin_decompressor_decompressed_16 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  process(zz_IBusSimplePlugin_decompressor_decompressed_16)
  begin
    zz_IBusSimplePlugin_decompressor_decompressed_17(4) <= zz_IBusSimplePlugin_decompressor_decompressed_16;
    zz_IBusSimplePlugin_decompressor_decompressed_17(3) <= zz_IBusSimplePlugin_decompressor_decompressed_16;
    zz_IBusSimplePlugin_decompressor_decompressed_17(2) <= zz_IBusSimplePlugin_decompressor_decompressed_16;
    zz_IBusSimplePlugin_decompressor_decompressed_17(1) <= zz_IBusSimplePlugin_decompressor_decompressed_16;
    zz_IBusSimplePlugin_decompressor_decompressed_17(0) <= zz_IBusSimplePlugin_decompressor_decompressed_16;
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_18 <= pkg_cat(pkg_cat(pkg_cat(pkg_cat(pkg_cat(zz_IBusSimplePlugin_decompressor_decompressed_17,pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,5)),pkg_toStdLogicVector(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,2))),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,10)),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,4,3)),pkg_stdLogicVector("0"));
  zz_IBusSimplePlugin_decompressor_decompressed_19 <= pkg_stdLogicVector("00000");
  zz_IBusSimplePlugin_decompressor_decompressed_20 <= pkg_stdLogicVector("00001");
  zz_IBusSimplePlugin_decompressor_decompressed_21 <= pkg_stdLogicVector("00010");
  switch_Misc_l44 <= pkg_cat(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,1,0),pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,15,13));
  zz_IBusSimplePlugin_decompressor_decompressed_22 <= pkg_toStdLogic(pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,10) /= pkg_stdLogicVector("11"));
  switch_Misc_l204 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,11,10);
  switch_Misc_l204_1 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,6,5);
  process(switch_Misc_l204_1)
  begin
    case switch_Misc_l204_1 is
      when "00" =>
        zz_IBusSimplePlugin_decompressor_decompressed_23 <= pkg_stdLogicVector("000");
      when "01" =>
        zz_IBusSimplePlugin_decompressor_decompressed_23 <= pkg_stdLogicVector("100");
      when "10" =>
        zz_IBusSimplePlugin_decompressor_decompressed_23 <= pkg_stdLogicVector("110");
      when others =>
        zz_IBusSimplePlugin_decompressor_decompressed_23 <= pkg_stdLogicVector("111");
    end case;
  end process;

  process(switch_Misc_l204,zz_IBusSimplePlugin_decompressor_decompressed_23)
  begin
    case switch_Misc_l204 is
      when "00" =>
        zz_IBusSimplePlugin_decompressor_decompressed_24 <= pkg_stdLogicVector("101");
      when "01" =>
        zz_IBusSimplePlugin_decompressor_decompressed_24 <= pkg_stdLogicVector("101");
      when "10" =>
        zz_IBusSimplePlugin_decompressor_decompressed_24 <= pkg_stdLogicVector("111");
      when others =>
        zz_IBusSimplePlugin_decompressor_decompressed_24 <= zz_IBusSimplePlugin_decompressor_decompressed_23;
    end case;
  end process;

  zz_IBusSimplePlugin_decompressor_decompressed_25 <= pkg_extract(zz_IBusSimplePlugin_decompressor_decompressed,12);
  process(zz_IBusSimplePlugin_decompressor_decompressed_25)
  begin
    zz_IBusSimplePlugin_decompressor_decompressed_26(6) <= zz_IBusSimplePlugin_decompressor_decompressed_25;
    zz_IBusSimplePlugin_decompressor_decompressed_26(5) <= zz_IBusSimplePlugin_decompressor_decompressed_25;
    zz_IBusSimplePlugin_decompressor_decompressed_26(4) <= zz_IBusSimplePlugin_decompressor_decompressed_25;
    zz_IBusSimplePlugin_decompressor_decompressed_26(3) <= zz_IBusSimplePlugin_decompressor_decompressed_25;
    zz_IBusSimplePlugin_decompressor_decompressed_26(2) <= zz_IBusSimplePlugin_decompressor_decompressed_25;
    zz_IBusSimplePlugin_decompressor_decompressed_26(1) <= zz_IBusSimplePlugin_decompressor_decompressed_25;
    zz_IBusSimplePlugin_decompressor_decompressed_26(0) <= zz_IBusSimplePlugin_decompressor_decompressed_25;
  end process;

  IBusSimplePlugin_decompressor_output_valid <= (IBusSimplePlugin_decompressor_input_valid and (not ((IBusSimplePlugin_decompressor_throw2Bytes and (not IBusSimplePlugin_decompressor_bufferValid)) and (not IBusSimplePlugin_decompressor_isInputHighRvc))));
  IBusSimplePlugin_decompressor_output_payload_pc <= IBusSimplePlugin_decompressor_input_payload_pc;
  IBusSimplePlugin_decompressor_output_payload_isRvc <= IBusSimplePlugin_decompressor_isRvc;
  IBusSimplePlugin_decompressor_output_payload_rsp_inst <= pkg_mux(IBusSimplePlugin_decompressor_isRvc,IBusSimplePlugin_decompressor_decompressed,IBusSimplePlugin_decompressor_raw);
  IBusSimplePlugin_decompressor_input_ready <= (IBusSimplePlugin_decompressor_output_ready and (((not IBusSimplePlugin_iBusRsp_stages_1_input_valid) or IBusSimplePlugin_decompressor_flushNext) or ((not (IBusSimplePlugin_decompressor_bufferValid and IBusSimplePlugin_decompressor_isInputHighRvc)) and (not (((not IBusSimplePlugin_decompressor_unaligned) and IBusSimplePlugin_decompressor_isInputLowRvc) and IBusSimplePlugin_decompressor_isInputHighRvc)))));
  IBusSimplePlugin_decompressor_output_fire <= (IBusSimplePlugin_decompressor_output_valid and IBusSimplePlugin_decompressor_output_ready);
  IBusSimplePlugin_decompressor_bufferFill <= (((((not IBusSimplePlugin_decompressor_unaligned) and IBusSimplePlugin_decompressor_isInputLowRvc) and (not IBusSimplePlugin_decompressor_isInputHighRvc)) or (IBusSimplePlugin_decompressor_bufferValid and (not IBusSimplePlugin_decompressor_isInputHighRvc))) or ((IBusSimplePlugin_decompressor_throw2Bytes and (not IBusSimplePlugin_decompressor_isRvc)) and (not IBusSimplePlugin_decompressor_isInputHighRvc)));
  when_Fetcher_l283 <= (IBusSimplePlugin_decompressor_output_ready and IBusSimplePlugin_decompressor_input_valid);
  when_Fetcher_l286 <= (IBusSimplePlugin_decompressor_output_ready and IBusSimplePlugin_decompressor_input_valid);
  when_Fetcher_l291 <= (IBusSimplePlugin_externalFlush or IBusSimplePlugin_decompressor_consumeCurrent);
  IBusSimplePlugin_decompressor_output_ready <= ((pkg_toStdLogic(false) and (not IBusSimplePlugin_injector_decodeInput_valid)) or IBusSimplePlugin_injector_decodeInput_ready);
  IBusSimplePlugin_injector_decodeInput_valid <= zz_IBusSimplePlugin_injector_decodeInput_valid;
  IBusSimplePlugin_injector_decodeInput_payload_pc <= zz_IBusSimplePlugin_injector_decodeInput_payload_pc;
  IBusSimplePlugin_injector_decodeInput_payload_rsp_error <= zz_IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  IBusSimplePlugin_injector_decodeInput_payload_rsp_inst <= zz_IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  IBusSimplePlugin_injector_decodeInput_payload_isRvc <= zz_IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  when_Fetcher_l329 <= (not pkg_toStdLogic(false));
  when_Fetcher_l329_1 <= (not execute_arbitration_isStuck);
  when_Fetcher_l329_2 <= (not memory_arbitration_isStuck);
  when_Fetcher_l329_3 <= (not writeBack_arbitration_isStuck);
  IBusSimplePlugin_pcValids_0 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
  IBusSimplePlugin_pcValids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
  IBusSimplePlugin_pcValids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
  IBusSimplePlugin_pcValids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_3;
  IBusSimplePlugin_injector_decodeInput_ready <= (not decode_arbitration_isStuck);
  process(IBusSimplePlugin_injector_decodeInput_valid,switch_Fetcher_l362)
  begin
    decode_arbitration_isValid <= IBusSimplePlugin_injector_decodeInput_valid;
    case switch_Fetcher_l362 is
      when "010" =>
        decode_arbitration_isValid <= pkg_toStdLogic(true);
      when "011" =>
        decode_arbitration_isValid <= pkg_toStdLogic(true);
      when others =>
    end case;
  end process;

  iBus_cmd_valid <= IBusSimplePlugin_cmd_valid;
  IBusSimplePlugin_cmd_ready <= iBus_cmd_ready;
  iBus_cmd_payload_pc <= IBusSimplePlugin_cmd_payload_pc;
  IBusSimplePlugin_pending_next <= ((IBusSimplePlugin_pending_value + pkg_resize(unsigned(pkg_toStdLogicVector(IBusSimplePlugin_pending_inc)),3)) - pkg_resize(unsigned(pkg_toStdLogicVector(IBusSimplePlugin_pending_dec)),3));
  IBusSimplePlugin_cmdFork_canEmit <= (IBusSimplePlugin_iBusRsp_stages_0_output_ready and pkg_toStdLogic(IBusSimplePlugin_pending_value /= pkg_unsigned("111")));
  when_IBusSimplePlugin_l305 <= (IBusSimplePlugin_iBusRsp_stages_0_input_valid and ((not IBusSimplePlugin_cmdFork_canEmit) or (not IBusSimplePlugin_cmd_ready)));
  IBusSimplePlugin_cmd_valid <= (IBusSimplePlugin_iBusRsp_stages_0_input_valid and IBusSimplePlugin_cmdFork_canEmit);
  IBusSimplePlugin_cmd_fire <= (IBusSimplePlugin_cmd_valid and IBusSimplePlugin_cmd_ready);
  IBusSimplePlugin_pending_inc <= IBusSimplePlugin_cmd_fire;
  IBusSimplePlugin_cmd_payload_pc <= unsigned(pkg_cat(std_logic_vector(pkg_extract(IBusSimplePlugin_iBusRsp_stages_0_input_payload,31,2)),std_logic_vector(pkg_unsigned("00"))));
  IBusSimplePlugin_rspJoin_rspBuffer_flush <= (pkg_toStdLogic(IBusSimplePlugin_rspJoin_rspBuffer_discardCounter /= pkg_unsigned("000")) or IBusSimplePlugin_iBusRsp_flush);
  IBusSimplePlugin_rspJoin_rspBuffer_output_valid <= (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid and pkg_toStdLogic(IBusSimplePlugin_rspJoin_rspBuffer_discardCounter = pkg_unsigned("000")));
  IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error <= IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst <= IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_ready <= (IBusSimplePlugin_rspJoin_rspBuffer_output_ready or IBusSimplePlugin_rspJoin_rspBuffer_flush);
  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_fire <= (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid and IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_ready);
  IBusSimplePlugin_pending_dec <= IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_fire;
  IBusSimplePlugin_rspJoin_fetchRsp_pc <= IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  process(IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error,when_IBusSimplePlugin_l376)
  begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error <= IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error;
    if when_IBusSimplePlugin_l376 = '1' then
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error <= pkg_toStdLogic(false);
    end if;
  end process;

  IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst <= IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst;
  when_IBusSimplePlugin_l376 <= (not IBusSimplePlugin_rspJoin_rspBuffer_output_valid);
  IBusSimplePlugin_rspJoin_exceptionDetected <= pkg_toStdLogic(false);
  IBusSimplePlugin_rspJoin_join_valid <= (IBusSimplePlugin_iBusRsp_stages_1_output_valid and IBusSimplePlugin_rspJoin_rspBuffer_output_valid);
  IBusSimplePlugin_rspJoin_join_payload_pc <= IBusSimplePlugin_rspJoin_fetchRsp_pc;
  IBusSimplePlugin_rspJoin_join_payload_rsp_error <= IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  IBusSimplePlugin_rspJoin_join_payload_rsp_inst <= IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  IBusSimplePlugin_rspJoin_join_payload_isRvc <= IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  IBusSimplePlugin_rspJoin_join_fire <= (IBusSimplePlugin_rspJoin_join_valid and IBusSimplePlugin_rspJoin_join_ready);
  IBusSimplePlugin_iBusRsp_stages_1_output_ready <= pkg_mux(IBusSimplePlugin_iBusRsp_stages_1_output_valid,IBusSimplePlugin_rspJoin_join_fire,IBusSimplePlugin_rspJoin_join_ready);
  IBusSimplePlugin_rspJoin_join_fire_1 <= (IBusSimplePlugin_rspJoin_join_valid and IBusSimplePlugin_rspJoin_join_ready);
  IBusSimplePlugin_rspJoin_rspBuffer_output_ready <= IBusSimplePlugin_rspJoin_join_fire_1;
  zz_IBusSimplePlugin_iBusRsp_output_valid <= (not IBusSimplePlugin_rspJoin_exceptionDetected);
  IBusSimplePlugin_rspJoin_join_ready <= (IBusSimplePlugin_iBusRsp_output_ready and zz_IBusSimplePlugin_iBusRsp_output_valid);
  IBusSimplePlugin_iBusRsp_output_valid <= (IBusSimplePlugin_rspJoin_join_valid and zz_IBusSimplePlugin_iBusRsp_output_valid);
  IBusSimplePlugin_iBusRsp_output_payload_pc <= IBusSimplePlugin_rspJoin_join_payload_pc;
  IBusSimplePlugin_iBusRsp_output_payload_rsp_error <= IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  IBusSimplePlugin_iBusRsp_output_payload_rsp_inst <= IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  IBusSimplePlugin_iBusRsp_output_payload_isRvc <= IBusSimplePlugin_rspJoin_join_payload_isRvc;
  zz_dBus_cmd_valid <= pkg_toStdLogic(false);
  process(execute_ALIGNEMENT_FAULT)
  begin
    execute_DBusSimplePlugin_skipCmd <= pkg_toStdLogic(false);
    if execute_ALIGNEMENT_FAULT = '1' then
      execute_DBusSimplePlugin_skipCmd <= pkg_toStdLogic(true);
    end if;
  end process;

  dBus_cmd_valid <= (((((execute_arbitration_isValid and execute_MEMORY_ENABLE) and (not execute_arbitration_isStuckByOthers)) and (not execute_arbitration_isFlushed)) and (not execute_DBusSimplePlugin_skipCmd)) and (not zz_dBus_cmd_valid));
  dBus_cmd_payload_wr <= execute_MEMORY_STORE;
  dBus_cmd_payload_size <= unsigned(pkg_extract(execute_INSTRUCTION,13,12));
  process(dBus_cmd_payload_size,execute_RS2)
  begin
    case dBus_cmd_payload_size is
      when "00" =>
        zz_dBus_cmd_payload_data <= pkg_cat(pkg_cat(pkg_cat(pkg_extract(execute_RS2,7,0),pkg_extract(execute_RS2,7,0)),pkg_extract(execute_RS2,7,0)),pkg_extract(execute_RS2,7,0));
      when "01" =>
        zz_dBus_cmd_payload_data <= pkg_cat(pkg_extract(execute_RS2,15,0),pkg_extract(execute_RS2,15,0));
      when others =>
        zz_dBus_cmd_payload_data <= pkg_extract(execute_RS2,31,0);
    end case;
  end process;

  dBus_cmd_payload_data <= zz_dBus_cmd_payload_data;
  when_DBusSimplePlugin_l428 <= ((((execute_arbitration_isValid and execute_MEMORY_ENABLE) and (not dBus_cmd_ready)) and (not execute_DBusSimplePlugin_skipCmd)) and (not zz_dBus_cmd_valid));
  process(dBus_cmd_payload_size)
  begin
    case dBus_cmd_payload_size is
      when "00" =>
        zz_execute_DBusSimplePlugin_formalMask <= pkg_stdLogicVector("0001");
      when "01" =>
        zz_execute_DBusSimplePlugin_formalMask <= pkg_stdLogicVector("0011");
      when others =>
        zz_execute_DBusSimplePlugin_formalMask <= pkg_stdLogicVector("1111");
    end case;
  end process;

  execute_DBusSimplePlugin_formalMask <= std_logic_vector(shift_left(unsigned(zz_execute_DBusSimplePlugin_formalMask),to_integer(pkg_extract(dBus_cmd_payload_address,1,0))));
  dBus_cmd_payload_address <= unsigned(execute_SRC_ADD);
  when_DBusSimplePlugin_l481 <= (((memory_arbitration_isValid and memory_MEMORY_ENABLE) and (not memory_MEMORY_STORE)) and ((not dBus_rsp_ready) or pkg_toStdLogic(false)));
  process(memory_ALIGNEMENT_FAULT,when_DBusSimplePlugin_l514)
  begin
    DBusSimplePlugin_memoryExceptionPort_valid <= pkg_toStdLogic(false);
    if memory_ALIGNEMENT_FAULT = '1' then
      DBusSimplePlugin_memoryExceptionPort_valid <= pkg_toStdLogic(true);
    end if;
    if when_DBusSimplePlugin_l514 = '1' then
      DBusSimplePlugin_memoryExceptionPort_valid <= pkg_toStdLogic(false);
    end if;
  end process;

  process(memory_ALIGNEMENT_FAULT,memory_MEMORY_STORE)
  begin
    DBusSimplePlugin_memoryExceptionPort_payload_code <= pkg_unsigned("XXXX");
    if memory_ALIGNEMENT_FAULT = '1' then
      DBusSimplePlugin_memoryExceptionPort_payload_code <= pkg_resize(pkg_mux(memory_MEMORY_STORE,pkg_unsigned("110"),pkg_unsigned("100")),4);
    end if;
  end process;

  DBusSimplePlugin_memoryExceptionPort_payload_badAddr <= unsigned(memory_REGFILE_WRITE_DATA);
  when_DBusSimplePlugin_l514 <= (not ((memory_arbitration_isValid and memory_MEMORY_ENABLE) and (pkg_toStdLogic(true) or (not memory_arbitration_isStuckByOthers))));
  process(writeBack_MEMORY_READ_DATA,writeBack_MEMORY_ADDRESS_LOW)
  begin
    writeBack_DBusSimplePlugin_rspShifted <= writeBack_MEMORY_READ_DATA;
    case writeBack_MEMORY_ADDRESS_LOW is
      when "01" =>
        writeBack_DBusSimplePlugin_rspShifted(7 downto 0) <= pkg_extract(writeBack_MEMORY_READ_DATA,15,8);
      when "10" =>
        writeBack_DBusSimplePlugin_rspShifted(15 downto 0) <= pkg_extract(writeBack_MEMORY_READ_DATA,31,16);
      when "11" =>
        writeBack_DBusSimplePlugin_rspShifted(7 downto 0) <= pkg_extract(writeBack_MEMORY_READ_DATA,31,24);
      when others =>
    end case;
  end process;

  switch_Misc_l204_2 <= pkg_extract(writeBack_INSTRUCTION,13,12);
  zz_writeBack_DBusSimplePlugin_rspFormated <= (pkg_extract(writeBack_DBusSimplePlugin_rspShifted,7) and (not pkg_extract(writeBack_INSTRUCTION,14)));
  process(zz_writeBack_DBusSimplePlugin_rspFormated,writeBack_DBusSimplePlugin_rspShifted)
  begin
    zz_writeBack_DBusSimplePlugin_rspFormated_1(31) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(30) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(29) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(28) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(27) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(26) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(25) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(24) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(23) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(22) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(21) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(20) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(19) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(18) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(17) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(16) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(15) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(14) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(13) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(12) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(11) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(10) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(9) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(8) <= zz_writeBack_DBusSimplePlugin_rspFormated;
    zz_writeBack_DBusSimplePlugin_rspFormated_1(7 downto 0) <= pkg_extract(writeBack_DBusSimplePlugin_rspShifted,7,0);
  end process;

  zz_writeBack_DBusSimplePlugin_rspFormated_2 <= (pkg_extract(writeBack_DBusSimplePlugin_rspShifted,15) and (not pkg_extract(writeBack_INSTRUCTION,14)));
  process(zz_writeBack_DBusSimplePlugin_rspFormated_2,writeBack_DBusSimplePlugin_rspShifted)
  begin
    zz_writeBack_DBusSimplePlugin_rspFormated_3(31) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(30) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(29) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(28) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(27) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(26) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(25) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(24) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(23) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(22) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(21) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(20) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(19) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(18) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(17) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(16) <= zz_writeBack_DBusSimplePlugin_rspFormated_2;
    zz_writeBack_DBusSimplePlugin_rspFormated_3(15 downto 0) <= pkg_extract(writeBack_DBusSimplePlugin_rspShifted,15,0);
  end process;

  process(switch_Misc_l204_2,zz_writeBack_DBusSimplePlugin_rspFormated_1,zz_writeBack_DBusSimplePlugin_rspFormated_3,writeBack_DBusSimplePlugin_rspShifted)
  begin
    case switch_Misc_l204_2 is
      when "00" =>
        writeBack_DBusSimplePlugin_rspFormated <= zz_writeBack_DBusSimplePlugin_rspFormated_1;
      when "01" =>
        writeBack_DBusSimplePlugin_rspFormated <= zz_writeBack_DBusSimplePlugin_rspFormated_3;
      when others =>
        writeBack_DBusSimplePlugin_rspFormated <= writeBack_DBusSimplePlugin_rspShifted;
    end case;
  end process;

  when_DBusSimplePlugin_l560 <= (writeBack_arbitration_isValid and writeBack_MEMORY_ENABLE);
  process(CsrPlugin_forceMachineWire)
  begin
    CsrPlugin_privilege <= pkg_unsigned("11");
    if CsrPlugin_forceMachineWire = '1' then
      CsrPlugin_privilege <= pkg_unsigned("11");
    end if;
  end process;

  CsrPlugin_misa_base <= pkg_unsigned("01");
  CsrPlugin_misa_extensions <= pkg_stdLogicVector("00000000000000000001000010");
  zz_when_CsrPlugin_l952_4 <= (CsrPlugin_mip_MTIP and CsrPlugin_mie_MTIE);
  zz_when_CsrPlugin_l952_5 <= (CsrPlugin_mip_MSIP and CsrPlugin_mie_MSIE);
  zz_when_CsrPlugin_l952_6 <= (CsrPlugin_mip_MEIP and CsrPlugin_mie_MEIE);
  CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped <= pkg_unsigned("11");
  CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege <= pkg_mux(pkg_toStdLogic(CsrPlugin_privilege < CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped),CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped,CsrPlugin_privilege);
  process(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode,decodeExceptionPort_valid,decode_arbitration_isFlushed)
  begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_decode <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
    if decodeExceptionPort_valid = '1' then
      CsrPlugin_exceptionPortCtrl_exceptionValids_decode <= pkg_toStdLogic(true);
    end if;
    if decode_arbitration_isFlushed = '1' then
      CsrPlugin_exceptionPortCtrl_exceptionValids_decode <= pkg_toStdLogic(false);
    end if;
  end process;

  process(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute,CsrPlugin_selfException_valid,execute_arbitration_isFlushed)
  begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_execute <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
    if CsrPlugin_selfException_valid = '1' then
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute <= pkg_toStdLogic(true);
    end if;
    if execute_arbitration_isFlushed = '1' then
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute <= pkg_toStdLogic(false);
    end if;
  end process;

  process(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory,DBusSimplePlugin_memoryExceptionPort_valid,memory_arbitration_isFlushed)
  begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_memory <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
    if DBusSimplePlugin_memoryExceptionPort_valid = '1' then
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory <= pkg_toStdLogic(true);
    end if;
    if memory_arbitration_isFlushed = '1' then
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory <= pkg_toStdLogic(false);
    end if;
  end process;

  process(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack,writeBack_arbitration_isFlushed)
  begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
    if writeBack_arbitration_isFlushed = '1' then
      CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack <= pkg_toStdLogic(false);
    end if;
  end process;

  when_CsrPlugin_l909 <= (not decode_arbitration_isStuck);
  when_CsrPlugin_l909_1 <= (not execute_arbitration_isStuck);
  when_CsrPlugin_l909_2 <= (not memory_arbitration_isStuck);
  when_CsrPlugin_l909_3 <= (not writeBack_arbitration_isStuck);
  when_CsrPlugin_l922 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack),pkg_cat(pkg_toStdLogicVector(CsrPlugin_exceptionPortCtrl_exceptionValids_memory),pkg_cat(pkg_toStdLogicVector(CsrPlugin_exceptionPortCtrl_exceptionValids_execute),pkg_toStdLogicVector(CsrPlugin_exceptionPortCtrl_exceptionValids_decode)))) /= pkg_stdLogicVector("0000"));
  CsrPlugin_exceptionPendings_0 <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  CsrPlugin_exceptionPendings_1 <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  CsrPlugin_exceptionPendings_2 <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  CsrPlugin_exceptionPendings_3 <= CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  when_CsrPlugin_l946 <= (CsrPlugin_mstatus_MIE or pkg_toStdLogic(CsrPlugin_privilege < pkg_unsigned("11")));
  when_CsrPlugin_l952 <= ((zz_when_CsrPlugin_l952 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  when_CsrPlugin_l952_1 <= ((zz_when_CsrPlugin_l952_1 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  when_CsrPlugin_l952_2 <= ((zz_when_CsrPlugin_l952_2 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  when_CsrPlugin_l952_3 <= ((zz_when_CsrPlugin_l952_3 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  when_CsrPlugin_l952_4 <= ((zz_when_CsrPlugin_l952_4 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  when_CsrPlugin_l952_5 <= ((zz_when_CsrPlugin_l952_5 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  when_CsrPlugin_l952_6 <= ((zz_when_CsrPlugin_l952_6 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  CsrPlugin_exception <= (CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack and CsrPlugin_allowException);
  CsrPlugin_lastStageWasWfi <= pkg_toStdLogic(false);
  CsrPlugin_pipelineLiberator_active <= ((CsrPlugin_interrupt_valid and CsrPlugin_allowInterrupts) and decode_arbitration_isValid);
  when_CsrPlugin_l980 <= (not execute_arbitration_isStuck);
  when_CsrPlugin_l980_1 <= (not memory_arbitration_isStuck);
  when_CsrPlugin_l980_2 <= (not writeBack_arbitration_isStuck);
  when_CsrPlugin_l985 <= ((not CsrPlugin_pipelineLiberator_active) or decode_arbitration_removeIt);
  process(CsrPlugin_pipelineLiberator_pcValids_2,when_CsrPlugin_l991,CsrPlugin_hadException)
  begin
    CsrPlugin_pipelineLiberator_done <= CsrPlugin_pipelineLiberator_pcValids_2;
    if when_CsrPlugin_l991 = '1' then
      CsrPlugin_pipelineLiberator_done <= pkg_toStdLogic(false);
    end if;
    if CsrPlugin_hadException = '1' then
      CsrPlugin_pipelineLiberator_done <= pkg_toStdLogic(false);
    end if;
  end process;

  when_CsrPlugin_l991 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack),pkg_cat(pkg_toStdLogicVector(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory),pkg_toStdLogicVector(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute))) /= pkg_stdLogicVector("000"));
  CsrPlugin_interruptJump <= ((CsrPlugin_interrupt_valid and CsrPlugin_pipelineLiberator_done) and CsrPlugin_allowInterrupts);
  process(CsrPlugin_interrupt_targetPrivilege,CsrPlugin_hadException,CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege)
  begin
    CsrPlugin_targetPrivilege <= CsrPlugin_interrupt_targetPrivilege;
    if CsrPlugin_hadException = '1' then
      CsrPlugin_targetPrivilege <= CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
    end if;
  end process;

  process(CsrPlugin_interrupt_code,CsrPlugin_hadException,CsrPlugin_exceptionPortCtrl_exceptionContext_code)
  begin
    CsrPlugin_trapCause <= pkg_resize(CsrPlugin_interrupt_code,5);
    if CsrPlugin_hadException = '1' then
      CsrPlugin_trapCause <= pkg_resize(CsrPlugin_exceptionPortCtrl_exceptionContext_code,5);
    end if;
  end process;

  process(CsrPlugin_targetPrivilege,CsrPlugin_mtvec_mode)
  begin
    CsrPlugin_xtvec_mode <= pkg_stdLogicVector("XX");
    case CsrPlugin_targetPrivilege is
      when "11" =>
        CsrPlugin_xtvec_mode <= CsrPlugin_mtvec_mode;
      when others =>
    end case;
  end process;

  process(CsrPlugin_targetPrivilege,CsrPlugin_mtvec_base)
  begin
    CsrPlugin_xtvec_base <= pkg_unsigned("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    case CsrPlugin_targetPrivilege is
      when "11" =>
        CsrPlugin_xtvec_base <= CsrPlugin_mtvec_base;
      when others =>
    end case;
  end process;

  when_CsrPlugin_l1019 <= (CsrPlugin_hadException or CsrPlugin_interruptJump);
  when_CsrPlugin_l1064 <= (writeBack_arbitration_isValid and pkg_toStdLogic(writeBack_ENV_CTRL = EnvCtrlEnum_seq_XRET));
  switch_CsrPlugin_l1068 <= pkg_extract(writeBack_INSTRUCTION,29,28);
  contextSwitching <= CsrPlugin_jumpInterface_valid;
  when_CsrPlugin_l1116 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector((writeBack_arbitration_isValid and pkg_toStdLogic(writeBack_ENV_CTRL = EnvCtrlEnum_seq_XRET))),pkg_cat(pkg_toStdLogicVector((memory_arbitration_isValid and pkg_toStdLogic(memory_ENV_CTRL = EnvCtrlEnum_seq_XRET))),pkg_toStdLogicVector((execute_arbitration_isValid and pkg_toStdLogic(execute_ENV_CTRL = EnvCtrlEnum_seq_XRET))))) /= pkg_stdLogicVector("000"));
  execute_CsrPlugin_blockedBySideEffects <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_isValid),pkg_toStdLogicVector(memory_arbitration_isValid)) /= pkg_stdLogicVector("00")) or pkg_toStdLogic(false));
  process(execute_CsrPlugin_csr_836,execute_CsrPlugin_csr_772,execute_CsrPlugin_csr_768,execute_CsrPlugin_csr_773,execute_CsrPlugin_csr_833,execute_CsrPlugin_csr_834,execute_CSR_READ_OPCODE,CsrPlugin_csrMapping_allowCsrSignal,when_CsrPlugin_l1297,when_CsrPlugin_l1302)
  begin
    execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(true);
    if execute_CsrPlugin_csr_836 = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
    if execute_CsrPlugin_csr_772 = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
    if execute_CsrPlugin_csr_768 = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
    if execute_CsrPlugin_csr_773 = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
    if execute_CsrPlugin_csr_833 = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
    if execute_CsrPlugin_csr_834 = '1' then
      if execute_CSR_READ_OPCODE = '1' then
        execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
      end if;
    end if;
    if CsrPlugin_csrMapping_allowCsrSignal = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
    if when_CsrPlugin_l1297 = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(true);
    end if;
    if when_CsrPlugin_l1302 = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
  end process;

  process(when_CsrPlugin_l1136,when_CsrPlugin_l1137)
  begin
    execute_CsrPlugin_illegalInstruction <= pkg_toStdLogic(false);
    if when_CsrPlugin_l1136 = '1' then
      if when_CsrPlugin_l1137 = '1' then
        execute_CsrPlugin_illegalInstruction <= pkg_toStdLogic(true);
      end if;
    end if;
  end process;

  process(when_CsrPlugin_l1144)
  begin
    CsrPlugin_selfException_valid <= pkg_toStdLogic(false);
    if when_CsrPlugin_l1144 = '1' then
      CsrPlugin_selfException_valid <= pkg_toStdLogic(true);
    end if;
  end process;

  process(when_CsrPlugin_l1144,CsrPlugin_privilege)
  begin
    CsrPlugin_selfException_payload_code <= pkg_unsigned("XXXX");
    if when_CsrPlugin_l1144 = '1' then
      case CsrPlugin_privilege is
        when "00" =>
          CsrPlugin_selfException_payload_code <= pkg_unsigned("1000");
        when others =>
          CsrPlugin_selfException_payload_code <= pkg_unsigned("1011");
      end case;
    end if;
  end process;

  CsrPlugin_selfException_payload_badAddr <= unsigned(execute_INSTRUCTION);
  when_CsrPlugin_l1136 <= (execute_arbitration_isValid and pkg_toStdLogic(execute_ENV_CTRL = EnvCtrlEnum_seq_XRET));
  when_CsrPlugin_l1137 <= pkg_toStdLogic(CsrPlugin_privilege < unsigned(pkg_extract(execute_INSTRUCTION,29,28)));
  when_CsrPlugin_l1144 <= (execute_arbitration_isValid and pkg_toStdLogic(execute_ENV_CTRL = EnvCtrlEnum_seq_ECALL));
  process(execute_arbitration_isValid,execute_IS_CSR,execute_CSR_WRITE_OPCODE,when_CsrPlugin_l1297)
  begin
    execute_CsrPlugin_writeInstruction <= ((execute_arbitration_isValid and execute_IS_CSR) and execute_CSR_WRITE_OPCODE);
    if when_CsrPlugin_l1297 = '1' then
      execute_CsrPlugin_writeInstruction <= pkg_toStdLogic(false);
    end if;
  end process;

  process(execute_arbitration_isValid,execute_IS_CSR,execute_CSR_READ_OPCODE,when_CsrPlugin_l1297)
  begin
    execute_CsrPlugin_readInstruction <= ((execute_arbitration_isValid and execute_IS_CSR) and execute_CSR_READ_OPCODE);
    if when_CsrPlugin_l1297 = '1' then
      execute_CsrPlugin_readInstruction <= pkg_toStdLogic(false);
    end if;
  end process;

  execute_CsrPlugin_writeEnable <= (execute_CsrPlugin_writeInstruction and (not execute_arbitration_isStuck));
  execute_CsrPlugin_readEnable <= (execute_CsrPlugin_readInstruction and (not execute_arbitration_isStuck));
  CsrPlugin_csrMapping_hazardFree <= (not execute_CsrPlugin_blockedBySideEffects);
  execute_CsrPlugin_readToWriteData <= CsrPlugin_csrMapping_readDataSignal;
  switch_Misc_l204_3 <= pkg_extract(execute_INSTRUCTION,13);
  process(switch_Misc_l204_3,execute_SRC1,execute_INSTRUCTION,execute_CsrPlugin_readToWriteData)
  begin
    case switch_Misc_l204_3 is
      when '0' =>
        zz_CsrPlugin_csrMapping_writeDataSignal <= execute_SRC1;
      when others =>
        zz_CsrPlugin_csrMapping_writeDataSignal <= pkg_mux(pkg_extract(execute_INSTRUCTION,12),(execute_CsrPlugin_readToWriteData and pkg_not(execute_SRC1)),(execute_CsrPlugin_readToWriteData or execute_SRC1));
    end case;
  end process;

  CsrPlugin_csrMapping_writeDataSignal <= zz_CsrPlugin_csrMapping_writeDataSignal;
  when_CsrPlugin_l1176 <= (execute_arbitration_isValid and execute_IS_CSR);
  when_CsrPlugin_l1180 <= (execute_arbitration_isValid and (execute_IS_CSR or pkg_toStdLogic(false)));
  execute_CsrPlugin_csrAddress <= pkg_extract(execute_INSTRUCTION,31,20);
  zz_decode_BRANCH_CTRL_3 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000100000001010000")) = pkg_stdLogicVector("00000000000000000100000001010000"));
  zz_decode_BRANCH_CTRL_4 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000011000")) = pkg_stdLogicVector("00000000000000000000000000000000"));
  zz_decode_BRANCH_CTRL_5 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000000100")) = pkg_stdLogicVector("00000000000000000000000000000100"));
  zz_decode_BRANCH_CTRL_6 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001001000")) = pkg_stdLogicVector("00000000000000000000000001001000"));
  zz_decode_BRANCH_CTRL_7 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000001000000000000")) = pkg_stdLogicVector("00000000000000000000000000000000"));
  zz_decode_BRANCH_CTRL_2 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_decode_BRANCH_CTRL_6),pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2 = zz_zz_decode_BRANCH_CTRL_2_1))) /= pkg_stdLogicVector("00"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_2 = zz_zz_decode_BRANCH_CTRL_2_3)) /= pkg_stdLogicVector("0"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_4) /= pkg_stdLogicVector("0"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_zz_decode_BRANCH_CTRL_2_5 /= zz_zz_decode_BRANCH_CTRL_2_10)),pkg_cat(pkg_toStdLogicVector(zz_zz_decode_BRANCH_CTRL_2_11),pkg_cat(zz_zz_decode_BRANCH_CTRL_2_17,zz_zz_decode_BRANCH_CTRL_2_18))))));
  zz_decode_SRC1_CTRL_2 <= pkg_extract(zz_decode_BRANCH_CTRL_2,1,0);
  zz_decode_SRC1_CTRL_1 <= zz_decode_SRC1_CTRL_2;
  zz_decode_ALU_CTRL_2 <= pkg_extract(zz_decode_BRANCH_CTRL_2,6,5);
  zz_decode_ALU_CTRL_1 <= zz_decode_ALU_CTRL_2;
  zz_decode_SRC2_CTRL_2 <= pkg_extract(zz_decode_BRANCH_CTRL_2,8,7);
  zz_decode_SRC2_CTRL_1 <= zz_decode_SRC2_CTRL_2;
  zz_decode_ENV_CTRL_2 <= pkg_extract(zz_decode_BRANCH_CTRL_2,17,16);
  zz_decode_ENV_CTRL_1 <= zz_decode_ENV_CTRL_2;
  zz_decode_ALU_BITWISE_CTRL_2 <= pkg_extract(zz_decode_BRANCH_CTRL_2,20,19);
  zz_decode_ALU_BITWISE_CTRL_1 <= zz_decode_ALU_BITWISE_CTRL_2;
  zz_decode_SHIFT_CTRL_2 <= pkg_extract(zz_decode_BRANCH_CTRL_2,27,26);
  zz_decode_SHIFT_CTRL_1 <= zz_decode_SHIFT_CTRL_2;
  zz_decode_BRANCH_CTRL_8 <= pkg_extract(zz_decode_BRANCH_CTRL_2,30,29);
  zz_decode_BRANCH_CTRL_1 <= zz_decode_BRANCH_CTRL_8;
  decodeExceptionPort_valid <= (decode_arbitration_isValid and (not decode_LEGAL_INSTRUCTION));
  decodeExceptionPort_payload_code <= pkg_unsigned("0010");
  decodeExceptionPort_payload_badAddr <= unsigned(decode_INSTRUCTION);
  when_RegFilePlugin_l63 <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,11,7) = pkg_stdLogicVector("00000"));
  decode_RegFilePlugin_regFileReadAddress1 <= unsigned(pkg_extract(decode_INSTRUCTION_ANTICIPATED,19,15));
  decode_RegFilePlugin_regFileReadAddress2 <= unsigned(pkg_extract(decode_INSTRUCTION_ANTICIPATED,24,20));
  decode_RegFilePlugin_rs1Data <= zz_RegFilePlugin_regFile_port0;
  decode_RegFilePlugin_rs2Data <= zz_RegFilePlugin_regFile_port0_1;
  process(zz_lastStageRegFileWrite_valid,writeBack_arbitration_isFiring,zz_2)
  begin
    lastStageRegFileWrite_valid <= (zz_lastStageRegFileWrite_valid and writeBack_arbitration_isFiring);
    if zz_2 = '1' then
      lastStageRegFileWrite_valid <= pkg_toStdLogic(true);
    end if;
  end process;

  process(zz_lastStageRegFileWrite_payload_address,zz_2)
  begin
    lastStageRegFileWrite_payload_address <= unsigned(pkg_extract(zz_lastStageRegFileWrite_payload_address,11,7));
    if zz_2 = '1' then
      lastStageRegFileWrite_payload_address <= pkg_unsigned("00000");
    end if;
  end process;

  process(zz_decode_RS2_2,zz_2)
  begin
    lastStageRegFileWrite_payload_data <= zz_decode_RS2_2;
    if zz_2 = '1' then
      lastStageRegFileWrite_payload_data <= pkg_stdLogicVector("00000000000000000000000000000000");
    end if;
  end process;

  process(execute_ALU_BITWISE_CTRL,execute_SRC1,execute_SRC2)
  begin
    case execute_ALU_BITWISE_CTRL is
      when AluBitwiseCtrlEnum_seq_AND_1 =>
        execute_IntAluPlugin_bitwise <= (execute_SRC1 and execute_SRC2);
      when AluBitwiseCtrlEnum_seq_OR_1 =>
        execute_IntAluPlugin_bitwise <= (execute_SRC1 or execute_SRC2);
      when others =>
        execute_IntAluPlugin_bitwise <= (execute_SRC1 xor execute_SRC2);
    end case;
  end process;

  process(execute_ALU_CTRL,execute_IntAluPlugin_bitwise,execute_SRC_LESS,execute_SRC_ADD_SUB)
  begin
    case execute_ALU_CTRL is
      when AluCtrlEnum_seq_BITWISE =>
        zz_execute_REGFILE_WRITE_DATA <= execute_IntAluPlugin_bitwise;
      when AluCtrlEnum_seq_SLT_SLTU =>
        zz_execute_REGFILE_WRITE_DATA <= pkg_resize(pkg_toStdLogicVector(execute_SRC_LESS),32);
      when others =>
        zz_execute_REGFILE_WRITE_DATA <= execute_SRC_ADD_SUB;
    end case;
  end process;

  execute_MulPlugin_a <= execute_RS1;
  execute_MulPlugin_b <= execute_RS2;
  switch_MulPlugin_l87 <= pkg_extract(execute_INSTRUCTION,13,12);
  process(switch_MulPlugin_l87)
  begin
    case switch_MulPlugin_l87 is
      when "01" =>
        execute_MulPlugin_aSigned <= pkg_toStdLogic(true);
      when "10" =>
        execute_MulPlugin_aSigned <= pkg_toStdLogic(true);
      when others =>
        execute_MulPlugin_aSigned <= pkg_toStdLogic(false);
    end case;
  end process;

  process(switch_MulPlugin_l87)
  begin
    case switch_MulPlugin_l87 is
      when "01" =>
        execute_MulPlugin_bSigned <= pkg_toStdLogic(true);
      when "10" =>
        execute_MulPlugin_bSigned <= pkg_toStdLogic(false);
      when others =>
        execute_MulPlugin_bSigned <= pkg_toStdLogic(false);
    end case;
  end process;

  execute_MulPlugin_aULow <= unsigned(pkg_extract(execute_MulPlugin_a,15,0));
  execute_MulPlugin_bULow <= unsigned(pkg_extract(execute_MulPlugin_b,15,0));
  execute_MulPlugin_aSLow <= signed(pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(false)),pkg_extract(execute_MulPlugin_a,15,0)));
  execute_MulPlugin_bSLow <= signed(pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(false)),pkg_extract(execute_MulPlugin_b,15,0)));
  execute_MulPlugin_aHigh <= signed(pkg_cat(pkg_toStdLogicVector((execute_MulPlugin_aSigned and pkg_extract(execute_MulPlugin_a,31))),pkg_extract(execute_MulPlugin_a,31,16)));
  execute_MulPlugin_bHigh <= signed(pkg_cat(pkg_toStdLogicVector((execute_MulPlugin_bSigned and pkg_extract(execute_MulPlugin_b,31))),pkg_extract(execute_MulPlugin_b,31,16)));
  writeBack_MulPlugin_result <= (pkg_resize(writeBack_MUL_LOW,66) + pkg_shiftLeft(writeBack_MUL_HH,32));
  when_MulPlugin_l147 <= (writeBack_arbitration_isValid and writeBack_IS_MUL);
  switch_MulPlugin_l148 <= pkg_extract(writeBack_INSTRUCTION,13,12);
  memory_DivPlugin_frontendOk <= pkg_toStdLogic(true);
  process(when_MulDivIterativePlugin_l128,when_MulDivIterativePlugin_l132)
  begin
    memory_DivPlugin_div_counter_willIncrement <= pkg_toStdLogic(false);
    if when_MulDivIterativePlugin_l128 = '1' then
      if when_MulDivIterativePlugin_l132 = '1' then
        memory_DivPlugin_div_counter_willIncrement <= pkg_toStdLogic(true);
      end if;
    end if;
  end process;

  process(when_MulDivIterativePlugin_l162)
  begin
    memory_DivPlugin_div_counter_willClear <= pkg_toStdLogic(false);
    if when_MulDivIterativePlugin_l162 = '1' then
      memory_DivPlugin_div_counter_willClear <= pkg_toStdLogic(true);
    end if;
  end process;

  memory_DivPlugin_div_counter_willOverflowIfInc <= pkg_toStdLogic(memory_DivPlugin_div_counter_value = pkg_unsigned("100001"));
  memory_DivPlugin_div_counter_willOverflow <= (memory_DivPlugin_div_counter_willOverflowIfInc and memory_DivPlugin_div_counter_willIncrement);
  process(memory_DivPlugin_div_counter_willOverflow,memory_DivPlugin_div_counter_value,memory_DivPlugin_div_counter_willIncrement,memory_DivPlugin_div_counter_willClear)
  begin
    if memory_DivPlugin_div_counter_willOverflow = '1' then
      memory_DivPlugin_div_counter_valueNext <= pkg_unsigned("000000");
    else
      memory_DivPlugin_div_counter_valueNext <= (memory_DivPlugin_div_counter_value + pkg_resize(unsigned(pkg_toStdLogicVector(memory_DivPlugin_div_counter_willIncrement)),6));
    end if;
    if memory_DivPlugin_div_counter_willClear = '1' then
      memory_DivPlugin_div_counter_valueNext <= pkg_unsigned("000000");
    end if;
  end process;

  when_MulDivIterativePlugin_l126 <= pkg_toStdLogic(memory_DivPlugin_div_counter_value = pkg_unsigned("100000"));
  when_MulDivIterativePlugin_l126_1 <= (not memory_arbitration_isStuck);
  when_MulDivIterativePlugin_l128 <= (memory_arbitration_isValid and memory_IS_DIV);
  when_MulDivIterativePlugin_l129 <= ((not memory_DivPlugin_frontendOk) or (not memory_DivPlugin_div_done));
  when_MulDivIterativePlugin_l132 <= (memory_DivPlugin_frontendOk and (not memory_DivPlugin_div_done));
  zz_memory_DivPlugin_div_stage_0_remainderShifted <= pkg_extract(memory_DivPlugin_rs1,31,0);
  memory_DivPlugin_div_stage_0_remainderShifted <= unsigned(pkg_cat(std_logic_vector(pkg_extract(memory_DivPlugin_accumulator,31,0)),pkg_toStdLogicVector(pkg_extract(zz_memory_DivPlugin_div_stage_0_remainderShifted,31))));
  memory_DivPlugin_div_stage_0_remainderMinusDenominator <= (memory_DivPlugin_div_stage_0_remainderShifted - pkg_resize(memory_DivPlugin_rs2,33));
  memory_DivPlugin_div_stage_0_outRemainder <= pkg_mux((not pkg_extract(memory_DivPlugin_div_stage_0_remainderMinusDenominator,32)),pkg_resize(memory_DivPlugin_div_stage_0_remainderMinusDenominator,32),pkg_resize(memory_DivPlugin_div_stage_0_remainderShifted,32));
  memory_DivPlugin_div_stage_0_outNumerator <= pkg_resize(unsigned(pkg_cat(std_logic_vector(zz_memory_DivPlugin_div_stage_0_remainderShifted),pkg_toStdLogicVector((not pkg_extract(memory_DivPlugin_div_stage_0_remainderMinusDenominator,32))))),32);
  when_MulDivIterativePlugin_l151 <= pkg_toStdLogic(memory_DivPlugin_div_counter_value = pkg_unsigned("100000"));
  zz_memory_DivPlugin_div_result <= pkg_mux(pkg_extract(memory_INSTRUCTION,13),pkg_extract(memory_DivPlugin_accumulator,31,0),pkg_extract(memory_DivPlugin_rs1,31,0));
  when_MulDivIterativePlugin_l162 <= (not memory_arbitration_isStuck);
  zz_memory_DivPlugin_rs2 <= (pkg_extract(execute_RS2,31) and execute_IS_RS2_SIGNED);
  zz_memory_DivPlugin_rs1 <= (pkg_toStdLogic(false) or ((execute_IS_DIV and pkg_extract(execute_RS1,31)) and execute_IS_RS1_SIGNED));
  process(execute_IS_RS1_SIGNED,execute_RS1)
  begin
    zz_memory_DivPlugin_rs1_1(32) <= (execute_IS_RS1_SIGNED and pkg_extract(execute_RS1,31));
    zz_memory_DivPlugin_rs1_1(31 downto 0) <= execute_RS1;
  end process;

  process(decode_SRC1_CTRL,zz_decode_SRC1,decode_IS_RVC,decode_INSTRUCTION)
  begin
    case decode_SRC1_CTRL is
      when Src1CtrlEnum_seq_RS =>
        zz_decode_SRC1_1 <= zz_decode_SRC1;
      when Src1CtrlEnum_seq_PC_INCREMENT =>
        zz_decode_SRC1_1 <= pkg_resize(pkg_mux(decode_IS_RVC,pkg_stdLogicVector("010"),pkg_stdLogicVector("100")),32);
      when Src1CtrlEnum_seq_IMU =>
        zz_decode_SRC1_1 <= pkg_cat(pkg_extract(decode_INSTRUCTION,31,12),std_logic_vector(pkg_unsigned("000000000000")));
      when others =>
        zz_decode_SRC1_1 <= pkg_resize(pkg_extract(decode_INSTRUCTION,19,15),32);
    end case;
  end process;

  zz_decode_SRC2_2 <= pkg_extract(decode_INSTRUCTION,31);
  process(zz_decode_SRC2_2)
  begin
    zz_decode_SRC2_3(19) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(18) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(17) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(16) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(15) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(14) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(13) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(12) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(11) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(10) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(9) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(8) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(7) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(6) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(5) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(4) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(3) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(2) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(1) <= zz_decode_SRC2_2;
    zz_decode_SRC2_3(0) <= zz_decode_SRC2_2;
  end process;

  zz_decode_SRC2_4 <= pkg_extract(pkg_cat(pkg_extract(decode_INSTRUCTION,31,25),pkg_extract(decode_INSTRUCTION,11,7)),11);
  process(zz_decode_SRC2_4)
  begin
    zz_decode_SRC2_5(19) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(18) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(17) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(16) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(15) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(14) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(13) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(12) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(11) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(10) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(9) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(8) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(7) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(6) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(5) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(4) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(3) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(2) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(1) <= zz_decode_SRC2_4;
    zz_decode_SRC2_5(0) <= zz_decode_SRC2_4;
  end process;

  process(decode_SRC2_CTRL,zz_decode_SRC2_1,zz_decode_SRC2_3,decode_INSTRUCTION,zz_decode_SRC2_5,zz_decode_SRC2)
  begin
    case decode_SRC2_CTRL is
      when Src2CtrlEnum_seq_RS =>
        zz_decode_SRC2_6 <= zz_decode_SRC2_1;
      when Src2CtrlEnum_seq_IMI =>
        zz_decode_SRC2_6 <= pkg_cat(zz_decode_SRC2_3,pkg_extract(decode_INSTRUCTION,31,20));
      when Src2CtrlEnum_seq_IMS =>
        zz_decode_SRC2_6 <= pkg_cat(zz_decode_SRC2_5,pkg_cat(pkg_extract(decode_INSTRUCTION,31,25),pkg_extract(decode_INSTRUCTION,11,7)));
      when others =>
        zz_decode_SRC2_6 <= std_logic_vector(zz_decode_SRC2);
    end case;
  end process;

  process(execute_SRC1,execute_SRC_USE_SUB_LESS,execute_SRC2,execute_SRC2_FORCE_ZERO)
  begin
    execute_SrcPlugin_addSub <= std_logic_vector(((signed(execute_SRC1) + signed(pkg_mux(execute_SRC_USE_SUB_LESS,pkg_not(execute_SRC2),execute_SRC2))) + pkg_mux(execute_SRC_USE_SUB_LESS,pkg_signed("00000000000000000000000000000001"),pkg_signed("00000000000000000000000000000000"))));
    if execute_SRC2_FORCE_ZERO = '1' then
      execute_SrcPlugin_addSub <= execute_SRC1;
    end if;
  end process;

  execute_SrcPlugin_less <= pkg_mux(pkg_toStdLogic(pkg_extract(execute_SRC1,31) = pkg_extract(execute_SRC2,31)),pkg_extract(execute_SrcPlugin_addSub,31),pkg_mux(execute_SRC_LESS_UNSIGNED,pkg_extract(execute_SRC2,31),pkg_extract(execute_SRC1,31)));
  execute_FullBarrelShifterPlugin_amplitude <= unsigned(pkg_extract(execute_SRC2,4,0));
  process(execute_SRC1)
  begin
    zz_execute_FullBarrelShifterPlugin_reversed(0) <= pkg_extract(execute_SRC1,31);
    zz_execute_FullBarrelShifterPlugin_reversed(1) <= pkg_extract(execute_SRC1,30);
    zz_execute_FullBarrelShifterPlugin_reversed(2) <= pkg_extract(execute_SRC1,29);
    zz_execute_FullBarrelShifterPlugin_reversed(3) <= pkg_extract(execute_SRC1,28);
    zz_execute_FullBarrelShifterPlugin_reversed(4) <= pkg_extract(execute_SRC1,27);
    zz_execute_FullBarrelShifterPlugin_reversed(5) <= pkg_extract(execute_SRC1,26);
    zz_execute_FullBarrelShifterPlugin_reversed(6) <= pkg_extract(execute_SRC1,25);
    zz_execute_FullBarrelShifterPlugin_reversed(7) <= pkg_extract(execute_SRC1,24);
    zz_execute_FullBarrelShifterPlugin_reversed(8) <= pkg_extract(execute_SRC1,23);
    zz_execute_FullBarrelShifterPlugin_reversed(9) <= pkg_extract(execute_SRC1,22);
    zz_execute_FullBarrelShifterPlugin_reversed(10) <= pkg_extract(execute_SRC1,21);
    zz_execute_FullBarrelShifterPlugin_reversed(11) <= pkg_extract(execute_SRC1,20);
    zz_execute_FullBarrelShifterPlugin_reversed(12) <= pkg_extract(execute_SRC1,19);
    zz_execute_FullBarrelShifterPlugin_reversed(13) <= pkg_extract(execute_SRC1,18);
    zz_execute_FullBarrelShifterPlugin_reversed(14) <= pkg_extract(execute_SRC1,17);
    zz_execute_FullBarrelShifterPlugin_reversed(15) <= pkg_extract(execute_SRC1,16);
    zz_execute_FullBarrelShifterPlugin_reversed(16) <= pkg_extract(execute_SRC1,15);
    zz_execute_FullBarrelShifterPlugin_reversed(17) <= pkg_extract(execute_SRC1,14);
    zz_execute_FullBarrelShifterPlugin_reversed(18) <= pkg_extract(execute_SRC1,13);
    zz_execute_FullBarrelShifterPlugin_reversed(19) <= pkg_extract(execute_SRC1,12);
    zz_execute_FullBarrelShifterPlugin_reversed(20) <= pkg_extract(execute_SRC1,11);
    zz_execute_FullBarrelShifterPlugin_reversed(21) <= pkg_extract(execute_SRC1,10);
    zz_execute_FullBarrelShifterPlugin_reversed(22) <= pkg_extract(execute_SRC1,9);
    zz_execute_FullBarrelShifterPlugin_reversed(23) <= pkg_extract(execute_SRC1,8);
    zz_execute_FullBarrelShifterPlugin_reversed(24) <= pkg_extract(execute_SRC1,7);
    zz_execute_FullBarrelShifterPlugin_reversed(25) <= pkg_extract(execute_SRC1,6);
    zz_execute_FullBarrelShifterPlugin_reversed(26) <= pkg_extract(execute_SRC1,5);
    zz_execute_FullBarrelShifterPlugin_reversed(27) <= pkg_extract(execute_SRC1,4);
    zz_execute_FullBarrelShifterPlugin_reversed(28) <= pkg_extract(execute_SRC1,3);
    zz_execute_FullBarrelShifterPlugin_reversed(29) <= pkg_extract(execute_SRC1,2);
    zz_execute_FullBarrelShifterPlugin_reversed(30) <= pkg_extract(execute_SRC1,1);
    zz_execute_FullBarrelShifterPlugin_reversed(31) <= pkg_extract(execute_SRC1,0);
  end process;

  execute_FullBarrelShifterPlugin_reversed <= pkg_mux(pkg_toStdLogic(execute_SHIFT_CTRL = ShiftCtrlEnum_seq_SLL_1),zz_execute_FullBarrelShifterPlugin_reversed,execute_SRC1);
  process(memory_SHIFT_RIGHT)
  begin
    zz_decode_RS2_3(0) <= pkg_extract(memory_SHIFT_RIGHT,31);
    zz_decode_RS2_3(1) <= pkg_extract(memory_SHIFT_RIGHT,30);
    zz_decode_RS2_3(2) <= pkg_extract(memory_SHIFT_RIGHT,29);
    zz_decode_RS2_3(3) <= pkg_extract(memory_SHIFT_RIGHT,28);
    zz_decode_RS2_3(4) <= pkg_extract(memory_SHIFT_RIGHT,27);
    zz_decode_RS2_3(5) <= pkg_extract(memory_SHIFT_RIGHT,26);
    zz_decode_RS2_3(6) <= pkg_extract(memory_SHIFT_RIGHT,25);
    zz_decode_RS2_3(7) <= pkg_extract(memory_SHIFT_RIGHT,24);
    zz_decode_RS2_3(8) <= pkg_extract(memory_SHIFT_RIGHT,23);
    zz_decode_RS2_3(9) <= pkg_extract(memory_SHIFT_RIGHT,22);
    zz_decode_RS2_3(10) <= pkg_extract(memory_SHIFT_RIGHT,21);
    zz_decode_RS2_3(11) <= pkg_extract(memory_SHIFT_RIGHT,20);
    zz_decode_RS2_3(12) <= pkg_extract(memory_SHIFT_RIGHT,19);
    zz_decode_RS2_3(13) <= pkg_extract(memory_SHIFT_RIGHT,18);
    zz_decode_RS2_3(14) <= pkg_extract(memory_SHIFT_RIGHT,17);
    zz_decode_RS2_3(15) <= pkg_extract(memory_SHIFT_RIGHT,16);
    zz_decode_RS2_3(16) <= pkg_extract(memory_SHIFT_RIGHT,15);
    zz_decode_RS2_3(17) <= pkg_extract(memory_SHIFT_RIGHT,14);
    zz_decode_RS2_3(18) <= pkg_extract(memory_SHIFT_RIGHT,13);
    zz_decode_RS2_3(19) <= pkg_extract(memory_SHIFT_RIGHT,12);
    zz_decode_RS2_3(20) <= pkg_extract(memory_SHIFT_RIGHT,11);
    zz_decode_RS2_3(21) <= pkg_extract(memory_SHIFT_RIGHT,10);
    zz_decode_RS2_3(22) <= pkg_extract(memory_SHIFT_RIGHT,9);
    zz_decode_RS2_3(23) <= pkg_extract(memory_SHIFT_RIGHT,8);
    zz_decode_RS2_3(24) <= pkg_extract(memory_SHIFT_RIGHT,7);
    zz_decode_RS2_3(25) <= pkg_extract(memory_SHIFT_RIGHT,6);
    zz_decode_RS2_3(26) <= pkg_extract(memory_SHIFT_RIGHT,5);
    zz_decode_RS2_3(27) <= pkg_extract(memory_SHIFT_RIGHT,4);
    zz_decode_RS2_3(28) <= pkg_extract(memory_SHIFT_RIGHT,3);
    zz_decode_RS2_3(29) <= pkg_extract(memory_SHIFT_RIGHT,2);
    zz_decode_RS2_3(30) <= pkg_extract(memory_SHIFT_RIGHT,1);
    zz_decode_RS2_3(31) <= pkg_extract(memory_SHIFT_RIGHT,0);
  end process;

  process(when_HazardSimplePlugin_l57,when_HazardSimplePlugin_l58,when_HazardSimplePlugin_l48,when_HazardSimplePlugin_l57_1,when_HazardSimplePlugin_l58_1,when_HazardSimplePlugin_l48_1,when_HazardSimplePlugin_l57_2,when_HazardSimplePlugin_l58_2,when_HazardSimplePlugin_l48_2,when_HazardSimplePlugin_l105)
  begin
    HazardSimplePlugin_src0Hazard <= pkg_toStdLogic(false);
    if when_HazardSimplePlugin_l57 = '1' then
      if when_HazardSimplePlugin_l58 = '1' then
        if when_HazardSimplePlugin_l48 = '1' then
          HazardSimplePlugin_src0Hazard <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l57_1 = '1' then
      if when_HazardSimplePlugin_l58_1 = '1' then
        if when_HazardSimplePlugin_l48_1 = '1' then
          HazardSimplePlugin_src0Hazard <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l57_2 = '1' then
      if when_HazardSimplePlugin_l58_2 = '1' then
        if when_HazardSimplePlugin_l48_2 = '1' then
          HazardSimplePlugin_src0Hazard <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l105 = '1' then
      HazardSimplePlugin_src0Hazard <= pkg_toStdLogic(false);
    end if;
  end process;

  process(when_HazardSimplePlugin_l57,when_HazardSimplePlugin_l58,when_HazardSimplePlugin_l51,when_HazardSimplePlugin_l57_1,when_HazardSimplePlugin_l58_1,when_HazardSimplePlugin_l51_1,when_HazardSimplePlugin_l57_2,when_HazardSimplePlugin_l58_2,when_HazardSimplePlugin_l51_2,when_HazardSimplePlugin_l108)
  begin
    HazardSimplePlugin_src1Hazard <= pkg_toStdLogic(false);
    if when_HazardSimplePlugin_l57 = '1' then
      if when_HazardSimplePlugin_l58 = '1' then
        if when_HazardSimplePlugin_l51 = '1' then
          HazardSimplePlugin_src1Hazard <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l57_1 = '1' then
      if when_HazardSimplePlugin_l58_1 = '1' then
        if when_HazardSimplePlugin_l51_1 = '1' then
          HazardSimplePlugin_src1Hazard <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l57_2 = '1' then
      if when_HazardSimplePlugin_l58_2 = '1' then
        if when_HazardSimplePlugin_l51_2 = '1' then
          HazardSimplePlugin_src1Hazard <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if when_HazardSimplePlugin_l108 = '1' then
      HazardSimplePlugin_src1Hazard <= pkg_toStdLogic(false);
    end if;
  end process;

  HazardSimplePlugin_writeBackWrites_valid <= (zz_lastStageRegFileWrite_valid and writeBack_arbitration_isFiring);
  HazardSimplePlugin_writeBackWrites_payload_address <= pkg_extract(zz_lastStageRegFileWrite_payload_address,11,7);
  HazardSimplePlugin_writeBackWrites_payload_data <= zz_decode_RS2_2;
  HazardSimplePlugin_addr0Match <= pkg_toStdLogic(HazardSimplePlugin_writeBackBuffer_payload_address = pkg_extract(decode_INSTRUCTION,19,15));
  HazardSimplePlugin_addr1Match <= pkg_toStdLogic(HazardSimplePlugin_writeBackBuffer_payload_address = pkg_extract(decode_INSTRUCTION,24,20));
  when_HazardSimplePlugin_l47 <= pkg_toStdLogic(true);
  when_HazardSimplePlugin_l48 <= pkg_toStdLogic(pkg_extract(writeBack_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,19,15));
  when_HazardSimplePlugin_l51 <= pkg_toStdLogic(pkg_extract(writeBack_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,24,20));
  when_HazardSimplePlugin_l45 <= (writeBack_arbitration_isValid and writeBack_REGFILE_WRITE_VALID);
  when_HazardSimplePlugin_l57 <= (writeBack_arbitration_isValid and writeBack_REGFILE_WRITE_VALID);
  when_HazardSimplePlugin_l58 <= (pkg_toStdLogic(false) or (not when_HazardSimplePlugin_l47));
  when_HazardSimplePlugin_l48_1 <= pkg_toStdLogic(pkg_extract(memory_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,19,15));
  when_HazardSimplePlugin_l51_1 <= pkg_toStdLogic(pkg_extract(memory_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,24,20));
  when_HazardSimplePlugin_l45_1 <= (memory_arbitration_isValid and memory_REGFILE_WRITE_VALID);
  when_HazardSimplePlugin_l57_1 <= (memory_arbitration_isValid and memory_REGFILE_WRITE_VALID);
  when_HazardSimplePlugin_l58_1 <= (pkg_toStdLogic(false) or (not memory_BYPASSABLE_MEMORY_STAGE));
  when_HazardSimplePlugin_l48_2 <= pkg_toStdLogic(pkg_extract(execute_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,19,15));
  when_HazardSimplePlugin_l51_2 <= pkg_toStdLogic(pkg_extract(execute_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,24,20));
  when_HazardSimplePlugin_l45_2 <= (execute_arbitration_isValid and execute_REGFILE_WRITE_VALID);
  when_HazardSimplePlugin_l57_2 <= (execute_arbitration_isValid and execute_REGFILE_WRITE_VALID);
  when_HazardSimplePlugin_l58_2 <= (pkg_toStdLogic(false) or (not execute_BYPASSABLE_EXECUTE_STAGE));
  when_HazardSimplePlugin_l105 <= (not decode_RS1_USE);
  when_HazardSimplePlugin_l108 <= (not decode_RS2_USE);
  when_HazardSimplePlugin_l113 <= (decode_arbitration_isValid and (HazardSimplePlugin_src0Hazard or HazardSimplePlugin_src1Hazard));
  when_DebugPlugin_l225 <= (DebugPlugin_haltIt and (not DebugPlugin_isPipBusy));
  DebugPlugin_allowEBreak <= (DebugPlugin_debugUsed and (not DebugPlugin_disableEbreak));
  process(debug_bus_cmd_valid,switch_DebugPlugin_l268,debug_bus_cmd_payload_wr,IBusSimplePlugin_injectionPort_ready)
  begin
    debug_bus_cmd_ready_read_buffer <= pkg_toStdLogic(true);
    if debug_bus_cmd_valid = '1' then
      case switch_DebugPlugin_l268 is
        when "000001" =>
          if debug_bus_cmd_payload_wr = '1' then
            debug_bus_cmd_ready_read_buffer <= IBusSimplePlugin_injectionPort_ready;
          end if;
        when others =>
      end case;
    end if;
  end process;

  process(DebugPlugin_busReadDataReg,when_DebugPlugin_l244,DebugPlugin_resetIt,DebugPlugin_haltIt,DebugPlugin_isPipBusy,DebugPlugin_haltedByBreak,DebugPlugin_stepIt,zz_3,DebugPlugin_hardwareBreakpoints_0_pc,DebugPlugin_hardwareBreakpoints_0_valid,DebugPlugin_hardwareBreakpoints_1_pc,DebugPlugin_hardwareBreakpoints_1_valid,DebugPlugin_hardwareBreakpoints_2_pc,DebugPlugin_hardwareBreakpoints_2_valid,DebugPlugin_hardwareBreakpoints_3_pc,DebugPlugin_hardwareBreakpoints_3_valid,DebugPlugin_hardwareBreakpoints_4_pc,DebugPlugin_hardwareBreakpoints_4_valid,DebugPlugin_hardwareBreakpoints_5_pc,DebugPlugin_hardwareBreakpoints_5_valid,DebugPlugin_hardwareBreakpoints_6_pc,DebugPlugin_hardwareBreakpoints_6_valid,DebugPlugin_hardwareBreakpoints_7_pc,DebugPlugin_hardwareBreakpoints_7_valid,DebugPlugin_hardwareBreakpoints_8_pc,DebugPlugin_hardwareBreakpoints_8_valid,DebugPlugin_hardwareBreakpoints_9_pc,DebugPlugin_hardwareBreakpoints_9_valid,DebugPlugin_hardwareBreakpoints_10_pc,DebugPlugin_hardwareBreakpoints_10_valid,DebugPlugin_hardwareBreakpoints_11_pc,DebugPlugin_hardwareBreakpoints_11_valid,DebugPlugin_hardwareBreakpoints_12_pc,DebugPlugin_hardwareBreakpoints_12_valid,DebugPlugin_hardwareBreakpoints_13_pc,DebugPlugin_hardwareBreakpoints_13_valid,DebugPlugin_hardwareBreakpoints_14_pc,DebugPlugin_hardwareBreakpoints_14_valid,DebugPlugin_hardwareBreakpoints_15_pc,DebugPlugin_hardwareBreakpoints_15_valid)
  begin
    debug_bus_rsp_data <= DebugPlugin_busReadDataReg;
    if when_DebugPlugin_l244 = '1' then
      debug_bus_rsp_data(0) <= DebugPlugin_resetIt;
      debug_bus_rsp_data(1) <= DebugPlugin_haltIt;
      debug_bus_rsp_data(2) <= DebugPlugin_isPipBusy;
      debug_bus_rsp_data(3) <= DebugPlugin_haltedByBreak;
      debug_bus_rsp_data(4) <= DebugPlugin_stepIt;
    end if;
    case zz_3 is
      when "010000" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_0_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_0_valid;
      when "010001" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_1_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_1_valid;
      when "010010" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_2_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_2_valid;
      when "010011" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_3_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_3_valid;
      when "010100" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_4_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_4_valid;
      when "010101" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_5_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_5_valid;
      when "010110" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_6_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_6_valid;
      when "010111" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_7_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_7_valid;
      when "011000" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_8_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_8_valid;
      when "011001" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_9_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_9_valid;
      when "011010" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_10_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_10_valid;
      when "011011" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_11_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_11_valid;
      when "011100" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_12_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_12_valid;
      when "011101" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_13_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_13_valid;
      when "011110" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_14_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_14_valid;
      when "011111" =>
        debug_bus_rsp_data(31 downto 1) <= std_logic_vector(DebugPlugin_hardwareBreakpoints_15_pc);
        debug_bus_rsp_data(0) <= DebugPlugin_hardwareBreakpoints_15_valid;
      when others =>
    end case;
  end process;

  when_DebugPlugin_l244 <= (not zz_when_DebugPlugin_l244);
  process(debug_bus_cmd_valid,switch_DebugPlugin_l268,debug_bus_cmd_payload_wr)
  begin
    IBusSimplePlugin_injectionPort_valid <= pkg_toStdLogic(false);
    if debug_bus_cmd_valid = '1' then
      case switch_DebugPlugin_l268 is
        when "000001" =>
          if debug_bus_cmd_payload_wr = '1' then
            IBusSimplePlugin_injectionPort_valid <= pkg_toStdLogic(true);
          end if;
        when others =>
      end case;
    end if;
  end process;

  IBusSimplePlugin_injectionPort_payload <= debug_bus_cmd_payload_data;
  switch_DebugPlugin_l268 <= pkg_extract(debug_bus_cmd_payload_address,7,2);
  when_DebugPlugin_l272 <= pkg_extract(debug_bus_cmd_payload_data,16);
  when_DebugPlugin_l272_1 <= pkg_extract(debug_bus_cmd_payload_data,24);
  when_DebugPlugin_l273 <= pkg_extract(debug_bus_cmd_payload_data,17);
  when_DebugPlugin_l273_1 <= pkg_extract(debug_bus_cmd_payload_data,25);
  when_DebugPlugin_l274 <= pkg_extract(debug_bus_cmd_payload_data,25);
  when_DebugPlugin_l275 <= pkg_extract(debug_bus_cmd_payload_data,25);
  when_DebugPlugin_l276 <= pkg_extract(debug_bus_cmd_payload_data,18);
  when_DebugPlugin_l276_1 <= pkg_extract(debug_bus_cmd_payload_data,26);
  when_DebugPlugin_l296 <= (execute_arbitration_isValid and execute_DO_EBREAK);
  when_DebugPlugin_l299 <= pkg_toStdLogic(pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_isValid),pkg_toStdLogicVector(memory_arbitration_isValid)) /= pkg_stdLogicVector("00")) = pkg_toStdLogic(false));
  when_DebugPlugin_l312 <= (DebugPlugin_stepIt and IBusSimplePlugin_incomingInstruction);
  debug_resetOut <= DebugPlugin_resetIt_regNext;
  when_DebugPlugin_l328 <= (DebugPlugin_haltIt or DebugPlugin_stepIt);
  execute_BranchPlugin_eq <= pkg_toStdLogic(execute_SRC1 = execute_SRC2);
  switch_Misc_l204_4 <= pkg_extract(execute_INSTRUCTION,14,12);
  process(switch_Misc_l204_4,execute_BranchPlugin_eq,execute_SRC_LESS)
  begin
    if (switch_Misc_l204_4 = pkg_stdLogicVector("000")) then
        zz_execute_BRANCH_DO <= execute_BranchPlugin_eq;
    elsif (switch_Misc_l204_4 = pkg_stdLogicVector("001")) then
        zz_execute_BRANCH_DO <= (not execute_BranchPlugin_eq);
    elsif (pkg_toStdLogic((switch_Misc_l204_4 and pkg_stdLogicVector("101")) = pkg_stdLogicVector("101")) = '1') then
        zz_execute_BRANCH_DO <= (not execute_SRC_LESS);
    else
        zz_execute_BRANCH_DO <= execute_SRC_LESS;
    end if;
  end process;

  process(execute_BRANCH_CTRL,zz_execute_BRANCH_DO)
  begin
    case execute_BRANCH_CTRL is
      when BranchCtrlEnum_seq_INC =>
        zz_execute_BRANCH_DO_1 <= pkg_toStdLogic(false);
      when BranchCtrlEnum_seq_JAL =>
        zz_execute_BRANCH_DO_1 <= pkg_toStdLogic(true);
      when BranchCtrlEnum_seq_JALR =>
        zz_execute_BRANCH_DO_1 <= pkg_toStdLogic(true);
      when others =>
        zz_execute_BRANCH_DO_1 <= zz_execute_BRANCH_DO;
    end case;
  end process;

  execute_BranchPlugin_branch_src1 <= pkg_mux(pkg_toStdLogic(execute_BRANCH_CTRL = BranchCtrlEnum_seq_JALR),unsigned(execute_RS1),execute_PC);
  zz_execute_BranchPlugin_branch_src2 <= pkg_extract(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_extract(execute_INSTRUCTION,19,12)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,20))),pkg_extract(execute_INSTRUCTION,30,21)),19);
  process(zz_execute_BranchPlugin_branch_src2)
  begin
    zz_execute_BranchPlugin_branch_src2_1(10) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(9) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(8) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(7) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(6) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(5) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(4) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(3) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(2) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(1) <= zz_execute_BranchPlugin_branch_src2;
    zz_execute_BranchPlugin_branch_src2_1(0) <= zz_execute_BranchPlugin_branch_src2;
  end process;

  zz_execute_BranchPlugin_branch_src2_2 <= pkg_extract(execute_INSTRUCTION,31);
  process(zz_execute_BranchPlugin_branch_src2_2)
  begin
    zz_execute_BranchPlugin_branch_src2_3(19) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(18) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(17) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(16) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(15) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(14) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(13) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(12) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(11) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(10) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(9) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(8) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(7) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(6) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(5) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(4) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(3) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(2) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(1) <= zz_execute_BranchPlugin_branch_src2_2;
    zz_execute_BranchPlugin_branch_src2_3(0) <= zz_execute_BranchPlugin_branch_src2_2;
  end process;

  zz_execute_BranchPlugin_branch_src2_4 <= pkg_extract(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,7))),pkg_extract(execute_INSTRUCTION,30,25)),pkg_extract(execute_INSTRUCTION,11,8)),11);
  process(zz_execute_BranchPlugin_branch_src2_4)
  begin
    zz_execute_BranchPlugin_branch_src2_5(18) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(17) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(16) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(15) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(14) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(13) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(12) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(11) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(10) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(9) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(8) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(7) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(6) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(5) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(4) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(3) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(2) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(1) <= zz_execute_BranchPlugin_branch_src2_4;
    zz_execute_BranchPlugin_branch_src2_5(0) <= zz_execute_BranchPlugin_branch_src2_4;
  end process;

  process(execute_BRANCH_CTRL,zz_execute_BranchPlugin_branch_src2_1,execute_INSTRUCTION,zz_execute_BranchPlugin_branch_src2_3,zz_execute_BranchPlugin_branch_src2_5)
  begin
    case execute_BRANCH_CTRL is
      when BranchCtrlEnum_seq_JAL =>
        zz_execute_BranchPlugin_branch_src2_6 <= pkg_cat(pkg_cat(zz_execute_BranchPlugin_branch_src2_1,pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_extract(execute_INSTRUCTION,19,12)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,20))),pkg_extract(execute_INSTRUCTION,30,21))),pkg_toStdLogicVector(pkg_toStdLogic(false)));
      when BranchCtrlEnum_seq_JALR =>
        zz_execute_BranchPlugin_branch_src2_6 <= pkg_cat(zz_execute_BranchPlugin_branch_src2_3,pkg_extract(execute_INSTRUCTION,31,20));
      when others =>
        zz_execute_BranchPlugin_branch_src2_6 <= pkg_cat(pkg_cat(zz_execute_BranchPlugin_branch_src2_5,pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,7))),pkg_extract(execute_INSTRUCTION,30,25)),pkg_extract(execute_INSTRUCTION,11,8))),pkg_toStdLogicVector(pkg_toStdLogic(false)));
    end case;
  end process;

  execute_BranchPlugin_branch_src2 <= unsigned(zz_execute_BranchPlugin_branch_src2_6);
  execute_BranchPlugin_branchAdder <= (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  BranchPlugin_jumpInterface_valid <= ((memory_arbitration_isValid and memory_BRANCH_DO) and (not pkg_toStdLogic(false)));
  BranchPlugin_jumpInterface_payload <= memory_BRANCH_CALC;
  when_Pipeline_l124 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_1 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_2 <= ((not writeBack_arbitration_isStuck) and (not CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack));
  when_Pipeline_l124_3 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_4 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_5 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_6 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_7 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_8 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_9 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_10 <= (not execute_arbitration_isStuck);
  zz_decode_SRC1_CTRL <= zz_decode_SRC1_CTRL_1;
  when_Pipeline_l124_11 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_12 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_13 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_14 <= (not writeBack_arbitration_isStuck);
  zz_decode_to_execute_ALU_CTRL_1 <= decode_ALU_CTRL;
  zz_decode_ALU_CTRL <= zz_decode_ALU_CTRL_1;
  when_Pipeline_l124_15 <= (not execute_arbitration_isStuck);
  zz_execute_ALU_CTRL <= decode_to_execute_ALU_CTRL;
  zz_decode_SRC2_CTRL <= zz_decode_SRC2_CTRL_1;
  when_Pipeline_l124_16 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_17 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_18 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_19 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_20 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_21 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_22 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_23 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_24 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_25 <= (not execute_arbitration_isStuck);
  zz_decode_to_execute_ENV_CTRL_1 <= decode_ENV_CTRL;
  zz_execute_to_memory_ENV_CTRL_1 <= execute_ENV_CTRL;
  zz_memory_to_writeBack_ENV_CTRL_1 <= memory_ENV_CTRL;
  zz_decode_ENV_CTRL <= zz_decode_ENV_CTRL_1;
  when_Pipeline_l124_26 <= (not execute_arbitration_isStuck);
  zz_execute_ENV_CTRL <= decode_to_execute_ENV_CTRL;
  when_Pipeline_l124_27 <= (not memory_arbitration_isStuck);
  zz_memory_ENV_CTRL <= execute_to_memory_ENV_CTRL;
  when_Pipeline_l124_28 <= (not writeBack_arbitration_isStuck);
  zz_writeBack_ENV_CTRL <= memory_to_writeBack_ENV_CTRL;
  when_Pipeline_l124_29 <= (not execute_arbitration_isStuck);
  zz_decode_to_execute_ALU_BITWISE_CTRL_1 <= decode_ALU_BITWISE_CTRL;
  zz_decode_ALU_BITWISE_CTRL <= zz_decode_ALU_BITWISE_CTRL_1;
  when_Pipeline_l124_30 <= (not execute_arbitration_isStuck);
  zz_execute_ALU_BITWISE_CTRL <= decode_to_execute_ALU_BITWISE_CTRL;
  when_Pipeline_l124_31 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_32 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_33 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_34 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_35 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_36 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_37 <= (not execute_arbitration_isStuck);
  zz_decode_to_execute_SHIFT_CTRL_1 <= decode_SHIFT_CTRL;
  zz_execute_to_memory_SHIFT_CTRL_1 <= execute_SHIFT_CTRL;
  zz_decode_SHIFT_CTRL <= zz_decode_SHIFT_CTRL_1;
  when_Pipeline_l124_38 <= (not execute_arbitration_isStuck);
  zz_execute_SHIFT_CTRL <= decode_to_execute_SHIFT_CTRL;
  when_Pipeline_l124_39 <= (not memory_arbitration_isStuck);
  zz_memory_SHIFT_CTRL <= execute_to_memory_SHIFT_CTRL;
  zz_decode_to_execute_BRANCH_CTRL_1 <= decode_BRANCH_CTRL;
  zz_decode_BRANCH_CTRL <= zz_decode_BRANCH_CTRL_1;
  when_Pipeline_l124_40 <= (not execute_arbitration_isStuck);
  zz_execute_BRANCH_CTRL <= decode_to_execute_BRANCH_CTRL;
  when_Pipeline_l124_41 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_42 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_43 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_44 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_45 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_46 <= (not execute_arbitration_isStuck);
  when_Pipeline_l124_47 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_48 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_49 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_50 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_51 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_52 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_53 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_54 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_55 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_56 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_57 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_58 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_59 <= (not memory_arbitration_isStuck);
  when_Pipeline_l124_60 <= (not writeBack_arbitration_isStuck);
  when_Pipeline_l124_61 <= (not writeBack_arbitration_isStuck);
  decode_arbitration_isFlushed <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushNext),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushNext),pkg_toStdLogicVector(execute_arbitration_flushNext))) /= pkg_stdLogicVector("000")) or pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushIt),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushIt),pkg_cat(pkg_toStdLogicVector(execute_arbitration_flushIt),pkg_toStdLogicVector(decode_arbitration_flushIt)))) /= pkg_stdLogicVector("0000")));
  execute_arbitration_isFlushed <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushNext),pkg_toStdLogicVector(memory_arbitration_flushNext)) /= pkg_stdLogicVector("00")) or pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushIt),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushIt),pkg_toStdLogicVector(execute_arbitration_flushIt))) /= pkg_stdLogicVector("000")));
  memory_arbitration_isFlushed <= (pkg_toStdLogic(pkg_toStdLogicVector(writeBack_arbitration_flushNext) /= pkg_stdLogicVector("0")) or pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushIt),pkg_toStdLogicVector(memory_arbitration_flushIt)) /= pkg_stdLogicVector("00")));
  writeBack_arbitration_isFlushed <= (pkg_toStdLogic(false) or pkg_toStdLogic(pkg_toStdLogicVector(writeBack_arbitration_flushIt) /= pkg_stdLogicVector("0")));
  decode_arbitration_isStuckByOthers <= (decode_arbitration_haltByOther or (((pkg_toStdLogic(false) or execute_arbitration_isStuck) or memory_arbitration_isStuck) or writeBack_arbitration_isStuck));
  decode_arbitration_isStuck <= (decode_arbitration_haltItself or decode_arbitration_isStuckByOthers);
  decode_arbitration_isMoving <= ((not decode_arbitration_isStuck) and (not decode_arbitration_removeIt));
  decode_arbitration_isFiring <= ((decode_arbitration_isValid and (not decode_arbitration_isStuck)) and (not decode_arbitration_removeIt));
  execute_arbitration_isStuckByOthers <= (execute_arbitration_haltByOther or ((pkg_toStdLogic(false) or memory_arbitration_isStuck) or writeBack_arbitration_isStuck));
  execute_arbitration_isStuck <= (execute_arbitration_haltItself or execute_arbitration_isStuckByOthers);
  execute_arbitration_isMoving <= ((not execute_arbitration_isStuck) and (not execute_arbitration_removeIt));
  execute_arbitration_isFiring <= ((execute_arbitration_isValid and (not execute_arbitration_isStuck)) and (not execute_arbitration_removeIt));
  memory_arbitration_isStuckByOthers <= (memory_arbitration_haltByOther or (pkg_toStdLogic(false) or writeBack_arbitration_isStuck));
  memory_arbitration_isStuck <= (memory_arbitration_haltItself or memory_arbitration_isStuckByOthers);
  memory_arbitration_isMoving <= ((not memory_arbitration_isStuck) and (not memory_arbitration_removeIt));
  memory_arbitration_isFiring <= ((memory_arbitration_isValid and (not memory_arbitration_isStuck)) and (not memory_arbitration_removeIt));
  writeBack_arbitration_isStuckByOthers <= (writeBack_arbitration_haltByOther or pkg_toStdLogic(false));
  writeBack_arbitration_isStuck <= (writeBack_arbitration_haltItself or writeBack_arbitration_isStuckByOthers);
  writeBack_arbitration_isMoving <= ((not writeBack_arbitration_isStuck) and (not writeBack_arbitration_removeIt));
  writeBack_arbitration_isFiring <= ((writeBack_arbitration_isValid and (not writeBack_arbitration_isStuck)) and (not writeBack_arbitration_removeIt));
  when_Pipeline_l151 <= ((not execute_arbitration_isStuck) or execute_arbitration_removeIt);
  when_Pipeline_l154 <= ((not decode_arbitration_isStuck) and (not decode_arbitration_removeIt));
  when_Pipeline_l151_1 <= ((not memory_arbitration_isStuck) or memory_arbitration_removeIt);
  when_Pipeline_l154_1 <= ((not execute_arbitration_isStuck) and (not execute_arbitration_removeIt));
  when_Pipeline_l151_2 <= ((not writeBack_arbitration_isStuck) or writeBack_arbitration_removeIt);
  when_Pipeline_l154_2 <= ((not memory_arbitration_isStuck) and (not memory_arbitration_removeIt));
  process(switch_Fetcher_l362)
  begin
    IBusSimplePlugin_injectionPort_ready <= pkg_toStdLogic(false);
    case switch_Fetcher_l362 is
      when "100" =>
        IBusSimplePlugin_injectionPort_ready <= pkg_toStdLogic(true);
      when others =>
    end case;
  end process;

  when_Fetcher_l360 <= pkg_toStdLogic(switch_Fetcher_l362 /= pkg_unsigned("000"));
  when_Fetcher_l378 <= (not decode_arbitration_isStuck);
  when_Fetcher_l398 <= pkg_toStdLogic(switch_Fetcher_l362 /= pkg_unsigned("000"));
  when_CsrPlugin_l1264 <= (not execute_arbitration_isStuck);
  when_CsrPlugin_l1264_1 <= (not execute_arbitration_isStuck);
  when_CsrPlugin_l1264_2 <= (not execute_arbitration_isStuck);
  when_CsrPlugin_l1264_3 <= (not execute_arbitration_isStuck);
  when_CsrPlugin_l1264_4 <= (not execute_arbitration_isStuck);
  when_CsrPlugin_l1264_5 <= (not execute_arbitration_isStuck);
  process(execute_CsrPlugin_csr_836,LocalInt0_regNext,LocalInt1_regNext,LocalInt2_regNext,LocalInt3_regNext,CsrPlugin_mip_MEIP,CsrPlugin_mip_MTIP,CsrPlugin_mip_MSIP)
  begin
    zz_CsrPlugin_csrMapping_readDataInit <= pkg_stdLogicVector("00000000000000000000000000000000");
    if execute_CsrPlugin_csr_836 = '1' then
      zz_CsrPlugin_csrMapping_readDataInit(16 downto 16) <= pkg_toStdLogicVector(LocalInt0_regNext);
      zz_CsrPlugin_csrMapping_readDataInit(17 downto 17) <= pkg_toStdLogicVector(LocalInt1_regNext);
      zz_CsrPlugin_csrMapping_readDataInit(18 downto 18) <= pkg_toStdLogicVector(LocalInt2_regNext);
      zz_CsrPlugin_csrMapping_readDataInit(19 downto 19) <= pkg_toStdLogicVector(LocalInt3_regNext);
      zz_CsrPlugin_csrMapping_readDataInit(11 downto 11) <= pkg_toStdLogicVector(CsrPlugin_mip_MEIP);
      zz_CsrPlugin_csrMapping_readDataInit(7 downto 7) <= pkg_toStdLogicVector(CsrPlugin_mip_MTIP);
      zz_CsrPlugin_csrMapping_readDataInit(3 downto 3) <= pkg_toStdLogicVector(CsrPlugin_mip_MSIP);
    end if;
  end process;

  process(execute_CsrPlugin_csr_772,LocalInt0_enable,LocalInt1_enable,LocalInt2_enable,LocalInt3_enable,CsrPlugin_mie_MEIE,CsrPlugin_mie_MTIE,CsrPlugin_mie_MSIE)
  begin
    zz_CsrPlugin_csrMapping_readDataInit_1 <= pkg_stdLogicVector("00000000000000000000000000000000");
    if execute_CsrPlugin_csr_772 = '1' then
      zz_CsrPlugin_csrMapping_readDataInit_1(16 downto 16) <= pkg_toStdLogicVector(LocalInt0_enable);
      zz_CsrPlugin_csrMapping_readDataInit_1(17 downto 17) <= pkg_toStdLogicVector(LocalInt1_enable);
      zz_CsrPlugin_csrMapping_readDataInit_1(18 downto 18) <= pkg_toStdLogicVector(LocalInt2_enable);
      zz_CsrPlugin_csrMapping_readDataInit_1(19 downto 19) <= pkg_toStdLogicVector(LocalInt3_enable);
      zz_CsrPlugin_csrMapping_readDataInit_1(11 downto 11) <= pkg_toStdLogicVector(CsrPlugin_mie_MEIE);
      zz_CsrPlugin_csrMapping_readDataInit_1(7 downto 7) <= pkg_toStdLogicVector(CsrPlugin_mie_MTIE);
      zz_CsrPlugin_csrMapping_readDataInit_1(3 downto 3) <= pkg_toStdLogicVector(CsrPlugin_mie_MSIE);
    end if;
  end process;

  process(execute_CsrPlugin_csr_768,CsrPlugin_mstatus_MPP,CsrPlugin_mstatus_MPIE,CsrPlugin_mstatus_MIE)
  begin
    zz_CsrPlugin_csrMapping_readDataInit_2 <= pkg_stdLogicVector("00000000000000000000000000000000");
    if execute_CsrPlugin_csr_768 = '1' then
      zz_CsrPlugin_csrMapping_readDataInit_2(12 downto 11) <= std_logic_vector(CsrPlugin_mstatus_MPP);
      zz_CsrPlugin_csrMapping_readDataInit_2(7 downto 7) <= pkg_toStdLogicVector(CsrPlugin_mstatus_MPIE);
      zz_CsrPlugin_csrMapping_readDataInit_2(3 downto 3) <= pkg_toStdLogicVector(CsrPlugin_mstatus_MIE);
    end if;
  end process;

  process(execute_CsrPlugin_csr_773,CsrPlugin_mtvec_base,CsrPlugin_mtvec_mode)
  begin
    zz_CsrPlugin_csrMapping_readDataInit_3 <= pkg_stdLogicVector("00000000000000000000000000000000");
    if execute_CsrPlugin_csr_773 = '1' then
      zz_CsrPlugin_csrMapping_readDataInit_3(31 downto 2) <= std_logic_vector(CsrPlugin_mtvec_base);
      zz_CsrPlugin_csrMapping_readDataInit_3(1 downto 0) <= CsrPlugin_mtvec_mode;
    end if;
  end process;

  process(execute_CsrPlugin_csr_833,CsrPlugin_mepc)
  begin
    zz_CsrPlugin_csrMapping_readDataInit_4 <= pkg_stdLogicVector("00000000000000000000000000000000");
    if execute_CsrPlugin_csr_833 = '1' then
      zz_CsrPlugin_csrMapping_readDataInit_4(31 downto 0) <= std_logic_vector(CsrPlugin_mepc);
    end if;
  end process;

  process(execute_CsrPlugin_csr_834,CsrPlugin_mcause_interrupt,CsrPlugin_mcause_exceptionCode)
  begin
    zz_CsrPlugin_csrMapping_readDataInit_5 <= pkg_stdLogicVector("00000000000000000000000000000000");
    if execute_CsrPlugin_csr_834 = '1' then
      zz_CsrPlugin_csrMapping_readDataInit_5(31 downto 31) <= pkg_toStdLogicVector(CsrPlugin_mcause_interrupt);
      zz_CsrPlugin_csrMapping_readDataInit_5(4 downto 0) <= std_logic_vector(CsrPlugin_mcause_exceptionCode);
    end if;
  end process;

  CsrPlugin_csrMapping_readDataInit <= (((zz_CsrPlugin_csrMapping_readDataInit or zz_CsrPlugin_csrMapping_readDataInit_1) or (zz_CsrPlugin_csrMapping_readDataInit_2 or zz_CsrPlugin_csrMapping_readDataInit_3)) or (zz_CsrPlugin_csrMapping_readDataInit_4 or zz_CsrPlugin_csrMapping_readDataInit_5));
  when_CsrPlugin_l1297 <= pkg_toStdLogic(CsrPlugin_privilege < unsigned(pkg_extract(execute_CsrPlugin_csrAddress,9,8)));
  when_CsrPlugin_l1302 <= ((not execute_arbitration_isValid) or (not execute_IS_CSR));
  process(iBus_cmd_m2sPipe_ready,when_Stream_l342)
  begin
    iBus_cmd_ready <= iBus_cmd_m2sPipe_ready;
    if when_Stream_l342 = '1' then
      iBus_cmd_ready <= pkg_toStdLogic(true);
    end if;
  end process;

  when_Stream_l342 <= (not iBus_cmd_m2sPipe_valid);
  iBus_cmd_m2sPipe_valid <= iBus_cmd_rValid;
  iBus_cmd_m2sPipe_payload_pc <= iBus_cmd_rData_pc;
  iBusWishbone_ADR <= pkg_shiftRight(iBus_cmd_m2sPipe_payload_pc,2);
  iBusWishbone_CTI <= pkg_stdLogicVector("000");
  iBusWishbone_BTE <= pkg_stdLogicVector("00");
  iBusWishbone_SEL <= pkg_stdLogicVector("1111");
  iBusWishbone_WE <= pkg_toStdLogic(false);
  iBusWishbone_DAT_MOSI <= pkg_stdLogicVector("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  iBusWishbone_CYC_read_buffer <= iBus_cmd_m2sPipe_valid;
  iBusWishbone_STB <= iBus_cmd_m2sPipe_valid;
  iBus_cmd_m2sPipe_ready <= (iBus_cmd_m2sPipe_valid and iBusWishbone_ACK);
  iBus_rsp_valid <= (iBusWishbone_CYC_read_buffer and iBusWishbone_ACK);
  iBus_rsp_payload_inst <= iBusWishbone_DAT_MISO;
  iBus_rsp_payload_error <= pkg_toStdLogic(false);
  dBus_cmd_halfPipe_fire <= (dBus_cmd_halfPipe_valid and dBus_cmd_halfPipe_ready);
  dBus_cmd_ready <= (not dBus_cmd_rValid);
  dBus_cmd_halfPipe_valid <= dBus_cmd_rValid;
  dBus_cmd_halfPipe_payload_wr <= dBus_cmd_rData_wr;
  dBus_cmd_halfPipe_payload_address <= dBus_cmd_rData_address;
  dBus_cmd_halfPipe_payload_data <= dBus_cmd_rData_data;
  dBus_cmd_halfPipe_payload_size <= dBus_cmd_rData_size;
  dBusWishbone_ADR <= pkg_shiftRight(dBus_cmd_halfPipe_payload_address,2);
  dBusWishbone_CTI <= pkg_stdLogicVector("000");
  dBusWishbone_BTE <= pkg_stdLogicVector("00");
  process(dBus_cmd_halfPipe_payload_size)
  begin
    case dBus_cmd_halfPipe_payload_size is
      when "00" =>
        zz_dBusWishbone_SEL <= pkg_stdLogicVector("0001");
      when "01" =>
        zz_dBusWishbone_SEL <= pkg_stdLogicVector("0011");
      when others =>
        zz_dBusWishbone_SEL <= pkg_stdLogicVector("1111");
    end case;
  end process;

  process(zz_dBusWishbone_SEL,dBus_cmd_halfPipe_payload_address,when_DBusSimplePlugin_l189)
  begin
    dBusWishbone_SEL <= std_logic_vector(shift_left(unsigned(zz_dBusWishbone_SEL),to_integer(pkg_extract(dBus_cmd_halfPipe_payload_address,1,0))));
    if when_DBusSimplePlugin_l189 = '1' then
      dBusWishbone_SEL <= pkg_stdLogicVector("1111");
    end if;
  end process;

  when_DBusSimplePlugin_l189 <= (not dBus_cmd_halfPipe_payload_wr);
  dBusWishbone_WE_read_buffer <= dBus_cmd_halfPipe_payload_wr;
  dBusWishbone_DAT_MOSI <= dBus_cmd_halfPipe_payload_data;
  dBus_cmd_halfPipe_ready <= (dBus_cmd_halfPipe_valid and dBusWishbone_ACK);
  dBusWishbone_CYC <= dBus_cmd_halfPipe_valid;
  dBusWishbone_STB <= dBus_cmd_halfPipe_valid;
  dBus_rsp_ready <= ((dBus_cmd_halfPipe_valid and (not dBusWishbone_WE_read_buffer)) and dBusWishbone_ACK);
  dBus_rsp_data <= dBusWishbone_DAT_MISO;
  dBus_rsp_error <= pkg_toStdLogic(false);
  process(clk, reset)
  begin
    if reset = '1' then
      LocalInt0_regNext <= pkg_toStdLogic(false);
      LocalInt0_enable <= pkg_toStdLogic(false);
      LocalInt1_regNext <= pkg_toStdLogic(false);
      LocalInt1_enable <= pkg_toStdLogic(false);
      LocalInt2_regNext <= pkg_toStdLogic(false);
      LocalInt2_enable <= pkg_toStdLogic(false);
      LocalInt3_regNext <= pkg_toStdLogic(false);
      LocalInt3_enable <= pkg_toStdLogic(false);
      IBusSimplePlugin_fetchPc_pcReg <= pkg_unsigned("00000000000000010000000000000000");
      IBusSimplePlugin_fetchPc_correctionReg <= pkg_toStdLogic(false);
      IBusSimplePlugin_fetchPc_booted <= pkg_toStdLogic(false);
      IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(false);
      IBusSimplePlugin_decodePc_pcReg <= pkg_unsigned("00000000000000010000000000000000");
      zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_2 <= pkg_toStdLogic(false);
      IBusSimplePlugin_decompressor_bufferValid <= pkg_toStdLogic(false);
      IBusSimplePlugin_decompressor_throw2BytesReg <= pkg_toStdLogic(false);
      zz_IBusSimplePlugin_injector_decodeInput_valid <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_2 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_3 <= pkg_toStdLogic(false);
      IBusSimplePlugin_pending_value <= pkg_unsigned("000");
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= pkg_unsigned("000");
      CsrPlugin_mtvec_mode <= pkg_stdLogicVector("01");
      CsrPlugin_mtvec_base <= pkg_unsigned("000000000000000100000000000100");
      CsrPlugin_mstatus_MIE <= pkg_toStdLogic(false);
      CsrPlugin_mstatus_MPIE <= pkg_toStdLogic(false);
      CsrPlugin_mstatus_MPP <= pkg_unsigned("11");
      CsrPlugin_mie_MEIE <= pkg_toStdLogic(false);
      CsrPlugin_mie_MTIE <= pkg_toStdLogic(false);
      CsrPlugin_mie_MSIE <= pkg_toStdLogic(false);
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= pkg_toStdLogic(false);
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= pkg_toStdLogic(false);
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= pkg_toStdLogic(false);
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= pkg_toStdLogic(false);
      CsrPlugin_interrupt_valid <= pkg_toStdLogic(false);
      CsrPlugin_pipelineLiberator_pcValids_0 <= pkg_toStdLogic(false);
      CsrPlugin_pipelineLiberator_pcValids_1 <= pkg_toStdLogic(false);
      CsrPlugin_pipelineLiberator_pcValids_2 <= pkg_toStdLogic(false);
      CsrPlugin_hadException <= pkg_toStdLogic(false);
      execute_CsrPlugin_wfiWake <= pkg_toStdLogic(false);
      zz_2 <= pkg_toStdLogic(true);
      memory_DivPlugin_div_counter_value <= pkg_unsigned("000000");
      HazardSimplePlugin_writeBackBuffer_valid <= pkg_toStdLogic(false);
      execute_arbitration_isValid <= pkg_toStdLogic(false);
      memory_arbitration_isValid <= pkg_toStdLogic(false);
      writeBack_arbitration_isValid <= pkg_toStdLogic(false);
      switch_Fetcher_l362 <= pkg_unsigned("000");
      iBus_cmd_rValid <= pkg_toStdLogic(false);
      dBus_cmd_rValid <= pkg_toStdLogic(false);
    elsif rising_edge(clk) then
      LocalInt0_regNext <= LocalInt0;
      LocalInt1_regNext <= LocalInt1;
      LocalInt2_regNext <= LocalInt2;
      LocalInt3_regNext <= LocalInt3;
      if IBusSimplePlugin_fetchPc_correction = '1' then
        IBusSimplePlugin_fetchPc_correctionReg <= pkg_toStdLogic(true);
      end if;
      if IBusSimplePlugin_fetchPc_output_fire = '1' then
        IBusSimplePlugin_fetchPc_correctionReg <= pkg_toStdLogic(false);
      end if;
      IBusSimplePlugin_fetchPc_booted <= pkg_toStdLogic(true);
      if when_Fetcher_l131 = '1' then
        IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(false);
      end if;
      if IBusSimplePlugin_fetchPc_output_fire_1 = '1' then
        IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(true);
      end if;
      if when_Fetcher_l131_1 = '1' then
        IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(false);
      end if;
      if when_Fetcher_l158 = '1' then
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end if;
      if when_Fetcher_l180 = '1' then
        IBusSimplePlugin_decodePc_pcReg <= IBusSimplePlugin_decodePc_pcPlus;
      end if;
      if when_Fetcher_l192 = '1' then
        IBusSimplePlugin_decodePc_pcReg <= IBusSimplePlugin_jump_pcLoad_payload;
      end if;
      if IBusSimplePlugin_iBusRsp_flush = '1' then
        zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_2 <= pkg_toStdLogic(false);
      end if;
      if zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready = '1' then
        zz_IBusSimplePlugin_iBusRsp_stages_0_output_ready_2 <= (IBusSimplePlugin_iBusRsp_stages_0_output_valid and (not pkg_toStdLogic(false)));
      end if;
      if IBusSimplePlugin_decompressor_output_fire = '1' then
        IBusSimplePlugin_decompressor_throw2BytesReg <= ((((not IBusSimplePlugin_decompressor_unaligned) and IBusSimplePlugin_decompressor_isInputLowRvc) and IBusSimplePlugin_decompressor_isInputHighRvc) or (IBusSimplePlugin_decompressor_bufferValid and IBusSimplePlugin_decompressor_isInputHighRvc));
      end if;
      if when_Fetcher_l283 = '1' then
        IBusSimplePlugin_decompressor_bufferValid <= pkg_toStdLogic(false);
      end if;
      if when_Fetcher_l286 = '1' then
        if IBusSimplePlugin_decompressor_bufferFill = '1' then
          IBusSimplePlugin_decompressor_bufferValid <= pkg_toStdLogic(true);
        end if;
      end if;
      if when_Fetcher_l291 = '1' then
        IBusSimplePlugin_decompressor_throw2BytesReg <= pkg_toStdLogic(false);
        IBusSimplePlugin_decompressor_bufferValid <= pkg_toStdLogic(false);
      end if;
      if decode_arbitration_removeIt = '1' then
        zz_IBusSimplePlugin_injector_decodeInput_valid <= pkg_toStdLogic(false);
      end if;
      if IBusSimplePlugin_decompressor_output_ready = '1' then
        zz_IBusSimplePlugin_injector_decodeInput_valid <= (IBusSimplePlugin_decompressor_output_valid and (not IBusSimplePlugin_externalFlush));
      end if;
      if when_Fetcher_l329 = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= pkg_toStdLogic(true);
      end if;
      if IBusSimplePlugin_decodePc_flushed = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= pkg_toStdLogic(false);
      end if;
      if when_Fetcher_l329_1 = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end if;
      if IBusSimplePlugin_decodePc_flushed = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= pkg_toStdLogic(false);
      end if;
      if when_Fetcher_l329_2 = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end if;
      if IBusSimplePlugin_decodePc_flushed = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= pkg_toStdLogic(false);
      end if;
      if when_Fetcher_l329_3 = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
      end if;
      if IBusSimplePlugin_decodePc_flushed = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= pkg_toStdLogic(false);
      end if;
      IBusSimplePlugin_pending_value <= IBusSimplePlugin_pending_next;
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter - pkg_resize(unsigned(pkg_toStdLogicVector((IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid and pkg_toStdLogic(IBusSimplePlugin_rspJoin_rspBuffer_discardCounter /= pkg_unsigned("000"))))),3));
      if IBusSimplePlugin_iBusRsp_flush = '1' then
        IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (IBusSimplePlugin_pending_value - pkg_resize(unsigned(pkg_toStdLogicVector(IBusSimplePlugin_pending_dec)),3));
      end if;
      assert (not (((dBus_rsp_ready and memory_MEMORY_ENABLE) and memory_arbitration_isValid) and memory_arbitration_isStuck)) = '1' report "DBusSimplePlugin doesn't allow memory stage stall when read happend"  severity FAILURE;
      assert (not (((writeBack_arbitration_isValid and writeBack_MEMORY_ENABLE) and (not writeBack_MEMORY_STORE)) and writeBack_arbitration_isStuck)) = '1' report "DBusSimplePlugin doesn't allow writeback stage stall when read happend"  severity FAILURE;
      if when_CsrPlugin_l909 = '1' then
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= pkg_toStdLogic(false);
      else
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
      end if;
      if when_CsrPlugin_l909_1 = '1' then
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= (CsrPlugin_exceptionPortCtrl_exceptionValids_decode and (not decode_arbitration_isStuck));
      else
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
      end if;
      if when_CsrPlugin_l909_2 = '1' then
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= (CsrPlugin_exceptionPortCtrl_exceptionValids_execute and (not execute_arbitration_isStuck));
      else
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
      end if;
      if when_CsrPlugin_l909_3 = '1' then
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= (CsrPlugin_exceptionPortCtrl_exceptionValids_memory and (not memory_arbitration_isStuck));
      else
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= pkg_toStdLogic(false);
      end if;
      CsrPlugin_interrupt_valid <= pkg_toStdLogic(false);
      if when_CsrPlugin_l946 = '1' then
        if when_CsrPlugin_l952 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if when_CsrPlugin_l952_1 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if when_CsrPlugin_l952_2 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if when_CsrPlugin_l952_3 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if when_CsrPlugin_l952_4 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if when_CsrPlugin_l952_5 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if when_CsrPlugin_l952_6 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
      end if;
      if CsrPlugin_pipelineLiberator_active = '1' then
        if when_CsrPlugin_l980 = '1' then
          CsrPlugin_pipelineLiberator_pcValids_0 <= pkg_toStdLogic(true);
        end if;
        if when_CsrPlugin_l980_1 = '1' then
          CsrPlugin_pipelineLiberator_pcValids_1 <= CsrPlugin_pipelineLiberator_pcValids_0;
        end if;
        if when_CsrPlugin_l980_2 = '1' then
          CsrPlugin_pipelineLiberator_pcValids_2 <= CsrPlugin_pipelineLiberator_pcValids_1;
        end if;
      end if;
      if when_CsrPlugin_l985 = '1' then
        CsrPlugin_pipelineLiberator_pcValids_0 <= pkg_toStdLogic(false);
        CsrPlugin_pipelineLiberator_pcValids_1 <= pkg_toStdLogic(false);
        CsrPlugin_pipelineLiberator_pcValids_2 <= pkg_toStdLogic(false);
      end if;
      if CsrPlugin_interruptJump = '1' then
        CsrPlugin_interrupt_valid <= pkg_toStdLogic(false);
      end if;
      CsrPlugin_hadException <= CsrPlugin_exception;
      if when_CsrPlugin_l1019 = '1' then
        case CsrPlugin_targetPrivilege is
          when "11" =>
            CsrPlugin_mstatus_MIE <= pkg_toStdLogic(false);
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          when others =>
        end case;
      end if;
      if when_CsrPlugin_l1064 = '1' then
        case switch_CsrPlugin_l1068 is
          when "11" =>
            CsrPlugin_mstatus_MPP <= pkg_unsigned("00");
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPIE <= pkg_toStdLogic(true);
          when others =>
        end case;
      end if;
      execute_CsrPlugin_wfiWake <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_when_CsrPlugin_l952_6),pkg_cat(pkg_toStdLogicVector(zz_when_CsrPlugin_l952_5),pkg_cat(pkg_toStdLogicVector(zz_when_CsrPlugin_l952_4),pkg_cat(pkg_toStdLogicVector(zz_when_CsrPlugin_l952_3),pkg_cat(pkg_toStdLogicVector(zz_when_CsrPlugin_l952_2),pkg_cat(pkg_toStdLogicVector(zz_when_CsrPlugin_l952_1),pkg_toStdLogicVector(zz_when_CsrPlugin_l952))))))) /= pkg_stdLogicVector("0000000")) or CsrPlugin_thirdPartyWake);
      zz_2 <= pkg_toStdLogic(false);
      memory_DivPlugin_div_counter_value <= memory_DivPlugin_div_counter_valueNext;
      HazardSimplePlugin_writeBackBuffer_valid <= HazardSimplePlugin_writeBackWrites_valid;
      if when_Pipeline_l151 = '1' then
        execute_arbitration_isValid <= pkg_toStdLogic(false);
      end if;
      if when_Pipeline_l154 = '1' then
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end if;
      if when_Pipeline_l151_1 = '1' then
        memory_arbitration_isValid <= pkg_toStdLogic(false);
      end if;
      if when_Pipeline_l154_1 = '1' then
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end if;
      if when_Pipeline_l151_2 = '1' then
        writeBack_arbitration_isValid <= pkg_toStdLogic(false);
      end if;
      if when_Pipeline_l154_2 = '1' then
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end if;
      case switch_Fetcher_l362 is
        when "000" =>
          if IBusSimplePlugin_injectionPort_valid = '1' then
            switch_Fetcher_l362 <= pkg_unsigned("001");
          end if;
        when "001" =>
          switch_Fetcher_l362 <= pkg_unsigned("010");
        when "010" =>
          switch_Fetcher_l362 <= pkg_unsigned("011");
        when "011" =>
          if when_Fetcher_l378 = '1' then
            switch_Fetcher_l362 <= pkg_unsigned("100");
          end if;
        when "100" =>
          switch_Fetcher_l362 <= pkg_unsigned("000");
        when others =>
      end case;
      if execute_CsrPlugin_csr_772 = '1' then
        if execute_CsrPlugin_writeEnable = '1' then
          LocalInt0_enable <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,16);
          LocalInt1_enable <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,17);
          LocalInt2_enable <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,18);
          LocalInt3_enable <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,19);
          CsrPlugin_mie_MEIE <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,11);
          CsrPlugin_mie_MTIE <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,7);
          CsrPlugin_mie_MSIE <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,3);
        end if;
      end if;
      if execute_CsrPlugin_csr_768 = '1' then
        if execute_CsrPlugin_writeEnable = '1' then
          CsrPlugin_mstatus_MPP <= unsigned(pkg_extract(CsrPlugin_csrMapping_writeDataSignal,12,11));
          CsrPlugin_mstatus_MPIE <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,7);
          CsrPlugin_mstatus_MIE <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,3);
        end if;
      end if;
      if execute_CsrPlugin_csr_773 = '1' then
        if execute_CsrPlugin_writeEnable = '1' then
          CsrPlugin_mtvec_base <= unsigned(pkg_extract(CsrPlugin_csrMapping_writeDataSignal,31,2));
          CsrPlugin_mtvec_mode <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,1,0);
        end if;
      end if;
      if iBus_cmd_ready = '1' then
        iBus_cmd_rValid <= iBus_cmd_valid;
      end if;
      if dBus_cmd_valid = '1' then
        dBus_cmd_rValid <= pkg_toStdLogic(true);
      end if;
      if dBus_cmd_halfPipe_fire = '1' then
        dBus_cmd_rValid <= pkg_toStdLogic(false);
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if IBusSimplePlugin_decompressor_input_valid = '1' then
        IBusSimplePlugin_decompressor_bufferValidLatch <= IBusSimplePlugin_decompressor_bufferValid;
      end if;
      if IBusSimplePlugin_decompressor_input_valid = '1' then
        IBusSimplePlugin_decompressor_throw2BytesLatch <= IBusSimplePlugin_decompressor_throw2Bytes;
      end if;
      if when_Fetcher_l286 = '1' then
        IBusSimplePlugin_decompressor_bufferData <= pkg_extract(IBusSimplePlugin_decompressor_input_payload_rsp_inst,31,16);
      end if;
      if IBusSimplePlugin_decompressor_output_ready = '1' then
        zz_IBusSimplePlugin_injector_decodeInput_payload_pc <= IBusSimplePlugin_decompressor_output_payload_pc;
        zz_IBusSimplePlugin_injector_decodeInput_payload_rsp_error <= IBusSimplePlugin_decompressor_output_payload_rsp_error;
        zz_IBusSimplePlugin_injector_decodeInput_payload_rsp_inst <= IBusSimplePlugin_decompressor_output_payload_rsp_inst;
        zz_IBusSimplePlugin_injector_decodeInput_payload_isRvc <= IBusSimplePlugin_decompressor_output_payload_isRvc;
      end if;
      if IBusSimplePlugin_injector_decodeInput_ready = '1' then
        IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_decompressor_raw;
      end if;
      CsrPlugin_mip_MEIP <= externalInterrupt;
      CsrPlugin_mip_MTIP <= timerInterrupt;
      CsrPlugin_mip_MSIP <= softwareInterrupt;
      CsrPlugin_mcycle <= (CsrPlugin_mcycle + pkg_unsigned("0000000000000000000000000000000000000000000000000000000000000001"));
      if writeBack_arbitration_isFiring = '1' then
        CsrPlugin_minstret <= (CsrPlugin_minstret + pkg_unsigned("0000000000000000000000000000000000000000000000000000000000000001"));
      end if;
      if decodeExceptionPort_valid = '1' then
        CsrPlugin_exceptionPortCtrl_exceptionContext_code <= decodeExceptionPort_payload_code;
        CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= decodeExceptionPort_payload_badAddr;
      end if;
      if CsrPlugin_selfException_valid = '1' then
        CsrPlugin_exceptionPortCtrl_exceptionContext_code <= CsrPlugin_selfException_payload_code;
        CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= CsrPlugin_selfException_payload_badAddr;
      end if;
      if DBusSimplePlugin_memoryExceptionPort_valid = '1' then
        CsrPlugin_exceptionPortCtrl_exceptionContext_code <= DBusSimplePlugin_memoryExceptionPort_payload_code;
        CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= DBusSimplePlugin_memoryExceptionPort_payload_badAddr;
      end if;
      if when_CsrPlugin_l946 = '1' then
        if when_CsrPlugin_l952 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("10000");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if when_CsrPlugin_l952_1 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("10001");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if when_CsrPlugin_l952_2 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("10010");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if when_CsrPlugin_l952_3 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("10011");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if when_CsrPlugin_l952_4 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("00111");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if when_CsrPlugin_l952_5 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("00011");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if when_CsrPlugin_l952_6 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("01011");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
      end if;
      if when_CsrPlugin_l1019 = '1' then
        case CsrPlugin_targetPrivilege is
          when "11" =>
            CsrPlugin_mcause_interrupt <= (not CsrPlugin_hadException);
            CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
            CsrPlugin_mepc <= writeBack_PC;
            if CsrPlugin_hadException = '1' then
              CsrPlugin_mtval <= CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
            end if;
          when others =>
        end case;
      end if;
      if when_MulDivIterativePlugin_l126 = '1' then
        memory_DivPlugin_div_done <= pkg_toStdLogic(true);
      end if;
      if when_MulDivIterativePlugin_l126_1 = '1' then
        memory_DivPlugin_div_done <= pkg_toStdLogic(false);
      end if;
      if when_MulDivIterativePlugin_l128 = '1' then
        if when_MulDivIterativePlugin_l132 = '1' then
          memory_DivPlugin_rs1(31 downto 0) <= memory_DivPlugin_div_stage_0_outNumerator;
          memory_DivPlugin_accumulator(31 downto 0) <= memory_DivPlugin_div_stage_0_outRemainder;
          if when_MulDivIterativePlugin_l151 = '1' then
            memory_DivPlugin_div_result <= pkg_resize(std_logic_vector(signed((unsigned(pkg_cat(pkg_toStdLogicVector(memory_DivPlugin_div_needRevert),std_logic_vector(pkg_mux(memory_DivPlugin_div_needRevert,pkg_not(zz_memory_DivPlugin_div_result),zz_memory_DivPlugin_div_result)))) + pkg_resize(unsigned(pkg_toStdLogicVector(memory_DivPlugin_div_needRevert)),33)))),32);
          end if;
        end if;
      end if;
      if when_MulDivIterativePlugin_l162 = '1' then
        memory_DivPlugin_accumulator <= pkg_unsigned("00000000000000000000000000000000000000000000000000000000000000000");
        memory_DivPlugin_rs1 <= (unsigned(pkg_mux(zz_memory_DivPlugin_rs1,pkg_not(zz_memory_DivPlugin_rs1_1),zz_memory_DivPlugin_rs1_1)) + pkg_resize(unsigned(pkg_toStdLogicVector(zz_memory_DivPlugin_rs1)),33));
        memory_DivPlugin_rs2 <= (unsigned(pkg_mux(zz_memory_DivPlugin_rs2,pkg_not(execute_RS2),execute_RS2)) + pkg_resize(unsigned(pkg_toStdLogicVector(zz_memory_DivPlugin_rs2)),32));
        memory_DivPlugin_div_needRevert <= ((zz_memory_DivPlugin_rs1 xor (zz_memory_DivPlugin_rs2 and (not pkg_extract(execute_INSTRUCTION,13)))) and (not ((pkg_toStdLogic(execute_RS2 = pkg_stdLogicVector("00000000000000000000000000000000")) and execute_IS_RS2_SIGNED) and (not pkg_extract(execute_INSTRUCTION,13)))));
      end if;
      HazardSimplePlugin_writeBackBuffer_payload_address <= HazardSimplePlugin_writeBackWrites_payload_address;
      HazardSimplePlugin_writeBackBuffer_payload_data <= HazardSimplePlugin_writeBackWrites_payload_data;
      if when_Pipeline_l124 = '1' then
        decode_to_execute_PC <= zz_decode_SRC2;
      end if;
      if when_Pipeline_l124_1 = '1' then
        execute_to_memory_PC <= execute_PC;
      end if;
      if when_Pipeline_l124_2 = '1' then
        memory_to_writeBack_PC <= memory_PC;
      end if;
      if when_Pipeline_l124_3 = '1' then
        decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
      end if;
      if when_Pipeline_l124_4 = '1' then
        execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
      end if;
      if when_Pipeline_l124_5 = '1' then
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end if;
      if when_Pipeline_l124_6 = '1' then
        decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
      end if;
      if when_Pipeline_l124_7 = '1' then
        execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
      end if;
      if when_Pipeline_l124_8 = '1' then
        memory_to_writeBack_FORMAL_PC_NEXT <= zz_memory_to_writeBack_FORMAL_PC_NEXT;
      end if;
      if when_Pipeline_l124_9 = '1' then
        decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
      end if;
      if when_Pipeline_l124_10 = '1' then
        decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
      end if;
      if when_Pipeline_l124_11 = '1' then
        decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
      end if;
      if when_Pipeline_l124_12 = '1' then
        decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
      end if;
      if when_Pipeline_l124_13 = '1' then
        execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
      end if;
      if when_Pipeline_l124_14 = '1' then
        memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
      end if;
      if when_Pipeline_l124_15 = '1' then
        decode_to_execute_ALU_CTRL <= zz_decode_to_execute_ALU_CTRL;
      end if;
      if when_Pipeline_l124_16 = '1' then
        decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
      end if;
      if when_Pipeline_l124_17 = '1' then
        execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
      end if;
      if when_Pipeline_l124_18 = '1' then
        memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
      end if;
      if when_Pipeline_l124_19 = '1' then
        decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
      end if;
      if when_Pipeline_l124_20 = '1' then
        decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
      end if;
      if when_Pipeline_l124_21 = '1' then
        execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
      end if;
      if when_Pipeline_l124_22 = '1' then
        decode_to_execute_MEMORY_STORE <= decode_MEMORY_STORE;
      end if;
      if when_Pipeline_l124_23 = '1' then
        execute_to_memory_MEMORY_STORE <= execute_MEMORY_STORE;
      end if;
      if when_Pipeline_l124_24 = '1' then
        memory_to_writeBack_MEMORY_STORE <= memory_MEMORY_STORE;
      end if;
      if when_Pipeline_l124_25 = '1' then
        decode_to_execute_IS_CSR <= decode_IS_CSR;
      end if;
      if when_Pipeline_l124_26 = '1' then
        decode_to_execute_ENV_CTRL <= zz_decode_to_execute_ENV_CTRL;
      end if;
      if when_Pipeline_l124_27 = '1' then
        execute_to_memory_ENV_CTRL <= zz_execute_to_memory_ENV_CTRL;
      end if;
      if when_Pipeline_l124_28 = '1' then
        memory_to_writeBack_ENV_CTRL <= zz_memory_to_writeBack_ENV_CTRL;
      end if;
      if when_Pipeline_l124_29 = '1' then
        decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
      end if;
      if when_Pipeline_l124_30 = '1' then
        decode_to_execute_ALU_BITWISE_CTRL <= zz_decode_to_execute_ALU_BITWISE_CTRL;
      end if;
      if when_Pipeline_l124_31 = '1' then
        decode_to_execute_IS_MUL <= decode_IS_MUL;
      end if;
      if when_Pipeline_l124_32 = '1' then
        execute_to_memory_IS_MUL <= execute_IS_MUL;
      end if;
      if when_Pipeline_l124_33 = '1' then
        memory_to_writeBack_IS_MUL <= memory_IS_MUL;
      end if;
      if when_Pipeline_l124_34 = '1' then
        decode_to_execute_IS_DIV <= decode_IS_DIV;
      end if;
      if when_Pipeline_l124_35 = '1' then
        execute_to_memory_IS_DIV <= execute_IS_DIV;
      end if;
      if when_Pipeline_l124_36 = '1' then
        decode_to_execute_IS_RS1_SIGNED <= decode_IS_RS1_SIGNED;
      end if;
      if when_Pipeline_l124_37 = '1' then
        decode_to_execute_IS_RS2_SIGNED <= decode_IS_RS2_SIGNED;
      end if;
      if when_Pipeline_l124_38 = '1' then
        decode_to_execute_SHIFT_CTRL <= zz_decode_to_execute_SHIFT_CTRL;
      end if;
      if when_Pipeline_l124_39 = '1' then
        execute_to_memory_SHIFT_CTRL <= zz_execute_to_memory_SHIFT_CTRL;
      end if;
      if when_Pipeline_l124_40 = '1' then
        decode_to_execute_BRANCH_CTRL <= zz_decode_to_execute_BRANCH_CTRL;
      end if;
      if when_Pipeline_l124_41 = '1' then
        decode_to_execute_RS1 <= zz_decode_SRC1;
      end if;
      if when_Pipeline_l124_42 = '1' then
        decode_to_execute_RS2 <= zz_decode_SRC2_1;
      end if;
      if when_Pipeline_l124_43 = '1' then
        decode_to_execute_SRC2_FORCE_ZERO <= decode_SRC2_FORCE_ZERO;
      end if;
      if when_Pipeline_l124_44 = '1' then
        decode_to_execute_SRC1 <= decode_SRC1;
      end if;
      if when_Pipeline_l124_45 = '1' then
        decode_to_execute_SRC2 <= decode_SRC2;
      end if;
      if when_Pipeline_l124_46 = '1' then
        decode_to_execute_DO_EBREAK <= decode_DO_EBREAK;
      end if;
      if when_Pipeline_l124_47 = '1' then
        execute_to_memory_ALIGNEMENT_FAULT <= execute_ALIGNEMENT_FAULT;
      end if;
      if when_Pipeline_l124_48 = '1' then
        execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
      end if;
      if when_Pipeline_l124_49 = '1' then
        memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
      end if;
      if when_Pipeline_l124_50 = '1' then
        execute_to_memory_REGFILE_WRITE_DATA <= zz_decode_RS2_1;
      end if;
      if when_Pipeline_l124_51 = '1' then
        memory_to_writeBack_REGFILE_WRITE_DATA <= zz_decode_RS2;
      end if;
      if when_Pipeline_l124_52 = '1' then
        execute_to_memory_MUL_LL <= execute_MUL_LL;
      end if;
      if when_Pipeline_l124_53 = '1' then
        execute_to_memory_MUL_LH <= execute_MUL_LH;
      end if;
      if when_Pipeline_l124_54 = '1' then
        execute_to_memory_MUL_HL <= execute_MUL_HL;
      end if;
      if when_Pipeline_l124_55 = '1' then
        execute_to_memory_MUL_HH <= execute_MUL_HH;
      end if;
      if when_Pipeline_l124_56 = '1' then
        memory_to_writeBack_MUL_HH <= memory_MUL_HH;
      end if;
      if when_Pipeline_l124_57 = '1' then
        execute_to_memory_SHIFT_RIGHT <= execute_SHIFT_RIGHT;
      end if;
      if when_Pipeline_l124_58 = '1' then
        execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
      end if;
      if when_Pipeline_l124_59 = '1' then
        execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
      end if;
      if when_Pipeline_l124_60 = '1' then
        memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
      end if;
      if when_Pipeline_l124_61 = '1' then
        memory_to_writeBack_MUL_LOW <= memory_MUL_LOW;
      end if;
      if when_Fetcher_l398 = '1' then
        zz_IBusSimplePlugin_injector_decodeInput_payload_rsp_inst <= IBusSimplePlugin_injectionPort_payload;
      end if;
      if when_CsrPlugin_l1264 = '1' then
        execute_CsrPlugin_csr_836 <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,31,20) = pkg_stdLogicVector("001101000100"));
      end if;
      if when_CsrPlugin_l1264_1 = '1' then
        execute_CsrPlugin_csr_772 <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,31,20) = pkg_stdLogicVector("001100000100"));
      end if;
      if when_CsrPlugin_l1264_2 = '1' then
        execute_CsrPlugin_csr_768 <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,31,20) = pkg_stdLogicVector("001100000000"));
      end if;
      if when_CsrPlugin_l1264_3 = '1' then
        execute_CsrPlugin_csr_773 <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,31,20) = pkg_stdLogicVector("001100000101"));
      end if;
      if when_CsrPlugin_l1264_4 = '1' then
        execute_CsrPlugin_csr_833 <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,31,20) = pkg_stdLogicVector("001101000001"));
      end if;
      if when_CsrPlugin_l1264_5 = '1' then
        execute_CsrPlugin_csr_834 <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,31,20) = pkg_stdLogicVector("001101000010"));
      end if;
      if execute_CsrPlugin_csr_836 = '1' then
        if execute_CsrPlugin_writeEnable = '1' then
          CsrPlugin_mip_MSIP <= pkg_extract(CsrPlugin_csrMapping_writeDataSignal,3);
        end if;
      end if;
      if execute_CsrPlugin_csr_833 = '1' then
        if execute_CsrPlugin_writeEnable = '1' then
          CsrPlugin_mepc <= unsigned(pkg_extract(CsrPlugin_csrMapping_writeDataSignal,31,0));
        end if;
      end if;
      if iBus_cmd_ready = '1' then
        iBus_cmd_rData_pc <= iBus_cmd_payload_pc;
      end if;
      if dBus_cmd_ready = '1' then
        dBus_cmd_rData_wr <= dBus_cmd_payload_wr;
        dBus_cmd_rData_address <= dBus_cmd_payload_address;
        dBus_cmd_rData_data <= dBus_cmd_payload_data;
        dBus_cmd_rData_size <= dBus_cmd_payload_size;
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      DebugPlugin_firstCycle <= pkg_toStdLogic(false);
      if debug_bus_cmd_ready_read_buffer = '1' then
        DebugPlugin_firstCycle <= pkg_toStdLogic(true);
      end if;
      DebugPlugin_secondCycle <= DebugPlugin_firstCycle;
      DebugPlugin_isPipBusy <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_isValid),pkg_cat(pkg_toStdLogicVector(memory_arbitration_isValid),pkg_cat(pkg_toStdLogicVector(execute_arbitration_isValid),pkg_toStdLogicVector(decode_arbitration_isValid)))) /= pkg_stdLogicVector("0000")) or IBusSimplePlugin_incomingInstruction);
      if writeBack_arbitration_isValid = '1' then
        DebugPlugin_busReadDataReg <= zz_decode_RS2_2;
      end if;
      zz_when_DebugPlugin_l244 <= pkg_extract(debug_bus_cmd_payload_address,2);
      zz_3 <= pkg_extract(debug_bus_cmd_payload_address,7,2);
      if debug_bus_cmd_valid = '1' then
        case switch_DebugPlugin_l268 is
          when "010000" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_0_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "010001" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_1_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "010010" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_2_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "010011" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_3_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "010100" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_4_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "010101" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_5_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "010110" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_6_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "010111" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_7_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011000" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_8_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011001" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_9_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011010" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_10_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011011" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_11_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011100" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_12_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011101" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_13_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011110" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_14_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when "011111" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_15_pc <= unsigned(pkg_extract(debug_bus_cmd_payload_data,31,1));
            end if;
          when others =>
        end case;
      end if;
      if when_DebugPlugin_l296 = '1' then
        DebugPlugin_busReadDataReg <= std_logic_vector(execute_PC);
      end if;
      DebugPlugin_resetIt_regNext <= DebugPlugin_resetIt;
    end if;
  end process;

  process(clk, debugReset)
  begin
    if debugReset = '1' then
      DebugPlugin_resetIt <= pkg_toStdLogic(false);
      DebugPlugin_haltIt <= pkg_toStdLogic(false);
      DebugPlugin_stepIt <= pkg_toStdLogic(false);
      DebugPlugin_godmode <= pkg_toStdLogic(false);
      DebugPlugin_haltedByBreak <= pkg_toStdLogic(false);
      DebugPlugin_debugUsed <= pkg_toStdLogic(false);
      DebugPlugin_disableEbreak <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_0_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_1_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_2_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_3_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_4_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_5_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_6_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_7_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_8_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_9_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_10_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_11_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_12_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_13_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_14_valid <= pkg_toStdLogic(false);
      DebugPlugin_hardwareBreakpoints_15_valid <= pkg_toStdLogic(false);
      zz_4 <= pkg_toStdLogic(false);
    elsif rising_edge(clk) then
      if when_DebugPlugin_l225 = '1' then
        DebugPlugin_godmode <= pkg_toStdLogic(true);
      end if;
      if debug_bus_cmd_valid = '1' then
        DebugPlugin_debugUsed <= pkg_toStdLogic(true);
      end if;
      if debug_bus_cmd_valid = '1' then
        case switch_DebugPlugin_l268 is
          when "000000" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_stepIt <= pkg_extract(debug_bus_cmd_payload_data,4);
              if when_DebugPlugin_l272 = '1' then
                DebugPlugin_resetIt <= pkg_toStdLogic(true);
              end if;
              if when_DebugPlugin_l272_1 = '1' then
                DebugPlugin_resetIt <= pkg_toStdLogic(false);
              end if;
              if when_DebugPlugin_l273 = '1' then
                DebugPlugin_haltIt <= pkg_toStdLogic(true);
              end if;
              if when_DebugPlugin_l273_1 = '1' then
                DebugPlugin_haltIt <= pkg_toStdLogic(false);
              end if;
              if when_DebugPlugin_l274 = '1' then
                DebugPlugin_haltedByBreak <= pkg_toStdLogic(false);
              end if;
              if when_DebugPlugin_l275 = '1' then
                DebugPlugin_godmode <= pkg_toStdLogic(false);
              end if;
              if when_DebugPlugin_l276 = '1' then
                DebugPlugin_disableEbreak <= pkg_toStdLogic(true);
              end if;
              if when_DebugPlugin_l276_1 = '1' then
                DebugPlugin_disableEbreak <= pkg_toStdLogic(false);
              end if;
            end if;
          when "010000" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_0_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "010001" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_1_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "010010" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_2_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "010011" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_3_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "010100" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_4_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "010101" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_5_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "010110" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_6_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "010111" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_7_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011000" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_8_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011001" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_9_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011010" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_10_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011011" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_11_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011100" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_12_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011101" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_13_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011110" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_14_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when "011111" =>
            if debug_bus_cmd_payload_wr = '1' then
              DebugPlugin_hardwareBreakpoints_15_valid <= pkg_extract(debug_bus_cmd_payload_data,0);
            end if;
          when others =>
        end case;
      end if;
      if when_DebugPlugin_l296 = '1' then
        if when_DebugPlugin_l299 = '1' then
          DebugPlugin_haltIt <= pkg_toStdLogic(true);
          DebugPlugin_haltedByBreak <= pkg_toStdLogic(true);
        end if;
      end if;
      if when_DebugPlugin_l312 = '1' then
        if decode_arbitration_isValid = '1' then
          DebugPlugin_haltIt <= pkg_toStdLogic(true);
        end if;
      end if;
      zz_4 <= (DebugPlugin_stepIt and decode_arbitration_isFiring);
    end if;
  end process;

end arch;

