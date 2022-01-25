#----------------------------------------------------------------------------------
# (c) Bernhard Lang, Hochschule Osnabrueck
#----------------------------------------------------------------------------------
vcom -work work ScaleClock.vhd

vcom -work work VexRiscvWB_small.vhd
vcom -work work GDB_RSP/GDB_RSP_Debug.vhd

vcom -work work Memory/Memory.vhd

vcom -work work Peripherals/GPIO.vhd

vcom -work work Peripherals/Timer.vhd 

vcom -work work Interconnect.vhd
vcom -work work VexRiscvWB_System.vhd

vcom -work work VexRiscvWB_TOP.vhd

vcom -work work string_stream_pack.vhd
vcom -work work UART_Sender_nur_Simulation.vhd
vcom -work work VexRiscvWB_TOP_TB.vhd

vsim -novopt -t ps -noglitch work.vexriscvwb_top_tb

set NumericStdNoWarnings 1

config wave -signalnamewidth 1

if (1) { add wave /vexriscvwb_top_tb/ExtClk            }
if (1) { add wave /vexriscvwb_top_tb/DUT/Reset         }
if (1) { add wave /vexriscvwb_top_tb/DUT/SysClk        }
if (1) { add wave /vexriscvwb_top_tb/DUT/SysReset      }
if (1) { add wave /vexriscvwb_top_tb/DUT/clkgen/locked }

if (1) {
  add wave -divider "== DBG_TXD, DBG_RXD =="
  add wave          /vexriscvwb_top_tb/s_valid
  add wave          /vexriscvwb_top_tb/s_last 
  add wave -ASCII   /vexriscvwb_top_tb/s_data 
  add wave          /vexriscvwb_top_tb/s_ready
  add wave          /vexriscvwb_top_tb/DBG_Sender/marker
  add wave          /vexriscvwb_top_tb/DBG_RXD
  add wave          /vexriscvwb_top_tb/Stimulate/message
  add wave          /vexriscvwb_top_tb/DBG_TXD
  add wave -ASCII   /vexriscvwb_top_tb/response
}

add wave -divider "== VexRiscV =="
add wave -hexadecimal /vexriscvwb_top_tb/DUT/MCU/CPU/execute_PC
add wave -hexadecimal /vexriscvwb_top_tb/DUT/MCU/CPU/execute_INSTRUCTION
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/execute_arbitration_isValid
add wave -hexadecimal /vexriscvwb_top_tb/DUT/MCU/CPU/RegFilePlugin_regFile(1)
add wave -hexadecimal /vexriscvwb_top_tb/DUT/MCU/CPU/RegFilePlugin_regFile(2)
add wave -hexadecimal /vexriscvwb_top_tb/DUT/MCU/CPU/RegFilePlugin_regFile(3)

add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/DebugPlugin_resetIt
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/DebugPlugin_haltIt
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/DebugPlugin_stepIt
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/DebugPlugin_godmode
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/DebugPlugin_haltedByBreak
add wave -divider "==================="
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/debug_bus_cmd_valid
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/debug_bus_cmd_payload_wr
add wave -hexadecimal /vexriscvwb_top_tb/DUT/MCU/CPU/debug_bus_cmd_payload_address
add wave -hexadecimal /vexriscvwb_top_tb/DUT/MCU/CPU/debug_bus_rsp_data
add wave              /vexriscvwb_top_tb/DUT/MCU/CPU/debug_bus_cmd_ready

if (1) {
  add wave -divider "== VexRiscV dbgbus =="
  add wave                      /vexriscvwb_top_tb/DUT/Debug_IF/dbgbus_cmd_valid
  add wave                      /vexriscvwb_top_tb/DUT/Debug_IF/dbgbus_cmd_payload_wr
  add wave -hexadecimal         /vexriscvwb_top_tb/DUT/Debug_IF/dbgbus_cmd_payload_address
  add wave -hexadecimal         /vexriscvwb_top_tb/DUT/Debug_IF/dbgbus_cmd_payload_data
  add wave                      /vexriscvwb_top_tb/DUT/Debug_IF/dbgbus_cmd_ready
  add wave -hexadecimal         /vexriscvwb_top_tb/DUT/Debug_IF/dbgbus_rsp_data
  add wave                      /vexriscvwb_top_tb/DUT/MCU/CPU_Halted
}

if (0) {
  add wave -divider "== RXD TXD =="
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/in_valid
  add wave -ASCII /vexriscvwb_top_tb/DUT/Debug_IF/in_data
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/in_ready
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRRF_valid

  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRRF_last
  add wave -ASCII /vexriscvwb_top_tb/DUT/Debug_IF/GRRF_data
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRRF_ready


  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRI/rspC_valid
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRI/rspC_last
  add wave -ASCII /vexriscvwb_top_tb/DUT/Debug_IF/GRI/rspC_data
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRI/rspC_ready
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRI/rspR_valid
  add wave -ASCII /vexriscvwb_top_tb/DUT/Debug_IF/GRI/rspR_data
  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRI/rspR_ready

  add wave /vexriscvwb_top_tb/DUT/Debug_IF/GRI/control_path/state
}

run 700 us

wave zoom full



