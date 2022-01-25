package vexriscv.demo

import vexriscv.plugin._
import vexriscv.{VexRiscv, VexRiscvConfig, plugin}
import spinal.core._
import spinal.lib._

object VexRiscvWB_small{
  def main(args: Array[String]) {
    //val report = SpinalConfig(mode = Verilog).generate{
    val report = SpinalConfig(mode = VHDL).generate{

      //CPU configuration
      val cpuConfig = VexRiscvConfig(
        plugins = List(
          new IBusSimplePlugin(
            resetVector = 0x00010000l,
            cmdForkOnSecondStage = false,
            cmdForkPersistence = false,
            prediction = NONE,
            catchAccessFault = false,
            compressedGen = true
          ),
          new DBusSimplePlugin(
            catchAddressMisaligned = true,
            catchAccessFault = false
          ),
          new CsrPlugin(
            CsrPluginConfig(
              catchIllegalAccess = false,
              mvendorid          = null,
              marchid            = null,
              mimpid             = null,
              mhartid            = null,
              misaExtensionsInit = 66,
              misaAccess         = CsrAccess.NONE,
              mtvecAccess        = CsrAccess.READ_WRITE, // required for xtvecModeGen
              mtvecInit          = 0x00010011l,
              mepcAccess         = CsrAccess.READ_WRITE, // required to access EPC from software 
              mscratchGen        = false,
              mcauseAccess       = CsrAccess.READ_ONLY,
              mbadaddrAccess     = CsrAccess.NONE,
              mcycleAccess       = CsrAccess.NONE,
              minstretAccess     = CsrAccess.NONE,
              ecallGen           = true,                   // enable
              wfiGenAsWait       = false,
              ucycleAccess       = CsrAccess.NONE,
              uinstretAccess     = CsrAccess.NONE,
              xtvecModeGen       = true
            )
          ),
          new DecoderSimplePlugin(
            catchIllegalInstruction = true
          ),
          new RegFilePlugin(
            regFileReadyKind = plugin.SYNC,
            zeroBoot = false
          ),
          new IntAluPlugin,
          new MulPlugin,
          new DivPlugin,
          new SrcPlugin(
            separatedAddSub = false,
            executeInsertion = false
          ),
          //new LightShifterPlugin,
          new FullBarrelShifterPlugin,
          new HazardSimplePlugin(
            bypassExecute           = true,
            bypassMemory            = true,
            bypassWriteBack         = true,
            bypassWriteBackBuffer   = true,
            pessimisticUseSrc       = false,
            pessimisticWriteRegFile = false,
            pessimisticAddressMatch = false
          ),
          new DebugPlugin(
            debugClockDomain        = ClockDomain.current.clone(reset = Bool().setName("debugReset")),
            hardwareBreakpointCount = 16
          ),
          new BranchPlugin(
            earlyBranch = false,
            catchAddressMisaligned = true
          ),
          new UserInterruptPlugin(
            interruptName = "LocalInt0",
            code = 16
          ),
          new UserInterruptPlugin(
            interruptName = "LocalInt1",
            code = 17
          ),
          new UserInterruptPlugin(
            interruptName = "LocalInt2",
            code = 18
          ),
          new UserInterruptPlugin(
            interruptName = "LocalInt3",
            code = 19
          ),
          new YamlPlugin("cpu0.yaml")
        )
      )

      //CPU instanciation
      val cpu = new VexRiscv(cpuConfig)

      //CPU modifications to be an AhbLite3 one
      cpu.rework {
        for (plugin <- cpuConfig.plugins) plugin match {
          case plugin: IBusSimplePlugin => {
            plugin.iBus.setAsDirectionLess() //Unset IO properties of iBus
            master(plugin.iBus.toWishbone()).setName("iBusWishbone")
          }
          case plugin: DBusSimplePlugin => {
            plugin.dBus.setAsDirectionLess()
            master(plugin.dBus.toWishbone()).setName("dBusWishbone")
          }
          case _ =>
        }
      }
      cpu
    }
  }
}

