#!/usr/bin/env python3

# This file is Copyright (c) Greg Davill <greg.davill@gmail.com>
# License: BSD


# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["vivado"]

# Import lxbuildenv to integrate the deps/ directory
#import lxbuildenv

import sys
import os
import shutil
import argparse
import subprocess

import inspect

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import arty
#from litex_boards.targets.orangecrab import _CRG

from litex.build.xilinx.vivado import vivado_build_args, vivado_build_argdict

from litex.build.generic_platform import IOStandard, Subsignal, Pins, Misc

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
#from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *

from litedram.modules import MT41K128M16
from litedram.phy import s7ddrphy

from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.bitbang import I2CMaster

from litex.soc.doc import generate_docs


from migen.genlib.cdc import MultiReg


from modules.rgb import RGB
from modules.analog import AnalogSense
from modules.csr_cdc import CSRClockDomainWrapper
from modules.io_block import IOPort

from litex.soc.cores import spi_flash
from litex.soc.cores.gpio import GPIOTristate, GPIOOut, GPIOIn

from valentyusb.usbcore import io as usbio
from litex.soc.cores.usb_ohci import USBOHCI

# connect all remaninig GPIO pins out
extras = [
    ("gpio", 0, Pins("pmodb:0"), 
        IOStandard("LVCMOS33")
    ),
    ("usb", 0,
        Subsignal("d_p", Pins("pmoda:3")),
        Subsignal("d_n", Pins("pmoda:2")),
        Subsignal("pullup", Pins("pmoda:1")),
        IOStandard("LVCMOS33")
    ),
    ("usb_ohci", 0,
        Subsignal("dp", Pins("pmodc:0"), Misc("PULLDOWN")), #USB hosts need 15K pulldowns (220uA@3.3V), artix7 provides 68-330uA@3.3V (48K-10K
        Subsignal("dm", Pins("pmodc:4"), Misc("PULLDOWN")), #this is minimum 2.8V ViH with the 1.5K pullup on the device, so > 2.0V as the USB spec
        IOStandard("LVCMOS33"),
    ),
]


# CRG ---------------------------------------------------------------------------------------------

class CRG(Module):
	def __init__(self, platform, sys_clk_freq, with_usb_pll=False, with_dram=True):
		self.rst = Signal()
		self.clock_domains.cd_sys       = ClockDomain()
		if with_dram:
		    self.clock_domains.cd_sys4x     = ClockDomain(reset_less=True)
		    self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
		    self.clock_domains.cd_idelay    = ClockDomain()


		# Clk/Rst.
		clk100 = platform.request("clk100")

		# PLL.
		self.submodules.pll = pll = S7PLL(speedgrade=-1)
		self.comb += pll.reset.eq(self.rst)
		pll.register_clkin(clk100, 100e6)
		pll.create_clkout(self.cd_sys, sys_clk_freq)
		platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

		if with_dram:
		    pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
		    pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
		    pll.create_clkout(self.cd_idelay,    200e6)
		    self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

		#USB device/host clocks
		if with_usb_pll:
		    self.clock_domains.cd_usb_12       = ClockDomain(reset_less=False)
		    self.clock_domains.cd_usb_48       = ClockDomain(reset_less=False)
		    pll.create_clkout(self.cd_usb_12, 12e6, margin=1e-3)
		    pll.create_clkout(self.cd_usb_48, 48e6, margin=1e-3)        


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    csr_map = {
        "ctrl":           0,  # provided by default (optional)
        "crg":            1,  # user
        "identifier_mem": 4,  # provided by default (optional)
        "timer0":         5,  # provided by default (optional)
        "rgb":            10,
        "gpio":           11,
        "self_reset":     12,
        "version":        14,
        "lxspi":          15,
        "button":         17,
        "asense":         18,
    }
    csr_map.update(SoCCore.csr_map)

    mem_map = {
        "rom":      0x00000000,  # (default shadow @0x80000000)
        "sram":     0x10000000,  # (default shadow @0xa0000000)
        "spiflash": 0x20000000,  # (default shadow @0xa0000000)
        "main_ram": 0x40000000,  # (default shadow @0xc0000000)
        "csr":      0xe0000000,  # (default shadow @0xe0000000)
        "usb_ohci": 0xd0000000, #set to 0xd0000000 to avoid any shadow conflict 
    }
    mem_map.update(SoCCore.mem_map)

    interrupt_map = {
        "timer0": 2,
        "usb": 3,
        #"usb_ohci_ctrl": 4,
    }
    interrupt_map.update(SoCCore.interrupt_map)

    def __init__(self, sys_clk_freq, toolchain="vivado", **kwargs):
        platform = arty.Platform()
        platform.add_extension(extras)

        # Disconnect Serial Debug (Stub required so BIOS is kept happy)
        #kwargs['uart_name']="stream"

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs)

        # connect UART stream to NULL
        self.comb += self.uart.source.ready.eq(1)
        
        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = crg = CRG(platform, sys_clk_freq, with_usb_pll=True,
        	with_dram=not self.integrated_main_ram_size)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.submodules.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype        = "DDR3",
                nphases        = 4,
                sys_clk_freq   = sys_clk_freq)
            self.add_csr("ddrphy")
            self.add_sdram("sdram", 
                phy                     = self.ddrphy,
                module                  = MT41K128M16(sys_clk_freq, "1:4"),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True
            )


        # RGB LED
        self.submodules.rgb = RGB(platform.request("rgb_led", 0))
        #self.submodules.gpio = GPIOTristateCustom(platform.request("gpio", 0))

        self.submodules.gpio= IOPort(platform.request("gpio",0))

        try:
            self.submodules.button = GPIOIn(platform.request("user_btn"))
        except:
            ...
        
        # Analog Mux
        #self.submodules.asense = AnalogSense(platform.request("analog"))
        
        # drive PROGRAMN HIGH
        #self.comb += platform.request("rst_n").eq(1)

        if True:
            from litespi.modules import S25FL128L
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="4x", module=S25FL128L(Codes.READ_1_1_4), rate="1:2", with_master=True)

        self.constants["FLASH_BOOT_ADDRESS"] = self.mem_map['spiflash'] + 0x00100000

        with_usb_host = True
        if with_usb_host:
            self.submodules.usb_ohci = USBOHCI(platform, platform.request("usb_ohci"), usb_clk_freq=int(48e6), usb_clock_name="usb_48")
            self.bus.add_slave("usb_ohci_ctrl", self.usb_ohci.wb_ctrl, region=SoCRegion(origin=self.mem_map["usb_ohci"], size=0x100000, cached=False))
            self.bus.add_master("usb_ohci_dma", master=self.usb_ohci.wb_dma)

            #self.comb += self.cpu.interrupt[self.interrupt_map["usb_ohci_ctrl"]].eq(self.usb_ohci.interrupt)
            self.comb += self.cpu.interrupt[4].eq(self.usb_ohci.interrupt) #works
            self.constants["USB_OHCI_CTRL_INTERRUPT"] = 4
            self.constants["USB_OHCI_BASE"] = self.mem_map["usb_ohci"]
            
        # Attach USB to a seperate CSR bus that's decoupled from our CPU clock
        usb_pads = platform.request("usb")
        usb_iobuf = usbio.IoBuf(usb_pads.d_p, usb_pads.d_n, usb_pads.pullup)
        """
        self.submodules.usb0 = CSRClockDomainWrapper(usb_iobuf)
        self.comb += self.cpu.interrupt[self.interrupt_map['usb']].eq(self.usb0.irq)

        from litex.soc.integration.soc_core import SoCRegion
        self.bus.add_slave('usb', self.usb0.bus, SoCRegion(origin=0x90000000, size=0x1000, cached=False))

        """
        if True:
            from valentyusb.usbcore.cpu import eptri
            self.submodules.usb = eptri.TriEndpointInterface(usb_iobuf, cdc=True, debug=False) #use cdc=True even for composite devices




# Build --------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Arty A7")
    parser.add_argument("--toolchain",           default="vivado",                 help="FPGA toolchain (vivado, symbiflow or yosys+nextpnr).")
    parser.add_argument("--build",               action="store_true",              help="Build bitstream.")
    parser.add_argument("--load",                action="store_true",              help="Load bitstream.")
    parser.add_argument("--flash",               action="store_true",              help="Flash bitstream.")
    parser.add_argument("--variant",             default="a7-35",                  help="Board variant (a7-35 or a7-100).")
    parser.add_argument("--sys-clk-freq",        default=100e6,                    help="System clock frequency.")
    ethopts = parser.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",      action="store_true",              help="Enable Ethernet support.")
    ethopts.add_argument("--with-etherbone",     action="store_true",              help="Enable Etherbone support.")
    parser.add_argument("--eth-ip",              default="192.168.1.50", type=str, help="Ethernet/Etherbone IP address.")
    parser.add_argument("--eth-dynamic-ip",      action="store_true",              help="Enable dynamic Ethernet IP addresses setting.")
    sdopts = parser.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",     action="store_true",              help="Enable SPI-mode SDCard support.")
    sdopts.add_argument("--with-sdcard",         action="store_true",              help="Enable SDCard support.")
    parser.add_argument("--sdcard-adapter",      type=str,                         help="SDCard PMOD adapter (digilent or numato).")
    parser.add_argument("--with-jtagbone",       action="store_true",              help="Enable JTAGbone support.")
    parser.add_argument("--with-spi-flash",      action="store_true",              help="Enable SPI Flash (MMAPed).")
    builder_args(parser)
    soc_core_args(parser)
    vivado_build_args(parser)
    args = parser.parse_args()
    
    soc = BaseSoC(toolchain=args.toolchain, sys_clk_freq=int(float(args.sys_clk_freq)),**soc_core_argdict(args))

    builder = Builder(soc, **builder_argdict(args))

          
    # Build gateware
    builder_kargs = vivado_build_argdict(args)
    vns = builder.build(**builder_kargs)
    soc.do_exit(vns)   

    generate_docs(soc, "build/documentation/", project_name="OrangeCrab Test SoC", author="Greg Davill")

    input_config = os.path.join(builder.output_dir, "gateware", f"{soc.platform.name}.config")
    output_bitstream = os.path.join(builder.gateware_dir, f"{soc.platform.name}.bit")

    dfu_file = os.path.join(builder.gateware_dir, f"{soc.platform.name}.dfu")
    shutil.copyfile(output_bitstream, dfu_file)
    os.system(f"dfu-suffix -v 1209 -p 5af0 -a {dfu_file}")

def argdict(args):
    r = soc_sdram_argdict(args)
    for a in ["revision"]:
        arg = getattr(args, a, None)
        if arg is not None:
            r[a] = arg
    return r

if __name__ == "__main__":
    main()
