#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# Copyright (c) 2020 Konrad Beckmann <konrad.beckmann@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

""" Pergola platform definitions.

This is a non-core platform. To use it, you'll need to set your LUNA_PLATFORM variable:

    > export LUNA_PLATFORM="luna.gateware.platform.pergola:PergolaPlatform"
"""

import os
import subprocess

from nmigen import Elaboratable, ClockDomain, Module, ClockSignal, Instance, Signal
from nmigen.build import Resource, Subsignal, Pins, Attrs, Clock, Connector

from nmigen.vendor.lattice_ecp5 import LatticeECP5Platform

from .core import LUNAPlatform


class PergolaDomainGenerator(Elaboratable):
    def __init__(self, *, clock_frequencies=None, clock_signal_name=None):
        pass

    def elaborate(self, platform):
        m = Module()

        # Grab our default input clock.
        input_clock = platform.request(platform.default_clk, dir="i")

        # Create our domains; but don't do anything else for them, for now.
        m.domains.sync   = ClockDomain()
        m.domains.usb    = ClockDomain()
        m.domains.usb_io = ClockDomain()
        m.domains.fast   = ClockDomain()

        # ecppll -i 16 -o 48 --clkout1 12
        m.submodules.pll = Instance("EHXPLLL",

                # Clock in.
                i_CLKI=input_clock,

                # Generated clock outputs.
                o_CLKOP=ClockSignal("sync"), # 48 MHz
                o_CLKOS=ClockSignal("usb"),  # 12 MHz

                # Status.
                #o_LOCK=self._pll_lock,

                # PLL parameters...
                p_PLLRST_ENA="DISABLED",
                p_INTFB_WAKE="DISABLED",
                p_STDBY_ENABLE="DISABLED",
                p_DPHASE_SOURCE="DISABLED",
                p_CLKOS3_FPHASE=0,
                p_CLKOS3_CPHASE=0,
                p_CLKOS2_FPHASE=0,
                p_CLKOS2_CPHASE=7,
                p_CLKOS_FPHASE=0,
                p_CLKOS_CPHASE=5,
                p_CLKOP_FPHASE=0,
                p_CLKOP_CPHASE=5,
                p_PLL_LOCK_MODE=0,
                p_CLKOS_TRIM_DELAY="0",
                p_CLKOS_TRIM_POL="FALLING",
                p_CLKOP_TRIM_DELAY="0",
                p_CLKOP_TRIM_POL="FALLING",
                p_OUTDIVIDER_MUXD="DIVD",
                p_CLKOS3_ENABLE="DISABLED",
                p_OUTDIVIDER_MUXC="DIVC",
                p_CLKOS2_ENABLE="DISABLED",
                p_OUTDIVIDER_MUXB="DIVB",
                p_CLKOS_ENABLE="ENABLED",
                p_OUTDIVIDER_MUXA="DIVA",
                p_CLKOP_ENABLE="ENABLED",
                p_CLKOS3_DIV=1,
                p_CLKOS2_DIV=8,
                p_CLKOS_DIV=48,
                p_CLKOP_DIV=12,
                p_CLKFB_DIV=3,
                p_CLKI_DIV=1,
                p_FEEDBK_PATH="CLKOP",


                # Internal feedback.
                i_CLKFB=ClockSignal("sync"),

                # Control signals.
                i_RST=0,
                i_PHASESEL0=0,
                i_PHASESEL1=0,
                i_PHASEDIR=1,
                i_PHASESTEP=1,
                i_PHASELOADREG=1,
                i_STDBY=0,
                i_PLLWAKESYNC=0,

                # Output Enables.
                i_ENCLKOP=0,
                i_ENCLKOS=0,
                i_ENCLKOS2=0,
                i_ENCLKOS3=0,

                # Synthesis attributes.
                a_FREQUENCY_PIN_CLKI="16.000000",
                a_FREQUENCY_PIN_CLKOP="48.000000",
                a_FREQUENCY_PIN_CLKOS="12.000000",
                a_ICP_CURRENT="12",
                a_LPF_RESISTOR="8"
        )

        # We'll use our 48MHz clock for everything _except_ the usb_io domain...
        m.d.comb += [
            ClockSignal("usb_io")  .eq(ClockSignal("sync")),
            ClockSignal("fast")    .eq(ClockSignal("sync"))
        ]

        return m


class PergolaPlatform(LatticeECP5Platform, LUNAPlatform):
    name        = "Pergola RevA 0.1"

    device      = "LFE5U-12F"
    package     = "BG256"
    speed       = 8

    clock_domain_generator = PergolaDomainGenerator

    default_usb_connection = "usb"

    default_clk = "clk16"
    resources   = [
        Resource("clk16", 0, Pins("P11", dir="i"),
                 Clock(16e6), Attrs(GLOBAL=True, IO_TYPE="LVCMOS33")),

        Resource("led", 0, Pins("F15", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 1, Pins("E16", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 2, Pins("E15", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 3, Pins("D16", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 4, Pins("C16", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 5, Pins("C15", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 6, Pins("B16", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 7, Pins("B15", dir="o"), Attrs(IO_TYPE="LVCMOS33")),

        Resource("user_button", 0, Pins("F14"), Attrs(IO_TYPE="LVCMOS33")),

        # UARTResource(0,
        #     rx="T2", tx="R1",
        #     attrs=Attrs(IO_TYPE="LVCMOS33", PULLUP=1)
        # ),

        # # PSRAM #1
        # *SPIFlashResources(0,
        #     cs="A9", clk="B9", mosi="B10", miso="A10", wp="A11", hold="B8",
        #     attrs=Attrs(IO_TYPE="LVCMOS33")
        # ),

        # # PSRAM #2
        # *SPIFlashResources(1,
        #     cs="A2", clk="A4", mosi="A5", miso="B3", wp="B4", hold="A3",
        #     attrs=Attrs(IO_TYPE="LVCMOS33")
        # ),

        # PMOD1, USB-C pmod
        Resource("usb", 0,
            Subsignal("d_p",     Pins("P2", dir="io")),
            Subsignal("d_n",     Pins("L1", dir="io")),
            Subsignal("dp_pull", Pins("J2", dir="io")),
            Subsignal("dn_pull", Pins("H2", dir="io")),
            Attrs(IO_STANDARD="LVCMOS33")
        ),

        Resource("pmod", 0, Pins("P2  L1  J2  H2       N1  L2  J1  G1 ")), # PMOD1
        Resource("pmod", 1, Pins("G2  E2  C1  B1       F1  D1  C2  B2 ")), # PMOD2
        Resource("pmod", 2, Pins("D4  C6  B7  C7       C4  B6  A7  A8 ")), # PMOD3
        Resource("pmod", 3, Pins("B11 B12 B13 B14      A12 A13 A14 A15")), # PMOD4
        Resource("pmod", 4, Pins("F16 G16 J16 L16      G15 H15 J15 L15")), # PMOD5
    ]

    connectors = [
        Connector("pmod", 0, "P2  L1  J2  H2       N1  L2  J1  G1 "), # PMOD1
        Connector("pmod", 1, "G2  E2  C1  B1       F1  D1  C2  B2 "), # PMOD2
        Connector("pmod", 2, "D4  C6  B7  C7       C4  B6  A7  A8 "), # PMOD3
        Connector("pmod", 3, "B11 B12 B13 B14      A12 A13 A14 A15"), # PMOD4
        Connector("pmod", 4, "F16 G16 J16 L16      G15 H15 J15 L15"), # PMOD5
    ]

    @property
    def file_templates(self):
        idcodes = {
            "LFE5U-12F": "0x21111043",
            "LFE5U-25F": "0x41111043",
            "LFE5U-45F": "0x41112043",
            "LFE5U-85F": "0x41113043",
        }
        return {
            **super().file_templates,
            "{{name}}-openocd.cfg": """
            jtag newtap ecp5 tap -irlen 8 -expected-id {} ;
            """.format(idcodes[self.device])
        }

    def toolchain_program(self, products, name):
        openocd = os.environ.get("OPENOCD", "openocd")
        interface = os.environ.get("INTERFACE", "/dev/ttyACM0")
        if interface == "SiPEED" or interface == "busblaster":
            if interface == "SiPEED":
                args = ["-c", """
                        interface ftdi
                        ftdi_vid_pid 0x0403 0x6010
                        ftdi_layout_init 0x0018 0x05fb
                        ftdi_layout_signal nSRST -data 0x0010
                    """]
            elif interface == "busblaster":
                args = ["-f", "interface/ftdi/dp_busblaster.cfg"]

            with products.extract("{}-openocd.cfg".format(name), "{}.svf".format(name)) \
                    as (config_filename, vector_filename):
                subprocess.check_call([openocd,
                    *args,
                    "-f", config_filename,
                    "-c", "transport select jtag; adapter_khz 10000; init; svf -quiet {}; exit".format(vector_filename)
                ])
        elif interface == "pergola_bringup":
            # Early bringup code
            with products.extract("{}.bit".format(name)) as (bitstream):
                print(subprocess.check_call(["bash", "-c", """
                stty -F {} 300 raw -clocal -echo icrnl;
                sleep 0.01;
                cat {} & > /dev/null;
                CATPID=$! ;
                echo -n "$(stat -c%s {})\n" > {};
                cp {} {};
                sync;
                sleep 1;
                """.format(interface, interface, bitstream, interface, bitstream, interface)]))
                #   kill $CATPID || true;
        else:
            with products.extract("{}.bit".format(name)) as (bitstream):
                print(subprocess.check_call([
                    "hf2", "-v", "0x239a", "-p", "0x0058", "flash",
                    "--file", bitstream, "--address", "0x70000000", "-s"
                ]))


    def build(self, *args, **kwargs):
        LatticeECP5Platform.build(self, *args, **kwargs)
