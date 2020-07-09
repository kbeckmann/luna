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

from nmigen import Elaboratable, ClockDomain, Module, ClockSignal, Instance, Signal, ResetSignal
from nmigen.build import Resource, Subsignal, Pins, Attrs, Clock, Connector

from nmigen.vendor.lattice_ecp5 import LatticeECP5Platform

from .core import LUNAPlatform

from sys import float_info
from math import fabs

################ copied from pergola

class ECP5PLLConfig():
    def __init__(self, cd_name, freq, phase=0, error=0):
        """
        Parameters:
            cd_name: Name of the clock domain
            freq:    Requested frequency
            phase:   Requested phase (not available on CLKOP)
            error:   Acceptable frequency error
        """
        self.cd_name = cd_name
        self.freq = freq
        self.phase = phase
        self.error = error

    def __repr__(self):
        return "(ECP5PLLConfig {} {} {} {})".format(self.cd_name, self.freq, self.phase, self.error)


class ECP5PLL(Elaboratable):
    INPUT_MIN = 8.0
    INPUT_MAX = 400.0
    OUTPUT_MIN = 10.0
    OUTPUT_MAX = 400.0
    PFD_MIN = 3.125
    PFD_MAX = 400.0
    VCO_MIN = 400.0
    VCO_MAX = 800.0

    def __init__(self, clock_config=None, clock_signal_name=None, clock_signal_freq=None, skip_checks=False):
        """
        Parameters:
            clock_config:      Array of ECP5PLLConfig objects. Must have 1 to 4 elements.
            clock_signal_name: Input clock signal name. Uses default clock if not specified.
            skip_checks:       Skips limit checks and allows out-of-spec usage
        """
        self.clock_name = clock_signal_name
        self.clock_signal_freq = clock_signal_freq

        assert(1 <= len(clock_config) <= 4)
        assert(clock_config[0].phase == 0)

        self.clock_config = clock_config.copy()
        self.skip_checks = skip_checks

    def calc_pll_params(self, input, output):
        if (not self.INPUT_MIN <= input <= self.INPUT_MAX):
            logger.warning("Input clock violates frequency range: {} <= {:.3f} <= {}".format(
                self.INPUT_MIN, freq, self.INPUT_MAX))

        params = {}
        error = float_info.max

        for input_div in range(1, 129):
            fpfd = input / input_div
            if fpfd < self.PFD_MIN or fpfd > self.PFD_MAX:
                continue

            for feedback_div in range(1, 81):
                for output_div in range(1, 129):
                    fvco = fpfd * feedback_div * output_div

                    if not self.skip_checks and (fvco < self.VCO_MIN or fvco > self.VCO_MAX):
                        continue

                    freq = fvco / output_div
                    if (fabs(freq - output) < error or \
                        (fabs(freq - output) == error and \
                        fabs(fvco - 600) < fabs(params["fvco"] - 600))):

                        error = fabs(freq - output)
                        params["refclk_div"] = input_div
                        params["feedback_div"] = feedback_div
                        params["output_div"] = output_div
                        params["freq"] = freq
                        params["freq_requested"] = output
                        params["fvco"] = fvco
                        # shift the primary by 180 degrees. Lattice seems to do this
                        ns_phase = 1.0 / (freq * 1e6) * 0.5
                        params["primary_cphase"] = ns_phase * (fvco * 1e6)

        if not self.skip_checks:
            assert(self.OUTPUT_MIN <= freq <= self.OUTPUT_MAX)

        params["secondary"] = [{
            "div": 0,
            "freq": 0,
            "freq_requested": 0,
            "phase": 0,
            "cphase": 0,
            "fphase": 0,
            "enabled": False,
            "error": 0,
            }]*3
        params["error"] = error
        return params

    def generate_secondary_output(self, params, channel, output, phase):
        div = round(params["fvco"] / output)
        freq = params["fvco"] / div

        ns_shift = 1.0 / (freq * 1e6) * phase /  360.0
        phase_count = ns_shift * (params["fvco"] * 1e6)
        cphase = round(phase_count)
        fphase = round((phase_count - cphase) * 8)

        ns_actual = 1.0 / (params["fvco"] * 1e6) * (cphase + fphase / 8.0)
        phase_shift = 360 * ns_actual / (1.0 / (freq * 1e6))

        params["secondary"][channel] = {}
        params["secondary"][channel]["enabled"] = True
        params["secondary"][channel]["div"] = div
        params["secondary"][channel]["freq"] = freq
        params["secondary"][channel]["freq_requested"] = output
        params["secondary"][channel]["phase"] = phase_shift
        params["secondary"][channel]["cphase"] = cphase + params["primary_cphase"]
        params["secondary"][channel]["fphase"] = fphase
        params["secondary"][channel]["error"] = fabs(freq - output)

        if (not self.OUTPUT_MIN <= freq <= self.OUTPUT_MAX):
            logger.warning("ClockDomain {} violates frequency range: {} <= {:.3f} <= {}".format(
                self.clock_config[channel + 1].cd_name, self.OUTPUT_MIN, freq, self.OUTPUT_MAX))


    def elaborate(self, platform):
        m = Module()

        # Create clock out signals
        self.clk = {cfg.cd_name: Signal() for cfg in self.clock_config}
        self._pll_lock = Signal()

        # Create our clock domains.
        for cfg in self.clock_config:
            m.domains += ClockDomain(cfg.cd_name)
            m.d.comb += ClockSignal(domain=cfg.cd_name).eq(self.clk[cfg.cd_name])
            m.d.comb += ResetSignal(cfg.cd_name).eq(~self._pll_lock),

        # Grab our input clock
        clock_name = self.clock_name if self.clock_name else platform.default_clk
        
        try:
            self.clkin_frequency = platform.lookup(clock_name).clock.frequency / 1e6
            input_clock_pin = platform.request(clock_name)
        except:
            input_clock_pin = ClockSignal(clock_name)
            # TODO: Make this nicer, remove clock_signal_freq
            self.clkin_frequency = self.clock_signal_freq / 1e6

        # Calculate configuration parameters
        params = self.calc_pll_params(self.clkin_frequency, self.clock_config[0].freq)
        if len(self.clock_config) > 1:
            self.generate_secondary_output(params, 0, self.clock_config[1].freq, self.clock_config[1].phase)
        if len(self.clock_config) > 2:
            self.generate_secondary_output(params, 1, self.clock_config[2].freq, self.clock_config[2].phase)
        if len(self.clock_config) > 3:
            self.generate_secondary_output(params, 2, self.clock_config[3].freq, self.clock_config[3].phase)

        for i, p in enumerate([
                params,
                params["secondary"][0],
                params["secondary"][1],
                params["secondary"][2]
            ]):
            if p["error"] > 0:
                logger.warning("ClockDomain {} has an error of {:.3f} MHz ({} instead of {})"
                    .format(self.clock_config[i].cd_name, p["error"], p["freq"], p["freq_requested"]))
                if not self.skip_checks:
                    assert(p["error"] <= self.clock_config[i].error)

        m.submodules.pll = Instance("EHXPLLL",
            # Clock in.
            i_CLKI=input_clock_pin,

            # Generated clock outputs.
            o_CLKOP=self.clk[self.clock_config[0].cd_name],
            o_CLKOS=self.clk[self.clock_config[1].cd_name] if len(self.clock_config) > 1 else Signal(),
            o_CLKOS2=self.clk[self.clock_config[2].cd_name] if len(self.clock_config) > 2 else Signal(),
            o_CLKOS3=self.clk[self.clock_config[3].cd_name] if len(self.clock_config) > 3 else Signal(),

            # Status.
            o_LOCK=self._pll_lock,

            # PLL parameters...
            p_PLLRST_ENA="DISABLED",
            p_INTFB_WAKE="DISABLED",
            p_STDBY_ENABLE="DISABLED",
            p_DPHASE_SOURCE="DISABLED",

            p_OUTDIVIDER_MUXA="DIVA",
            p_OUTDIVIDER_MUXB="DIVB",
            p_OUTDIVIDER_MUXC="DIVC",
            p_OUTDIVIDER_MUXD="DIVD",

            p_CLKI_DIV=params["refclk_div"],
            p_CLKOP_ENABLE="ENABLED",
            p_CLKOP_DIV=params["output_div"],
            p_CLKOP_CPHASE=params["primary_cphase"],
            p_CLKOP_FPHASE=0,

            p_CLKOS_ENABLE="ENABLED" if params["secondary"][0]["enabled"] else "DISABLED",
            p_CLKOS_FPHASE=params["secondary"][0]["fphase"],
            p_CLKOS_CPHASE=params["secondary"][0]["cphase"],
            p_CLKOS_DIV=params["secondary"][0]["div"],

            p_CLKOS2_ENABLE="ENABLED" if params["secondary"][1]["enabled"] else "DISABLED",
            p_CLKOS2_FPHASE=params["secondary"][1]["fphase"],
            p_CLKOS2_CPHASE=params["secondary"][1]["cphase"],
            p_CLKOS2_DIV=params["secondary"][1]["div"],

            p_CLKOS3_ENABLE="ENABLED" if params["secondary"][2]["enabled"] else "DISABLED",
            p_CLKOS3_FPHASE=params["secondary"][2]["fphase"],
            p_CLKOS3_CPHASE=params["secondary"][2]["cphase"],
            p_CLKOS3_DIV=params["secondary"][2]["div"],

            p_FEEDBK_PATH="CLKOP", # TODO: external feedback
            p_CLKFB_DIV=params["feedback_div"],

            # Internal feedback.
            i_CLKFB=self.clk[self.clock_config[0].cd_name],

            # TODO: Reset
            i_RST=0,

            # TODO: Standby
            i_STDBY=0,

            # TODO: Dynamic mode
            i_PHASESEL0=0,
            i_PHASESEL1=0,
            i_PHASEDIR=0,
            i_PHASESTEP=0,
            i_PHASELOADREG=0,

            i_PLLWAKESYNC=0,

            # Output Enables.
            i_ENCLKOP=0,
            i_ENCLKOS=0,
            i_ENCLKOS2=0,
            i_ENCLKOS3=0,

            # Synthesis attributes.
            a_FREQUENCY_PIN_CLKI=str(self.clkin_frequency),
            a_FREQUENCY_PIN_CLKOP=str(self.clock_config[0].freq),
            a_FREQUENCY_PIN_CLKOS=str(self.clock_config[1].freq) if len(self.clock_config) > 1 else "0",
            a_FREQUENCY_PIN_CLKOS2=str(self.clock_config[2].freq) if len(self.clock_config) > 2 else "0",
            a_FREQUENCY_PIN_CLKOS3=str(self.clock_config[3].freq) if len(self.clock_config) > 3 else "0",
            a_ICP_CURRENT="12",
            a_LPF_RESISTOR="8",
            a_MFG_ENABLE_FILTEROPAMP="1",
            a_MFG_GMCREF_SEL="2",
        )
        return m

###############################

class PergolaDomainGenerator(Elaboratable):
    def __init__(self, *, clock_frequencies=None, clock_signal_name=None):
        pass

    def elaborate(self, platform):
        m = Module()

        m.submodules.pll = ECP5PLL([
            ECP5PLLConfig("sync", 48),
            ECP5PLLConfig("usb", 12),
        ])

        # We'll use our 48MHz clock for everything _except_ the usb domain...
        m.domains.usb_io = ClockDomain()
        m.domains.fast   = ClockDomain()
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
