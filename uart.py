#!/usr/bin/env python3
from ctypes import ArgumentError
from sys import flags
from nmigen import *
from nmigen.build import *
from nmigen_boards.icebreaker import *
from nmigen_boards.resources.memory import SPIFlashResources


def _divisor(freq_in, freq_out, max_ppm=None):
    divisor = freq_in // freq_out
    if divisor <= 0:
        raise ArgumentError("Output frequency is too high.")

    ppm = 100000 * ((freq_in / divisor) - freq_out) / freq_out
    if max_ppm is not None and ppm > max_ppm:
        raise ArgumentError("Output frequency deviation is too high.")

    return divisor


class UART(Elaboratable):
    def __init__(self, serial, clk_freq, baud_rate):
        self.tx_data = Signal(8, reset=ord('x'))
        self.tx_ready = Signal()
        self.tx_ack = Signal()
        self.tx_strobe = Signal()
        self.tx_bitno = None
        self.tx_latch = None
        self.tx_fsm = None

        self.serial = serial

        self.divisor = _divisor(
            freq_in=clk_freq, freq_out=baud_rate, max_ppm=50000)

    def elaborate(self, _platform: Platform) -> Module:
        m = Module()

        # TX

        tx_counter = Signal(range(self.divisor))
        m.d.comb += self.tx_strobe.eq(tx_counter == 0)
        with m.If(tx_counter == 0):
            m.d.sync += tx_counter.eq(self.divisor - 1)
        with m.Else():
            m.d.sync += tx_counter.eq(tx_counter - 1)

        self.tx_bitno = tx_bitno = Signal(3)
        self.tx_latch = tx_latch = Signal(8)
        with m.FSM(reset="IDLE") as self.tx_fsm:
            with m.State("IDLE"):
                m.d.comb += self.tx_ack.eq(1)
                with m.If(self.tx_ready):
                    m.d.sync += [
                        tx_counter.eq(self.divisor - 1),
                        tx_latch.eq(self.tx_data)
                    ]
                    m.next = "START"
                with m.Else():
                    m.d.sync += self.serial.tx.eq(1)

            with m.State("START"):
                with m.If(self.tx_strobe):
                    m.d.sync += self.serial.tx.eq(0)
                    m.next = "DATA"

            with m.State("DATA"):
                with m.If(self.tx_strobe):
                    m.d.sync += [
                        self.serial.tx.eq(tx_latch[0]),
                        tx_latch.eq(Cat(tx_latch[1:8], 0)),
                        tx_bitno.eq(tx_bitno + 1)
                    ]
                    with m.If(self.tx_bitno == 7):
                        m.next = "STOP"

            with m.State("STOP"):
                with m.If(self.tx_strobe):
                    m.d.sync += self.serial.tx.eq(1)
                    m.next = "IDLE"

        return m

class _LoopbackTest(Elaboratable):
    def __init__(self):
        self.empty = Signal(reset=1)
        self.data = Signal(8)
        self.rx_strobe = Signal()
        self.tx_strobe = Signal()
        self.uart = None

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        serial = platform.request("uart")
        leds = Cat([platform.request("led_r"), platform.request("led_g")])
        debug = platform.request("debug")

        self.uart = UART(serial, clk_freq=12000000, baud_rate=115200)
        m.submodules.uart = self.uart

        arr = Array([Const(ord(x)) for x in "siemaa!\n"])
        from math import log2
        bits = int(log2(len(arr)))
        assert 2 **bits == len(arr), "ERROR: arr len is not power of 2"
        idx = Signal(bits)
        
        with m.FSM():
            with m.State("WAIT"):
                a = Signal(20, reset=1)
                m.d.sync += a.eq(a + 1)
                with m.If(a == 0):
                    m.next = "RUN"
            with m.State("RUN"):
                with m.If(self.uart.tx_ack):
                    m.d.sync += [
                        idx.eq(idx + 1),
                    ]
                    m.d.comb += [
                        self.uart.tx_data.eq(arr[idx]),
                        self.uart.tx_ready.eq(1),
                    ]
                    m.next = "WAIT"

        # m.d.comb += [
        #     self.tx_strobe.eq(1),
        #     self.uart.tx_ready.eq(1)
        # ]

        m.d.comb += [
            debug.eq(Cat(
                Const(0, 1),
                serial.tx,
                Const(0, 1)
            ))
        ]

        return m


class Test(Elaboratable):
    def __init__(self) -> None:
        super().__init__()
    
    def elaborate(self, platform : Platform):
        m = Module()
        sync = m.d.sync
        comb = m.d.comb

        # SPIFlashResources

        flash = platform.request("spi_flash_1x")
        led = platform.request("led_r")
        led_gie = platform.request("led_g")
        # a = Signal(30)
        # sync += a.eq(a + 1)
        # with m.If(a[23] == 1):
        #     sync += led.eq(1)
        # with m.Else():
        #     sync += led.eq(0)

        

        FC_RD = 0x03 #  Read Data
        WE = 0x06
        ENTER_QPI = 0x38
        EXIT_QPI = 0xFF

        a = Signal()

        # neg_sync = ClockDomain("neg_sync", clk_edge="neg")
        # m.d.comb += ClockSignal("neg_sync").eq(clk_i)

        # neg_sync.clk = ClockSignal(domain="sync")
        # neg_sync.rst = ResetSignal("sync")
        # m.domains += neg_sync # .neg_sync = neg_sync

        a = Array(
            [
            Const(FC_RD, 8),
            Const(0, 8),
            Const(0, 8),
            # Const(0, 8), # XXX
            Const(4, 8),
            ]
        )

        which = Signal(2)
        bit = Signal(3)
        res = Signal(8)
        temp = Signal(8)

        # raise ValueError(dir(ClockDomain("sync")))

        m.domains += ClockDomain("neg_sync", clk_edge="neg")
        m.d.comb += ClockSignal("neg_sync").eq(ClockDomain("sync").clk)
        
        with m.FSM():
            with m.State("WARMUP"):
                ctr = Signal(20, reset=1)
                sync += ctr.eq(ctr + 1)
                with m.If(ctr == 0):
                    m.next = "START"
            with m.State("START"):
                sync += flash.cs.eq(0) # TODO czy jest pull-up?
                m.next = "INIT"
            with m.State("INIT"):
                sync += flash.clk.eq(ClockSignal())
                sync += temp.eq(a[0])
                m.next = "JOB"
            with m.State("JOB"):
                comb += flash.mosi.eq(temp[7])
                sync += bit.eq(bit + 1)
                with m.If(bit == 7):
                    sync += bit.eq(0)
                    sync += which.eq(which + 1)
                    with m.If(which == len(a) - 1):
                        m.next = "SENT"
                with m.Else():
                    sync += temp.eq(temp << 1)
            with m.State("SENT"):
                # with m.If(flash.miso):
                #     m.next = "DONE"
                sync += bit.eq(bit + 1)
                with m.If(bit == 7):
                    m.next = "DONE"
                # with m.Else():
                    # m.d.neg_sync += res.eq((res << 1) | flash.miso)
                m.d.neg_sync += res.eq((res << 1) | flash.miso)
            with m.State("DONE"):
                with m.If(res ==0): # czy addr=0x4 == 0x7e?
                    sync += led.eq(1)
                with m.Else():
                    sync += led_gie.eq(1)
                    
        return m


from nmigen import *
from nmigen.lib.cdc import ResetSynchronizer

class PLL(Elaboratable):

    """
    Instantiate the iCE40's phase-locked loop (PLL).
    This uses the iCE40's SB_PLL40_PAD primitive in simple feedback
    mode.
    The reference clock is directly connected to a package pin. To
    allocate that pin, request the pin with dir='-'; otherwise nMigen
    inserts an SB_IO on the pin.  E.g.,
        clk_pin = platform.request('clk12', dir='-')
    Because the PLL eats the external clock, that clock is not available
    for other uses.  So you might as well have the PLL generate the
    default 'sync' clock domain.
    This module also has a reset synchronizer -- the domain's reset line
    is not released until a few clocks after the PLL lock signal is
    good.
    """

    def __init__(self, freq_in_mhz, freq_out_mhz, domain_name='sync'):
        self.freq_in = freq_in_mhz
        self.freq_out = freq_out_mhz
        self.coeff = self._calc_freq_coefficients()
        self.clk_pin = Signal()
        self.domain_name = domain_name
        self.domain = ClockDomain(domain_name)
        self.ports = [
            self.clk_pin,
            self.domain.clk,
            self.domain.rst,
        ]

    def _calc_freq_coefficients(self):
        # cribbed from Icestorm's icepll.
        f_in, f_req = self.freq_in, self.freq_out
        assert 10 <= f_in <= 13
        assert 16 <= f_req <= 275
        from collections import namedtuple
        import warnings
        coefficients = namedtuple('coefficients', 'divr divf divq')
        divf_range = 128        # see comments in icepll.cc
        best_fout = float('inf')
        for divr in range(16):
            pfd = f_in / (divr + 1)
            if 10 <= pfd <= 133:
                for divf in range(divf_range):
                    vco = pfd * (divf + 1)
                    if 533 <= vco <= 1066:
                        for divq in range(1, 7):
                            fout = vco * 2**-divq
                            if abs(fout - f_req) < abs(best_fout - f_req):
                                best_fout = fout
                                best = coefficients(divr, divf, divq)
        if best_fout != f_req:
            warnings.warn(
                f'PLL: requested {f_req} MHz, got {best_fout} MHz)',
                stacklevel=3)
        return best

    def elaborate(self, platform):
        m = Module()

        # coeff = self._calc_freq_coefficients()

        pll_lock = Signal()
        clock = Signal() # ClockSignal(self.domain_name)
        
        # m.d.comb += platform.request(platform.default_clk).i.eq(clock)
        
        
        # m.d.comb += ClockSignal(self.domain_name).eq(clock)
        # m.d.comb += ResetSignal(self.domain_name).eq(~pll_lock)
        # pll = Instance("SB_PLL40_PAD",
        #     p_FEEDBACK_PATH='SIMPLE',
        #     p_DIVR=self.coeff.divr,
        #     p_DIVF=self.coeff.divf,
        #     p_DIVQ=self.coeff.divq,
        #     p_FILTER_RANGE=0b001,

        #     i_PACKAGEPIN=self.clk_pin,
        #     i_RESETB=Const(1),
        #     i_BYPASS=Const(0),

        #     o_PLLOUTGLOBAL=clock,
        #     o_LOCK=pll_lock
        # )
        # rs = ResetSynchronizer(~pll_lock, domain=self.domain_name)

        
        # m.submodules.pll = pll
        # m.submodules.rs = rs
        # m.submodules += [pll, rs]
        m.submodules.test = Test()
        return m

if __name__ == "__main__":
    plat = ICEBreakerPlatform()

    # raise ValueError(plat.hfosc_div)
    plat.hfosc_div = 1.2

    # The debug pins are on the PMOD1A in the following order on the connector:
    # 7 8 9 10 1 2 3 4
    # Yes that means that the pins at the edge of the board come first
    # and the pins further away from the edge second
    plat.add_resources([
        Resource("debug", 0, Pins("7 8 9 10 1 2 3 4", dir="o",
                                    conn=("pmod", 0)), Attrs(IO_STANDARD="SB_LVCMOS"))
    ])

    plat.build(Test(), do_program=True)
