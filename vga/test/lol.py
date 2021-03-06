#!/usr/bin/env python3

from nmigen import *


class Mod(Elaboratable):
    def __init__(self, w):

        self.a = Signal()
        self.b = Signal()
        
    def elaborate(self, platform):
        m = Module()
        sync = m.d.sync
        comb = m.d.comb

        sync += self.a.eq(~self.a)
        return m



if __name__ == "__main__":
    
    m = Mod(4)

    ports = [
        m.a,
        m.b,
    ]

    from nmigen.back.pysim import *
    
    sim = Simulator(m)
    sim.add_clock(1e-6)

    def test():
        # initial value
        # assert not (yield m.busy)
        # yield m.en.eq(1)
        # for i in range(26000000):
        for i in range(20):
            yield
        # a = yield m.out
        # print(a)
        # assert 10 == (yield m.out)

    sim.add_sync_process(test)
    with sim.write_vcd('mod.vcd'):
        sim.run()
        print("=== OK, sim done")

    from nmigen.back import rtlil
    a = open("mod.il", "w")
    a.write(rtlil.convert(m, ports=ports))
    print("=== OK, mod.il written")

    from nmigen.back import verilog
    a = open("mod.v", "w")
    a.write(verilog.convert(m, ports=ports))
    print("=== OK, mod.v written")
