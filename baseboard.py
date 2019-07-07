import os
import functools
import subprocess
import argparse

from types import SimpleNamespace

from nmigen.compat import *
from nmigen.compat.genlib.cdc import MultiReg
from nmigen.back import rtlil, verilog

from nmigen_boards.mercury import *

# TODO: Button/logging test via VGA.
# One button starts/stops logging
# One dumps data over PMOD.

class BaseboardDemo(Module):
    def __init__(self, plat):
        self.clock_domains.cd_sync = ClockDomain(reset_less=True)

        adc = plat.request("adc")
        ssd = plat.request("sevenseg")
        # pmod = plat.request("pmod_gpio")
        bits = Signal(4)
        ch_sel = Signal(3)
        binary_in = Signal(10)

        self.submodules.spiadc = SPICtrl(24, adc)
        self.submodules.timer = Timer(1.0/200)
        self.submodules.bin2bcd = Binary2Bcd()
        self.submodules.sevenseg = SevenSegDriver(ssd)
        self.submodules.degrees = Adc2Degrees()

        # ADC order is MSB-first (bit 0 == MSB)
        # Bit 7: Start bit
        spi_word = Cat(
            C(0, 8),
            C(0, 4),
            ch_sel, # Channel select
            C(1, 1), # Single-ended
            C(1, 1), # Start
            C(0, 7) # Padding
        )

        self.comb += [ClockSignal("sync").eq(plat.request("clk50"))]

        self.comb += [
            ch_sel.eq(Cat([plat.request("user_sw", i) for i in range(3)])),
            self.spiadc.din.eq(spi_word),
            self.sevenseg.din.eq(self.bin2bcd.dout),
            self.degrees.adc.eq(binary_in),

            Case(ch_sel, {
                2 : self.bin2bcd.din.eq(self.degrees.degrees),
                "default" : self.bin2bcd.din.eq(binary_in)
            })
        ]

        # self.comb += [
        #     pmod[0].eq(adc.clk),
        #     pmod[1].eq(adc.miso),
        #     pmod[2].eq(adc.mosi),
        #     pmod[3].eq(adc.cs_n)
        # ]

        self.sync += [
            self.spiadc.en.eq(0),
            self.bin2bcd.en.eq(0),
            If(self.spiadc.done & self.timer.stb,
                self.spiadc.en.eq(1),
                self.bin2bcd.en.eq(1),
                # leds.eq(self.spiadc.
                binary_in.eq(self.spiadc.dout)
            )
        ]


class Debounce(Module):
    def __init__(self, button):
        self.out = Signal(1)
        self.up_stb = Signal(1)
        self.down_stb = Signal(1)

        button_sys = Signal()
        last_state = Signal(1)
        cnt = Signal(16)

        self.comb += [self.out.eq(last_state)]
        self.comb += [self.down_stb.eq((cnt == 65535) & last_state == 1)]
        self.comb += [self.up_stb.eq((cnt == 65535) & last_state == 0)]

        self.sync += [
            cnt.eq(0),
            If(button_sys != last_state,
                cnt.eq(cnt + 1),
                If(cnt == 65535,
                    last_state.eq(~last_state),
                )
            )
        ]

        self.specials += Multireg(button, button_sys)





class Timer(Module):
    def __init__(self, per=1, clk_freq=50000000):
        self.stb = Signal(1)
        cnt = Signal(max = int(per*clk_freq), reset = int(per * clk_freq))

        self.sync += [
            self.stb.eq(0),
            cnt.eq(cnt - 1),
            If(cnt == 0,
                cnt.eq(int(per * clk_freq)),
                self.stb.eq(1)
            )
        ]


class Adc2Degrees(Module):
    def __init__(self):
        self.adc = Signal(10)
        self.degrees = Signal(10)

        # c = (z - 82)/4 is a good approx to Celsius
        self.comb += [
            self.degrees.eq(
                (self.adc - 82) >> 2
            )
        ]


class Binary2Bcd(Module):
    def __init__(self):
        self.en = Signal(1)
        self.din = Signal(10)
        self.dout = Signal(16)

        done = Signal(1, reset=1)
        add3 = Signal(1)
        add_done = Signal(1)

        tmp = Signal.like(self.din)
        tmp_out = Signal.like(self.dout)
        cnt = Signal(max = 10, reset = 10)

        ones = Signal(4)
        tens = Signal(4)
        hund = Signal(4)
        thou = Signal(4)

        add3_logic = functools.reduce(lambda x, y: x | y, [tmp_out[i:i+4] >= 5 for i in range(0, 16, 4)])

        add_logic = [
            If(tmp_out[i:i+4] >= 5,
                tmp_out[i:i+4].eq(tmp_out[i:i+4] + 3)
            ) for i in range(0, 16, 4)
        ]

        self.comb += [add3.eq(add3_logic)]

        self.sync += [
            # It's really not a big deal if this restarts if self.en if
            # held asserted.
            If(self.en,
                done.eq(0),
                tmp.eq(self.din),
                tmp_out.eq(0),
                cnt.eq(10),
                add_done.eq(0)
            ),

            If(~done,
                # After we add 3 we need to shift
                If(~add3 | add_done,
                    tmp.eq(Cat(0, tmp[0:-1])),
                    tmp_out.eq(Cat(tmp[-1], tmp_out[0:-1])),
                    add_done.eq(0),
                    cnt.eq(cnt - 1),

                    If((cnt - 1) == 0,
                        done.eq(1)
                    )
                ).Else(
                    add_logic,
                    add_done.eq(1)
                )
            ).Else(
                self.dout.eq(tmp_out)
            )
        ]

    def do_tb(self):
        def my_tb(dut):
            yield dut.en.eq(1)
            yield dut.din.eq(0x3FF)
            yield
            yield dut.en.eq(0)
            for i in range(50):
                yield

        run_simulation(self, my_tb(self), vcd_name="bcd.vcd", clocks={"sys" : 20})


class SevenSegDriver(Module):
    def __init__(self, pads, clk_freq=50000000):
        self.din = Signal(16)

        dout = Signal(8)
        curr_digit = Signal(4)
        digit_sel = Signal(2)
        enable = Signal(4, reset=15)

        ###

        self.submodules.timer = Timer(1.0/240, 50000000)

        switch_logic = [
            If(digit_sel == i//4,
                curr_digit.eq(self.din[i:i + 4]),
                enable[i//4].eq(0)
            ) for i in range(0, 16, 4)
        ]

        self.comb += [
            pads.en0.eq(enable[0]),
            pads.en1.eq(enable[1]),
            pads.en2.eq(enable[2]),
            pads.en3.eq(enable[3]),
            switch_logic
        ]

        self.comb += [
            pads.a.eq(dout[7]),
            pads.b.eq(dout[6]),
            pads.c.eq(dout[5]),
            pads.d.eq(dout[4]),
            pads.e.eq(dout[3]),
            pads.f.eq(dout[2]),
            pads.g.eq(dout[1]),
            pads.dp.eq(dout[0]),
        ]

        self.comb +=  Case(curr_digit, {
                0: dout.eq(0b11111100),
                1: dout.eq(0b01100000),
                2: dout.eq(0b11011010),
                3: dout.eq(0b11110010),
                4: dout.eq(0b01100110),
                5: dout.eq(0b10110110),
                6: dout.eq(0b10111110),
                7: dout.eq(0b11100000),
                8: dout.eq(0b11111110),
                9: dout.eq(0b11110110),
                0xA: dout.eq(0b11101110),
                0xB: dout.eq(0b00111110),
                0xC: dout.eq(0b10011100),
                0xD: dout.eq(0b01111010),
                0xE: dout.eq(0b10011110),
                0xF: dout.eq(0b10001110)
        })

        self.sync += [
            If(self.timer.stb,
                digit_sel.eq(digit_sel + 1)
            )
        ]


class SPICtrl(Module):
    def __init__(self, width, pads):
        self.din = Signal(width)
        self.en = Signal(1)
        self.done = Signal(1, reset = 1)
        self.dout = Signal(width)

        edge_cnt = Signal(max = width * 2, reset = width * 2)
        in_prog = Signal(1)
        tmp = Signal(width)
        div = Signal(6, reset = 63)
        in_bit = Signal(1)

        sclk_pedge = Signal(1)
        sclk_nedge = Signal(1)
        sclk_prev = Signal(1)

        self.comb += [
            sclk_pedge.eq(pads.clk & ~sclk_prev),
            sclk_nedge.eq(~pads.clk & sclk_prev),
            self.dout.eq(tmp)
        ]

        self.sync += [
            sclk_prev.eq(pads.clk)
        ]

        self.comb += [
            self.done.eq(~in_prog),
            pads.mosi.eq(tmp[-1]),
        ]

        self.sync += [
            If(~in_prog,
                pads.clk.eq(0),
                pads.cs.eq(0),
                div.eq(63),
                edge_cnt.eq(width * 2)
            ),

            If(self.en & ~in_prog,
                tmp.eq(self.din),
                in_prog.eq(1),
                pads.cs.eq(1)
            ),

            If(in_prog,
                div.eq(div - 1),
                If(div == 0,
                    edge_cnt.eq(edge_cnt - 1),
                    pads.clk.eq(~pads.clk)
                ),
            ),

            If(edge_cnt == 0,
                in_prog.eq(0),
            )
        ]

        self.sync += [
            If(sclk_nedge,
                tmp.eq(Cat(in_bit, tmp[:-1]))
            ),

            If(sclk_pedge,
                in_bit.eq(pads.miso)
            )
        ]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mercury Baseboard Demo in Migen")
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-p', dest='prog', action='store_true', help="Program Mercury after build.")
    group.add_argument('-g', dest='gen', action='store_true', help="Generate verilog only- do not build or program.")
    args = parser.parse_args()

    plat = MercuryPlatform()
    plat.add_resources(plat.sevenseg)
    plat.add_resources(plat.user_sw)
    m = BaseboardDemo(plat)

    if args.gen:
        plan = plat.build(m, do_build=False, do_program=False)
        products = plan.execute_local(run_script=False)
    else:
        plat.build(m, do_program=args.prog)
