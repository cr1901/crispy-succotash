import functools
import argparse
from typing import Any, Optional

from amaranth import *
from amaranth.lib.cdc import FFSynchronizer
from amaranth.sim import Simulator
from amaranth_boards.mercury import *

# TODO: Button/logging test via VGA.
# One button starts/stops logging
# One dumps data over PMOD.


class BaseboardDemo(Elaboratable):
    def __init__(self):
        self.timer = Timer(1.0 / 200)
        self.degrees = Adc2Degrees()

    def elaborate(self, plat):
        m = Module()
        cd_sync = ClockDomain(reset_less=True)
        m.domains += cd_sync

        adc = plat.request("spi_adc")
        ssd = plat.request("display_7seg")
        ssd_ctl = plat.request("display_7seg_ctrl")
        # pmod = plat.request("pmod_gpio")
        ch_sel = Signal(3)
        binary_in = Signal(10)

        m.submodules.spiadc = spiadc = SPICtrl(24, adc)
        m.submodules.timer = self.timer
        m.submodules.bin2bcd = bin2bcd = Binary2Bcd()
        m.submodules.sevenseg = sevenseg = SevenSegDriver(ssd, ssd_ctl)
        m.submodules.degrees = self.degrees

        # ADC order is MSB-first (bit 0 == MSB)
        # Bit 7: Start bit
        spi_word = Cat(
            C(0, 8),
            C(0, 4),
            ch_sel,  # Channel select
            C(1, 1),  # Single-ended
            C(1, 1),  # Start
            C(0, 7)  # Padding
        )

        m.d.comb += [ClockSignal("sync").eq(plat.request("clk50")),]

        m.d.comb += [
            ch_sel.eq(Cat([plat.request("switch", i) for i in range(3)])),
            spiadc.din.eq(spi_word),
            sevenseg.din.eq(bin2bcd.dout),
            self.degrees.adc.eq(binary_in),
        ]

        with m.Switch(ch_sel):
            with m.Case(2):
                m.d.comb += bin2bcd.din.eq(self.degrees.degrees)
            with m.Default():
                m.d.comb += bin2bcd.din.eq(binary_in)

        # m.d.comb += [
        #     pmod[0].eq(adc.clk),
        #     pmod[1].eq(adc.cipo),
        #     pmod[2].eq(adc.copi),
        #     pmod[3].eq(adc.cs_n)
        # ]

        m.d.sync += [
            spiadc.en.eq(0),
            bin2bcd.en.eq(0),
        ]

        with m.If(spiadc.done & self.timer.stb):
            m.d.sync += [
                spiadc.en.eq(1),
                bin2bcd.en.eq(1),
                # leds.eq(self.spiadc.
                binary_in.eq(spiadc.dout)
            ]

        return m


class Debounce(Elaboratable):
    def __init__(self, button):
        self.out = Signal(1)
        self.up_stb = Signal(1)
        self.down_stb = Signal(1)
        self.button = button

    def elaborate(self, plat):
        m = Module()

        if not self.button:
            self.button = plat.request("button")

        button_sync = Signal()
        last_state = Signal(1)
        cnt = Signal(16)

        m.d.comb += [self.out.eq(last_state)]
        m.d.comb += [self.down_stb.eq((cnt == 65535) & last_state == 1)]
        m.d.comb += [self.up_stb.eq((cnt == 65535) & last_state == 0)]

        m.d.sync += cnt.eq(0)

        with m.If(button_sync != last_state):
            m.d.sync += cnt.eq(cnt + 1)

            with m.If(cnt == 65535):
                m.d.sync += last_state.eq(~last_state)

        self.specials += FFSynchronizer(self.button, button_sync)

        return m


class Timer(Elaboratable):
    def __init__(self, period: int, clk_freq: Optional[int] = None):
        self.period = period
        self.clk_freq = clk_freq

        self.stb = Signal(1)

    def elaborate(self, platform):
        m = Module()

        if not self.clk_freq:
            self.clk_freq = platform.default_clk_frequency

        cnt = Signal(range(0, int(self.period * self.clk_freq)),
                     reset=int(self.period * self.clk_freq))

        m.d.sync += [
            self.stb.eq(0),
            cnt.eq(cnt - 1),
        ]

        with m.If(cnt == 0):
            m.d.sync += [
                cnt.eq(int(self.period * self.clk_freq)),
                self.stb.eq(1)
            ]

        return m


class Adc2Degrees(Elaboratable):
    def __init__(self):
        self.adc = Signal(10)
        self.degrees = Signal(10)

    def elaborate(self, plat):
        m = Module()

        # c = (z - 82)/4 is a good approx to Celsius
        m.d.comb += self.degrees.eq((self.adc - 82) >> 2)

        return m


class Binary2Bcd(Elaboratable):
    def __init__(self):
        self.en = Signal(1)
        self.din = Signal(10)
        self.dout = Signal(16)

    def elaborate(self, plat):
        m = Module()
        done = Signal(1, reset=1)
        add3 = Signal(1)
        add_done = Signal(1)

        tmp = Signal.like(self.din)
        tmp_out = Signal.like(self.dout)
        cnt = Signal(range(0, 10), reset=10)

        add3_logic = functools.reduce(lambda x, y: x | y,
                                      [tmp_out[i:i + 4] >= 5
                                       for i in range(0, 16, 4)])

        m.d.comb += add3.eq(add3_logic)

        with m.If(self.en):
            # It's really not a big deal if this restarts if self.en is
            # held asserted.
            m.d.sync += [
                done.eq(0),
                tmp.eq(self.din),
                tmp_out.eq(0),
                cnt.eq(10),
                add_done.eq(0)
            ]

        with m.If(~done):
            # After we add 3 we need to shift
            with m.If(~add3 | add_done):
                m.d.sync += [
                    tmp.eq(Cat(0, tmp[0:-1])),
                    tmp_out.eq(Cat(tmp[-1], tmp_out[0:-1])),
                    add_done.eq(0),
                    cnt.eq(cnt - 1),
                ]

                with m.If((cnt - 1) == 0):
                    m.d.sync += done.eq(1)

            with m.Else():
                m.d.sync += add_done.eq(1)
                for i in range(0, 16, 4):
                    with m.If(tmp_out[i:i + 4] >= 5):
                        m.d.sync += tmp_out[i:i + 4].eq(tmp_out[i:i + 4] + 3)

        with m.Else():
            m.d.sync += self.dout.eq(tmp_out)

        return m

    def do_tb(self):
        def my_tb():
            yield self.en.eq(1)
            yield self.din.eq(0x3FF)
            yield
            yield self.en.eq(0)
            for i in range(50):
                yield

        sim = Simulator(self)
        sim.add_clock(20e-9)
        sim.add_sync_process(my_tb)

        with sim.write_vcd("bcd.vcd", "bcd.gtkw"):
            sim.run()


class SevenSegDriver(Elaboratable):
    def __init__(self, pads, pads_ctl, clk_freq=50000000):
        self.din = Signal(16)
        self.pads = pads
        self.pads_ctl = pads_ctl
        self.clk_freq = clk_freq

        self.timer = Timer(1.0 / 240, clk_freq)

    def elaborate(self, plat):
        m = Module()

        # FIXME: Check for something SPI-pads-shaped to verify correct
        # input type.
        if isinstance(self.pads, str):
            self.pads = plat.request(self.pads)
        if isinstance(self.pads, str):
            self.pads = plat.request(self.pads_ctl)

        dout = Signal(8)
        curr_digit = Signal(4)
        digit_sel = Signal(2)
        enable = Signal(4, reset=15)

        m.submodules.timer = self.timer

        m.d.comb += self.pads_ctl.en.eq(enable)

        # Switch Logic
        for i in range(0, 16, 4):
            with m.If(digit_sel == i // 4):
                m.d.comb += [
                    curr_digit.eq(self.din[i:i + 4]),
                    enable[i // 4].eq(0)
                ]

        for i, p in enumerate(reversed(("a", "b", "c", "d",
                                       "e", "f", "g", "dp"))):
            m.d.comb += getattr(self.pads, p).eq(dout[i])

        with m.Switch(curr_digit):
            for i, pat in enumerate((0b11111100, 0b01100000, 0b11011010, 0b11110010,  # noqa: E501
                                    0b01100110, 0b10110110, 0b10111110, 0b11100000,  # noqa: E501
                                    0b11111110, 0b11110110, 0b11101110, 0b00111110,  # noqa: E501
                                    0b10011100, 0b01111010, 0b10011110, 0b10001110)):  # noqa: E501
                with m.Case(i):
                    m.d.comb += dout.eq(pat)

        with m.If(self.timer.stb):
            m.d.sync += digit_sel.eq(digit_sel + 1)

        return m


class SPICtrl(Elaboratable):
    def __init__(self, width: int, pads: Any):
        self.din = Signal(width)
        self.en = Signal(1)
        self.done = Signal(1, reset=1)
        self.dout = Signal(width)
        self.width = width
        self.pads = pads

    def elaborate(self, plat):
        m = Module()

        # FIXME: Check for something SPI-pads-shaped to verify correct
        # input type.
        if isinstance(self.pads, str):
            self.pads = plat.request(self.pads)

        edge_cnt = Signal(range(0, self.width * 2), reset=self.width * 2)
        in_prog = Signal(1)
        tmp = Signal(self.width)
        div = Signal(6, reset=63)
        in_bit = Signal(1)

        sclk_pedge = Signal(1)
        sclk_nedge = Signal(1)
        sclk_prev = Signal(1)

        m.d.comb += [
            sclk_pedge.eq(self.pads.clk & ~sclk_prev),
            sclk_nedge.eq(~self.pads.clk & sclk_prev),
            self.dout.eq(tmp)
        ]

        m.d.sync += [
            sclk_prev.eq(self.pads.clk)
        ]

        m.d.comb += [
            self.done.eq(~in_prog),
            self.pads.copi.eq(tmp[-1]),
        ]

        with m.FSM():
            with m.State("IDLE"):
                m.d.sync += [
                    self.pads.clk.eq(0),
                    self.pads.cs.eq(0),
                    div.eq(63),
                    edge_cnt.eq(self.width * 2)
                ]

                with m.If(self.en):
                    m.d.sync += [
                        tmp.eq(self.din),
                        self.pads.cs.eq(1)
                    ]
                    m.next = "XFER"

            with m.State("XFER"):
                m.d.sync += div.eq(div - 1)

                with m.If(div == 0):
                    m.d.sync += [
                        edge_cnt.eq(edge_cnt - 1),
                        self.pads.clk.eq(~self.pads.clk)
                    ]

                with m.If(edge_cnt == 0):
                    m.next = "IDLE"

        with m.If(sclk_nedge):
            m.d.sync += tmp.eq(Cat(in_bit, tmp[:-1]))

        with m.If(sclk_pedge):
            m.d.sync += in_bit.eq(self.pads.cipo)

        return m


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mercury Baseboard Demo in Migen")  # noqa: E501
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-p', dest='prog', action='store_true', help="Program Mercury after build.")  # noqa: E501
    group.add_argument('-g', dest='gen', action='store_true', help="Generate verilog only- do not build or program.")  # noqa: E501
    args = parser.parse_args()

    plat = MercuryPlatform()
    plat.add_resources(plat.baseboard_no_sram)
    m = BaseboardDemo()

    if args.gen:
        plan = plat.build(m, do_build=False, do_program=False)
        products = plan.execute_local(run_script=False)
    else:
        plat.build(m, do_program=args.prog)
