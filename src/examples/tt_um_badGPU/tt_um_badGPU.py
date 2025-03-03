'''
Created on March 1, 2025

@author: Emery Nagy
@copyright: Copyright (C) 2025
'''

import gc
from ttboard.demoboard import DemoBoard
from ttboard.mode import RPMode
import ttboard.util.time as time
import asyncio
from ttboard.pins.gpio_map import GPIOMapTT06
import machine
from collections import deque

class SPICommand():
    WRITE_POLY_A = 0x80
    CLEAR_POLY_A = 0x40
    WRITE_POLY_B = 0x81
    CLEAR_POLY_B = 0x41
    WRITE_POLY_C = 0x82
    CLEAR_POLY_C = 0x42
    WRITE_POLY_D = 0x83
    CLEAR_POLY_D = 0x43
    SET_BG_COLOR = 0x01

class Triangle():
    TRIANGLE_A = 1
    TRIANGLE_B = 2
    TRIANGLE_C = 3
    TRIANGLE_D = 4

triangle_write_map = {
    Triangle.TRIANGLE_A: SPICommand.WRITE_POLY_A,
    Triangle.TRIANGLE_B: SPICommand.WRITE_POLY_B,
    Triangle.TRIANGLE_C: SPICommand.WRITE_POLY_C,
    Triangle.TRIANGLE_D: SPICommand.WRITE_POLY_D
}

class Vertex:
    def __init__(self, x: int, y: int):
        self.x = int(round(x/8))
        self.y = int(round(y/8))

    def __lt__(self, other):
            return self.x < other.x

    def __repr__(self):
        print(f"Vertex: x={self.x}, y={self.y}")

class GPU:
    def __init__(self, tt: DemoBoard):
        self.tt = tt
        self.spi = machine.SoftSPI(baudrate=10000000, polarity=0, bits=8, phase=0, sck=machine.Pin(GPIOMapTT06.UIO3), mosi=machine.Pin(GPIOMapTT06.UIO1), miso=machine.Pin(GPIOMapTT06.UIO2), firstbit=machine.SPI.LSB)
        self.cs = machine.Pin(GPIOMapTT06.UIO0, mode=machine.Pin.OUT, value=1)
        self.queue = deque((), 10)

    async def run(self):

        # Used in the pin interrupts for signalling
        global rising_found
        global falling_found

        while True:

            # SPI comms are allowed
            if rising_found:
                rising_found = False

                # Enable CS for whole transfer phase
                while True:
                    try:
                        # Empty queue or go until transfer phase over
                        cmd = self.queue.pop()
                        self.cs(0)
                        self.spi.write(cmd)
                        self.cs(1)

                        if falling_found:
                            # Command may have not been written properly, put back into queue and get ready for next cycle
                            self.queue.append(cmd)
                            break
                    except:
                        break
                self.cs(1)


            await asyncio.sleep(0.0000001)

    def write(self, cmd):
        self.queue.append(cmd)

    def draw_triangle(
            self,
            triangle: Triangle,
            red_value: int,
            green_value: int,
            blue_value: int,
            vertices: list[Vertex]
        ):
        # Lookup command byte and pack color to 6 bits
        cmd = triangle_write_map[triangle]
        packed_color = (red_value) << 4 | (green_value) << 2 | (blue_value)

        # Re-arrange triangles, should be x0 > x1 > x2
        ordered_vertices = sorted(vertices, reverse=False)

        print(ordered_vertices)

        cmd_str = (
            (cmd) |
            (packed_color << 8) |
            (ordered_vertices[0].x << 14) |
            (ordered_vertices[1].x << 21) |
            (ordered_vertices[2].x << 28) |
            (ordered_vertices[0].y << 35) |
            (ordered_vertices[1].y << 41) |
            (ordered_vertices[2].y << 47)
        )

        cmd_str_serialized = cmd_str.to_bytes(7,'little')

        self.write(cmd_str_serialized)

gc.collect()

rising_found = False
falling_found = False

def pin_isr(pin: machine.Pin):
    global rising_found
    global falling_found
    if pin.value() == 1:
        rising_found = True
        falling_found = False
    else:
        falling_found = True
        rising_found = False


async def commander(gpu: GPU):

    while True:

        for i in range(20):
            gpu.draw_triangle(
                triangle=Triangle.TRIANGLE_A,
                red_value=2,
                green_value=0,
                blue_value=0,
                vertices=[Vertex(0, 480), Vertex(640, 480), Vertex(320, 15*(i+1))])
            gpu.draw_triangle(
                triangle=Triangle.TRIANGLE_B,
                red_value=2,
                green_value=2,
                blue_value=0,
                vertices=[Vertex(0, 480), Vertex(640, 480), Vertex(320, 10*(i+1))])


            await asyncio.sleep(0.015)

        await asyncio.sleep(1)

        for i in range(20):
            gpu.draw_triangle(
                triangle=Triangle.TRIANGLE_A,
                red_value=2,
                green_value=0,
                blue_value=0,
                vertices=[Vertex(0, 480), Vertex(640, 480), Vertex(320, 15*(19-i))])
            gpu.draw_triangle(
                triangle=Triangle.TRIANGLE_B,
                red_value=2,
                green_value=2,
                blue_value=0,
                vertices=[Vertex(0, 480), Vertex(640, 480), Vertex(320, 10*(19-i))])

            await asyncio.sleep(0.015)

        await asyncio.sleep(1)

async def runner(gpu: GPU):
    await asyncio.gather(commander(gpu), gpu.run())

def main():

    tt = DemoBoard.get()

    # Create GPU
    gpu = GPU(tt)

    tt.shuttle.tt_um_emern_top.enable()

    # Verify board settings
    tt.uio_oe_pico.value = 0b1011

    # Reset project
    tt.reset_project(putInReset=True)
    time.sleep_ms(1)
    tt.reset_project(putInReset=False)

    # Setup int pin
    int_pin = machine.Pin(GPIOMapTT06.UIO4, machine.Pin.IN, tt.pins.uio4.pull)
    int_pin.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=pin_isr)

    # Start GPU
    asyncio.run(runner(gpu))
