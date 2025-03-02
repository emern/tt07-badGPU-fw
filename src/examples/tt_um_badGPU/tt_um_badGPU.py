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


SPI_CMD_WRITE_POLY_A = 0x80
SPI_CMD_CLEAR_POLY_A = 0x40
SPI_CMD_WRITE_POLY_B = 0x81
SPI_CMD_CLEAR_POLY_B = 0x41
SPI_CMD_WRITE_POLY_C = 0x82
SPI_CMD_CLEAR_POLY_C = 0x42
SPI_CMD_WRITE_POLY_D = 0x83
SPI_CMD_CLEAR_POLY_D = 0x43
SPI_CMD_SET_BG_COLOR = 0x01

SPI_CS_PIN = 0
SPI_MOSI_PIN = 1
SPI_CLK_PIN = 3
SPI_INT_PIN = 4

class GPU:
    def __init__(self, tt: DemoBoard):
        self.tt = tt
        self.spi = machine.SoftSPI(baudrate=1000000, polarity=0, bits=8, phase=0, sck=machine.Pin(GPIOMapTT06.UIO3), mosi=machine.Pin(GPIOMapTT06.UIO1), miso=machine.Pin(GPIOMapTT06.UIO2), firstbit=machine.SPI.LSB)
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
                self.cs(0)
                while True:
                    try:
                        # Empty queue or go until transfer phase over
                        cmd = self.queue.pop()
                        self.spi.write(cmd)

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
        for color in range(256):
            await asyncio.sleep(1)
            gpu.write(bytes([0x01, color, 0x00, 0x00, 0x00, 0x00, 0x00]))


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

    # Setup SPI pins
    spi = machine.SoftSPI(baudrate=1000000, polarity=0, bits=8, phase=0, sck=machine.Pin(GPIOMapTT06.UIO3), mosi=machine.Pin(GPIOMapTT06.UIO1), miso=machine.Pin(GPIOMapTT06.UIO2), firstbit=machine.SPI.LSB)
    cs = machine.Pin(GPIOMapTT06.UIO0, mode=machine.Pin.OUT, value=1)

    # Start GPU
    asyncio.run(runner(gpu))
