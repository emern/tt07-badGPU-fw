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
import cmath
import ulab
from ulab import numpy as np
from ulab import scipy
from .adc_test import ASCSampler, ADCChannelMap

SAMPLE_FREQ = 44000 # 44 kHz
SAMPLE_PERIOD = int((1 / SAMPLE_FREQ) * 1000000) # in microseconds
NUMBER_BINS = 64
NUM_BARS = 4
BETA = 0.2

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

gc.collect()

spi_active = False

def pin_isr(pin: machine.Pin):
    global spi_active
    if pin.value() == 1:
        spi_active = True
    else:
        spi_active = False

def draw_triangle(
        triangle: Triangle,
        red_value: int,
        green_value: int,
        blue_value: int,
        vertices: list[Vertex],
    ):
    # Lookup command byte and pack color to 6 bits
    cmd = triangle_write_map[triangle]
    packed_color = (red_value) << 4 | (green_value) << 2 | (blue_value)

    # Re-arrange triangles, should be x0 > x1 > x2
    ordered_vertices = sorted(vertices, reverse=False)

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
    return cmd_str_serialized

def main():

    tt = DemoBoard.get()

    tt.shuttle.tt_um_emern_top.enable()
    tt.clock_project_PWM(25000000, max_rp2040_freq=200e6)

    # Verify board settings
    tt.uio_oe_pico.value = 0b1011

    # Reset project
    tt.reset_project(putInReset=True)
    time.sleep_ms(1)
    tt.reset_project(putInReset=False)

    # Setup int pin
    int_pin = machine.Pin(GPIOMapTT06.UIO4, machine.Pin.IN, tt.pins.uio4.pull)
    int_pin.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=pin_isr)
    spi = machine.SoftSPI(baudrate=10000000, polarity=0, bits=8, phase=0, sck=machine.Pin(GPIOMapTT06.UIO3), mosi=machine.Pin(GPIOMapTT06.UIO1), miso=machine.Pin(GPIOMapTT06.UIO2), firstbit=machine.SPI.LSB)
    cs = machine.Pin(GPIOMapTT06.UIO0, mode=machine.Pin.OUT, value=1)
    sampler = ASCSampler(ADCChannelMap.ADC_CHAN_PIN_29, NUMBER_BINS, SAMPLE_FREQ)

    cmd_a = draw_triangle(
        triangle=Triangle.TRIANGLE_A,
        red_value=2,
        green_value=0,
        blue_value=0,
        vertices=[Vertex(0, 480), Vertex(160, 480), Vertex(80, 480)])
    cmd_b = draw_triangle(
        triangle=Triangle.TRIANGLE_B,
        red_value=2,
        green_value=2,
        blue_value=0,
        vertices=[Vertex(160, 480), Vertex(320, 480), Vertex(240, 480)])
    cmd_c = draw_triangle(
        triangle=Triangle.TRIANGLE_C,
        red_value=0,
        green_value=0,
        blue_value=2,
        vertices=[Vertex(320, 480), Vertex(480, 480), Vertex(400, 480)])
    cmd_d = draw_triangle(
        triangle=Triangle.TRIANGLE_D,
        red_value=0,
        green_value=2,
        blue_value=2,
        vertices=[Vertex(480, 480), Vertex(640, 480), Vertex(560, 480)])

    # Create SOS sections for third order low-pass butterworth filter with 20 kHz cutoff
    sos = [[0.75082223, 0.75082223, 0, 1, 0.74859062, 0], [1, 2, 1, 1, 1.68204283, 0.75305383]]

    last_bar_one = 0
    last_bar_two = 0
    last_bar_three = 0
    last_bar_four = 0

    while True:

        t_sample_start = time.ticks_us()
        arr = np.frombuffer(sampler.sample_blocking(), dtype=np.uint16)
        t_sample_end = time.ticks_us()

        t_compute_start = time.ticks_us()
        # Apply first order low-pass filter
        arr = scipy.signal.sosfilt(sos, arr)

        # Calculate magnitude of FFT
        fft_res = ulab.utils.spectrogram(arr)

        # Convert to dBFS with reference to max power seen
        ref = max(np.max(fft_res[1:5]), 20000.0)
        real = np.log10(fft_res[1:6] / ref) * 260
        bar_one = -1* real[1]
        bar_two = -1 * real[2]
        bar_three = -1 * real[3]
        bar_four = -1 * real[4]

        bar_one_scaled = min(bar_one, (460))
        bar_two_scaled = min(bar_two, (460))
        bar_three_scaled = min(bar_three, (460))
        bar_four_scaled = min(bar_four, (460))

        # Do EMA filter on bar values to smooth
        bar_one_scaled = (BETA * bar_one_scaled) + ((1 - BETA) * last_bar_one)
        bar_two_scaled = (BETA * bar_two_scaled) + ((1 - BETA) * last_bar_two)
        bar_three_scaled = (BETA * bar_three_scaled) + ((1 - BETA) * last_bar_three)
        bar_four_scaled = (BETA * bar_four_scaled) + ((1 - BETA) * last_bar_four)

        last_bar_one = bar_one_scaled
        last_bar_two = bar_two_scaled
        last_bar_three = bar_three_scaled
        last_bar_four = bar_four_scaled

        t_compute_end = time.ticks_us()

        t_pack_start = time.ticks_us()
        if np.isfinite(bar_one):
            cmd_a = draw_triangle(
                triangle=Triangle.TRIANGLE_A,
                red_value=2,
                green_value=0,
                blue_value=0,
                vertices=[Vertex(0, 480), Vertex(160, 480), Vertex(80, bar_one_scaled)])
        if np.isfinite(bar_two):
            cmd_b = draw_triangle(
                triangle=Triangle.TRIANGLE_B,
                red_value=2,
                green_value=2,
                blue_value=0,
                vertices=[Vertex(160, 480), Vertex(320, 480), Vertex(240, bar_two_scaled)])
        if np.isfinite(bar_three):
            cmd_c = draw_triangle(
                triangle=Triangle.TRIANGLE_C,
                red_value=0,
                green_value=0,
                blue_value=2,
                vertices=[Vertex(320, 480), Vertex(480, 480), Vertex(400, bar_three_scaled)])
        if np.isfinite(bar_four):
            cmd_d = draw_triangle(
                triangle=Triangle.TRIANGLE_D,
                red_value=0,
                green_value=2,
                blue_value=0,
                vertices=[Vertex(480, 480), Vertex(640, 480), Vertex(560, bar_four_scaled)])
        t_pack_end = time.ticks_us()

        t_wait_start = time.ticks_us()
        # Wait for next opportunity to send data
        while not spi_active:
            # Wait for interrupt to be triggered
            pass
        t_wait_end = time.ticks_us()

        t_spi_start = time.ticks_us()
        try:
            # Empty queue or go until transfer phase over
            cs(0)
            spi.write(cmd_a)
            cs(1)

            cs(0)
            spi.write(cmd_b)
            cs(1)

            cs(0)
            spi.write(cmd_c)
            cs(1)

            cs(0)
            spi.write(cmd_d)
            cs(1)

        except:
            pass
        t_spi_end = time.ticks_us()

        # print(f"Sample time: {(t_sample_end - t_sample_start) * 1e-6} seconds")
        # print(f"Compute time: {(t_compute_end - t_compute_start) * 1e-6} seconds")
        # print(f"Pack time: {(t_pack_end - t_pack_start) * 1e-6} seconds")
        # print(f"Wait time: {(t_wait_end - t_wait_start) * 1e-6} seconds")
        # print(f"SPI time: {(t_spi_end - t_spi_start) * 1e-6} seconds")

        # print(ref)
        # print(f"FFT Result: {fft_res[1:5]}")
        # print(f"Bar 1: {bar_one}, Bar 2: {bar_two}, Bar 3: {bar_three}, Bar 4: {bar_four}")
        # print(f"(Scaled) Bar 1: {bar_one_scaled}, Bar 2: {bar_two_scaled}, Bar 3: {bar_three_scaled}, Bar 4: {bar_four_scaled}")
        # print(f"(Last) Bar 1: {last_bar_one}, Bar 2: {last_bar_two}, Bar 3: {last_bar_three}, Bar 4: {last_bar_four}")
