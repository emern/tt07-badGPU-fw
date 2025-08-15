from uctypes import BF_POS, BF_LEN, UINT32, BFUINT32, struct
import array
import ttboard.util.time as time
import uctypes

GPIO_BASE       = 0x40014000
GPIO_CHAN_WIDTH = 0x08
GPIO_PIN_COUNT  = 30
PAD_BASE        = 0x4001c000
PAD_PIN_WIDTH   = 0x04
ADC_BASE        = 0x4004c000
DMA_BASE        = 0x50000000
DMA_CHAN_WIDTH  = 0x40
DMA_CHAN_COUNT  = 12

# DMA: RP2040 datasheet 2.5.7
DMA_CTRL_TRIG_FIELDS = {
    "AHB_ERROR":   31<<BF_POS | 1<<BF_LEN | BFUINT32,
    "READ_ERROR":  30<<BF_POS | 1<<BF_LEN | BFUINT32,
    "WRITE_ERROR": 29<<BF_POS | 1<<BF_LEN | BFUINT32,
    "BUSY":        24<<BF_POS | 1<<BF_LEN | BFUINT32,
    "SNIFF_EN":    23<<BF_POS | 1<<BF_LEN | BFUINT32,
    "BSWAP":       22<<BF_POS | 1<<BF_LEN | BFUINT32,
    "IRQ_QUIET":   21<<BF_POS | 1<<BF_LEN | BFUINT32,
    "TREQ_SEL":    15<<BF_POS | 6<<BF_LEN | BFUINT32,
    "CHAIN_TO":    11<<BF_POS | 4<<BF_LEN | BFUINT32,
    "RING_SEL":    10<<BF_POS | 1<<BF_LEN | BFUINT32,
    "RING_SIZE":    6<<BF_POS | 4<<BF_LEN | BFUINT32,
    "INCR_WRITE":   5<<BF_POS | 1<<BF_LEN | BFUINT32,
    "INCR_READ":    4<<BF_POS | 1<<BF_LEN | BFUINT32,
    "DATA_SIZE":    2<<BF_POS | 2<<BF_LEN | BFUINT32,
    "HIGH_PRIORITY":1<<BF_POS | 1<<BF_LEN | BFUINT32,
    "EN":           0<<BF_POS | 1<<BF_LEN | BFUINT32
}
# Channel-specific DMA registers
DMA_CHAN_REGS = {
    "READ_ADDR_REG":       0x00|UINT32,
    "WRITE_ADDR_REG":      0x04|UINT32,
    "TRANS_COUNT_REG":     0x08|UINT32,
    "CTRL_TRIG_REG":       0x0c|UINT32,
    "CTRL_TRIG":          (0x0c,DMA_CTRL_TRIG_FIELDS)
}
# General DMA registers
DMA_REGS = {
    "INTR":               0x400|UINT32,
    "INTE0":              0x404|UINT32,
    "INTF0":              0x408|UINT32,
    "INTS0":              0x40c|UINT32,
    "INTE1":              0x414|UINT32,
    "INTF1":              0x418|UINT32,
    "INTS1":              0x41c|UINT32,
    "TIMER0":             0x420|UINT32,
    "TIMER1":             0x424|UINT32,
    "TIMER2":             0x428|UINT32,
    "TIMER3":             0x42c|UINT32,
    "MULTI_CHAN_TRIGGER": 0x430|UINT32,
    "SNIFF_CTRL":         0x434|UINT32,
    "SNIFF_DATA":         0x438|UINT32,
    "FIFO_LEVELS":        0x440|UINT32,
    "CHAN_ABORT":         0x444|UINT32
}

# GPIO status and control: RP2040 datasheet 2.19.6.1.10
GPIO_STATUS_FIELDS = {
    "IRQTOPROC":  26<<BF_POS | 1<<BF_LEN | BFUINT32,
    "IRQFROMPAD": 24<<BF_POS | 1<<BF_LEN | BFUINT32,
    "INTOPERI":   19<<BF_POS | 1<<BF_LEN | BFUINT32,
    "INFROMPAD":  17<<BF_POS | 1<<BF_LEN | BFUINT32,
    "OETOPAD":    13<<BF_POS | 1<<BF_LEN | BFUINT32,
    "OEFROMPERI": 12<<BF_POS | 1<<BF_LEN | BFUINT32,
    "OUTTOPAD":    9<<BF_POS | 1<<BF_LEN | BFUINT32,
    "OUTFROMPERI": 8<<BF_POS | 1<<BF_LEN | BFUINT32
}
GPIO_CTRL_FIELDS = {
    "IRQOVER":    28<<BF_POS | 2<<BF_LEN | BFUINT32,
    "INOVER":     16<<BF_POS | 2<<BF_LEN | BFUINT32,
    "OEOVER":     12<<BF_POS | 2<<BF_LEN | BFUINT32,
    "OUTOVER":     8<<BF_POS | 2<<BF_LEN | BFUINT32,
    "FUNCSEL":     0<<BF_POS | 5<<BF_LEN | BFUINT32
}
GPIO_REGS = {
    "GPIO_STATUS_REG":     0x00|UINT32,
    "GPIO_STATUS":        (0x00,GPIO_STATUS_FIELDS),
    "GPIO_CTRL_REG":       0x04|UINT32,
    "GPIO_CTRL":          (0x04,GPIO_CTRL_FIELDS)
}

# PAD control: RP2040 datasheet 2.19.6.3
PAD_FIELDS = {
    "OD":          7<<BF_POS | 1<<BF_LEN | BFUINT32,
    "IE":          6<<BF_POS | 1<<BF_LEN | BFUINT32,
    "DRIVE":       4<<BF_POS | 2<<BF_LEN | BFUINT32,
    "PUE":         3<<BF_POS | 1<<BF_LEN | BFUINT32,
    "PDE":         2<<BF_POS | 1<<BF_LEN | BFUINT32,
    "SCHMITT":     1<<BF_POS | 1<<BF_LEN | BFUINT32,
    "SLEWFAST":    0<<BF_POS | 1<<BF_LEN | BFUINT32
}
PAD_REGS = {
    "PAD_REG":             0x00|UINT32,
    "PAD":                (0x00,PAD_FIELDS)
}

# ADC: RP2040 datasheet 4.9.6
ADC_CS_FIELDS = {
    "RROBIN":     16<<BF_POS | 5<<BF_LEN | BFUINT32,
    "AINSEL":     12<<BF_POS | 3<<BF_LEN | BFUINT32,
    "ERR_STICKY": 10<<BF_POS | 1<<BF_LEN | BFUINT32,
    "ERR":         9<<BF_POS | 1<<BF_LEN | BFUINT32,
    "READY":       8<<BF_POS | 1<<BF_LEN | BFUINT32,
    "START_MANY":  3<<BF_POS | 1<<BF_LEN | BFUINT32,
    "START_ONCE":  2<<BF_POS | 1<<BF_LEN | BFUINT32,
    "TS_EN":       1<<BF_POS | 1<<BF_LEN | BFUINT32,
    "EN":          0<<BF_POS | 1<<BF_LEN | BFUINT32
}
ADC_FCS_FIELDS = {
    "THRESH":     24<<BF_POS | 4<<BF_LEN | BFUINT32,
    "LEVEL":      16<<BF_POS | 4<<BF_LEN | BFUINT32,
    "OVER":       11<<BF_POS | 1<<BF_LEN | BFUINT32,
    "UNDER":      10<<BF_POS | 1<<BF_LEN | BFUINT32,
    "FULL":        9<<BF_POS | 1<<BF_LEN | BFUINT32,
    "EMPTY":       8<<BF_POS | 1<<BF_LEN | BFUINT32,
    "DREQ_EN":     3<<BF_POS | 1<<BF_LEN | BFUINT32,
    "ERR":         2<<BF_POS | 1<<BF_LEN | BFUINT32,
    "SHIFT":       1<<BF_POS | 1<<BF_LEN | BFUINT32,
    "EN":          0<<BF_POS | 1<<BF_LEN | BFUINT32,
}
ADC_REGS = {
    "CS_REG":              0x00|UINT32,
    "CS":                 (0x00,ADC_CS_FIELDS),
    "RESULT_REG":          0x04|UINT32,
    "FCS_REG":             0x08|UINT32,
    "FCS":                (0x08,ADC_FCS_FIELDS),
    "FIFO_REG":            0x0c|UINT32,
    "DIV_REG":             0x10|UINT32,
    "INTR_REG":            0x14|UINT32,
    "INTE_REG":            0x18|UINT32,
    "INTF_REG":            0x1c|UINT32,
    "INTS_REG":            0x20|UINT32
}
DREQ_PIO0_TX0, DREQ_PIO0_RX0, DREQ_PIO1_TX0 = 0, 4, 8
DREQ_PIO1_RX0, DREQ_SPI0_TX,  DREQ_SPI0_RX  = 12, 16, 17
DREQ_SPI1_TX,  DREQ_SPI1_RX,  DREQ_UART0_TX = 18, 19, 20
DREQ_UART0_RX, DREQ_UART1_TX, DREQ_UART1_RX = 21, 22, 23
DREQ_I2C0_TX,  DREQ_I2C0_RX,  DREQ_I2C1_TX  = 32, 33, 34
DREQ_I2C1_RX,  DREQ_ADC                     = 35, 36

DMA_CHANS = [struct(DMA_BASE + n*DMA_CHAN_WIDTH, DMA_CHAN_REGS) for n in range(0,DMA_CHAN_COUNT)]
DMA_DEVICE = struct(DMA_BASE, DMA_REGS)
GPIO_PINS = [struct(GPIO_BASE + n*GPIO_CHAN_WIDTH, GPIO_REGS) for n in range(0,GPIO_PIN_COUNT)]
PAD_PINS =  [struct(PAD_BASE + n*PAD_PIN_WIDTH, PAD_REGS) for n in range(0,GPIO_PIN_COUNT)]
ADC_DEVICE = struct(ADC_BASE, ADC_REGS)
ADC_FIFO_ADDR = ADC_BASE + 0x0c

GPIO_FUNC_SPI, GPIO_FUNC_UART, GPIO_FUNC_I2C = 1, 2, 3
GPIO_FUNC_PWM, GPIO_FUNC_SIO, GPIO_FUNC_PIO0 = 4, 5, 6
GPIO_FUNC_NULL = 0x1f

# Enumerations for the ADC channel mapping to GPIO pins
class ADCChannelMap:
    ADC_CHAN_PIN_26 = 0
    ADC_CHAN_PIN_27 = 1
    ADC_CHAN_PIN_28 = 2
    ADC_CHAN_PIN_29 = 3
    ADC_CHAN_TEMP_SENSOR = 4

class ASCSampler:

    def __init__(self, adc_chan, nsamples, sample_rate):
        self.adc_chan = adc_chan
        self.nsamples = nsamples
        self.sample_rate = sample_rate

        # Note: Data array get reused in between transfers to avoid reallocation
        self.adc_buf = array.array('H', (0 for _ in range(self.nsamples)))

        adc_pin = 26 + self.adc_chan
        self.adc = ADC_DEVICE
        self.pin = GPIO_PINS[adc_pin]
        self.pad = PAD_PINS[adc_pin]
        self.dma_chan = DMA_CHANS[0]
        self.pin.GPIO_CTRL_REG = GPIO_FUNC_NULL
        self.pad.PAD_REG = 0

        # Setup ADC hardware
        # Warning: Register set magic below
        self.adc.CS_REG = self.adc.FCS_REG = 0 # Clear CS and FCS registers
        self.adc.CS.EN = 1 # Enable ADC
        self.adc.CS.AINSEL = self.adc_chan # Select correct input channel
        self.adc.CS.START_ONCE = 1 # Start single shot ADC conversion now to warmup

        self.adc.FCS.EN = self.adc.FCS.DREQ_EN = 1 # Enable FIFO and DMA transfer requests

        # Setup ADC autosampling rate - note that we are only using the upper 16 bits of the 16.8 register value here
        # Total period = 1 + DIV_REG cycles of the 48 mHz ADC clock
        self.adc.DIV_REG = (48000000 // self.sample_rate - 1) << 8
        # THRESH = 1: Set DREQ when LEVEL (number of samples in FIFO) >= THRESH
        # OVER/UNDER = 1: Sets if FIFO has over/underflowed
        self.adc.FCS.THRESH = self.adc.FCS.OVER = self.adc.FCS.UNDER = 1

        # Setup DMA channel
        self.dma_chan.READ_ADDR_REG = ADC_FIFO_ADDR # Always read from ADC FIFO
        self.dma_chan.WRITE_ADDR_REG = uctypes.addressof(self.adc_buf) # Write to static ADC buffer
        self.dma_chan.TRANS_COUNT_REG = self.nsamples # Number of samples to transfer

        self.dma_chan.CTRL_TRIG_REG = 0 # Clear out TRIG register
        self.dma_chan.CTRL_TRIG.CHAIN_TO = 0 # Do not chain to another DMA channel
        self.dma_chan.CTRL_TRIG.INCR_WRITE = 1 # Increment write address for each transfer
        self.dma_chan.CTRL_TRIG.IRQ_QUIET = 1 # Do not generate IRQ on transfer complete
        self.dma_chan.CTRL_TRIG.TREQ_SEL = DREQ_ADC # Enable ADC DMA request
        self.dma_chan.CTRL_TRIG.DATA_SIZE = 1 # Transfer 16 bits per transfer

        # Clear off the ADC reading in the FIFO
        while self.adc.FCS.LEVEL:
            x = self.adc.FIFO_REG

    def sample_blocking(self) -> array.array:
        self.dma_chan.WRITE_ADDR_REG = uctypes.addressof(self.adc_buf)
        self.dma_chan.CTRL_TRIG.EN = 1
        self.adc.CS.START_MANY = 1
        while self.dma_chan.CTRL_TRIG.BUSY:
            pass

        self.adc.CS.START_MANY = 0
        self.dma_chan.CTRL_TRIG.EN = 0

        return self.adc_buf
