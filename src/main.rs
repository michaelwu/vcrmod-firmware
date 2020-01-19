#![feature(asm)]
#![feature(panic_info_message)]

#![no_std]
#![no_main]

#[no_mangle]
pub extern fn abort() {
    loop { }
}

use core::mem;
use core::ptr;
use core::panic::PanicInfo;

macro_rules! println {
    ($fmt:expr) => (print!(concat!($fmt, "\n\r")));
    ($fmt:expr, $($arg:tt)*) => (print!(concat!($fmt, "\n\r"), $($arg)*));
}

macro_rules! print {
    ($($arg:tt)*) => ({let mut writer = Uart1Writer; writer.write_fmt(format_args!($($arg)*)).unwrap()});
}

#[allow(unused_must_use)]
#[no_mangle]
#[panic_handler]
pub extern fn panic_fmt(info: &PanicInfo) -> ! {
    let mut writer = Uart1Writer;
    if let Some(l) = info.location() {
        writer.write_fmt(format_args!("Panic at {}:{} -\r\n", l.file(), l.line()));
    } else {
        writer.write_str("Panic at unknown location\r\n");
    }
    if let Some(fmt) = info.message() {
        core::fmt::write(&mut writer, *fmt);
    }
    writer.write_str("\r\n");
    loop {}
}

extern {
    static __stack_top: u8;
    static __text_end: u8;
    static mut __data_begin: u8;
    static mut __data_end: u8;
    static mut __bss_begin: u8;
    static mut __bss_end: u8;
}

pub type ISR = unsafe extern fn();

pub struct InitialStack(pub *const u8);
unsafe impl Sync for InitialStack {}

#[repr(C)]
pub struct M0VectorTable {
    pub initial_stack: InitialStack,
    pub reset: ISR,
    pub nmi: ISR,
    pub hard_fault: ISR,
    pub reserved0: [usize; 7],
    pub sv_call: ISR,
    pub reserved1: [usize; 2],
    pub pend_sv: ISR,
    pub systick: ISR,
}

#[repr(C)]
pub struct KL26VectorTable {
    pub base: M0VectorTable,
    pub dma: [ISR; 4],
    pub reserved0: usize,
    pub ftfa: ISR,
    pub pmc: ISR,
    pub llwu: ISR,
    pub i2c: [ISR; 2],
    pub spi: [ISR; 2],
    pub uart: [ISR; 3],
    pub adc: [ISR; 1],
    pub cmp: [ISR; 1],
    pub tpm: [ISR; 3],
    pub rtc_alarm: ISR,
    pub rtc_seconds: ISR,
    pub pit: ISR,
    pub i2s: [ISR; 1],
    pub usb_otg: ISR,
    pub dac: [ISR; 1],
    pub tsi: [ISR; 1],
    pub mcg: ISR,
    pub lptmr: [ISR; 1],
    pub reserved1: usize,
    pub pin_detect_a: ISR,
    pub pin_detect_cd: ISR,
}

#[used]
#[link_section=".vector_table"]
pub static _VECTOR: KL26VectorTable = KL26VectorTable {
    base: M0VectorTable {
        initial_stack: unsafe { InitialStack(&__stack_top) },
        reset: reset_isr,
        nmi: unimplemented_isr,
        hard_fault: hardfault_isr,
        reserved0: [0; 7],
        sv_call: unimplemented_isr,
        reserved1: [0; 2],
        pend_sv: unimplemented_isr,
        systick: systick_isr,
    },
    dma: [unimplemented_isr; 4],
    reserved0: 0,
    ftfa: unimplemented_isr,
    pmc: unimplemented_isr,
    llwu: unimplemented_isr,
    i2c: [unimplemented_isr; 2],
    spi: [unimplemented_isr; 2],
    uart: [unimplemented_isr; 3],
    adc: [unimplemented_isr; 1],
    cmp: [unimplemented_isr; 1],
    tpm: [unimplemented_isr; 3],
    rtc_alarm: unimplemented_isr,
    rtc_seconds: unimplemented_isr,
    pit: unimplemented_isr,
    i2s: [unimplemented_isr; 1],
    usb_otg: unimplemented_isr,
    dac: [unimplemented_isr; 1],
    tsi: [unimplemented_isr; 1],
    mcg: unimplemented_isr,
    lptmr: [unimplemented_isr; 1],
    reserved1: 0,
    pin_detect_a: unimplemented_isr,
    pin_detect_cd: unimplemented_isr,
};

#[used]
#[link_section=".flash_config"]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF
];

#[repr(C)]
pub struct Register<T: Copy, Align: Copy = T> {
    v: T,
    _a: [Align; 0],
}

impl<T: Copy, Align: Copy> Register<T, Align> {
    pub unsafe fn set(&self, v: T) {
        ptr::write_volatile(&self.v as *const _ as *mut _, v)
    }

    pub unsafe fn get(&self) -> T {
        ptr::read_volatile(&self.v as *const _)
    }
}

#[repr(C)]
pub struct PortModule {
    pub pcr: [Register<u32>; 32],
    pub gpclr: Register<u32>,
    pub gpchr: Register<u32>,
    pub reserved: [u32; 6],
    pub isfr: Register<u32>,
}

#[repr(C)]
pub struct GpioModule {
    pub pdor: Register<u32>,
    pub psor: Register<u32>,
    pub pcor: Register<u32>,
    pub ptor: Register<u32>,
    pub pdir: Register<u32>,
    pub pddr: Register<u32>,
}

#[repr(C)]
pub struct SpiModule {
    pub s: Register<u8>,
    pub br: Register<u8>,
    pub c2: Register<u8>,
    pub c1: Register<u8>,
    pub ml: Register<u8>,
    pub mh: Register<u8>,
    pub dl: Register<u8>,
    pub dh: Register<u8>,
    pub reserved: [u8; 2],
    pub ci: Register<u8>,
    pub c3: Register<u8>,
}

#[repr(C)]
pub struct SimpleUartModule {
    pub bdh: Register<u8>,
    pub bdl: Register<u8>,
    pub c1: Register<u8>,
    pub c2: Register<u8>,
    pub s1: Register<u8>,
    pub s2: Register<u8>,
    pub c3: Register<u8>,
    pub d: Register<u8>,
    pub c4: Register<u8>,
}

#[repr(C)]
pub struct UsbModule {
    pub perid: Register<u8, u32>,
    pub idcomp: Register<u8, u32>,
    pub rev: Register<u8, u32>,
    pub addinfo: Register<u8, u32>,
    pub otgistat: Register<u8, u32>,
    pub otgicr: Register<u8, u32>,
    pub otgstat: Register<u8, u32>,
    pub otgctl: Register<u8, u32>,
    pub reserved0: [u8; 0x60],
    pub istat: Register<u8, u32>,
    pub inten: Register<u8, u32>,
    pub errstat: Register<u8, u32>,
    pub erren: Register<u8, u32>,
    pub stat: Register<u8, u32>,
    pub ctl: Register<u8, u32>,
    pub addr: Register<u8, u32>,
    pub bdtpage1: Register<u8, u32>,
    pub frmnuml: Register<u8, u32>,
    pub frmnumh: Register<u8, u32>,
    pub token: Register<u8, u32>,
    pub softhld: Register<u8, u32>,
    pub bdtpage2: Register<u8, u32>,
    pub bdtpage3: Register<u8, u32>,
    pub reserved1: [u32; 2],
    pub endpt: [Register<u8, u32>; 16],
    pub usbctrl: Register<u8, u32>,
    pub observe: Register<u8, u32>,
    pub control: Register<u8, u32>,
    pub usbtrc0: Register<u8, u32>,
    pub reserved2: u32,
    pub usbfrmadjust: Register<u8, u32>,
}

extern {
    pub static NVIC_ISER: Register<u32>;
    pub static NVIC_ICER: Register<u32>;
    pub static NVIC_ISPR: Register<u32>;
    pub static NVIC_ICPR: Register<u32>;
    pub static NVIC_IPR: [Register<u32>; 8];
    pub static SYST_CSR: Register<u32>;
    pub static SYST_RVR: Register<u32>;
    pub static SYST_CVR: Register<u32>;

    pub static PORTA: PortModule;
    pub static PORTB: PortModule;
    pub static PORTC: PortModule;
    pub static PORTD: PortModule;
    pub static PORTE: PortModule;
    pub static SPI0: SpiModule;
    pub static SPI1: SpiModule;
    pub static GPIOA: GpioModule;
    pub static GPIOB: GpioModule;
    pub static GPIOC: GpioModule;
    pub static GPIOD: GpioModule;
    pub static GPIOE: GpioModule;
    pub static UART1: SimpleUartModule;
    pub static UART2: SimpleUartModule;
    pub static USB0: UsbModule;
    pub static SIM_SCGC4: Register<u32>;
    pub static SIM_SCGC5: Register<u32>;
    pub static SIM_SCGC6: Register<u32>;
    pub static SIM_SCGC7: Register<u32>;
    pub static SIM_CLKDIV1: Register<u32>;
    pub static SIM_SOPT2: Register<u32>;
    pub static SIM_COPC: Register<u32>;
    pub static OSC0_CR: Register<u8>;
    pub static MCG_C1: Register<u8>;
    pub static MCG_C2: Register<u8>;
    pub static MCG_C3: Register<u8>;
    pub static MCG_C4: Register<u8>;
    pub static MCG_C5: Register<u8>;
    pub static MCG_C6: Register<u8>;
    pub static MCG_S: Register<u8>;
    pub static MCG_SC: Register<u8>;
    pub static MCG_ATCVH: Register<u8>;
    pub static MCG_ATCVL: Register<u8>;
    pub static MCG_C7: Register<u8>;
    pub static MCG_C8: Register<u8>;
    pub static MCG_C9: Register<u8>;
    pub static MCG_C10: Register<u8>;
    pub static PMC_REGSC: Register<u8>;
}

pub const SIM_SCGC4_SPI1: u32 = 1 << 23;
pub const SIM_SCGC4_SPI0: u32 = 1 << 22;
pub const SIM_SCGC4_CMP: u32 = 1 << 19;
pub const SIM_SCGC4_USBOTG: u32 = 1 << 18;
pub const SIM_SCGC4_UART2: u32 = 1 << 12;
pub const SIM_SCGC4_UART1: u32 = 1 << 11;
pub const SIM_SCGC4_UART0: u32 = 1 << 10;
pub const SIM_SCGC4_I2C1: u32 = 1 << 7;
pub const SIM_SCGC4_I2C0: u32 = 1 << 6;

pub const SIM_SCGC5_PORTE: u32 = 1 << 13;
pub const SIM_SCGC5_PORTD: u32 = 1 << 12;
pub const SIM_SCGC5_PORTC: u32 = 1 << 11;
pub const SIM_SCGC5_PORTB: u32 = 1 << 10;
pub const SIM_SCGC5_PORTA: u32 = 1 << 9;
pub const SIM_SCGC5_TSI: u32 = 1 << 5;
pub const SIM_SCGC5_LPTMR: u32 = 1 << 0;

pub const SIM_SCGC6_DAC0: u32 = 1 << 31;
pub const SIM_SCGC6_RTC: u32 = 1 << 29;
pub const SIM_SCGC6_ADC0: u32 = 1 << 27;
pub const SIM_SCGC6_TPM2: u32 = 1 << 26;
pub const SIM_SCGC6_TPM1: u32 = 1 << 25;
pub const SIM_SCGC6_TPM0: u32 = 1 << 24;
pub const SIM_SCGC6_PIT: u32 = 1 << 23;
pub const SIM_SCGC6_I2S: u32 = 1 << 15;
pub const SIM_SCGC6_DMAMUX: u32 = 1 << 1;
pub const SIM_SCGC6_FTF: u32 = 1 << 0;

pub const SIM_SOPT2_USBSRC: u32 = 1 << 18;
pub const SIM_SOPT2_PLLFLLSEL: u32 = 1 << 16;
pub const SIM_SOPT2_RTCCLKOUTSEL: u32 = 1 << 4;

pub const PORT_PCR_ISF: u32 = 1 << 24;
pub const PORT_PCR_GPIO: u32 = 1 << 8;
pub const PORT_PCR_DSE: u32 = 1 << 6;
pub const PORT_PCR_PFE: u32 = 1 << 4;
pub const PORT_PCR_SRE: u32 = 1 << 2;
pub const PORT_PCR_PE: u32 = 1 << 1;
pub const PORT_PCR_PS: u32 = 1 << 0;

pub const OSC_CR_ERCLKEN: u8 = 1 << 7;
pub const OSC_CR_EREFSTEN: u8 = 1 << 5;
pub const OSC_CR_SC2P: u8 = 1 << 3;
pub const OSC_CR_SC4P: u8 = 1 << 2;
pub const OSC_CR_SC8P: u8 = 1 << 1;
pub const OSC_CR_SC16P: u8 = 1 << 0;

pub const MCG_C1_IREFS: u8 = 1 << 2;
pub const MCG_C1_IRCLKEN: u8 = 1 << 1;
pub const MCG_C1_IREFSTEN: u8 = 1 << 0;

pub const MCG_C2_LOCRE0: u8 = 1 << 7;
pub const MCG_C2_FCFTRIM: u8 = 1 << 6;
pub const MCG_C2_HGO0: u8 = 1 << 3;
pub const MCG_C2_EREFS0: u8 = 1 << 2;
pub const MCG_C2_LP: u8 = 1 << 1;
pub const MCG_C2_IRCS: u8 = 1 << 0;

pub const MCG_C6_LOLIE0: u8 = 1 << 7;
pub const MCG_C6_PLLS: u8 = 1 << 6;
pub const MCG_C6_CME0: u8 = 1 << 5;

pub const MCG_S_LOLS0: u8 = 1 << 7;
pub const MCG_S_LOCK0: u8 = 1 << 6;
pub const MCG_S_PLLST: u8 = 1 << 5;
pub const MCG_S_IREFST: u8 = 1 << 4;
pub const MCG_S_OSCINIT0: u8 = 1 << 1;
pub const MCG_S_IRCST: u8 = 1 << 0;

pub const PMC_REGSC_BGEN: u8 = 1 << 4;
pub const PMC_REGSC_ACKISO: u8 = 1 << 3;
pub const PMC_REGSC_REGONS: u8 = 1 << 2;
pub const PMC_REGSC_BGBE: u8 = 1 << 0;

pub const SYST_CSR_COUNTFLAG: u32 = 1 << 16;
pub const SYST_CSR_CLKSOURCE: u32 = 1 << 2;
pub const SYST_CSR_TICKINT: u32 = 1 << 1;
pub const SYST_CSR_ENABLE: u32 = 1 << 0;

pub const SPI_C1_SPIE: u8 = 1 << 7;
pub const SPI_C1_SPE: u8 = 1 << 6;
pub const SPI_C1_SPTIE: u8 = 1 << 5;
pub const SPI_C1_MSTR: u8 = 1 << 4;
pub const SPI_C1_CPOL: u8 = 1 << 3;
pub const SPI_C1_CPHA: u8 = 1 << 2;
pub const SPI_C1_SSOE: u8 = 1 << 1;
pub const SPI_C1_LSBFE: u8 = 1 << 0;

pub const SPI_C2_SPMIE: u8 = 1 << 7;
pub const SPI_C2_SPIMODE: u8 = 1 << 6;
pub const SPI_C2_TXDMAE: u8 = 1 << 5;
pub const SPI_C2_MODFEN: u8 = 1 << 4;
pub const SPI_C2_BIDIROE: u8 = 1 << 3;
pub const SPI_C2_RXDMAE: u8 = 1 << 2;
pub const SPI_C2_SPISWAI: u8 = 1 << 1;
pub const SPI_C2_SPC0: u8 = 1 << 0;

pub const SPI_S_SPRF: u8 = 1 << 7;
pub const SPI_S_SPMF: u8 = 1 << 6;
pub const SPI_S_SPTEF: u8 = 1 << 5;
pub const SPI_S_MODF: u8 = 1 << 4;

pub const UART_C2_TIE: u8 = 1 << 7;
pub const UART_C2_TCIE: u8 = 1 << 6;
pub const UART_C2_RIE: u8 = 1 << 5;
pub const UART_C2_ILIE: u8 = 1 << 4;
pub const UART_C2_TE: u8 = 1 << 3;
pub const UART_C2_RE: u8 = 1 << 2;
pub const UART_C2_RWU: u8 = 1 << 1;
pub const UART_C2_SBK: u8 = 1 << 0;

pub const UART_S1_TDRE: u8 = 1 << 7;
pub const UART_S1_TC: u8 = 1 << 6;
pub const UART_S1_RDRF: u8 = 1 << 5;
pub const UART_S1_IDLE: u8 = 1 << 4;
pub const UART_S1_OR: u8 = 1 << 3;
pub const UART_S1_NF: u8 = 1 << 2;
pub const UART_S1_FE: u8 = 1 << 1;
pub const UART_S1_PF: u8 = 1 << 0;

pub const USB_OTGISTAT_IDCHG: u8 = 1 << 7;
pub const USB_OTGISTAT_ONEMSEC: u8 = 1 << 6;
pub const USB_OTGISTAT_LINE_STATE_CHG: u8 = 1 << 5;
pub const USB_OTGISTAT_SESSVLDCHG: u8 = 1 << 3;
pub const USB_OTGISTAT_B_SESS_CHG: u8 = 1 << 2;
pub const USB_OTGISTAT_AVBUSCHG: u8 = 1 << 0;

pub const USB_OTGICR_IDEN: u8 = 1 << 7;
pub const USB_OTGICR_ONEMSECEN: u8 = 1 << 6;
pub const USB_OTGICR_LINESTATEEN: u8 = 1 << 5;
pub const USB_OTGICR_SESSVLDEN: u8 = 1 << 3;
pub const USB_OTGICR_BSESSEN: u8 = 1 << 2;
pub const USB_OTGICR_AVBUSEN: u8 = 1 << 0;

pub const USB_OTGSTAT_ID: u8 = 1 << 7;
pub const USB_OTGSTAT_LINESTATESTABLE: u8 = 1 << 5;
pub const USB_OTGSTAT_SESS_VLD: u8 = 1 << 3;
pub const USB_OTGSTAT_BSESSEND: u8 = 1 << 2;
pub const USB_OTGSTAT_AVBUSVLD: u8 = 1 << 0;

pub const USB_ISTAT_STALL: u8 = 1 << 7;
pub const USB_ISTAT_ATTACH: u8 = 1 << 6;
pub const USB_ISTAT_RESUME: u8 = 1 << 5;
pub const USB_ISTAT_SLEEP: u8 = 1 << 4;
pub const USB_ISTAT_TOKDNE: u8 = 1 << 3;
pub const USB_ISTAT_SOFTOK: u8 = 1 << 2;
pub const USB_ISTAT_ERROR: u8 = 1 << 1;
pub const USB_ISTAT_USBRST: u8 = 1 << 0;

pub const USB_INTEN_STALLEN: u8 = 1 << 7;
pub const USB_INTEN_ATTACHEN: u8 = 1 << 6;
pub const USB_INTEN_RESUMEEN: u8 = 1 << 5;
pub const USB_INTEN_SLEEPEN: u8 = 1 << 4;
pub const USB_INTEN_TOKDNEEN: u8 = 1 << 3;
pub const USB_INTEN_SOFTOKEN: u8 = 1 << 2;
pub const USB_INTEN_ERROREN: u8 = 1 << 1;
pub const USB_INTEN_USBRSTEN: u8 = 1 << 0;

pub const USB_CTL_JSTATE: u8 = 1 << 7;
pub const USB_CTL_SE0: u8 = 1 << 6;
pub const USB_CTL_TXSUSPENDTOKENBUSY: u8 = 1 << 5;
pub const USB_CTL_RESET: u8 = 1 << 4;
pub const USB_CTL_HOSTMODEEN: u8 = 1 << 3;
pub const USB_CTL_RESUME: u8 = 1 << 2;
pub const USB_CTL_ODDRST: u8 = 1 << 1;
pub const USB_CTL_USBENSOFEN: u8 = 1 << 0;

pub const USB_ERRSTAT_BTSERR: u8 = 1 << 7;
pub const USB_ERRSTAT_DMAERR: u8 = 1 << 5;
pub const USB_ERRSTAT_BTOERR: u8 = 1 << 4;
pub const USB_ERRSTAT_DFN8: u8 = 1 << 3;
pub const USB_ERRSTAT_CRC16: u8 = 1 << 2;
pub const USB_ERRSTAT_CRC5EOF: u8 = 1 << 1;
pub const USB_ERRSTAT_PIDERR: u8 = 1 << 0;

pub const USB_ERREN_BTSERREN: u8 = 1 << 7;
pub const USB_ERREN_DMAERREN: u8 = 1 << 5;
pub const USB_ERREN_BTOERREN: u8 = 1 << 4;
pub const USB_ERREN_DFN8EN: u8 = 1 << 3;
pub const USB_ERREN_CRC16EN: u8 = 1 << 2;
pub const USB_ERREN_CRC5EOFEN: u8 = 1 << 1;
pub const USB_ERREN_PIDERREN: u8 = 1 << 0;

pub const USB_STAT_TX: u8 = 1 << 3;
pub const USB_STAT_ODD: u8 = 1 << 2;

pub const USB_ENDPT_HOSTWOHUB: u8 = 1 << 7;
pub const USB_ENDPT_RETRYDIS: u8 = 1 << 6;
pub const USB_ENDPT_EPCTLDIS: u8 = 1 << 4;
pub const USB_ENDPT_EPRXEN: u8 = 1 << 3;
pub const USB_ENDPT_EPTXEN: u8 = 1 << 2;
pub const USB_ENDPT_EPSTALL: u8 = 1 << 1;
pub const USB_ENDPT_EPHSHK: u8 = 1 << 0;

pub const USB_USBCTRL_SUSP: u8 = 1 << 7;
pub const USB_USBCTRL_PDE: u8 = 1 << 6;

pub const USB_CONTROL_DPPULLUPNONOTG: u8 = 1 << 4;

pub const USB_USBTRC0_RESERVED: u8 = 1 << 6;

pub const USB_USBTRC0_USBRESET: u8 = 1 << 7;
pub const USB_USBTRC0_USBRESMEN: u8 = 1 << 5;
pub const USB_USBTRC0_SYNC_DET: u8 = 1 << 1;
pub const USB_USBTRC0_USB_RESUME_INT: u8 = 1 << 0;

static mut TICKS: u32 = 0;

pub unsafe extern fn reset_isr() {
    SIM_COPC.set(0);

    SIM_SCGC4.set(SIM_SCGC4_SPI0 | SIM_SCGC4_USBOTG);
    SIM_SCGC5.set(SIM_SCGC5_PORTA | SIM_SCGC5_PORTB | SIM_SCGC5_PORTC |
                  SIM_SCGC5_PORTD | SIM_SCGC5_PORTE);

    SIM_SCGC6.set(SIM_SCGC6_ADC0 | SIM_SCGC6_TPM0 | SIM_SCGC6_TPM1 |
                  SIM_SCGC6_TPM2 | SIM_SCGC6_FTF | SIM_SCGC6_DMAMUX);

    if (PMC_REGSC.get() & PMC_REGSC_ACKISO) != 0 {
        PMC_REGSC.set(PMC_REGSC_ACKISO);
    }


    OSC0_CR.set(OSC_CR_ERCLKEN);

    MCG_C2.set(MCG_C2_EREFS0 | (2 << 4));
    MCG_C1.set((2 << 6) | (4 << 3));

    while (MCG_S.get() & MCG_S_OSCINIT0) == 0 { }
    while (MCG_S.get() & MCG_S_IREFST) != 0 { }
    while (MCG_S.get() & (3 << 2)) != (2 << 2) { }

    MCG_C5.set(3);
    MCG_C6.set(MCG_C6_PLLS | 0);

    while (MCG_S.get() & MCG_S_PLLST) == 0 { }
    while (MCG_S.get() & MCG_S_LOCK0) == 0 { }

    SIM_CLKDIV1.set((1 << 28) | (1 << 16));

    MCG_C1.set(4 << 3);

    while (MCG_S.get() & (3 << 2)) != (3 << 2) { }

    SIM_SOPT2.set(SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | (6 << 5) | (1 << 26) | (1 << 24));

    SYST_RVR.set(48000 - 1);
    SYST_CVR.set(0);
    SYST_CSR.set(SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE);

    ptr::copy_nonoverlapping(&__text_end, &mut __data_begin,
                             mem::transmute::<_, usize>(&__data_end) -
                             mem::transmute::<_, usize>(&__data_begin));
    ptr::write_bytes(&mut __bss_begin, 0,
                     mem::transmute::<_, usize>(&__bss_end) -
                     mem::transmute::<_, usize>(&__bss_begin));

    asm!("CPSIE i");

    main();

    loop { }
}

unsafe extern fn unimplemented_isr() {
    loop { }
}

#[allow(unused_assignments)]
unsafe extern fn hardfault_isr() {
    let mut sp: *const usize = ptr::null();

    asm!("mov $0, sp" : "=r"(sp));

    println!("HARDFAULT");
    println!("{:08x}", sp as usize);
    for i in 0..16 {
        println!("{:08x}", *sp.offset(i) as usize);
    }
    loop { }
}

unsafe extern fn systick_isr() {
    TICKS += 1;
}

fn get_ticks() -> u32 {
    unsafe { ptr::read_volatile(&TICKS) }
}

fn delay(time: u32) {
    let start = get_ticks();
    while (get_ticks() - start) < time {
        unsafe { asm!("WFI") };
    }
}

struct Uart1Writer;

use core::fmt::{Write, Error};
impl Write for Uart1Writer {
    fn write_str(&mut self, s: &str) -> Result<(), Error> {
        unsafe {
            for byte in s.as_bytes() {
                while (UART1.s1.get() & UART_S1_TDRE) == 0 { }
                UART1.d.set(*byte);
            }
        }
        Ok(())
    }
}

#[inline(never)]
unsafe fn main() {
    // Setup LEDs and H bridge
    GPIOB.pddr.set((1 << 19) | (1 << 18) | (1 << 3));
    PORTB.pcr[3].set(PORT_PCR_GPIO | PORT_PCR_DSE | PORT_PCR_SRE);
    PORTB.pcr[18].set(PORT_PCR_GPIO | PORT_PCR_DSE | PORT_PCR_SRE);
    PORTB.pcr[19].set(PORT_PCR_GPIO | PORT_PCR_DSE | PORT_PCR_SRE);
    GPIOB.psor.set(1 << 3);

    SIM_SCGC4.set(SIM_SCGC4.get() | SIM_SCGC4_UART1);

    // UART1 RX
    PORTE.pcr[1].set((3 << 8) | PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE);
    // UART1 TX
    PORTE.pcr[0].set((3 << 8) | PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_DSE | PORT_PCR_SRE);

    // SPI0
    PORTC.pcr[4].set((2 << 8) | PORT_PCR_DSE);
    PORTC.pcr[5].set((2 << 8) | PORT_PCR_DSE);
    PORTC.pcr[6].set((2 << 8) | PORT_PCR_DSE);

    UART1.bdh.set(0);
    UART1.bdl.set(13);

    UART1.c2.set(UART_C2_TE | UART_C2_RE);

    delay(100);

    println!("Ready!");

    // Fire up filament power supply
    GPIOD.pddr.set(1 << 3);
    PORTD.pcr[3].set(PORT_PCR_GPIO | PORT_PCR_DSE | PORT_PCR_SRE);
    GPIOD.psor.set(1 << 3);
    delay(100);

    // Setup H bridge
    GPIOB.pcor.set((1 << 19) | (1 << 18));
    GPIOB.psor.set(1 << 19);
    delay(100);

    // Fire up anode/grid power supply
    GPIOA.pddr.set(1 << 1);
    PORTA.pcr[1].set(PORT_PCR_GPIO | PORT_PCR_DSE | PORT_PCR_SRE);
    GPIOA.psor.set(1 << 1);

    // Setup SPI

    loop {
        GPIOB.pcor.set(1 << 3);
        delay(200);
        GPIOB.psor.set(1 << 3);
        delay(200);
    }
}
