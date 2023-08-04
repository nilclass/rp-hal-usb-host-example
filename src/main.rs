#![no_std]
#![no_main]

use defmt as _;
use defmt_rtt as _;
use panic_probe as _;

// mod descriptor_info;

#[rtic::app(
    device = rp_pico::hal::pac, dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use usbh::{
        UsbHost,
        PollResult,
    };

    use embedded_hal::digital::v2::OutputPin;

    use rp_pico::hal::{
        self,
        clocks,
        watchdog::Watchdog,
        usb::host::UsbHostBus,
    };
    use rp_pico::XOSC_CRYSTAL_FREQ;

    use defmt::info;

    use rp2040_monotonic::Rp2040Monotonic;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    // Shared resources go here
    #[shared]
    struct Shared {
    }

    // Local resources go here
    #[local]
    struct Local {
        usb_host: UsbHost<UsbHostBus>,
        driver: usbh::driver::Kbd,
        pin: hal::gpio::Pin<
            hal::gpio::bank0::Gpio2,
            hal::gpio::FunctionSioOutput,
            hal::gpio::PullDown,
            >,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        unsafe { hal::sio::spinlock_reset() };

        defmt::info!("init");

        // Setup the clock. This is required.
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
            .unwrap();

        let sio = hal::Sio::new(ctx.device.SIO);

        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let pin = pins.gpio2.into_push_pull_output();

        // Setup the monotonic timer
        let mono = Rp2040Monotonic::new(ctx.device.TIMER);

        // Create UsbHost instance. Conceptually the `UsbHost` is not specific to any host implementation.
        let usb_host = UsbHost::new(
            // UsbHostBus here is the `usbh::bus::HostBus` implementation for the rp2040.
            UsbHostBus::new(
                ctx.device.USBCTRL_REGS,
                ctx.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                &mut ctx.device.RESETS
            )
        );

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                usb_host,
                driver: usbh::driver::Kbd::new(),
                pin,
            },
            init::Monotonics(mono),
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    #[task(binds = USBCTRL_IRQ, local = [usb_host, driver, pin])]
    fn usbctrl_irq(ctx: usbctrl_irq::Context) {
        _ = ctx.local.pin.set_high();

        match ctx.local.usb_host.poll(ctx.local.driver) {
            PollResult::NoDevice => {
                // No device is attached (yet)
                return;
            },
            PollResult::Busy => {
                // Bus is currently busy communicating with the device
            },
            PollResult::Idle => {
                // Bus is idle, we can issue commands via the host interface
            },

            PollResult::BusError(error) => {
                defmt::error!("Bus error: {}", error);
            }
        }

        match ctx.local.driver.take_event() {
            None => {},
            Some(event) => {
                use usbh::driver::KbdEvent;
                match event {
                    KbdEvent::DeviceAdded(dev_addr) => {
                        info!("Keyboard with address {} added", dev_addr);
                        ctx.local.driver.set_idle(dev_addr, 0, ctx.local.usb_host);
                    }
                    KbdEvent::DeviceRemoved(dev_addr) => {
                        info!("Keyboard with address {} removed", dev_addr);
                    }
                    KbdEvent::InputChanged(dev_addr, report) => {
                        info!("Input on keyboard {}:\n  {}", dev_addr, report);
                    }
                    _ => {}
                }
            }
        }

        _ = ctx.local.pin.set_low();
    }
}
