//! USB host support example, using the keyboard driver
//!
//! This example illustrates initializing the USB host stack, and using
//! the keyboard driver.
//!
//! It will log a lot of things via defmt (including keypresses).
//!
//! It also interprets pressing the NumLock key, by toggling the NumLock LED.
//!
#![no_std]
#![no_main]

use defmt as _;
use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(
    device = rp_pico::hal::pac, dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use usbh::{
        driver::kbd::{KbdDriver, KbdEvent, KbdLed},
        driver::log::{EventMask, LogDriver},
        PollResult, UsbHost,
    };

    use embedded_hal::digital::v2::OutputPin;

    use rp_pico::hal::{self, clocks, usb::host::UsbHostBus, watchdog::Watchdog};
    use rp_pico::XOSC_CRYSTAL_FREQ;

    use defmt::info;

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        usb_host: UsbHost<UsbHostBus>,
        log_driver: LogDriver,
        kbd_driver: KbdDriver,
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

        // Create UsbHost instance. Conceptually the `UsbHost` is not specific to any host implementation.
        let usb_host = UsbHost::new(
            // UsbHostBus here is the `usbh::bus::HostBus` implementation for the rp2040.
            UsbHostBus::new(
                ctx.device.USBCTRL_REGS,
                ctx.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                &mut ctx.device.RESETS,
            ),
        );

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                usb_host,
                kbd_driver: KbdDriver::new(),
                log_driver: LogDriver::new(EventMask::all()),
                pin,
            },
            init::Monotonics(),
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

    #[task(binds = USBCTRL_IRQ, local = [usb_host, kbd_driver, log_driver, pin, num_state: bool = false])]
    fn usbctrl_irq(ctx: usbctrl_irq::Context) {
        _ = ctx.local.pin.set_high();

        match ctx
            .local
            .usb_host
            .poll(&mut [ctx.local.log_driver, ctx.local.kbd_driver])
        {
            PollResult::NoDevice => {
                // No device is attached (yet)
                return;
            }
            PollResult::Busy => {
                // Bus is currently busy communicating with the device
            }
            PollResult::Idle => {
                // Bus is idle, we can issue commands via the host interface
            }

            PollResult::BusError(error) => {
                defmt::error!("Bus error: {}", error);
            }

            PollResult::DiscoveryError(dev_addr) => {
                defmt::error!("Discovery for device {} failed", dev_addr);
            }

            _ => {}
        }

        match ctx.local.kbd_driver.take_event() {
            None => {}
            Some(event) => {
                match event {
                    KbdEvent::DeviceAdded(dev_addr) => {
                        info!("Keyboard with address {} added", dev_addr);
                        ctx.local
                            .kbd_driver
                            .set_idle(dev_addr, 0, ctx.local.usb_host)
                            .ok()
                            .unwrap();
                    }
                    KbdEvent::DeviceRemoved(dev_addr) => {
                        info!("Keyboard with address {} removed", dev_addr);
                    }
                    KbdEvent::InputChanged(dev_addr, report) => {
                        info!("Input on keyboard {}:\n  {}", dev_addr, report);

                        // toggle Num lock LED when NumLock key is pressed
                        if report.pressed_keys().find(|key| *key == 83).is_some() {
                            *ctx.local.num_state = !*ctx.local.num_state;
                            ctx.local
                                .kbd_driver
                                .set_led(
                                    dev_addr,
                                    KbdLed::NumLock,
                                    *ctx.local.num_state,
                                    ctx.local.usb_host,
                                )
                                .ok()
                                .unwrap();
                        }
                    }
                    _ => {}
                }
            }
        }

        _ = ctx.local.pin.set_low();
    }
}
