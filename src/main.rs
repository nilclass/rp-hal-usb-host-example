#![no_std]
#![no_main]

use defmt as _;
use defmt_rtt as _;
use panic_probe as _;

mod descriptor_info;

#[rtic::app(
    device = rp_pico::hal::pac, dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use usbh::{
        UsbHost,
        PollResult,
        types::{DeviceAddress, DescriptorType, SetupPacket},
        driver::Driver,
    };

    use usb_device::{UsbDirection, control::{Recipient, RequestType}};

    use super::descriptor_info::{print_device_descriptor, print_config_descriptor};

    use embedded_hal::digital::v2::OutputPin;

    use rp_pico::hal::{
        self,
        clocks,
        watchdog::Watchdog,
        usb::host::UsbHostBus,
        Clock,
    };
    use rp_pico::XOSC_CRYSTAL_FREQ;

    use rp2040_monotonic::{fugit::ExtU64, Rp2040Monotonic};

    use defmt::info;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    pub struct BootKeyboardDriver {
        state: KeyboardState,
        delay: cortex_m::delay::Delay,
    }

    enum KeyboardState {
        // no USB device attached
        Detached,
        // device attached, but not detected yet
        Attached,
        // got device descriptor, now getting size for configuration descriptor
        GetConfigSize,
        // got config desc size, getting configuration
        GetConfigDesc,
        // detected USB keyboard, setting configuration
        Detected,
        // Device configuration selected, setting protocol
        Configured0,
        // Protocol set, setting idle
        Configured1,
        // Blink blink
        Blink,
        // Blink done.
        Ready(bool),
        Wait,
        // detected other device (not a keyboard)
        OtherDevice,
    }

    impl BootKeyboardDriver {
        fn new(delay: cortex_m::delay::Delay) -> Self {
            Self {
                state: KeyboardState::Detached,
                delay,
            }
        }
    }

    impl<B: usbh::bus::HostBus> usbh::driver::Driver<B> for BootKeyboardDriver {
        fn attached(&mut self, device_address: DeviceAddress, host: &mut UsbHost<B>) {
            info!("[BootKeyboardDriver] New device attached, with address {}", device_address);
            self.state = KeyboardState::Attached;
            host.get_descriptor(Some(device_address), Recipient::Device, DescriptorType::Device, 18).unwrap();
        }

        fn detached(&mut self, device_address: DeviceAddress) {
            info!("[BootKeyboardDriver] USB device detached");
            self.state = KeyboardState::Detached;
        }

        fn transfer_in_complete(&mut self, device_address: DeviceAddress, length: usize, host: &mut UsbHost<B>) {
            info!("[BootKeyboardDriver] IN transfer complete");
            let buf = unsafe { host.bus().control_buffer(length) };
            match self.state {
                KeyboardState::Detached => unreachable!(),
                KeyboardState::Attached => {
                    print_device_descriptor(buf);
                    let configuration_count = buf[17];
                    info!("Device has {} configurations (ignoring all but the first for now)", configuration_count);
                    host.get_descriptor(Some(device_address), Recipient::Device, DescriptorType::Configuration, 9);
                    self.state = KeyboardState::GetConfigSize;
                }
                KeyboardState::GetConfigSize => {
                    let size = ((buf[3] as u16) << 8) | buf[2] as u16;
                    host.get_descriptor(Some(device_address), Recipient::Device, DescriptorType::Configuration, size);
                    self.state = KeyboardState::GetConfigDesc;
                },
                KeyboardState::GetConfigDesc => {
                    print_config_descriptor(buf);
                    // TODO: figure out if this is really a keyboard (check device class, HID descriptor details etc. -- it's all in `buf`).
                    self.state = KeyboardState::Detected;
                    info!("[BootKeyboardDriver] Keyboard detected, setting configuration");
                    host.set_configuration(device_address, 1);
                },
                KeyboardState::Ready(_) => {
                    info!("Got report: {}", buf);
                    host.bus().enable_sof();
                    self.delay.delay_ms(250);
                    host.control_in(Some(device_address), SetupPacket::new(
                        UsbDirection::In,
                        RequestType::Class,
                        Recipient::Interface,
                        0x01, // GetReport
                        1 << 8, // input report
                        0,
                        8,
                    ), 8);
                }
                _ => {},
            }
        }

        fn transfer_out_complete(&mut self, device_address: DeviceAddress, host: &mut UsbHost<B>) {
            info!("[BootKeyboardDriver] OUT transfer complete");
            match self.state {
                KeyboardState::Detected => {
                    info!("[BootKeyboardDriver] Set configuration. Now selecting protocol");
                    self.state = KeyboardState::Configured0;
                    host.control_out(Some(device_address), SetupPacket::new(
                        UsbDirection::Out,
                        RequestType::Class,
                        Recipient::Interface,
                        0x0b, // SetProtocol,
                        0, // Boot protocol,
                        0,
                        0,
                    ), &[]);
                }
                KeyboardState::Configured0 => {
                    self.state = KeyboardState::Configured1;
                    host.control_out(Some(device_address), SetupPacket::new(
                        UsbDirection::Out,
                        RequestType::Class,
                        Recipient::Interface,
                        0x0a, // SetIdle
                        0,
                        0,
                        0,
                    ), &[]);
                },
                KeyboardState::Configured1 => {
                    self.state = KeyboardState::Blink;
                    info!("[BootKeyboardDriver] Configured1. Should be able to receive stuff now. Blinking once to confirm");
                    host.control_out(Some(device_address), SetupPacket::new(
                        UsbDirection::Out,
                        RequestType::Class,
                        Recipient::Interface,
                        0x09, // SetReport
                        2 << 8, // 2 == output report
                        0,
                        1,
                    ), &[0b111 /* all "lock" LEDs on */]);
                },
                KeyboardState::Blink => {
                    host.control_out(Some(device_address), SetupPacket::new(
                        UsbDirection::Out,
                        RequestType::Class,
                        Recipient::Interface,
                        0x09, // SetReport
                        2 << 8, // 2 == output report
                        0,
                        1,
                    ), &[0b001, /* only NUM lock on */]);
                    self.state = KeyboardState::Ready(true);
                }
                KeyboardState::Ready(pid) => {
                    info!("[BootKeyboardDriver] Ready. Waiting for keypresses.");
                    let result = host.bus().create_interrupt_pipe(device_address, 0x1, 8, 20); // let's assume endpoint 1 is correct
                    info!("RESULT {}", result);
                    host.bus().dump_dpram();
                    // host.bus().enable_sof();
                    // cortex_m::asm::delay(100000);
                    //defmt::debug!("EXPECTING PID{}", if pid { 1 } else { 0 });
                    // host.interrupt_in(device_address, 0x1, 8, pid);
                    self.state = KeyboardState::Ready(!pid);
                    // host.control_in(Some(device_address), SetupPacket::new(
                    //     UsbDirection::In,
                    //     RequestType::Class,
                    //     Recipient::Interface,
                    //     0x01, // GetReport
                    //     1 << 8, // input report
                    //     0,
                    //     8,
                    // ), 8);
                }
                _ => {}
            }
        }

        fn pipe_event(&mut self, device_address: DeviceAddress, buf: &[u8]) {
            info!("Input Report: {}", buf);
        }

        fn interrupt_in_complete(&mut self, device_address: DeviceAddress, length: usize, host: &mut UsbHost<B>) {
            let KeyboardState::Ready(pid) = self.state else {
                panic!()
            };

            let next_pid = if host.bus().received_len() > 0 {
                let buf = unsafe { host.bus().control_buffer(length) };
                info!("Input Report: {}", buf);
                !pid
            } else {
                pid
            };
            host.bus().enable_sof();
            self.delay.delay_ms(100);
            host.interrupt_in(device_address, 0x1, 8, next_pid);
            self.state = KeyboardState::Ready(next_pid);
        }

        fn interrupt_out_complete(&mut self, dev_addr: DeviceAddress, host: &mut UsbHost<B>) {
            unreachable!()
        }
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        usb_host: UsbHost<UsbHostBus>,
        driver: BootKeyboardDriver,
        pin: hal::gpio::Pin<
            hal::gpio::bank0::Gpio2,
            hal::gpio::FunctionSioOutput,
            hal::gpio::PullDown,
            >,
    }

    // Local resources go here
    #[local]
    struct Local {
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

        let delay = cortex_m::delay::Delay::new(ctx.core.SYST, clocks.system_clock.freq().to_Hz());

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
                usb_host,
                driver: BootKeyboardDriver::new(delay),
                pin,
            },
            Local {
                // Initialization of local resources go here
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

    fn poll_usb_host(usb_host: &mut UsbHost<UsbHostBus>, driver: &mut BootKeyboardDriver, from_delay_task: bool) {
        match usb_host.poll(from_delay_task, driver) {
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
            PollResult::PollAgain(delay) => {
                // The host requested to be polled again after the specified delay.
                // This is currently used to implement delays during the enumeration phase.
                poll_usb_task::spawn_after((delay.to_millis() as u64).millis()).ok();
            }
        }
    }

    #[task(binds = USBCTRL_IRQ, shared = [usb_host, driver, pin])]
    fn usbctrl_irq(mut ctx: usbctrl_irq::Context) {
        (&mut ctx.shared.usb_host, &mut ctx.shared.driver, &mut ctx.shared.pin).lock(|usb_host, driver, pin| {
            pin.set_high();
            poll_usb_host(usb_host, driver, false);
            pin.set_low();
        });
    }

    #[task(shared = [usb_host, driver, pin])]
    fn poll_usb_task(mut ctx: poll_usb_task::Context) {
        (&mut ctx.shared.usb_host, &mut ctx.shared.driver, &mut ctx.shared.pin).lock(|usb_host, driver, pin| {
            pin.set_high();
            poll_usb_host(usb_host, driver, true);
            pin.set_low();
        });
    }
}
