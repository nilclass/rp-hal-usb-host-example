
pub fn print_device_descriptor(bytes: &[u8]) -> usize {
    let size = bytes[0];
    if size != bytes.len() as u8 {
        defmt::error!("Invalid device descriptor? bLength={} does not match buffer size {}", size, bytes.len());
    } else if size != 18 {
        defmt::info!("Device descriptor too short ({} bytes), not printing for now.", size);
    } else {
        defmt::info!("Device descriptor:\n - Desc. Type:\t\t{}\n - USB Version:\t\t{}{}.{}.{}\n - Device Class:\t{}\n - vendor/product:\t{:#X}:{:#X}\n - #configurations:\t{}",
                     bytes[1], (bytes[3] & 0xF0) >> 4, bytes[3] & 0x0F, (bytes[2] & 0xF0) >> 4, bytes[2] & 0x0F, bytes[4],
                     ((bytes[9] as u16) << 8) | (bytes[8] as u16),
                     ((bytes[11] as u16) << 8) | (bytes[10] as u16),
                     bytes[17]
        );
    }
    bytes.len()
}

pub fn print_config_descriptor(bytes: &[u8]) -> usize {
    defmt::info!("Config descriptor:\n - Num Interfaces:\t{}\n - Config Value:\t{}\n - Self powered:\t{}\n - Remote wakeup:\t{}\n - Max power (mA):\t{}\n",
                 bytes[4], bytes[5], (bytes[7] >> 6) & 1 == 1, (bytes[7] >> 5) & 1 == 1, bytes[8] * 2
    );
    let mut consumed = 9; // size of config descriptor itself

    loop {
        if bytes.len() > consumed {
            consumed += print_descriptor(&bytes[consumed..]);
        } else {
            break
        }
    }

    consumed
}

fn print_interface_descriptor(bytes: &[u8]) -> usize {
    defmt::info!(
        "Interface descriptor:\n - Interface num:\t{}\n - Alt setting: \t{}\n - Num endpoints:\t{}\n - Interface class:\t{}\n - Iface subclass:\t{}\n - Iface protocol:\t{}\n",
        bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]
    );
    9
}

fn print_endpoint_descriptor(bytes: &[u8]) -> usize {
    defmt::info!(
        "Endpoint descriptor:\n - EP Address:\t{} ({})\n - Transfer type:\t{}\n - Sync type:\t{}\n - Usage type:\t{}\n - Max packet:\t{}\n - Poll interval:\t{}\n",
        bytes[2] & 0b111,
        if (bytes[2] >> 7) & 1 == 1 { "IN" } else { "OUT" },
        match bytes[3] & 0b11 {
            0 => "control",
            1 => "isochronous",
            2 => "bulk",
            3 => "interrupt",
            _ => unreachable!(),
        },
        match (bytes[3] >> 2) & 0b11 {
            0 => "none",
            1 => "asynchronous",
            2 => "adaptive",
            3 => "synchronous",
            _ => unreachable!(),
        },
        match (bytes[3] >> 4) & 0b11 {
            0 => "data",
            1 => "feedback",
            2 => "explicit feedback",
            3 => "(reserved)",
            _ => unreachable!(),
        },
        bytes[4] as u16 | ((bytes[5] as u16) << 8),
        bytes[6]
    );
    7
}

fn print_hid_kbd_descriptor(bytes: &[u8]) -> usize {
    defmt::info!(
        "HID (Keyboard) Descriptor:\n - HID version:\t{}{}.{}.{}\n - Country code:\t{}\n - Report type:\t{:#X}\n - Report length:\t{}\n",
        (bytes[3] & 0xF0) >> 4, bytes[3] & 0x0F, (bytes[2] & 0xF0) >> 4, bytes[2] & 0x0F,
        bytes[4], bytes[6], bytes[7]
    );
    9
}

fn print_descriptor(bytes: &[u8]) -> usize {
    match bytes[1] {
        1 => print_device_descriptor(bytes),
        2 => print_config_descriptor(bytes),
        4 => print_interface_descriptor(bytes),
        5 => print_endpoint_descriptor(bytes),
        0x21 => print_hid_kbd_descriptor(bytes),
        n => {
            defmt::error!("Not implemented: {} ({} bytes)", n, bytes[0]);
            bytes[0] as usize
        }
    }
}

fn print_hid_report(bytes: &[u8]) {
    let mut consumed = 0;
    loop {
        consumed += print_hid_item(&bytes[consumed..]);
        if consumed == bytes.len() {
            return
        }
    }
}

fn print_hid_item(bytes: &[u8]) -> usize {
    if bytes[0] == 0b11111110 {
        // long item
        let size = bytes[1];
        let tag = bytes[2];
        let data = &bytes[3..(3 + size as usize)];
        defmt::info!("HID ITEM (long): size={} tag={} data={}", size, tag, data);
        3 + size as usize
    } else {
        // short item
        let size = bytes[0] & 0b11;

        let type_ = match (bytes[0] >> 2) & 0b11 {
            0 => "main",
            1 => "global",
            2 => "local",
            3 => "reserved",
            _ => unreachable!(),
        };

        let tag = (bytes[0] >> 4) & 0b1111;

        defmt::info!("HID ITEM: size={}, type={}, tag={}, data={}", size, type_, tag, &bytes[1..(1 + size) as usize]);

        match (bytes[0] >> 2) & 0b11 {
            0 => { // MAIN
                if size > 0 {
                    match bytes[1] | 0b11111100 {
                        0b10000000 => defmt::info!("  INPUT, size {}", bytes[1] | 0b11),
                        0b10010000 => defmt::info!("  OUTPUT, size {}", bytes[1] | 0b11),
                        0b10110000 => defmt::info!("  FEATURE, size {}", bytes[1] | 0b11),
                        0b10100000 => defmt::info!("  COLLECTION, size {}", bytes[1] | 0b11),
                        0b11000000 => defmt::info!("  END COLLECTION, size {}", bytes[1] | 0b11),
                        _ => defmt::error!("Not sure how to interpret {:#X}", bytes[1]),
                    }
                }
            },
            1 => { // GLOBAL
            },
            2 => { // LOCAL
            },
            3 => { /* reserved */ }
            _ => unreachable!(),
        }

        1 + size as usize
    }
}
