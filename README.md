# Rust for puck.js

The [puck.js](https://www.espruino.com/Puck.js) is a cute nRF52832 form factor available from Espruino.

## Prerequisites

You'll need to hook up to a debugger or the [programming jig](https://shop.espruino.com/puckjs-jig) and an [nrf52 dk board](https://www.nordicsemi.com/Products/Development-hardware/nRF52-DK#Downloads) or similar.

Embassy requires a specific version of nightly. You won't have to do anything though as the rust-toolchain file will use the correct versions when you build.

- Install any dependencies and [cargo-flash](https://github.com/probe-rs/cargo-flash#prerequisites)
- Install probe-rs-cli `cargo install probe-rs-cli`
- On linux you need the following udev rules saved to somewhere like /etc/udev/rules.d/50-cmsis-dap.rules and then reload your udev rules with something like `sudo udevadm control -R`

```bash
# 0d28:0204 DAPLink
SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", ATTR{idProduct}=="0204", MODE:="666"
```

Finally we'll need a softdevice

- Download [SoftDevice S132](https://www.nordicsemi.com/Products/Development-software/S132/Download) from Nordic. Supported versions are 7.x.x
- `probe-rs-cli download --format hex s132_nrf52_7.3.0_softdevice.hex --chip nRF52832_xxAA --chip-erase`

## Flashing

Optionally, when you want to flash the chip so it can work stand alone we use [cargo-flash](https://github.com/probe-rs/cargo-flash#prerequisites) with `cargo flash --release --chip nRF52832_xxAA`

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
