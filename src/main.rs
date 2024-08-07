#![no_std]
#![no_main]

use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Pull};
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{Config, DataBits, InterruptHandler as UARTInterruptHandler, Parity, StopBits, Uart};
use embassy_time::{with_timeout, Duration};
use heapless::Vec;
use {defmt_rtt as _, panic_probe as _};

use r503::{Color, Identifier, Instruction, LightPattern};

bind_interrupts!(pub struct Irqs {
    UART0_IRQ  => UARTInterruptHandler<UART0>;
});

const START: u16 = 0xEF01;
const ADDRESS: u32 = 0xFFFFFFFF;

// Checksum is calculated on 'length (2 bytes) + data (??)'.
fn compute_checksum(buf: Vec<u8, 256>) -> u16 {
    let mut checksum = 0u16;

    let check_end = buf.len();
    let checked_bytes = &buf[6..check_end];
    for byte in checked_bytes {
        checksum += (*byte) as u16;
    }
    return checksum;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Start");

    let p = embassy_rp::init(Default::default());

    // Initialize the fingerprint scanner.
    let mut config  = Config::default();
    config.baudrate         = 57600;
    config.stop_bits        = StopBits::STOP1;
    config.data_bits        = DataBits::DataBits8;
    config.parity           = Parity::ParityNone;

    let mut wakeup = Input::new(p.PIN_18, Pull::Down);

    let (
        mut tx,
        mut rx
    ) = Uart::new(
        p.UART0,
        p.PIN_16,
        p.PIN_17,
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        config
    ).split();

    loop {
        let _ = wakeup.wait_for_any_edge().await;
        let color = match wakeup.get_level() {
            Level::Low  => Color::Red,
            Level::High => Color::Blue,
        };

        let mut data_buf: Vec<u8, 32> = heapless::Vec::new();

        // Set the data first, because the length is dependent on that.
        // However, we write the length bits before we do the data.
        data_buf.clear();
        let _ = data_buf.push(LightPattern::AlwaysOn.into());   // Breathing Light
        let _ = data_buf.push(0x00);                            // 0 = Very Fast, 255 = Very Slow; Max Time = 5 Seconds.
        let _ = data_buf.push(color.clone().into());            // colour=Red, Blue, Purple
        let _ = data_buf.push(0x00);                            // times=Infinite

        let send_buf = send(
            Identifier::Command,
            Instruction::AuraLedConfig,
            data_buf
        );

        // Send command buffer.
        let data_write: [u8; 16] = send_buf.clone().into_array().unwrap();
        debug!("W={=[u8]:02X}", data_write[..]);
        match tx.write(&data_write).await {
            Ok(..) => (), // No News is Good News.
            Err(e) => error!("Write error: {:?}", e),
        }

        // Read command buffer.
        let mut read_buf: [u8; 1] = [0; 1]; // Can only read one byte at a time!
        let mut data_read: Vec<u8, 64> = heapless::Vec::new(); // Save buffer.

        loop {
            match with_timeout(Duration::from_millis(500), rx.read(&mut read_buf)).await {
                Ok(..) => {
                    let _ = data_read.push(read_buf[0]).unwrap();
                }
                Err(..) => break,
            }
        }
        debug!("R={=[u8]:02X}", data_read[..]);

        if color == Color::Red {
            let mut data_buf: Vec<u8, 32> = heapless::Vec::new();
            data_buf.clear();
            let _ = data_buf.push(LightPattern::AlwaysOn.into());   // Breathing Light
            let _ = data_buf.push(0x00);                            // 0 = Very Fast, 255 = Very Slow; Max Time = 5 Seconds.
            let _ = data_buf.push(Color::Green.into());             // colour=Red, Blue, Purple
            let _ = data_buf.push(0x00);                            // times=Infinite
            let send_buf = send(
                Identifier::Command,
                Instruction::AuraLedConfig,
                data_buf
            );
            let data_write: [u8; 16] = send_buf.clone().into_array().unwrap();
            match tx.write(&data_write).await {
                Ok(..) => (), // No News is Good News.
                Err(e) => error!("Write error: {:?}", e),
            }
        }

    }
}

fn send(pid: Identifier, command: Instruction, data: Vec<u8, 32>) -> Vec<u8, 256> {
    let mut send_buf: Vec<u8, 256> = heapless::Vec::new();

    // Start    2 bytes Fixed value of 0xEF01; High byte transferred first.
    let _ = send_buf.extend_from_slice(&START.to_be_bytes()[..]);

    // Address  4 bytes Default value is 0xFFFFFFFF, which can be modified by command.
    //                  High byte transferred first and at wrong adder value, module
    //                  will reject to transfer.
    let _ = send_buf.extend_from_slice(&ADDRESS.to_be_bytes()[..]);

    // PID      1 byte  01H Command packet;
    //                  02H Data packet; Data packet shall not appear alone in executing
    //                      processs, must follow command packet or acknowledge packet.
    //                  07H Acknowledge packet;
    //                  08H End of Data packet.
    let _ = send_buf.extend_from_slice(&[pid.into()]);

    // LENGTH   2 bytes Refers to the length of package content (command packets and data packets)
    //                  plus the length of Checksum (2 bytes). Unit is byte. Max length is 256 bytes.
    //                  And high byte is transferred first.
    let len: u16 = (1 + data.len() + 2).try_into().unwrap();
    let _ = send_buf.extend_from_slice(&len.to_be_bytes()[..]);

    // DATA     -       It can be commands, data, command’s parameters, acknowledge result, etc.
    //                  (fingerprint character value, template are all deemed as data);

    // DATA
    let _ = send_buf.push(command.into());

    // DATA
    let _ = send_buf.extend_from_slice(&data);

    // SUM      2 bytes The arithmetic sum of package identifier, package length and all package
    //                  contens. Overflowing bits are omitted. high byte is transferred first.
    let chk = compute_checksum(send_buf.clone());
    let _ = send_buf.extend_from_slice(&chk.to_be_bytes()[..]);

    send_buf
}
