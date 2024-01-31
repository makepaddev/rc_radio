#![no_std]
#![no_main]

use hal::{IO, clock::ClockControl, peripherals::Peripherals, prelude::*, systimer::SystemTimer, Rng, Delay, ledc, ledc::LEDC};

use embedded_io::*;
use embedded_svc::{
    ipv4::Interface,
    wifi::{AccessPointInfo, AuthMethod, ClientConfiguration, Configuration, Wifi},
};

use esp_backtrace as _;
use esp_println::{print, println};
use esp_wifi::{
    current_millis, initialize, wifi_set_log_verbose,
    wifi::{utils::create_network_interface, WifiError, WifiStaDevice},
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use smoltcp::{
    socket::udp::PacketMetadata,
    iface::SocketStorage,
    
};

const SSID: &str = "xxx";
const PASSWORD: &str = "xxx";
const STATIC_IP: [u8;4] = [10,0,0,201];
const GATEWAY_IP: [u8;4] = [10,0,0,1];
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut front_wheel = io.pins.gpio3.into_push_pull_output();
    let mut rear_wheel = io.pins.gpio4.into_push_pull_output();

    // Set clocks at maximum frequency
    let clocks = ClockControl::max(system.clock_control).freeze();
    
    let mut ledc = LEDC::new(peripherals.LEDC, &clocks);
    ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);
    
    let mut lstimer0 = ledc.get_timer::<ledc::LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty14Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: 50u32.Hz(),
        })
        .unwrap();
    
    let mut front_wheel_chan = ledc.get_channel(ledc::channel::Number::Channel0, &mut front_wheel);
    front_wheel_chan
        .configure(ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: 8,
            pin_config: ledc::channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut rear_wheel_chan = ledc.get_channel(ledc::channel::Number::Channel0, &mut rear_wheel);
    rear_wheel_chan
        .configure(ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: 8,
            pin_config: ledc::channel::config::PinConfig::PushPull,
        })
        .unwrap();

    front_wheel_chan.set_duty_hw(1350);
    rear_wheel_chan.set_duty_hw(1350);

    // Initialize the timers used for Wifi
    // ANCHOR: wifi_init
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
   
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();
    //wifi_set_log_verbose();
    // ANCHOR_END: wifi_init

    // Configure Wifi
    // ANCHOR: wifi_config
    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let mut wifi_stack = WifiStack::new(iface, device, sockets, current_millis);
    // ANCHOR_END: wifi_config

    let mut auth_method = AuthMethod::WPA2Personal;
    let mut channel = None;
    if PASSWORD.is_empty() {
        auth_method = AuthMethod::None;
        channel = Some(6);
    }
    // ANCHOR: client_config_start
    let client_config = Configuration::Client(ClientConfiguration {
        // ANCHOR_END: client_config_start
        ssid: SSID.into(),
        password: PASSWORD.into(),
        auth_method,
        channel,
        ..Default::default() // ANCHOR: client_config_end
    });

    let res = controller.set_configuration(&client_config);
    println!("Wi-Fi set_configuration returned {:?}", res);
    // ANCHOR_END: client_config_end

    // ANCHOR: wifi_connect
    controller.start().unwrap();
    println!("Is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            //println!("{:?}", ap);
        }
    }

    println!("{:?}", controller.get_capabilities());
    println!("Wi-Fi connect: {:?}", controller.connect());

    // Wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());
    // ANCHOR_END: wifi_connect

    // ANCHOR: ip
    // Wait for getting an ip address

    wifi_stack
    .set_iface_configuration(&embedded_svc::ipv4::Configuration::Client(
        embedded_svc::ipv4::ClientConfiguration::Fixed(embedded_svc::ipv4::ClientSettings {
            ip: embedded_svc::ipv4::Ipv4Addr::from(STATIC_IP),
            subnet: embedded_svc::ipv4::Subnet {
                gateway: embedded_svc::ipv4::Ipv4Addr::from(GATEWAY_IP),
                mask: embedded_svc::ipv4::Mask(24),
            },
            dns: None,
            secondary_dns: None,
        }),
    ))
    .unwrap();
/*
    println!("Wait to get an ip address");
    loop {
        √.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }*/
    // ANCHOR_END: ip

    println!("Start busy loop on main");

    let mut rx_buffer = [0u8; 30000];
    let mut tx_buffer = [0u8; 30000];
    let mut rx_meta = [PacketMetadata::EMPTY;50];
    let mut tx_meta = [PacketMetadata::EMPTY;50];
    // lets get an UDP socket
    let mut socket = wifi_stack.get_udp_socket(&mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    
    //rear_wheel_chan.set_duty_hw(1500);

    socket.work();
    socket.bind(44441).unwrap();
    let mut delay = Delay::new(&clocks);
    loop {
        let mut buffer = [0u8; 4];
        while let Ok(len) = socket.receive(&mut buffer) {
            println!("GOT PACKET {} {:?}", buffer[0], len);
            let input = buffer[0] as f32 / 255.0;
            rear_wheel_chan.set_duty_hw(1000 + (700.0 * input) as u32);
            //rear_wheel_chan.set_duty_hw(value as u32);
            //channel0.set_duty_hw(value as u32);
        }
        let wait_end = current_millis() + 30;
        while current_millis() < wait_end {
            socket.work();
            delay.delay_ms(5u32);
        }
    }
/*-
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    loop {
        println!("Making HTTP request");
        socket.work();

        socket
            .open(IpAddress::Ipv4(Ipv4Address::new(142, 250, 185, 115)), 80)
            .unwrap();

        socket
            .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
            .unwrap();
        socket.flush().unwrap();

        // ANCHOR: reponse
        let wait_end = current_millis() + 20 * 1000;
        loop {
            let mut buffer = [0u8; 512];
            if let Ok(len) = socket.read(&mut buffer) {
                let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
                print!("{}", to_print);
            } else {
                break;
            }

            if current_millis() > wait_end {
                println!("Timeout");
                break;
            }
        }
        println!();
        // ANCHOR_END: reponse

        // ANCHOR: socket_close
        socket.disconnect();

        let wait_end = current_millis() + 5 * 1000;
        while current_millis() < wait_end {
            socket.work();
        }
        // ANCHOR_END: socket_close
    }
    */
}
