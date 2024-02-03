#![no_std]
#![no_main]

use embedded_svc::ipv4 as ipv4_svc;
use embedded_svc::wifi as wifi_svc;
use embedded_svc::{wifi::Wifi, ipv4::Interface};
use esp_backtrace as _;
use esp_println::{println};
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::{WifiStaDevice};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, EspWifiInitFor};
use hal::clock::ClockControl;
use hal::{Rng, IO, Delay, i2c::I2C ,peripherals::I2C0, ledc, ledc::LEDC};
use hal::{peripherals::Peripherals,  prelude::*};
use smoltcp::iface::SocketStorage;
use smoltcp::socket::udp::PacketMetadata;
use smoltcp::wire::IpAddress;

const WIFI_SSID: &str = "x";
const WIFI_PASSWORD: &str = "x";
const WIFI_DHCP: bool = false;
const WIFI_STATIC_IP: [u8;4] = [10,0,0,201];
const WIFI_GATEWAY_IP: [u8;4] = [10,0,0,1];
const IMU_TARGET_IP: IpAddress = IpAddress::v4(10,0,0,118);

const FRONT_CENTER: f32 = 950.0;
const FRONT_RANGE: f32 = 600.0;
const REAR_CENTER: f32 = 1300.0;
const REAR_RANGE: f32 = 600.0;
const THROTTLE_CENTER: f32 = 1250.0;
const THROTTLE_RANGE: f32 = 600.0;

#[entry]
fn main() -> ! {
    // System inputs/clocks/etc

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let timer = hal::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let mut ledc = LEDC::new(peripherals.LEDC, &clocks);
    ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);

    // init connection to BNO055 IMU sensor

    let mut i2c = I2C::new_with_timeout(
        peripherals.I2C0,
        io.pins.gpio6,
        io.pins.gpio5,
        50u32.kHz(),
        &clocks,
        Some(20),
    );

    // set up PWMS


    let mut front_wheel_gpio = io.pins.gpio3.into_push_pull_output();
    let mut rear_wheel_gpio = io.pins.gpio4.into_push_pull_output();
    let mut throttle_gpio = io.pins.gpio2.into_push_pull_output();
    let mut lstimer0 = ledc.get_timer::<ledc::LowSpeed>(ledc::timer::Number::Timer0);
       
    lstimer0.configure(ledc::timer::config::Config {
        duty: ledc::timer::config::Duty::Duty14Bit,
        clock_source: ledc::timer::LSClockSource::APBClk,
        frequency: 50u32.Hz(),
    }).unwrap();
            
    let mut front_wheel_chan = ledc.get_channel(ledc::channel::Number::Channel0, &mut front_wheel_gpio);
    front_wheel_chan.configure(ledc::channel::config::Config {
        timer: &lstimer0,
        duty_pct: 8,
        pin_config: ledc::channel::config::PinConfig::PushPull,
    }).unwrap();

    let mut rear_wheel_chan = ledc.get_channel(ledc::channel::Number::Channel1, &mut rear_wheel_gpio);
    rear_wheel_chan.configure(ledc::channel::config::Config {
        timer: &lstimer0,
        duty_pct: 8,
        pin_config: ledc::channel::config::PinConfig::PushPull,
    }).unwrap();

    let mut throttle_chan = ledc.get_channel(ledc::channel::Number::Channel2, &mut throttle_gpio);
    throttle_chan.configure(ledc::channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        pin_config: ledc::channel::config::PinConfig::PushPull,
    }).unwrap();

    front_wheel_chan.set_duty_hw(FRONT_CENTER as u32);
    rear_wheel_chan.set_duty_hw(REAR_CENTER as u32);
    throttle_chan.set_duty_hw(THROTTLE_CENTER as u32);

    // initialise wifi

    let wifi_init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    ).unwrap();

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (wifi_iface, wifi_device, mut wifi_controller, wifi_sockets) =
        create_network_interface(&wifi_init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let mut wifi_stack = WifiStack::new(wifi_iface, wifi_device, wifi_sockets, current_millis);

    let wifi_config = wifi_svc::Configuration::Client(wifi_svc::ClientConfiguration {
        ssid: WIFI_SSID.try_into().unwrap(),
        password: WIFI_PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    let res = wifi_controller.set_configuration(&wifi_config);
    println!("wifi_set_configuration returned {:?}", res);

    wifi_controller.start().unwrap();
    println!("is wifi started: {:?}", wifi_controller.is_started());

    /*
    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<wifi_svc::AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }*/

    println!("{:?}", wifi_controller.get_capabilities());
    println!("wifi_connect {:?}", wifi_controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = wifi_controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("Error: {:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", wifi_controller.is_connected());

    // static IP
    if WIFI_DHCP{
        println!("Wait to get an ip address");
        loop {
            wifi_stack.work();
    
            if wifi_stack.is_iface_up() {
                println!("got ip {:?}", wifi_stack.get_ip_info());
                break;
            }
        }
    }
    else{
        wifi_stack.set_iface_configuration(&ipv4_svc::Configuration::Client(
            ipv4_svc::ClientConfiguration::Fixed(ipv4_svc::ClientSettings {
                ip: ipv4_svc::Ipv4Addr::from(WIFI_STATIC_IP),
                subnet: ipv4_svc::Subnet {
                    gateway: ipv4_svc::Ipv4Addr::from(WIFI_GATEWAY_IP),
                    mask: ipv4_svc::Mask(24),
                },
                dns: None,
                secondary_dns: None,
            }),
        )).unwrap();
    }

    println!("Init BNO055");
    init_bno055(&mut i2c, &mut delay);
    
    println!("Start busy loop on main");
    let mut rx_buffer = [0u8; 2000];
    let mut tx_buffer = [0u8; 2000];
    let mut rx_meta = [PacketMetadata::EMPTY;5];
    let mut tx_meta = [PacketMetadata::EMPTY;5];
    // lets get an UDP socket
    let mut socket = wifi_stack.get_udp_socket(&mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    
    socket.work();
    socket.bind(44441).unwrap();
    let mut wrap_counter = 0;
    loop {
        let mut buffer = [0u8; 3];
        while let Ok(_len) = socket.receive(&mut buffer) {
            rear_wheel_chan.set_duty_hw((REAR_CENTER - REAR_RANGE * 0.5 + (REAR_RANGE * buffer[0] as f32 / 255.0) ) as u32);
            front_wheel_chan.set_duty_hw((FRONT_CENTER - FRONT_RANGE * 0.5 + (FRONT_RANGE * buffer[1] as f32 / 255.0)) as u32);
            throttle_chan.set_duty_hw((THROTTLE_CENTER - THROTTLE_RANGE * 0.5 + (THROTTLE_RANGE * buffer[2] as f32 / 255.0)) as u32);

            if !check_bno055(&mut i2c){
                println!("BNO055 not working");
            }
            else{
                let data = get_bno055_data(&mut i2c, wrap_counter);
                // alright lets send out a udp packet with the data
                println!("SENDING DATA {:?}", data);
                let _ = socket.send(IMU_TARGET_IP, 44442, &data);
                wrap_counter = wrap_counter.wrapping_add(1);
            }
        }
        let wait_end = current_millis() + 10;
        while current_millis() < wait_end {
            socket.work();
        }
    }
}


const BNO055_ADDRESS_A:u8 = 0x28;
const BNO055_ID:u8 = 0xA0;
const BNO055_CHIP_ID_ADDR:u8 = 0x00;
//const BNO055_OPERATION_MODE_NDOF:u8 = 0x0C;
const BNO055_OPERATION_MODE_IMUPLUS:u8 = 0x08;
const BNO055_OPR_MODE_ADDR:u8 = 0x3D;
const BNO055_OPERATION_MODE_CONFIG:u8 = 0x0;
const BNO055_SYS_TRIGGER_ADDR:u8 = 0x3f;
const BNO055_PAGE_ID_ADDR:u8 = 0x07;
const BNO055_POWER_MODE_NORMAL:u8 = 0x00;
const BNO055_PWR_MODE_ADDR:u8 = 0x3E;
//const BNO055_SYS_STAT_ADDR:u8 = 0x39;

const BNO055_GYRO_DATA_X_LSB_ADDR:u8 = 0x14;
const BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR:u8 = 0x28;
const BNO055_ACCEL_DATA_X_LSB_ADDR:u8 = 0x08;
const BNO055_GRAVITY_DATA_X_LSB_ADDR:u8 = 0x2E;

fn read_bytes(i2c:&mut I2C<'_, I2C0>, addr: u8, out:&mut [u8], steps:usize){
    for i in 0..steps{
        let mut data = [0u8;1];
        i2c.write_read(BNO055_ADDRESS_A, &[addr], &mut data).ok();
        out[i] = data[0];
    }
}

fn get_bno055_data(i2c:&mut I2C<'_, I2C0>, order_counter:u8)->[u8;25]{
    let mut out = [0u8; 25];
    let mut c = 0;
    out[c] = order_counter; c+=1;
    read_bytes(i2c, BNO055_GYRO_DATA_X_LSB_ADDR, &mut out[c..], 6); c+=6;
    read_bytes(i2c, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, &mut out[c..], 6); c+=6;
    read_bytes(i2c, BNO055_ACCEL_DATA_X_LSB_ADDR, &mut out[c..], 6); c+=6;
    read_bytes(i2c, BNO055_GRAVITY_DATA_X_LSB_ADDR, &mut out[c..], 6); c+=6;
    out
}

fn check_bno055(i2c:&mut I2C<'_, I2C0>)->bool{
    let mut data = [0u8; 1];
    i2c.write_read(BNO055_ADDRESS_A, &[BNO055_CHIP_ID_ADDR], &mut data).ok();
    data[0] == BNO055_ID
}
/*
fn get_bno055_status(i2c:&mut I2C<'_, I2C0>)->u8{
    let mut data = [0u8; 1];
    i2c.write_read(BNO055_ADDRESS_A, &[BNO055_SYS_STAT_ADDR], &mut data).ok();
    data[0]
}*/

fn init_bno055(i2c:&mut I2C<'_, I2C0>, delay:&mut Delay){
    i2c.write(BNO055_ADDRESS_A, &[BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG]).ok();
    i2c.write(BNO055_ADDRESS_A, &[BNO055_SYS_TRIGGER_ADDR, 0x20]).ok();

    loop{
        let mut data = [0u8; 1];
        i2c.write_read(BNO055_ADDRESS_A, &[BNO055_CHIP_ID_ADDR], &mut data).ok();
        if data[0] == BNO055_ID{
            break
        }
        delay.delay_ms(10u32);
    }
    delay.delay_ms(50u32);

    i2c.write(BNO055_ADDRESS_A, &[BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL]).ok();
    delay.delay_ms(10u32);

    i2c.write(BNO055_ADDRESS_A, &[BNO055_PAGE_ID_ADDR, 0]).ok();
    i2c.write(BNO055_ADDRESS_A, &[BNO055_SYS_TRIGGER_ADDR, 0]).ok();
    delay.delay_ms(10u32);
    i2c.write(BNO055_ADDRESS_A, &[BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_IMUPLUS]).ok();
    delay.delay_ms(20u32);
}