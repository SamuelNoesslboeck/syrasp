use core::f32::consts::PI;

use clap::{command, arg, value_parser};
use log::{debug, info};
use rppal::gpio::Gpio;
use syact::prelude::*;

// Define distance and max speed defaults
const DELTA_DEF : Delta = Delta(2.0 * PI);
const OMEGA_DEF : Velocity = Velocity(20.0);

#[tokio::main]
async fn main() -> Result<(), syact::Error> {
    // Init logger
        env_logger::init();

        info!("# SYRASP - stepper-fixed_dist");
    // 

    // Parse cmd args
    let matches = command!() 
        .about("Moves a stepper motor with a generic PWM controller connected to the pins 'pin_step' and 'pin_dir' by the given distance 
        'delta' with the maximum speed 'omega', optionally enabling microstepping with the microstepcount 'micro'")
        .arg(arg!([pin_step] "Pin number of the step pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_dir] "Pin number of the direction pin").value_parser(value_parser!(u8)))
        .arg(arg!([delta] "Delta (distance) of the movement in rad (2pi [1 rev] per default)").value_parser(value_parser!(f32)))
        .arg(arg!([omega] "Velocity (velocity) of the movement in rad/s (10 rad/s per default)").value_parser(value_parser!(f32)))
        .get_matches();

    let pin_step : u8 = *matches.get_one("pin_step").expect("A valid step pin has to be provided");
    let pin_dir : u8 = *matches.get_one("pin_dir").expect("A valid direction pin has to be provided");

    let delta : Delta  = Delta(*matches.get_one("delta").unwrap_or(&DELTA_DEF.0));
    let omega : Velocity = Velocity(*matches.get_one("omega").unwrap_or(&OMEGA_DEF.0));

    // Load data
    let inertia = std::env::var("INERTIA").ok().map(|v| v.parse::<Inertia>().unwrap()).unwrap_or(Inertia::ZERO);
    let force = std::env::var("FORCE").ok().map(|v| Force(v.parse::<f32>().unwrap())).unwrap_or(Force::ZERO);
    let micro_opt = std::env::var("MICRO").ok().map(|v| v.parse::<MicroSteps>().unwrap());

    info!("> Parsing data from env done!");

    let gpio = Gpio::new().unwrap();
    info!("> Accessing GPIO done!");

    // Create the controls for a stepper motor
    let mut stepper = Stepper::new(
        GenericPWM::new(
            gpio.get(pin_step).unwrap().into_output(), 
            gpio.get(pin_dir).unwrap().into_output()
        )?, 
        StepperConst::MOT_17HE15_1504S
    ).unwrap();

    // Link the component to a system
    stepper.set_config(StepperConfig { 
        voltage: 12.0,    // System voltage in volts
        overload_current: None
    }); 
    stepper.setup()?;

    if let Some(micro) = micro_opt {
        stepper.set_microsteps(micro);
    }

    // Apply some loads
    stepper.apply_inertia(inertia);
    stepper.apply_gen_force(force)?;

    stepper.set_velocity_max(omega);

    debug!("> Data used: {{ Delta: {}, Omega: {} }}", delta, omega);

    info!("> Starting the movement ... ");
    stepper.drive_rel(delta, Factor::MAX).await?;      
    info!("|  > Movement done!");

    Ok(())
}  