//! Prints the output devices, calibration inputs and the default output as seen by the adapter.
//!
//! `cargo run -p wye-mac --example devices [--virtual] [--watch SECS]`
//! (`WYE_SHOW_VIRTUAL=1` also shows virtual devices such as BlackHole.)

#[cfg(target_os = "macos")]
fn main() {
    use wye_mac::MacPlatform;
    use wye_platform::{DeviceInfo, Platform};

    let args: Vec<String> = std::env::args().collect();
    let p = match MacPlatform::new() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    };
    if args.iter().any(|a| a == "--virtual") {
        p.set_show_virtual(true);
    }
    let def = p.default_output().ok().flatten();
    let row = |d: &DeviceInfo| {
        let mut flags = Vec::new();
        if Some(&d.id) == def.as_ref() {
            flags.push("DEFAULT");
        }
        if d.busy {
            flags.push("BUSY");
        }
        if !d.can_be_default {
            flags.push("no-default");
        }
        println!(
            "  {:<34} uid={:<48} {:?} {:?} rate={} ch={} buf={} after={:.2} ms total={:.2} ms {}",
            format!("{:?}", d.name),
            d.id.0,
            d.transport,
            d.mode,
            d.nominal_rate,
            d.output_channels,
            d.buffer_frames,
            d.reported_latency.after_timestamp.as_secs_f64() * 1e3,
            d.reported_latency.total.as_secs_f64() * 1e3,
            flags.join(" ")
        );
    };
    println!("output devices:");
    for d in p.output_devices().unwrap_or_default() {
        row(&d);
        if let Some((s, m)) = p.output_volume(&d.id) {
            println!("      volume={s:.3} muted={m}");
        }
    }
    println!("calibration inputs (output_channels = input channels):");
    for d in p.calibration_inputs().unwrap_or_default() {
        row(&d);
        if let Ok(l) = p.input_latency(&d.id) {
            println!("      input latency {:.2} ms", l.as_secs_f64() * 1e3);
        }
    }
    println!("default output: {def:?}");
    println!("microphone permission: {:?}", p.microphone_permission());
    println!("capture permission: {:?}", p.capture_permission());

    let secs: u64 = args
        .iter()
        .position(|a| a == "--watch")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    if secs > 0 {
        println!("watching events for {secs} s ...");
        let _g = p
            .watch(Box::new(|ev| println!("event: {ev:?}")))
            .expect("watch");
        std::thread::sleep(std::time::Duration::from_secs(secs));
    }
}

#[cfg(not(target_os = "macos"))]
fn main() {}
