//! Reads our Config.toml and generates userconfig.rs by converting the values to constants

use std::env;
use std::fs;
use std::path::Path;
use core::include;

// So we can use the date for the serial number
use chrono::{DateTime, Utc};

use toml;

// A little hack so we can use our structs from config.rs without having to duplicate them here:
include!(concat!(env!("CARGO_MANIFEST_DIR"), "/src/config_structs.rs"));
// NOTE: config_structs.rs includes the serde import so we don't need it here

fn main() {
    let now: DateTime<Utc> = Utc::now();
    // env::set_var("SERIALNOW", now.to_rfc3339()); // Used with the Riskeyboard firmware serial number
    println!("cargo:rustc-env=SERIALNOW={}", now.timestamp()); // Used with the Riskeyboard firmware serial number
    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("userconfig.rs");
    let mut out = String::new();
    let contents = fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/Config.toml")).unwrap();
    let decoded: Config = toml::from_str(&contents[..]).unwrap();
    // TODO: Figure out a way to iterate over configurable items instead of having them hard coded like this
    for fname in KeyboardConfig::field_names().iter() {
        let meta = &decoded.keyboard.gen_meta_tuple(fname);
        let const_out = format!(
            "pub const {}_{}: {} = {};\n",
            meta.0.to_uppercase().split("CONFIG").next().unwrap(),
            meta.1.to_uppercase(),
            meta.2,
            meta.3);
        out.push_str(&const_out);
    }
    for fname in MouseConfig::field_names().iter() {
        let meta = &decoded.mouse.gen_meta_tuple(fname);
        let const_out = format!(
            "pub const {}_{}: {} = {};\n",
            meta.0.to_uppercase().split("CONFIG").next().unwrap(),
            meta.1.to_uppercase(),
            meta.2,
            meta.3);
        out.push_str(&const_out);
    }
    for fname in LedsConfig::field_names().iter() {
        let meta = &decoded.leds.gen_meta_tuple(fname);
        let const_out = format!(
            "pub const {}_{}: {} = {};\n",
            meta.0.to_uppercase().split("CONFIG").next().unwrap(),
            meta.1.to_uppercase(),
            meta.2,
            meta.3);
        out.push_str(&const_out);
    }
    for fname in EncoderConfig::field_names().iter() {
        let meta = &decoded.encoder.gen_meta_tuple(fname);
        let const_out = format!(
            "pub const {}_{}: {} = {};\n",
            meta.0.to_uppercase().split("CONFIG").next().unwrap(),
            meta.1.to_uppercase(),
            meta.2,
            meta.3);
        out.push_str(&const_out);
    }
    for fname in DisplayConfig::field_names().iter() {
        let meta = &decoded.display.gen_meta_tuple(fname);
        let const_out = format!(
            "pub const {}_{}: {} = {};\n",
            meta.0.to_uppercase().split("CONFIG").next().unwrap(),
            meta.1.to_uppercase(),
            meta.2,
            meta.3);
        out.push_str(&const_out);
    }
    for fname in InfraredConfig::field_names().iter() {
        let meta = &decoded.infrared.gen_meta_tuple(fname);
        let const_out = format!(
            "pub const {}_{}: {} = {};\n",
            meta.0.to_uppercase().split("CONFIG").next().unwrap(),
            meta.1.to_uppercase(),
            meta.2,
            meta.3);
        out.push_str(&const_out);
    }
    for fname in DevConfig::field_names().iter() {
        let meta = &decoded.dev.gen_meta_tuple(fname);
        let const_out = format!(
            "pub const {}_{}: {} = {};\n",
            meta.0.to_uppercase().split("CONFIG").next().unwrap(),
            meta.1.to_uppercase(),
            meta.2,
            meta.3);
        out.push_str(&const_out);
    }
    fs::write(&dest_path, out).unwrap();
    println!("cargo:rerun-if-changed=Config.toml");
    println!("cargo:rerun-if-changed=build.rs");
}
