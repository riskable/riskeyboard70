//! Configurable constants

use core::include;

include!(concat!(env!("OUT_DIR"), "/userconfig.rs"));

// This one will never change so it's here instead of in the Config.toml:
pub const IR_SAMPLERATE: u32 = 20_000;

/* Brightness notes:
 * TODO: Make the brightness adjustable on-the-fly
 * These are Rainbow values:
 * At 128 the keeb uses ~916mAu
 * At 110 it's ~854mA
 * At 100 it's ~814mA
 * At 64 it's ~670mA
 * At 32 it's ~560-580mA
 * At 16 it's ~527mA
 * At 8 it's ~516mA
 * At 0 it's 507-541mA (typically ~530mA)
 * WHITE values:
 * 16: ~623mA
 * 32: ~780mA
 * 64: ~1170mA (that escalated fast!)
 * With both USB plugs powered: ~1440mA from the PC USB!
 * Black Pill without anything else: 32mA
 * Riskeypad: 167mA
 * Looks like each 49e hall effect sensor uses about ~5.87mA
*/

