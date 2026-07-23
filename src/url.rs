use crate::simulation::{RandomNBodyParams, RandomSwarmParams, Scenario, SimulationConfig};

// URL <-> SimulationConfig bridge. Plain query string — readable, debuggable,
// no base64/serde deps. Keys map 1:1 to SimulationConfig fields; only non-default
// values are emitted (so a default config round-trips to "").
//
// Scenario names are the discriminant in lowercase with no underscores:
//   centralswarm, dualcircle, trianglecircle, burrau1913, solarsystem,
//   figureeight, circumbinary, trojan, slingshot, galaxycollision,
//   randomswarm, randomnbody.
//
// Decode is tolerant: unknown keys after a valid scenario are ignored, so a
// stale URL with an old key won't break. Malformed numbers return None.

fn scenario_name(scenario: &Scenario) -> &'static str {
    match scenario {
        Scenario::CentralSwarm { .. } => "centralswarm",
        Scenario::DualCircle => "dualcircle",
        Scenario::TriangleCircle => "trianglecircle",
        Scenario::Burrau1913 => "burrau1913",
        Scenario::SolarSystem => "solarsystem",
        Scenario::FigureEight => "figureeight",
        Scenario::Circumbinary => "circumbinary",
        Scenario::Trojan => "trojan",
        Scenario::Slingshot => "slingshot",
        Scenario::GalaxyCollision { .. } => "galaxycollision",
        Scenario::RandomSwarm(_) => "randomswarm",
        Scenario::RandomNBody(_) => "randomnbody",
    }
}

fn parse_scenario(name: &str) -> Option<Scenario> {
    match name {
        "centralswarm" => Some(Scenario::CentralSwarm { swarm_size: 1000 }),
        "dualcircle" => Some(Scenario::DualCircle),
        "trianglecircle" => Some(Scenario::TriangleCircle),
        "burrau1913" => Some(Scenario::Burrau1913),
        "solarsystem" => Some(Scenario::SolarSystem),
        "figureeight" => Some(Scenario::FigureEight),
        "circumbinary" => Some(Scenario::Circumbinary),
        "trojan" => Some(Scenario::Trojan),
        "slingshot" => Some(Scenario::Slingshot),
        "galaxycollision" => Some(Scenario::GalaxyCollision { swarm_size: 2000 }),
        "randomswarm" => Some(Scenario::RandomSwarm(RandomSwarmParams::default())),
        "randomnbody" => Some(Scenario::RandomNBody(RandomNBodyParams::default())),
        _ => None,
    }
}

/// Encode a config as a URL query string (without `?`).
/// Empty string means "all default values".
///
/// Deltas are measured against the config's *own canonical form*
/// (`for_scenario(config.scenario)`), not against `SimulationConfig::default()`.
/// `for_scenario` derives `physics_dt`/`softening` from the scenario via the
/// stability criterion, so a fresh `for_scenario(X)` round-trips to just
/// `scenario=X` — those derived fields are only emitted if the caller overrode
/// them. Scenario sub-parameters (swarm_size, mass ranges, …) are likewise
/// compared against the per-variant base returned by `parse_scenario`, so an
/// override on `GalaxyCollision`/`RandomSwarm`/`RandomNBody` is emitted even
/// though `SimulationConfig::default()` is a `CentralSwarm`.
pub fn encode(config: &SimulationConfig) -> String {
    let default = SimulationConfig::default();
    let mut parts: Vec<String> = Vec::new();

    if config.scenario != default.scenario {
        let name = scenario_name(&config.scenario);
        parts.push(format!("scenario={name}"));
        // Per-variant base: what `decode` would reconstruct from `scenario=<name>`
        // alone. Emit only the deltas on top of that base.
        let base = parse_scenario(name)
            .expect("scenario_name + parse_scenario cover the same variants");
        match (&config.scenario, base) {
            (Scenario::CentralSwarm { swarm_size }, Scenario::CentralSwarm { swarm_size: d }) => {
                if *swarm_size != d { parts.push(format!("swarm_size={swarm_size}")); }
            }
            (Scenario::GalaxyCollision { swarm_size }, Scenario::GalaxyCollision { swarm_size: d }) => {
                if *swarm_size != d { parts.push(format!("swarm_size={swarm_size}")); }
            }
            (Scenario::RandomSwarm(p), Scenario::RandomSwarm(d)) => {
                if p.swarm_size != d.swarm_size { parts.push(format!("swarm_size={}", p.swarm_size)); }
                if p.radius_range != d.radius_range {
                    parts.push(format!("radius_min={}", p.radius_range.0));
                    parts.push(format!("radius_max={}", p.radius_range.1));
                }
                if p.central_mass_range != d.central_mass_range {
                    parts.push(format!("central_mass_min={}", p.central_mass_range.0));
                    parts.push(format!("central_mass_max={}", p.central_mass_range.1));
                }
                if p.light_mass_range != d.light_mass_range {
                    parts.push(format!("light_mass_min={}", p.light_mass_range.0));
                    parts.push(format!("light_mass_max={}", p.light_mass_range.1));
                }
                if p.seed != d.seed { parts.push(format!("seed={}", p.seed)); }
            }
            (Scenario::RandomNBody(p), Scenario::RandomNBody(d)) => {
                if p.count != d.count { parts.push(format!("count={}", p.count)); }
                if p.mass_range != d.mass_range {
                    parts.push(format!("mass_min={}", p.mass_range.0));
                    parts.push(format!("mass_max={}", p.mass_range.1));
                }
                if p.position_spread != d.position_spread { parts.push(format!("position_spread={}", p.position_spread)); }
                if p.velocity_range != d.velocity_range {
                    parts.push(format!("velocity_min={}", p.velocity_range.0));
                    parts.push(format!("velocity_max={}", p.velocity_range.1));
                }
                if p.seed != d.seed { parts.push(format!("seed={}", p.seed)); }
            }
            _ => {}
        }
    }

    // Derived-field deltas are measured against the canonical config for THIS
    // scenario, so a fresh `for_scenario(X)` adds nothing here.
    let canonical = SimulationConfig::for_scenario(config.scenario);
    if config.physics_dt != canonical.physics_dt { parts.push(format!("physics_dt={}", config.physics_dt)); }
    if config.time_scale != canonical.time_scale { parts.push(format!("time_scale={}", config.time_scale)); }
    if config.theta_threshold != canonical.theta_threshold { parts.push(format!("theta={}", config.theta_threshold)); }
    if config.softening != canonical.softening { parts.push(format!("softening={}", config.softening)); }

    parts.join("&")
}

fn parse_pair(key: &str, value: &str, config: &mut SimulationConfig, overrides: &mut Overrides) -> Result<(), ()> {
    match key {
        "scenario" => {
            config.scenario = parse_scenario(value).ok_or(())?;
        }
        "swarm_size" => {
            let v: usize = value.parse().map_err(|_| ())?;
            match &mut config.scenario {
                Scenario::CentralSwarm { swarm_size } => *swarm_size = v,
                Scenario::GalaxyCollision { swarm_size } => *swarm_size = v,
                Scenario::RandomSwarm(p) => p.swarm_size = v,
                _ => return Err(()),
            }
        }
        "radius_min" => {
            if let Scenario::RandomSwarm(p) = &mut config.scenario { p.radius_range.0 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "radius_max" => {
            if let Scenario::RandomSwarm(p) = &mut config.scenario { p.radius_range.1 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "central_mass_min" => {
            if let Scenario::RandomSwarm(p) = &mut config.scenario { p.central_mass_range.0 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "central_mass_max" => {
            if let Scenario::RandomSwarm(p) = &mut config.scenario { p.central_mass_range.1 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "light_mass_min" => {
            if let Scenario::RandomSwarm(p) = &mut config.scenario { p.light_mass_range.0 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "light_mass_max" => {
            if let Scenario::RandomSwarm(p) = &mut config.scenario { p.light_mass_range.1 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "seed" => {
            let v: u64 = value.parse().map_err(|_| ())?;
            match &mut config.scenario {
                Scenario::RandomSwarm(p) => p.seed = v,
                Scenario::RandomNBody(p) => p.seed = v,
                _ => return Err(()),
            }
        }
        "count" => {
            if let Scenario::RandomNBody(p) = &mut config.scenario { p.count = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "mass_min" => {
            if let Scenario::RandomNBody(p) = &mut config.scenario { p.mass_range.0 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "mass_max" => {
            if let Scenario::RandomNBody(p) = &mut config.scenario { p.mass_range.1 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "position_spread" => {
            if let Scenario::RandomNBody(p) = &mut config.scenario { p.position_spread = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "velocity_min" => {
            if let Scenario::RandomNBody(p) = &mut config.scenario { p.velocity_range.0 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        "velocity_max" => {
            if let Scenario::RandomNBody(p) = &mut config.scenario { p.velocity_range.1 = value.parse().map_err(|_| ())?; } else { return Err(()); }
        }
        // Scalar overrides are recorded rather than applied immediately: the
        // scenario's own derived physics_dt/softening are reconstructed from
        // `for_scenario` after parsing (see `decode`), so a URL that omits them
        // still round-trips a fresh `for_scenario(X)`.
        "physics_dt" => { overrides.physics_dt = Some(value.parse().map_err(|_| ())?); }
        "time_scale" => { overrides.time_scale = Some(value.parse().map_err(|_| ())?); }
        "theta" => { overrides.theta = Some(value.parse().map_err(|_| ())?); }
        "softening" => { overrides.softening = Some(value.parse().map_err(|_| ())?); }
        // unknown key: tolerate (stale URL from old version). The decoder just
        // skips it — documented behavior in tests/url_encode.rs::trailing_junk_ignored.
        _ => {}
    }
    Ok(())
}

#[derive(Default)]
struct Overrides {
    physics_dt: Option<f32>,
    time_scale: Option<f32>,
    theta: Option<f32>,
    softening: Option<f32>,
}

/// Decode a URL query string (without `?`). Empty/whitespace → default config.
/// Unknown scenario name → None. Malformed number → None.
/// Unknown keys after a valid scenario are ignored. The scenario's derived
/// `physics_dt`/`softening` come from `for_scenario`; explicit `physics_dt`/
/// `softening`/`time_scale`/`theta` keys override them.
pub fn decode(query: &str) -> Option<SimulationConfig> {
    let trimmed = query.trim();
    if trimmed.is_empty() { return Some(SimulationConfig::default()); }

    let mut config = SimulationConfig::default();
    let mut overrides = Overrides::default();
    for pair in trimmed.split('&') {
        if pair.is_empty() { continue; }
        let mut split = pair.splitn(2, '=');
        let key = split.next()?;
        let value = split.next()?;
        parse_pair(key, value, &mut config, &mut overrides).ok()?;
    }
    // Rebuild from the canonical config for the final scenario so a URL that
    // only says `scenario=dualcircle` picks up DualCircle's derived dt/softening
    // rather than the default CentralSwarm's, then layer any explicit overrides.
    let mut canonical = SimulationConfig::for_scenario(config.scenario);
    match overrides.physics_dt { Some(v) => canonical.physics_dt = v, None => {} }
    match overrides.time_scale { Some(v) => canonical.time_scale = v, None => {} }
    match overrides.theta { Some(v) => canonical.theta_threshold = v, None => {} }
    match overrides.softening { Some(v) => canonical.softening = v, None => {} }
    Some(canonical)
}

// ---- wasm <-> window bridge ----
// These functions touch the browser API and exist only under wasm32. The
// native build of main.rs never calls them (gated at the call site too), so
// they add no dead-code warnings when target_arch != "wasm32".

#[cfg(target_arch = "wasm32")]
pub fn read_url_query() -> String {
    let window = web_sys::window().expect("no global window object");
    let location = window.location();
    let search = location.search().unwrap_or_default();
    // strip leading '?'
    if let Some(stripped) = search.strip_prefix('?') { stripped.to_string() } else { search }
}

#[cfg(target_arch = "wasm32")]
pub fn write_url_query(query: &str) {
    let window = web_sys::window().expect("no global window object");
    let path = window.location().pathname().unwrap_or_default();
    let new_url = if query.is_empty() { path } else { format!("{}?{}", path, query) };
    let history = window.history().expect("no history object");
    let _ = history.replace_state_with_url(&wasm_bindgen::JsValue::NULL, "", &new_url);
}