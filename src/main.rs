use nmea_parser::*;
use serde::{Serialize, Serializer, Deserialize};
use serde_json::json;
use std::collections::{HashMap, HashSet};

use std::process::{Command, Stdio};
use std::{thread, time::Duration};
use std::fs;
use std::fs::File;
use std::io::{Write, Read};
use std::fmt::Display;
use std::thread::sleep;

#[derive(Debug, Deserialize)]
struct Config {
    instant_scan: bool,
    start_after_duration: Option<u64>,
    scan_duration: Option<u64>,
}


fn read_config(config_path: &str) -> Result<Config, Box<dyn std::error::Error>> {
    let mut file = File::open(config_path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    let config: Config = serde_json::from_str(&contents)?;
    Ok(config)
}

#[tokio::main]
async fn main() {
    let config = read_config("config.json").expect("Failed to read config");

    if config.instant_scan {
        // Comportement Instantané
        if let Err(e) = run_ais_script().await {
            println!("Script failed with error: {}", e);
        }
    } else {
        // Comportement Programmé
        let start_after = config.start_after_duration.unwrap_or_default();
        let duration = config.scan_duration.unwrap_or_default();
        if let Err(e) = run_ais_script_programmed(start_after, duration).await {
            println!("Programmed script failed with error: {}", e);
        }
    }
}

// Wrapper structs for the problematic types
#[derive(Debug)]
struct NavigationStatusWrapper(ais::NavigationStatus);

impl Serialize for NavigationStatusWrapper {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.0.to_string())
    }
}

#[derive(Debug)]
struct PositioningSystemMetaWrapper(Option<ais::PositioningSystemMeta>);

impl Serialize for PositioningSystemMetaWrapper {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        match &self.0 {
            Some(meta) => serializer.serialize_str(&meta.to_string()),
            None => serializer.serialize_none(),
        }
    }
}

#[derive(Debug)]
struct RotDirectionWrapper(Option<ais::RotDirection>);

impl Serialize for RotDirectionWrapper {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        match &self.0 {
            Some(direction) => serializer.serialize_str(&direction.to_string()),
            None => serializer.serialize_none(),
        }
    }
}

#[derive(Debug)]
struct StationWrapper(ais::Station);

impl Serialize for StationWrapper {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.0.to_string())
    }
}

#[derive(Serialize)]
struct SerializedVesselDynamicData {
    #[serde(serialize_with = "serialize_bool_as_string")]
    own_vessel: Option<bool>,
    mmsi: u32,
    cog: f64,
    #[serde(serialize_with = "serialize_option_as_string")]
    current_gnss_position: Option<bool>,
    heading_true: f64,
    #[serde(serialize_with = "serialize_bool_as_string")]
    high_position_accuracy: Option<bool>,
    sog_knots: f64,
    longitude: f64,
    latitude: f64,
    nav_status: Option<NavigationStatusWrapper>,
    positioning_system_meta: PositioningSystemMetaWrapper,
    radio_status: u32,
    #[serde(serialize_with = "serialize_bool_as_string")]
    raim_flag: Option<bool>,
    #[serde(serialize_with = "serialize_option_as_string")]
    rot: Option<f64>,
    rot_direction: RotDirectionWrapper,
    #[serde(serialize_with = "serialize_bool_as_string")]
    special_manoeuvre: Option<bool>,
    station: Option<StationWrapper>,
    timestamp_seconds: u8,
}

#[derive(Serialize)]
struct SerializedStandardSarAircraftPositionReport {
    own_vessel: bool,
    station: StationWrapper,
    mmsi: u32,
    altitude: u16,
    sog_knots: u16,
    high_position_accuracy: bool,
    latitude: f64,
    longitude: f64,
    cog: f64,
    timestamp_seconds: u8,
    regional: u8,
    dte: bool,
    assigned: bool,
    raim_flag: bool,
    radio_status: u32,
    
}

#[derive(Serialize, Deserialize)]
struct VesselData {
    own_vessel: Option<String>,
    mmsi: u32,
    cog: f64,
    current_gnss_position: Option<String>,
    heading_true: Option<f64>,
    high_position_accuracy: Option<String>,
    sog_knots: f64,
    longitude: f64,
    latitude: f64,
    nav_status: Option<String>,
    positioning_system_meta: Option<String>,
    radio_status: f64,
    raim_flag: Option<String>,
    rot: Option<String>,
    rot_direction: Option<String>,
    special_manoeuvre: Option<String>,
    station: String,
    timestamp_seconds: f64,
}

fn serialize_option_as_string<T, S>(opt: &Option<T>, serializer: S) -> Result<S::Ok, S::Error>
where
    T: Display,
    S: Serializer,
{
    let string = match opt {
        Some(value) => value.to_string(),
        None => "Unknown".to_owned(),
    };
    serializer.serialize_str(&string)
}

fn serialize_bool_as_string<S>(value: &Option<bool>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let string = match value {
        Some(true) => "true".to_owned(),
        Some(false) => "false".to_owned(),
        None => "Unknown".to_owned(),
    };
    serializer.serialize_str(&string)
}

fn serialize_vessel_data(parser: &mut NmeaParser, sentences: &str) -> String {
    match parser.parse_sentence(sentences) {
        Ok(ParsedMessage::VesselDynamicData(vdd)) => {
            let serializable_parsed_message = SerializedVesselDynamicData {
                own_vessel: Some(vdd.own_vessel),
                mmsi: vdd.mmsi,
                cog: vdd.cog.unwrap_or_default(),
                current_gnss_position: vdd.current_gnss_position,
                heading_true: vdd.heading_true.unwrap_or_default(),
                high_position_accuracy: Some(vdd.high_position_accuracy),
                sog_knots: vdd.sog_knots.unwrap_or_default(),
                longitude: vdd.longitude.unwrap_or_default(),
                latitude: vdd.latitude.unwrap_or_default(),
                nav_status: Some(NavigationStatusWrapper(vdd.nav_status)),
                positioning_system_meta: PositioningSystemMetaWrapper(vdd.positioning_system_meta),
                radio_status: vdd.radio_status.unwrap_or_default(),
                raim_flag: Some(vdd.raim_flag),
                rot: vdd.rot,
                rot_direction: RotDirectionWrapper(vdd.rot_direction),
                special_manoeuvre: vdd.special_manoeuvre,
                station: Some(StationWrapper(vdd.station)),
                timestamp_seconds: vdd.timestamp_seconds,
            };
            serde_json::to_string_pretty(&serializable_parsed_message).unwrap()
        }
        _ => {
            panic!("Unknown message");
        }
    }
}

fn run_ais_catcher() -> Result<String, std::io::Error> {
    // Run the command
    let child = Command::new("sh")
        .arg("-c")
        .arg("./build/AIS-catcher -l > aisOutput.txt 2>&1")
        .spawn();

    match child {
        Ok(mut child) => {
            // Wait for 30 seconds
            sleep(Duration::from_secs(30));

            // Kill the command
            match child.kill() {
                Ok(_) => {
                    // Read the file
                    let contents = fs::read_to_string("aisOutput.txt")?;
                    Ok(contents)
                },
                Err(e) => Err(e),
            }
        },
        Err(e) => Err(e),
    }
}

async fn run_ais_catcher_programmed(duration: u64) -> Result<String, std::io::Error> {
    // Démarre AIS-catcher en arrière-plan
    let mut child = Command::new("./build/AIS-catcher")
        .arg("-l")
        .stdout(Stdio::piped()) // Rediriger la sortie standard si nécessaire
        .spawn()?;

    // Attend la durée spécifiée
    sleep(Duration::from_secs(duration));

    // Tente de terminer le processus proprement
    match child.kill() {
        Ok(_) => println!("AIS-catcher process was terminated."),
        Err(e) => println!("Failed to terminate AIS-catcher process: {}", e),
    }

    // Attendre la fin du processus et récupérer la sortie
    let output = child.wait_with_output()?;

    // Convertir la sortie en String
    let output_str = String::from_utf8_lossy(&output.stdout).to_string();

    Ok(output_str)
}


fn extract_nmea_trams(output: &str) -> Vec<&str> {
    let lines: Vec<&str> = output
        .lines()
        .filter(|line| line.starts_with("!"))
        .collect();
    lines
}

pub async fn run_ais_script() -> Result<bool, Box<dyn std::error::Error>> {
    let output_str: String = run_ais_catcher().unwrap();
    //let output_str: String = fs::read_to_string("aisOutput.txt").unwrap();

    let results = extract_nmea_trams(&output_str);
    let mut results_map: HashMap<u32, VesselData> = HashMap::new();

    for i in 0..results.len() {
        let mut parser = NmeaParser::new();
        let serialized_message = serialize_vessel_data(&mut parser, &results[i]);
        if serialized_message.len() > 0 {        
            let vessel_data: Result<VesselData, serde_json::Error> = serde_json::from_str(&serialized_message);            match vessel_data {
                Ok(data) => {
                    results_map.insert((i + 1) as u32, data);
                }
                Err(err) => {
                    println!("Failed to parse JSON: {}", err);
                }
            }
        } else {
            println!("Failed to parse message");
        }
    }
    
    let json = serde_json::to_string_pretty(&results_map).unwrap();
    println!("{}", json);

    write_json_to_file(&json, "ais_data.json")?;

    if json == "{}" {
        Ok(false)
    } else {
        Ok(true)
    }

}

pub async fn run_ais_script_programmed(start_after: u64, duration: u64) -> Result<bool, Box<dyn std::error::Error>> {
    println!("Waiting for {} seconds before starting...", start_after);
    thread::sleep(Duration::from_secs(start_after));

    println!("Starting AIS capture for {} seconds...", duration);
    let ais_output = run_ais_catcher_programmed(duration).await?;

    let results = extract_nmea_trams(&ais_output);
    let mut results_map: HashMap<String, Vec<SerializedVesselDynamicData>> = HashMap::new();
    let mut vessel_timestamps: HashMap<String, HashSet<u8>> = HashMap::new();

    for result in results.iter() {
        let mut parser = NmeaParser::new();
        if let Ok(ParsedMessage::VesselDynamicData(vdd)) = parser.parse_sentence(result) {
            let mmsi_str = vdd.mmsi.to_string();

            // Calcul des durées basé sur les timestamps
            vessel_timestamps.entry(mmsi_str.clone()).or_insert_with(HashSet::new).insert(vdd.timestamp_seconds);

            let serialized_data = SerializedVesselDynamicData {
                own_vessel: Some(vdd.own_vessel),
                mmsi: vdd.mmsi,
                cog: vdd.cog.unwrap_or_default(),
                current_gnss_position: vdd.current_gnss_position,
                heading_true: vdd.heading_true.unwrap_or_default(),
                high_position_accuracy: Some(vdd.high_position_accuracy),
                sog_knots: vdd.sog_knots.unwrap_or_default(),
                longitude: vdd.longitude.unwrap_or_default(),
                latitude: vdd.latitude.unwrap_or_default(),
                nav_status: Some(NavigationStatusWrapper(vdd.nav_status)),
                positioning_system_meta: PositioningSystemMetaWrapper(vdd.positioning_system_meta),
                radio_status: vdd.radio_status.unwrap_or_default(),
                raim_flag: Some(vdd.raim_flag),
                rot: vdd.rot,
                rot_direction: RotDirectionWrapper(vdd.rot_direction),
                special_manoeuvre: vdd.special_manoeuvre,
                station: Some(StationWrapper(vdd.station)),
                timestamp_seconds: vdd.timestamp_seconds,
            };

            results_map.entry(mmsi_str).or_insert_with(Vec::new).push(serialized_data);
        }
    }

    // Conversion des données et des durées en JSON, avec ajout des durées
    let mut json_data = Vec::new();
    for (mmsi, data_vec) in results_map {
        let durations = vessel_timestamps.get(&mmsi).unwrap();
        // Simplification: calcul des durées en supposant des intervalles de 1 seconde pour chaque timestamp
        let duration_str = durations.iter().map(|t| format!("{}", t)).collect::<Vec<_>>().join(", ");

        for data in data_vec {
            let mut vessel_json = json!(&data);
            vessel_json["durations"] = json!(duration_str);
            json_data.push(vessel_json);
        }
    }

    let json = serde_json::to_string_pretty(&json_data)?;
    write_json_to_file(&json, "ais_data_programmed.json")?;
    Ok(true)
}

fn write_json_to_file(json_data: &str, filename: &str) -> Result<(), std::io::Error> {
    let mut file = File::create(filename)?;
    file.write_all(json_data.as_bytes())?;
    Ok(())
}