use std::{fs::File, io::{Write, Read}, path::PathBuf, cmp::max};

use clap::{Parser, Args};
use regex::Regex;

use crate::spec::{DiagYml, MapData};

mod spec;

#[derive(Debug, Parser)]
pub struct ParserSettings {
    in_src_dir: String,
    out_yml_path: String,
}

fn main() {

    let settings = ParserSettings::parse();

    let mut yml = DiagYml::default();
    let src_dir = PathBuf::from(settings.in_src_dir);
    // Map parsing first
    let mut tmp_str = String::new();
    let mut maps_file = File::open(src_dir.join("diag").join("map_editor.h")).unwrap();
    maps_file.read_to_string(&mut tmp_str).unwrap();
    println!("{}",tmp_str);
    let lines: Vec<String> = tmp_str.split("\n").map(|x| x.to_string()).collect();
    println!("Parsing src/diag/map_editor.h");
    for (idx, line) in lines.iter().enumerate() {
        if line.contains("#define") && line.contains("_MAP_ID") {
            let mut parts = line.split(" ");
            assert!(parts.next().unwrap() == "#define");
            let map_name = parts.next().unwrap().replace("_MAP_ID", "");
            let map_idx = u8::from_str_radix(parts.next().unwrap().replace("0x", "").as_str(), 16).unwrap();
            println!("Found map {} (IDX {})", map_name, map_idx);
            let mut map_data = MapData {
                id: map_idx,
                name: map_name,
                ..Default::default()
            };
            if lines[max(0, idx-1)].contains("-*/") {
                println!("Map ID has data!");
                let mut scan_idx = idx-2;
                loop {
                    if lines[scan_idx].contains("/*") {
                        println!("Reached end of comment block");
                        break;
                    }
                    let md_line = &lines[scan_idx];
                    let capture = Regex::new(r"#(.*):\s(.*)").unwrap().captures(&md_line);
                    if let Some(data) = capture {
                        let key = &data[1];
                        let value = &data[2];
                        match key {
                            "name" => {
                                map_data.name = value.to_string();
                            }
                            "desc" => {
                                map_data.desc = Some(value.to_string());
                            }
                            "xdesc" => {
                                map_data.x_value_desc = Some(value.to_string());
                            },
                            "ydesc" => {
                                map_data.y_value_desc = Some(value.to_string());
                            },
                            "celldesc" => {
                                map_data.cell_value_desc = Some(value.to_string());
                            },
                            "xunit" => {
                                map_data.x_unit = Some(value.to_string());
                            },
                            "yunit" => {
                                map_data.y_unit = Some(value.to_string());
                            },
                            "cellunit" => {
                                map_data.cell_unit = Some(value.to_string());
                            }
                            _ => {
                                eprintln!("Skipping unknown tag '{}'", key);
                            }
                        }
                    }
                    scan_idx -= 1;
                }
            }
            yml.maps.push(map_data);
        }
    }


    let s = serde_yaml::to_string(&yml).unwrap();
    File::create(settings.out_yml_path).unwrap().write(s.as_bytes()).unwrap();


}
