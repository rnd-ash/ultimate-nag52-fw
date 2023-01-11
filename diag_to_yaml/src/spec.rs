use serde::{Serialize, Deserialize};


#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum ByteOrder {
    BigEndian,
    LittleEndian
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, PartialOrd)]
pub enum DataType {
    Enum(Vec<(u32, String)>),
    Linear { offset: f32, multiplier: f32 },
    HexDump,
    ASCII,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct DiagYml {
    pub gen_date: String,
    pub maps: Vec<MapData>,
}

impl Default for DiagYml {
    fn default() -> Self {
        Self { gen_date: "UNKNOWN".into(), maps: Default::default() }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, PartialOrd)]
pub struct DataField {
    pub start_bit: u32,
    pub len_bits: u32,
    pub byte_order: ByteOrder,
    pub name: String,
    pub desc: String,
    pub data_type: DataType,
    pub max: f32,
    pub min: f32
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct MapData {
    pub id: u8,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub desc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_value_desc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub y_value_desc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub x_value_desc: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub y_overrides: Vec<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub x_overrides: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub x_unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub y_unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_unit: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, PartialOrd)]
pub struct ConfigOption {
    pub id: u8,
    pub data: Vec<DataField>
}