use serde::de::{MapAccess, Visitor};
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

extern crate nalgebra as na;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectLocationPacket {
    /// The timestamp at which the object is located, in miliseconds
    pub time: u128,

    /// Name of the object
    pub name: String,

    /// Transform relative to the camera's reference frame
    #[serde(
        serialize_with = "serialize_isometry",
        deserialize_with = "deserialize_isometry"
    )]
    pub transform: na::Isometry3<f64>,
}

/// Serialize `na::Isometry3<f64>` type.
fn serialize_isometry<S: Serializer>(
    isometry: &na::Isometry3<f64>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    let mut map = serializer.serialize_map(Some(2))?;
    map.serialize_key("rq")?;
    map.serialize_value(isometry.rotation.as_vector().as_slice())?; // in the order of (i, j, k, 1)
    map.serialize_key("t")?;
    map.serialize_value(isometry.translation.vector.as_slice())?;
    map.end()
}

/// A utility struct for deserializing into `na::Isometry3<f64>` type
struct IsometryVisitor;

impl<'de> Visitor<'de> for IsometryVisitor {
    type Value = na::Isometry3<f64>;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        formatter.write_str("A map of format {\"rq\": [i, j, k, w], \"t\": [x, y, z]}\n")?;
        formatter.write_str("Where \"rq\" is the rotation quaternion, \"t\" is the translation vector relative to the camera's reference frame")
    }

    fn visit_map<M>(self, mut access: M) -> Result<Self::Value, M::Error>
    where
        M: MapAccess<'de>,
    {
        let mut rotation: Option<Vec<f64>> = None;
        let mut translation: Option<Vec<f64>> = None;
        while let Some((key, value)) = access.next_entry::<&str, Vec<f64>>()? {
            if key == "rq" {
                rotation = Some(value);
            } else if key == "t" {
                translation = Some(value);
            }
        }
        match (rotation, translation) {
            (Some(rotation), Some(translation)) => Ok(na::Isometry3::from_parts(
                na::Translation3::new(translation[0], translation[1], translation[2]),
                na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    rotation[3],
                    rotation[0],
                    rotation[1],
                    rotation[2],
                )),
            )),
            (Some(_), None) => Err(serde::de::Error::missing_field("t")),
            _ => Err(serde::de::Error::missing_field("rq")),
        }
    }
}

fn deserialize_isometry<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<na::Isometry3<f64>, D::Error> {
    deserializer.deserialize_map(IsometryVisitor)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serialize_json() {
        let packet = ObjectLocationPacket {
            time: 1145141919810,
            name: "object".to_string(),
            transform: na::Isometry3::identity(),
        };
        let serialized = serde_json::to_string(&packet).unwrap();
        let deserialized: serde_json::Value = serde_json::from_str(&serialized).unwrap();
        assert_eq!(
            deserialized,
            serde_json::json!({
                "time": 1145141919810u128,
                "name": "object",
                "transform": {
                    "rq": [0.0, 0.0, 0.0, 1.0],
                    "t": [0.0, 0.0, 0.0],
                }
            })
        );

        let packet = ObjectLocationPacket {
            time: 0,
            name: "&*\'|\"\\()[]~`.xXyY123啊啊".to_string(),
            transform: na::Isometry3::translation(1.0, 2.0, -3.0),
        };
        let serialized = serde_json::to_string(&packet).unwrap();
        let deserialized: serde_json::Value = serde_json::from_str(&serialized).unwrap();
        assert_eq!(
            deserialized,
            serde_json::json!({
                "time": 0u128,
                "name": "&*\'|\"\\()[]~`.xXyY123啊啊",
                "transform": {
                    "rq": [0.0, 0.0, 0.0, 1.0],
                    "t": [1.0, 2.0, -3.0],
                }
            })
        );
    }
}
