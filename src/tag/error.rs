use std::error::Error;
use std::fmt::{Debug, Display};
use std::ops::RangeInclusive;

use crate::tag::tagged_object::TagIndex;

/// This error occurs when the tagobj file's format is invalid.
pub struct InvalidFormatError {
    object: serde_json::Value,
    reason: String,
}

impl InvalidFormatError {
    pub fn new<S: Into<String>>(object: &serde_json::Value, reason: S) -> Self {
        Self {
            object: object.clone(),
            reason: reason.into(),
        }
    }
}

impl Debug for InvalidFormatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Invalid format for object:\n{}\nReason: {}",
            self.object, self.reason,
        )
    }
}

impl Display for InvalidFormatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl Error for InvalidFormatError {}

/// This error occurs when the version number in a .tagobj file is currently not supported.
pub struct UnsupportedVersionError {
    version: i64,
    supported_versions: RangeInclusive<i64>,
}

impl UnsupportedVersionError {
    pub fn new(version: i64, supported_versions: RangeInclusive<i64>) -> Self {
        Self {
            version: version,
            supported_versions,
        }
    }
}

impl Debug for UnsupportedVersionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Version number \"{}\" is not supported. Supported versions are from version {} to version {}.",
            self.version,
            self.supported_versions.start(),
            self.supported_versions.end(),
        )
    }
}

impl Display for UnsupportedVersionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl Error for UnsupportedVersionError {}

/// This error occurs when the same tag exists on multiple different object registered in the
/// tagged object locator.
pub struct ConflictingTagError {
    tag: Option<TagIndex>,
    object1: String,
    object2: String,
}

impl ConflictingTagError {
    pub fn new(tag: TagIndex, object1: String, object2: String) -> Self {
        Self {
            tag: Some(tag),
            object1,
            object2,
        }
    }

    pub fn new_name(name: &str) -> Self {
        Self {
            tag: None,
            object1: name.to_string(),
            object2: name.to_string(),
        }
    }
}

impl Debug for ConflictingTagError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(tag) = self.tag {
            write!(
                f,
                "Tag \"{}\" is in both object \"{}\" and object \"{}\"!",
                tag, self.object1, self.object2
            )
        } else {
            write!(
                f,
                "Conflicting object names: \"{}\" and \"{}\"",
                self.object1, self.object2
            )
        }
    }
}

impl Display for ConflictingTagError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl Error for ConflictingTagError {}
