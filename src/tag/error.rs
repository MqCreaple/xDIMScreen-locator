use std::ops::RangeInclusive;
use std::fmt::{Debug, Display};
use std::error::Error;

pub struct InvalidFormatError {
    object: serde_json::Value,
    reason: String,
}

impl InvalidFormatError {
    pub fn new<S: Into<String>>(object: &serde_json::Value, reason: S) -> Self {
        Self { object: object.clone(), reason: reason.into() }
    }
}

impl Debug for InvalidFormatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Invalid format for object:\n{}\nReason: {}",
            self.object,
            self.reason,
        )
    }
}

impl Display for InvalidFormatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl Error for InvalidFormatError {}

pub struct UnsupportedVersionError {
    version: i64,
    supported_versions: RangeInclusive<i64>,
}

impl UnsupportedVersionError {
    pub fn new(version: i64, supported_versions: RangeInclusive<i64>) -> Self {
        Self {
            version: version,
            supported_versions
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