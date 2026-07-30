//! The result of checking a level: informational sections plus findings.
//!
//! The check itself never prints. It returns a [`Report`], which the binary
//! formats and which tests assert against — the same data either way.

use std::fmt;

/// How much a finding matters.
///
/// Only [`Severity::Error`] fails the check. Warnings describe things worth an
/// author's attention that are nonetheless legitimate — dropping an object onto
/// terrain from a height, for instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Warning,
    Error,
}

impl fmt::Display for Severity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Severity::Warning => write!(f, "warning"),
            Severity::Error => write!(f, "error"),
        }
    }
}

/// One thing that is wrong, or might be.
#[derive(Debug, Clone)]
pub struct Finding {
    pub severity: Severity,
    /// Which check produced this, e.g. `"objects"`. Groups the output.
    pub category: &'static str,
    pub message: String,
}

/// A block of informational output — statistics, derived figures.
#[derive(Debug, Clone)]
pub struct Section {
    pub title: String,
    /// Label/value pairs, printed as an aligned table.
    pub rows: Vec<(String, String)>,
    /// Free-form lines printed after the table.
    pub notes: Vec<String>,
}

impl Section {
    pub fn new(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            rows: Vec::new(),
            notes: Vec::new(),
        }
    }

    pub fn row(&mut self, label: impl Into<String>, value: impl Into<String>) -> &mut Self {
        self.rows.push((label.into(), value.into()));
        self
    }

    pub fn note(&mut self, note: impl Into<String>) -> &mut Self {
        self.notes.push(note.into());
        self
    }
}

/// Everything one run of the check produced.
#[derive(Debug, Clone, Default)]
pub struct Report {
    pub sections: Vec<Section>,
    pub findings: Vec<Finding>,
}

impl Report {
    pub fn push_section(&mut self, section: Section) {
        self.sections.push(section);
    }

    pub fn error(&mut self, category: &'static str, message: impl Into<String>) {
        self.findings.push(Finding {
            severity: Severity::Error,
            category,
            message: message.into(),
        });
    }

    pub fn warn(&mut self, category: &'static str, message: impl Into<String>) {
        self.findings.push(Finding {
            severity: Severity::Warning,
            category,
            message: message.into(),
        });
    }

    pub fn error_count(&self) -> usize {
        self.count(Severity::Error)
    }

    pub fn warning_count(&self) -> usize {
        self.count(Severity::Warning)
    }

    fn count(&self, severity: Severity) -> usize {
        self.findings
            .iter()
            .filter(|f| f.severity == severity)
            .count()
    }

    /// Whether the level passed. Warnings do not fail a level.
    pub fn passed(&self) -> bool {
        self.error_count() == 0
    }
}
