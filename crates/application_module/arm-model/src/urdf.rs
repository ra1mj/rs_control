//! Minimal bounded URDF event parser.
//!
//! Collision/visual/deployment extension subtrees are intentionally ignored.
//! Kinematic and inertial elements are parsed strictly and retain no XML
//! representation after profile compilation.

use std::{collections::BTreeMap, str};

use quick_xml::{
    Reader, XmlVersion,
    encoding::{Decoder, EncodingError},
    events::{BytesStart, Event, attributes::AttrError},
};
use thiserror::Error;

const MAX_XML_EVENTS: usize = 16_384;
const MAX_XML_DEPTH: usize = 32;
const MAX_ATTRIBUTES_PER_ELEMENT: usize = 32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ParsedJointKind {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Spherical,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ParsedInertia {
    pub origin_xyz: [f64; 3],
    pub origin_rpy: [f64; 3],
    pub mass: f64,
    pub matrix: [f64; 6],
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ParsedLink {
    pub name: String,
    pub inertia: Option<ParsedInertia>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct ParsedLimit {
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    pub effort: Option<f64>,
    pub velocity: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ParsedMimic {
    pub joint: String,
    pub multiplier: Option<f64>,
    pub offset: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ParsedJoint {
    pub name: String,
    pub kind: ParsedJointKind,
    pub origin_xyz: [f64; 3],
    pub origin_rpy: [f64; 3],
    pub parent: String,
    pub child: String,
    pub axis: [f64; 3],
    pub limit: ParsedLimit,
    pub mimic: Option<ParsedMimic>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ParsedRobot {
    pub name: String,
    pub links: Vec<ParsedLink>,
    pub joints: Vec<ParsedJoint>,
}

/// Failure from the project-owned bounded XML/URDF parser.
#[derive(Debug, Error)]
pub enum UrdfParseError {
    /// XML syntax or entity decoding failed.
    #[error("invalid XML")]
    Xml(#[from] quick_xml::Error),
    /// Declared input encoding could not decode the document.
    #[error("invalid XML encoding")]
    Encoding(#[from] EncodingError),
    /// XML attribute iteration rejected a duplicate or malformed attribute.
    #[error("invalid XML attribute")]
    Attribute(#[from] AttrError),
    /// A name or attribute was not UTF-8.
    #[error("XML name or attribute is not UTF-8")]
    Utf8(#[from] str::Utf8Error),
    /// The document exceeded a parser work/storage bound.
    #[error("{field} exceeds maximum {maximum}")]
    Bound {
        /// Bounded dimension.
        field: &'static str,
        /// Accepted maximum.
        maximum: usize,
    },
    /// DTD/entity declarations are outside the accepted artifact format.
    #[error("DTD and custom entity declarations are forbidden")]
    DocType,
    /// A required element or attribute is missing.
    #[error("element {element} is missing required field {field}")]
    Missing {
        /// Element name.
        element: &'static str,
        /// Required field.
        field: &'static str,
    },
    /// A recognized element appeared in an invalid nesting context.
    #[error("element {element} appears in an invalid context")]
    Context {
        /// Misplaced element.
        element: &'static str,
    },
    /// A recognized element contained an unsupported attribute.
    #[error("element {element} contains unsupported attribute {attribute}")]
    UnknownAttribute {
        /// Element name.
        element: &'static str,
        /// Rejected attribute.
        attribute: String,
    },
    /// A scalar or vector attribute could not be parsed exactly.
    #[error("element {element} has invalid numeric field {field}")]
    Numeric {
        /// Element name.
        element: &'static str,
        /// Rejected field.
        field: &'static str,
    },
    /// Joint type spelling is unknown.
    #[error("joint type {value:?} is unknown")]
    JointType {
        /// Rejected spelling.
        value: String,
    },
    /// A singleton semantic element appeared multiple times.
    #[error("element {element} appears more than once in its owner")]
    DuplicateElement {
        /// Repeated element.
        element: &'static str,
    },
    /// Non-whitespace text is not part of the accepted URDF subset.
    #[error("unexpected text in URDF")]
    UnexpectedText,
}

#[derive(Default)]
struct InertiaBuilder {
    origin_xyz: [f64; 3],
    origin_rpy: [f64; 3],
    origin_seen: bool,
    mass: Option<f64>,
    matrix: Option<[f64; 6]>,
}

struct LinkBuilder {
    name: String,
    inertia: Option<ParsedInertia>,
}

struct JointBuilder {
    name: String,
    kind: ParsedJointKind,
    origin_xyz: [f64; 3],
    origin_rpy: [f64; 3],
    origin_seen: bool,
    parent: Option<String>,
    child: Option<String>,
    axis: [f64; 3],
    axis_seen: bool,
    limit: ParsedLimit,
    limit_seen: bool,
    mimic: Option<ParsedMimic>,
}

#[derive(Default)]
struct ParserState {
    robot_name: Option<String>,
    robot_open: bool,
    robot_closed: bool,
    links: Vec<ParsedLink>,
    joints: Vec<ParsedJoint>,
    current_link: Option<LinkBuilder>,
    current_joint: Option<JointBuilder>,
    current_inertia: Option<InertiaBuilder>,
    depth: usize,
    skipped_depth: usize,
    events: usize,
}

pub(crate) fn parse_urdf(source: &str) -> Result<ParsedRobot, UrdfParseError> {
    let mut reader = Reader::from_str(source);
    reader.config_mut().trim_text(true);
    reader.config_mut().check_end_names = true;
    let mut state = ParserState::default();

    loop {
        state.events += 1;
        if state.events > MAX_XML_EVENTS {
            return Err(UrdfParseError::Bound {
                field: "XML event count",
                maximum: MAX_XML_EVENTS,
            });
        }
        match reader.read_event()? {
            Event::Start(start) => {
                state.depth += 1;
                if state.depth > MAX_XML_DEPTH {
                    return Err(UrdfParseError::Bound {
                        field: "XML nesting depth",
                        maximum: MAX_XML_DEPTH,
                    });
                }
                if state.skipped_depth > 0 {
                    state.skipped_depth += 1;
                } else {
                    let element_depth = state.depth;
                    if !handle_start(&mut state, &start, reader.decoder(), element_depth, false)? {
                        if !state.robot_open || element_depth == 1 {
                            return Err(UrdfParseError::Context {
                                element: "document",
                            });
                        }
                        state.skipped_depth = 1;
                    }
                }
            }
            Event::Empty(start) => {
                if state.skipped_depth == 0 {
                    let element_depth =
                        state.depth.checked_add(1).ok_or(UrdfParseError::Bound {
                            field: "XML nesting depth",
                            maximum: MAX_XML_DEPTH,
                        })?;
                    if element_depth > MAX_XML_DEPTH {
                        return Err(UrdfParseError::Bound {
                            field: "XML nesting depth",
                            maximum: MAX_XML_DEPTH,
                        });
                    }
                    let recognized =
                        handle_start(&mut state, &start, reader.decoder(), element_depth, true)?;
                    if !recognized && (!state.robot_open || element_depth == 1) {
                        return Err(UrdfParseError::Context {
                            element: "document",
                        });
                    }
                }
            }
            Event::End(end) => {
                if state.skipped_depth > 0 {
                    state.skipped_depth -= 1;
                } else {
                    let element_depth = state.depth;
                    handle_end(&mut state, end.name().as_ref(), element_depth)?;
                }
                state.depth = state
                    .depth
                    .checked_sub(1)
                    .ok_or(UrdfParseError::Context { element: "end" })?;
            }
            Event::DocType(_) | Event::GeneralRef(_) => return Err(UrdfParseError::DocType),
            Event::Text(text) => {
                if !text.decode()?.trim().is_empty() && state.skipped_depth == 0 {
                    return Err(UrdfParseError::UnexpectedText);
                }
            }
            Event::CData(text) => {
                if !text.decode()?.trim().is_empty() && state.skipped_depth == 0 {
                    return Err(UrdfParseError::UnexpectedText);
                }
            }
            Event::Eof => break,
            Event::Decl(_) | Event::PI(_) | Event::Comment(_) => {}
        }
    }
    if state.depth != 0
        || state.robot_open
        || !state.robot_closed
        || state.current_link.is_some()
        || state.current_joint.is_some()
        || state.current_inertia.is_some()
    {
        return Err(UrdfParseError::Context {
            element: "document",
        });
    }
    Ok(ParsedRobot {
        name: state.robot_name.ok_or(UrdfParseError::Missing {
            element: "robot",
            field: "name",
        })?,
        links: state.links,
        joints: state.joints,
    })
}

fn handle_start(
    state: &mut ParserState,
    start: &BytesStart<'_>,
    decoder: Decoder,
    element_depth: usize,
    empty: bool,
) -> Result<bool, UrdfParseError> {
    match start.name().as_ref() {
        b"robot" => {
            let attrs = attributes(start, decoder, "robot", &["name"])?;
            if element_depth != 1
                || state.robot_name.is_some()
                || state.robot_open
                || state.robot_closed
                || state.current_link.is_some()
                || state.current_joint.is_some()
            {
                return Err(UrdfParseError::Context { element: "robot" });
            }
            state.robot_name = Some(required(&attrs, "robot", "name")?.to_owned());
            state.robot_open = !empty;
            state.robot_closed = empty;
            Ok(true)
        }
        b"link" => {
            let attrs = attributes(start, decoder, "link", &["name"])?;
            if element_depth != 2
                || !state.robot_open
                || state.current_link.is_some()
                || state.current_joint.is_some()
            {
                return Err(UrdfParseError::Context { element: "link" });
            }
            state.current_link = Some(LinkBuilder {
                name: required(&attrs, "link", "name")?.to_owned(),
                inertia: None,
            });
            if empty {
                finish_link(state)?;
            }
            Ok(true)
        }
        b"inertial" => {
            let _ = attributes(start, decoder, "inertial", &[])?;
            if element_depth != 3
                || state.current_link.is_none()
                || state.current_joint.is_some()
                || state.current_inertia.is_some()
            {
                return Err(UrdfParseError::Context {
                    element: "inertial",
                });
            }
            state.current_inertia = Some(InertiaBuilder::default());
            if empty {
                finish_inertia(state)?;
            }
            Ok(true)
        }
        b"joint" => {
            let attrs = attributes(start, decoder, "joint", &["name", "type"])?;
            if element_depth != 2
                || !state.robot_open
                || state.current_link.is_some()
                || state.current_joint.is_some()
            {
                return Err(UrdfParseError::Context { element: "joint" });
            }
            state.current_joint = Some(JointBuilder {
                name: required(&attrs, "joint", "name")?.to_owned(),
                kind: parse_joint_kind(required(&attrs, "joint", "type")?)?,
                origin_xyz: [0.0; 3],
                origin_rpy: [0.0; 3],
                origin_seen: false,
                parent: None,
                child: None,
                axis: [1.0, 0.0, 0.0],
                axis_seen: false,
                limit: ParsedLimit::default(),
                limit_seen: false,
                mimic: None,
            });
            if empty {
                finish_joint(state)?;
            }
            Ok(true)
        }
        b"origin" => {
            let attrs = attributes(start, decoder, "origin", &["xyz", "rpy"])?;
            let xyz = optional_vec3(&attrs, "origin", "xyz")?.unwrap_or([0.0; 3]);
            let rpy = optional_vec3(&attrs, "origin", "rpy")?.unwrap_or([0.0; 3]);
            if let Some(inertia) = &mut state.current_inertia {
                if element_depth != 4 {
                    return Err(UrdfParseError::Context { element: "origin" });
                }
                if inertia.origin_seen {
                    return Err(UrdfParseError::DuplicateElement { element: "origin" });
                }
                inertia.origin_seen = true;
                inertia.origin_xyz = xyz;
                inertia.origin_rpy = rpy;
            } else if let Some(joint) = &mut state.current_joint {
                if element_depth != 3 {
                    return Err(UrdfParseError::Context { element: "origin" });
                }
                if joint.origin_seen {
                    return Err(UrdfParseError::DuplicateElement { element: "origin" });
                }
                joint.origin_seen = true;
                joint.origin_xyz = xyz;
                joint.origin_rpy = rpy;
            } else {
                return Err(UrdfParseError::Context { element: "origin" });
            }
            Ok(true)
        }
        b"mass" => {
            let attrs = attributes(start, decoder, "mass", &["value"])?;
            let value = parse_f64(required(&attrs, "mass", "value")?, "mass", "value")?;
            if element_depth != 4 {
                return Err(UrdfParseError::Context { element: "mass" });
            }
            let inertia = state
                .current_inertia
                .as_mut()
                .ok_or(UrdfParseError::Context { element: "mass" })?;
            if inertia.mass.replace(value).is_some() {
                return Err(UrdfParseError::DuplicateElement { element: "mass" });
            }
            Ok(true)
        }
        b"inertia" => {
            let attrs = attributes(
                start,
                decoder,
                "inertia",
                &["ixx", "ixy", "ixz", "iyy", "iyz", "izz"],
            )?;
            let matrix = [
                parse_required_f64(&attrs, "inertia", "ixx")?,
                parse_required_f64(&attrs, "inertia", "ixy")?,
                parse_required_f64(&attrs, "inertia", "ixz")?,
                parse_required_f64(&attrs, "inertia", "iyy")?,
                parse_required_f64(&attrs, "inertia", "iyz")?,
                parse_required_f64(&attrs, "inertia", "izz")?,
            ];
            if element_depth != 4 {
                return Err(UrdfParseError::Context { element: "inertia" });
            }
            let inertia = state
                .current_inertia
                .as_mut()
                .ok_or(UrdfParseError::Context { element: "inertia" })?;
            if inertia.matrix.replace(matrix).is_some() {
                return Err(UrdfParseError::DuplicateElement { element: "inertia" });
            }
            Ok(true)
        }
        b"parent" => {
            let attrs = attributes(start, decoder, "parent", &["link"])?;
            if element_depth != 3 {
                return Err(UrdfParseError::Context { element: "parent" });
            }
            let joint = state
                .current_joint
                .as_mut()
                .ok_or(UrdfParseError::Context { element: "parent" })?;
            if joint
                .parent
                .replace(required(&attrs, "parent", "link")?.to_owned())
                .is_some()
            {
                return Err(UrdfParseError::DuplicateElement { element: "parent" });
            }
            Ok(true)
        }
        b"child" => {
            let attrs = attributes(start, decoder, "child", &["link"])?;
            if element_depth != 3 {
                return Err(UrdfParseError::Context { element: "child" });
            }
            let joint = state
                .current_joint
                .as_mut()
                .ok_or(UrdfParseError::Context { element: "child" })?;
            if joint
                .child
                .replace(required(&attrs, "child", "link")?.to_owned())
                .is_some()
            {
                return Err(UrdfParseError::DuplicateElement { element: "child" });
            }
            Ok(true)
        }
        b"axis" => {
            let attrs = attributes(start, decoder, "axis", &["xyz"])?;
            let axis = parse_vec3(required(&attrs, "axis", "xyz")?, "axis", "xyz")?;
            if element_depth != 3 {
                return Err(UrdfParseError::Context { element: "axis" });
            }
            let joint = state
                .current_joint
                .as_mut()
                .ok_or(UrdfParseError::Context { element: "axis" })?;
            if joint.axis_seen {
                return Err(UrdfParseError::DuplicateElement { element: "axis" });
            }
            joint.axis_seen = true;
            joint.axis = axis;
            Ok(true)
        }
        b"limit" => {
            let attrs = attributes(
                start,
                decoder,
                "limit",
                &["lower", "upper", "effort", "velocity"],
            )?;
            if element_depth != 3 {
                return Err(UrdfParseError::Context { element: "limit" });
            }
            let joint = state
                .current_joint
                .as_mut()
                .ok_or(UrdfParseError::Context { element: "limit" })?;
            if joint.limit_seen {
                return Err(UrdfParseError::DuplicateElement { element: "limit" });
            }
            joint.limit_seen = true;
            joint.limit = ParsedLimit {
                lower: optional_f64(&attrs, "limit", "lower")?,
                upper: optional_f64(&attrs, "limit", "upper")?,
                effort: optional_f64(&attrs, "limit", "effort")?,
                velocity: optional_f64(&attrs, "limit", "velocity")?,
            };
            Ok(true)
        }
        b"mimic" => {
            let attrs = attributes(start, decoder, "mimic", &["joint", "multiplier", "offset"])?;
            if element_depth != 3 {
                return Err(UrdfParseError::Context { element: "mimic" });
            }
            let mimic = ParsedMimic {
                joint: required(&attrs, "mimic", "joint")?.to_owned(),
                multiplier: optional_f64(&attrs, "mimic", "multiplier")?,
                offset: optional_f64(&attrs, "mimic", "offset")?,
            };
            let joint = state
                .current_joint
                .as_mut()
                .ok_or(UrdfParseError::Context { element: "mimic" })?;
            if joint.mimic.replace(mimic).is_some() {
                return Err(UrdfParseError::DuplicateElement { element: "mimic" });
            }
            Ok(true)
        }
        // Collision, visual, material, geometry and vendor/deployment
        // extension subtrees are not model inputs in this delivery.
        _ => Ok(false),
    }
}

fn handle_end(
    state: &mut ParserState,
    name: &[u8],
    element_depth: usize,
) -> Result<(), UrdfParseError> {
    match name {
        b"robot" if element_depth == 1 && state.robot_open => {
            if state.current_link.is_some()
                || state.current_joint.is_some()
                || state.current_inertia.is_some()
            {
                return Err(UrdfParseError::Context { element: "robot" });
            }
            state.robot_open = false;
            state.robot_closed = true;
            Ok(())
        }
        b"link" if element_depth == 2 => finish_link(state),
        b"joint" if element_depth == 2 => finish_joint(state),
        b"inertial" if element_depth == 3 => finish_inertia(state),
        b"robot" => Err(UrdfParseError::Context { element: "robot" }),
        b"link" => Err(UrdfParseError::Context { element: "link" }),
        b"joint" => Err(UrdfParseError::Context { element: "joint" }),
        b"inertial" => Err(UrdfParseError::Context {
            element: "inertial",
        }),
        _ => Ok(()),
    }
}

fn finish_link(state: &mut ParserState) -> Result<(), UrdfParseError> {
    if state.current_inertia.is_some() {
        return Err(UrdfParseError::Context { element: "link" });
    }
    let link = state
        .current_link
        .take()
        .ok_or(UrdfParseError::Context { element: "link" })?;
    state.links.push(ParsedLink {
        name: link.name,
        inertia: link.inertia,
    });
    Ok(())
}

fn finish_inertia(state: &mut ParserState) -> Result<(), UrdfParseError> {
    let inertia = state
        .current_inertia
        .take()
        .ok_or(UrdfParseError::Context {
            element: "inertial",
        })?;
    let parsed = ParsedInertia {
        origin_xyz: inertia.origin_xyz,
        origin_rpy: inertia.origin_rpy,
        mass: inertia.mass.ok_or(UrdfParseError::Missing {
            element: "inertial",
            field: "mass",
        })?,
        matrix: inertia.matrix.ok_or(UrdfParseError::Missing {
            element: "inertial",
            field: "inertia",
        })?,
    };
    let link = state.current_link.as_mut().ok_or(UrdfParseError::Context {
        element: "inertial",
    })?;
    if link.inertia.replace(parsed).is_some() {
        return Err(UrdfParseError::DuplicateElement {
            element: "inertial",
        });
    }
    Ok(())
}

fn finish_joint(state: &mut ParserState) -> Result<(), UrdfParseError> {
    let joint = state
        .current_joint
        .take()
        .ok_or(UrdfParseError::Context { element: "joint" })?;
    state.joints.push(ParsedJoint {
        name: joint.name,
        kind: joint.kind,
        origin_xyz: joint.origin_xyz,
        origin_rpy: joint.origin_rpy,
        parent: joint.parent.ok_or(UrdfParseError::Missing {
            element: "joint",
            field: "parent",
        })?,
        child: joint.child.ok_or(UrdfParseError::Missing {
            element: "joint",
            field: "child",
        })?,
        axis: joint.axis,
        limit: joint.limit,
        mimic: joint.mimic,
    });
    Ok(())
}

fn attributes(
    start: &BytesStart<'_>,
    decoder: Decoder,
    element: &'static str,
    allowed: &[&str],
) -> Result<BTreeMap<String, String>, UrdfParseError> {
    let mut values = BTreeMap::new();
    for (index, attribute) in start.attributes().with_checks(true).enumerate() {
        if index >= MAX_ATTRIBUTES_PER_ELEMENT {
            return Err(UrdfParseError::Bound {
                field: "attributes per element",
                maximum: MAX_ATTRIBUTES_PER_ELEMENT,
            });
        }
        let attribute = attribute?;
        let key = str::from_utf8(attribute.key.as_ref())?.to_owned();
        if !allowed.contains(&key.as_str()) {
            return Err(UrdfParseError::UnknownAttribute {
                element,
                attribute: key,
            });
        }
        let value = attribute
            .decoded_and_normalized_value(XmlVersion::Implicit1_0, decoder)?
            .into_owned();
        values.insert(key, value);
    }
    Ok(values)
}

fn required<'a>(
    values: &'a BTreeMap<String, String>,
    element: &'static str,
    field: &'static str,
) -> Result<&'a str, UrdfParseError> {
    values
        .get(field)
        .map(String::as_str)
        .ok_or(UrdfParseError::Missing { element, field })
}

fn optional_f64(
    values: &BTreeMap<String, String>,
    element: &'static str,
    field: &'static str,
) -> Result<Option<f64>, UrdfParseError> {
    values
        .get(field)
        .map(|value| parse_f64(value, element, field))
        .transpose()
}

fn parse_required_f64(
    values: &BTreeMap<String, String>,
    element: &'static str,
    field: &'static str,
) -> Result<f64, UrdfParseError> {
    parse_f64(required(values, element, field)?, element, field)
}

fn parse_f64(
    value: &str,
    element: &'static str,
    field: &'static str,
) -> Result<f64, UrdfParseError> {
    value
        .parse()
        .map_err(|_| UrdfParseError::Numeric { element, field })
}

fn optional_vec3(
    values: &BTreeMap<String, String>,
    element: &'static str,
    field: &'static str,
) -> Result<Option<[f64; 3]>, UrdfParseError> {
    values
        .get(field)
        .map(|value| parse_vec3(value, element, field))
        .transpose()
}

fn parse_vec3(
    value: &str,
    element: &'static str,
    field: &'static str,
) -> Result<[f64; 3], UrdfParseError> {
    let mut parts = value.split_whitespace();
    let x = parts
        .next()
        .ok_or(UrdfParseError::Numeric { element, field })?;
    let y = parts
        .next()
        .ok_or(UrdfParseError::Numeric { element, field })?;
    let z = parts
        .next()
        .ok_or(UrdfParseError::Numeric { element, field })?;
    if parts.next().is_some() {
        return Err(UrdfParseError::Numeric { element, field });
    }
    Ok([
        parse_f64(x, element, field)?,
        parse_f64(y, element, field)?,
        parse_f64(z, element, field)?,
    ])
}

fn parse_joint_kind(value: &str) -> Result<ParsedJointKind, UrdfParseError> {
    match value {
        "revolute" => Ok(ParsedJointKind::Revolute),
        "continuous" => Ok(ParsedJointKind::Continuous),
        "prismatic" => Ok(ParsedJointKind::Prismatic),
        "fixed" => Ok(ParsedJointKind::Fixed),
        "floating" => Ok(ParsedJointKind::Floating),
        "planar" => Ok(ParsedJointKind::Planar),
        "spherical" => Ok(ParsedJointKind::Spherical),
        _ => Err(UrdfParseError::JointType {
            value: value.to_owned(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::{UrdfParseError, parse_urdf};

    #[test]
    fn rejects_doctype_unknown_critical_attributes_and_large_attribute_sets() {
        assert!(matches!(
            parse_urdf(r#"<!DOCTYPE robot><robot name="x"><link name="base"/></robot>"#),
            Err(UrdfParseError::DocType)
        ));
        assert!(matches!(
            parse_urdf(r#"<robot name="x" executable="yes"><link name="base"/></robot>"#),
            Err(UrdfParseError::UnknownAttribute { .. })
        ));
    }

    #[test]
    fn rejects_out_of_context_and_duplicate_zero_valued_elements() {
        assert!(matches!(
            parse_urdf(r#"<link name="base"/>"#),
            Err(UrdfParseError::Context { .. })
        ));
        assert!(matches!(
            parse_urdf(
                r#"
                <robot name="x">
                  <link name="base"/>
                  <joint name="nested" type="fixed">
                    <parent link="base"><child link="base"/></parent>
                  </joint>
                </robot>
                "#,
            ),
            Err(UrdfParseError::Context { .. })
        ));
        assert!(matches!(
            parse_urdf(
                r#"
                <robot name="x">
                  <link name="base"/>
                  <link name="tip"/>
                  <joint name="duplicate" type="fixed">
                    <origin xyz="0 0 0"/>
                    <origin rpy="0 0 0"/>
                    <parent link="base"/>
                    <child link="tip"/>
                  </joint>
                </robot>
                "#,
            ),
            Err(UrdfParseError::DuplicateElement { element: "origin" })
        ));
    }

    #[test]
    fn ignores_collision_visual_and_vendor_extension_subtrees() {
        let robot = parse_urdf(
            r#"
            <robot name="fixture">
              <link name="base">
                <visual><geometry><box size="1 1 1"/></geometry></visual>
                <collision><geometry><sphere radius="1"/></geometry></collision>
              </link>
              <gazebo reference="base"><plugin name="x"/></gazebo>
            </robot>
            "#,
        )
        .unwrap();
        assert_eq!(robot.links.len(), 1);
        assert!(robot.links[0].inertia.is_none());
    }
}
