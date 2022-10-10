//! Define C structs that are transmitted over the socket.

use std::ffi::c_uint;

pub const BOXES_MAX_NUM: usize = 80;

#[derive(Debug, Clone)]
#[repr(C)]
pub struct YoloResult {
    class_count: c_uint,
    box_count: c_uint,
    boxes: [BoundingBox; BOXES_MAX_NUM],
}

impl YoloResult {
    pub fn class_count(&self) -> usize {
        self.class_count as usize
    }

    pub fn box_count(&self) -> usize {
        self.box_count as usize
    }

    pub fn boxes(&self) -> &[BoundingBox] {
        let count = self.box_count();
        &self.boxes[0..count]
    }
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct BoundingBox {
    /// top-left x corner
    pub x1: f32,
    /// top-left y corner
    pub y1: f32,
    /// bottom-right x corner
    pub x2: f32,
    /// bottom-right y corner
    pub y2: f32,
    /// probability score
    pub score: f32,
    /// class number (of many) with highest probability
    pub class_num: c_uint,
}
