use alloc::vec::Vec;

pub const DISPLAY_WIDTH: usize = 64;
pub const DISPLAY_HEIGHT: usize = 32;
pub const DISPLAYS: usize = 2;

/// Frame width is increased because frames are stacked horizontally
pub const FRAME_WIDTH: usize = DISPLAY_WIDTH * DISPLAYS;
pub const FRAME_HEIGHT: usize = DISPLAY_HEIGHT;

/// Maximum number of frames allowed for the whole "face"
const MAX_FRAMES_PER_FACE: usize = 4096;

/// Maximum number of uncompressed frame equivalents to allow
const MAX_UNCOMPRESSED_FRAMES: usize = 5;

/// Capacity for storing pixel data
const PIXEL_DATA_CAPACITY: usize = FRAME_WIDTH * FRAME_HEIGHT * MAX_UNCOMPRESSED_FRAMES;

#[derive(defmt::Format, Clone, Copy)]
pub struct Expression(u8);

impl Expression {
    pub const IDLE: Expression = Expression(0);
    pub const TALKING: Expression = Expression(1);

    pub fn from_value(value: u8) -> Option<Expression> {
        match value {
            0 => Some(Self::IDLE),
            1 => Some(Self::TALKING),
            _ => None,
        }
    }
}

pub struct Face {
    /// Pixel data for each face expression
    pub pixels: Vec<RLEPixelData>,

    /// Frame data
    frames: Vec<FaceFrame>,

    /// Current frame being written to
    current_frame: Option<usize>,

    /// Expression data
    expressions: [Option<FaceExpression>; 2],

    /// Current expression begin written to
    current_expression: Option<usize>,
}

impl Default for Face {
    fn default() -> Self {
        let pixels = Vec::with_capacity(PIXEL_DATA_CAPACITY);
        defmt::info!(
            "allocated {} bytes capacity for pixel data",
            pixels.capacity() * core::mem::size_of::<RLEPixelData>()
        );

        Self {
            pixels,
            frames: Vec::with_capacity(MAX_FRAMES_PER_FACE),
            current_frame: Default::default(),
            expressions: Default::default(),
            current_expression: Default::default(),
        }
    }
}

#[derive(defmt::Format)]
pub enum BeginFrameError {
    /// No active expression
    NoExpression,
    /// Limit for total frames reached
    FrameLimit,
    /// Limit for per-expression frames reached
    ExpressionFrameLimit,
}

#[derive(defmt::Format)]
pub enum PushPixelError {
    /// No active frame
    NoFrame,
    /// Reached limit for total pixels
    PixelLimit,
    /// Reacted limit for pixels within the frame
    FramePixelLimit,
}

#[derive(defmt::Format)]
pub enum BeginExpressionError {
    ExpressionExists,
}

impl Face {
    /// Get all the frames for an expression
    pub fn get_expression_frames(&self, expression: Expression) -> Option<&[FaceFrame]> {
        let expression_index = expression.0 as usize;
        let expression = self.expressions[expression_index].as_ref()?;

        self.frames
            .get(expression.index..(expression.index + expression.length as usize))
    }

    /// Get all pixels for a frame
    pub fn get_frame_pixels(&self, frame: &FaceFrame) -> Option<&[RLEPixelData]> {
        self.pixels
            .get(frame.index..(frame.index + frame.length as usize))
    }

    /// Begin writing an expression
    pub fn begin_expression(&mut self, expression: Expression) -> Result<(), BeginExpressionError> {
        let expression_index = expression.0 as usize;

        if self.expressions[expression_index].is_some() {
            return Err(BeginExpressionError::ExpressionExists);
        }

        let start_index = self.frames.len();
        self.expressions[expression_index] = Some(FaceExpression {
            index: start_index,
            length: 0,
        });

        self.current_expression = Some(expression_index);
        Ok(())
    }

    /// Begin a new frame on the current expression
    pub fn begin_frame(&mut self, duration: u8) -> Result<(), BeginFrameError> {
        let expression_index = self
            .current_expression
            .ok_or(BeginFrameError::NoExpression)?;

        let start_index = self.pixels.len();
        let index = self.frames.len();

        if index == self.frames.capacity() {
            return Err(BeginFrameError::FrameLimit);
        }

        self.frames.push(FaceFrame {
            index: start_index,
            length: 0,
            duration,
        });

        let expression = self.expressions[expression_index]
            .as_mut()
            .expect("expression must exist");

        expression.length = expression
            .length
            .checked_add(1)
            .ok_or(BeginFrameError::ExpressionFrameLimit)?;

        self.current_frame = Some(index);

        Ok(())
    }

    pub fn push_pixels(
        &mut self,
        pixels: impl Iterator<Item = RLEPixelData>,
    ) -> Result<(), PushPixelError> {
        let frame_index = self.current_frame.ok_or(PushPixelError::NoFrame)?;
        let frame = self.frames.get_mut(frame_index).expect("frame must exist");

        for pixel in pixels {
            if self.pixels.len() == self.pixels.capacity() {
                return Err(PushPixelError::PixelLimit);
            }

            self.pixels.push(pixel);

            frame.length = frame
                .length
                .checked_add(1)
                .ok_or(PushPixelError::FramePixelLimit)?;
        }

        Ok(())
    }

    pub fn reset(&mut self) {
        self.pixels.clear();
        self.frames.clear();

        for i in 0..self.expressions.len() {
            self.expressions[i] = None;
        }
    }
}

pub struct FaceFrame {
    /// Start index within the [`Face::pixels`] buffer for the pixel data of
    /// this frame
    pub index: usize,

    /// Total number of pixel data items from [`Face::pixels`] this frame
    /// uses
    pub length: u16,

    /// Milliseconds to display the frame for
    pub duration: u8,
}

pub struct FaceExpression {
    /// Start index of the frames within the [`Face::frames`] buffer for this
    /// expression
    pub index: usize,

    /// Length in frames of the expression (Number of items in [`Face::frames`])
    pub length: u8,
}

/// Represents a run-length encoded pixel data
#[derive(Debug, Copy, Clone, Default)]
pub struct RLEPixelData {
    /// Run of the pixel data
    pub length: u8,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl RLEPixelData {
    /// Try and obtain a [RLEPixelData] from a byte iterator
    pub fn from_iterator<'a, I: Iterator<Item = &'a u8>>(iter: &mut I) -> Option<RLEPixelData> {
        let length = *iter.next()?;
        let r = *iter.next()?;
        let g = *iter.next()?;
        let b = *iter.next()?;
        Some(Self { length, r, g, b })
    }
}

pub struct RLESlicePixelIterator<'a> {
    bytes: core::slice::Iter<'a, u8>,
}

impl<'a> RLESlicePixelIterator<'a> {
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes: bytes.iter(),
        }
    }
}

impl<'a> Iterator for RLESlicePixelIterator<'a> {
    type Item = RLEPixelData;

    fn next(&mut self) -> Option<Self::Item> {
        RLEPixelData::from_iterator(&mut self.bytes)
    }
}

/// RGB pixel data producer from RLE data
pub struct RLEPixelProducerIterator<'a> {
    // Current pixel data entry and number of produced values
    current: Option<(&'a RLEPixelData, u8)>,

    /// Iterator for next pixel data
    iterator: core::slice::Iter<'a, RLEPixelData>,
}

impl<'a> RLEPixelProducerIterator<'a> {
    pub fn new(pixel_data: &'a [RLEPixelData]) -> Self {
        Self {
            current: None,
            iterator: pixel_data.iter(),
        }
    }
}

impl<'a> Iterator for RLEPixelProducerIterator<'a> {
    type Item = (u8, u8, u8);

    fn next(&mut self) -> Option<Self::Item> {
        let pixel_data = match self.current.as_mut() {
            Some(value) if value.1 < value.0.length => value,

            // No value yet OR completely used the existing RLE data
            _ => {
                let value = self.iterator.next()?;
                self.current.insert((value, 0))
            }
        };

        let rgb = pixel_data.0;
        pixel_data.1 += 1;
        Some((rgb.r, rgb.g, rgb.b))
    }
}
