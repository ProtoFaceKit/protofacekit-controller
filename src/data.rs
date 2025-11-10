const DISPLAY_WIDTH: usize = 64;
const DISPLAY_HEIGHT: usize = 32;
const DISPLAYS: usize = 2;

/// Frame width is increased because frames are stacked horizontally
const FRAME_WIDTH: usize = DISPLAY_WIDTH * DISPLAYS;
const FRAME_HEIGHT: usize = DISPLAY_HEIGHT;

/// Maximum number of frames allowed for the whole "face"
const MAX_FRAMES_PER_FACE: usize = 300;

/// Maximum number of uncompressed frame equivalents to allow
const MAX_UNCOMPRESSED_FRAMES: usize = 200;

/// Capacity for storing pixel data
const PIXEL_DATA_CAPACITY: usize = FRAME_WIDTH * FRAME_HEIGHT * MAX_UNCOMPRESSED_FRAMES;

#[derive(Clone, Copy)]
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

#[derive(Default)]
pub struct Face {
    /// Pixel data for each face expression
    pixels: heapless::Vec<RLEPixelData, PIXEL_DATA_CAPACITY, usize>,

    /// Frame data
    frames: heapless::Vec<FaceFrame, MAX_FRAMES_PER_FACE, usize>,

    /// Current frame being written to
    current_frame: Option<usize>,

    /// Expression data
    expressions: [Option<FaceExpression>; 2],

    /// Current expression begin written to
    current_expression: Option<usize>,
}

pub enum BeginFrameError {
    /// No active expression
    NoExpression,
    /// Limit for total frames reached
    FrameLimit,
    /// Limit for per-expression frames reached
    ExpressionFrameLimit,
}

pub enum PushPixelError {
    /// No active frame
    NoFrame,
    /// Reached limit for total pixels
    PixelLimit,
    /// Reacted limit for pixels within the frame
    FramePixelLimit,
}

pub enum BeginExpressionError {
    ExpressionExists,
}

impl Face {
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

        self.frames
            .push(FaceFrame {
                index: start_index,
                length: 0,
                duration,
            })
            .map_err(|_| BeginFrameError::FrameLimit)?;

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
            self.pixels
                .push(pixel)
                .map_err(|_| PushPixelError::PixelLimit)?;

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
pub struct RLEPixelData {
    /// Run of the pixel data
    pub length: u16,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl RLEPixelData {
    /// Try and obtain a [RLEPixelData] from a byte iterator
    pub fn from_iterator<'a, I: Iterator<Item = &'a u8>>(iter: &mut I) -> Option<RLEPixelData> {
        let length_byte_1 = iter.next()?;
        let length_byte_2 = iter.next()?;

        let r = *iter.next()?;
        let g = *iter.next()?;
        let b = *iter.next()?;

        let length = u16::from_be_bytes([*length_byte_1, *length_byte_2]);

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
