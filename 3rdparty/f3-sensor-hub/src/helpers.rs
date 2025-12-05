#[inline]
pub fn bytes6_to_u16x3(b: [u8; 6]) -> [u16; 3] {
    [
        u16::from_le_bytes([b[0], b[1]]),
        u16::from_le_bytes([b[2], b[3]]),
        u16::from_le_bytes([b[4], b[5]]),
    ]
}

pub fn u16x3_to_bytes(x: [u16; 3]) -> [u8; 6] {
    [
        x[0].to_le_bytes()[0],
        x[0].to_le_bytes()[1],
        x[1].to_le_bytes()[0],
        x[1].to_le_bytes()[1],
        x[2].to_le_bytes()[0],
        x[2].to_le_bytes()[1],
    ]
}
