/// Read bounded JPEG headers before passing bytes to the external decoder.
/// Entropy validation remains FFmpeg's responsibility.
pub fn dimensions(data: &[u8]) -> Option<(u16, u16)> {
    if !data.starts_with(&[0xff, 0xd8]) || !data.ends_with(&[0xff, 0xd9]) {
        return None;
    }
    let mut offset = 2;
    while offset < data.len() {
        if data[offset] != 0xff {
            return None;
        }
        while *data.get(offset)? == 0xff {
            offset += 1;
        }
        let marker = *data.get(offset)?;
        offset += 1;
        if marker == 0xda || marker == 0xd9 {
            return None;
        }
        if marker == 0x01 || (0xd0..=0xd7).contains(&marker) {
            continue;
        }
        let length = u16::from_be_bytes([*data.get(offset)?, *data.get(offset + 1)?]) as usize;
        if length < 2 || offset.checked_add(length)? > data.len() {
            return None;
        }
        if matches!(marker, 0xc0..=0xc2) {
            if length < 8 || data[offset + 2] != 8 {
                return None;
            }
            let height = u16::from_be_bytes([data[offset + 3], data[offset + 4]]);
            let width = u16::from_be_bytes([data[offset + 5], data[offset + 6]]);
            let components = data[offset + 7];
            if width == 0
                || height == 0
                || !matches!(components, 1 | 3)
                || length != 8 + 3 * usize::from(components)
            {
                return None;
            }
            return Some((width, height));
        }
        offset += length;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::dimensions;

    #[test]
    fn rejects_truncation_and_invalid_components() {
        let valid = [
            0xff, 0xd8, 0xff, 0xc0, 0, 11, 8, 0, 16, 0, 32, 1, 1, 0x11, 0, 0xff, 0xd9,
        ];
        assert_eq!(dimensions(&valid), Some((32, 16)));
        for length in 0..valid.len() {
            assert_eq!(dimensions(&valid[..length]), None);
        }
        let mut invalid = valid;
        invalid[11] = 255;
        assert_eq!(dimensions(&invalid), None);
        invalid = valid;
        invalid[5] = 0;
        assert_eq!(dimensions(&invalid), None);
        invalid = valid;
        invalid[9] = 0;
        invalid[10] = 0;
        assert_eq!(dimensions(&invalid), None);
    }
}
