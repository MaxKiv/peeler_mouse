#[derive(Copy, Clone, Debug)]
pub struct VerticalGradient {
    pub value: i32,
    pub row_idx: usize,
}

impl PartialEq for VerticalGradient {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl PartialOrd for VerticalGradient {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.value.partial_cmp(&other.value)
    }
}
