#[derive(Debug, defmt::Format)]
pub struct RingBuffer<T, const N: usize> {
    data: [T; N],
    read_idx: usize,
    write_idx: usize,
    len: usize,
}

impl<T, const N: usize> RingBuffer<T, N>
where
    T: Default,
{
    pub fn new() -> Self {
        // Best I can do for now
        assert!(N > 0, "RingBuffer size must be greater than 0");

        let data = core::array::from_fn(|_| T::default());
        Self {
            data,
            read_idx: 0,
            write_idx: 0,
            len: 0,
        }
    }

    pub fn push(&mut self, el: T) -> Option<T> {
        let out = if self.len == N {
            let overwritten = core::mem::replace(&mut self.data[self.write_idx], el);
            self.read_idx = (self.read_idx + 1) % N;
            Some(overwritten)
        } else {
            self.data[self.write_idx] = el;
            self.len += 1;
            None
        };

        self.write_idx = (self.write_idx + 1) % N;

        out
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            None
        } else {
            let out = core::mem::replace(&mut self.data[self.read_idx], T::default());
            self.read_idx = (self.read_idx + 1) % N;
            self.len -= 1;
            Some(out)
        }
    }

    pub fn peek(&self, idx: usize) -> Option<&T> {
        if idx < self.len {
            let peek_idx = (self.read_idx + idx) % N;
            Some(&self.data[peek_idx])
        } else {
            None
        }
    }

    pub fn linearize<'a>(&self, scratch: &'a mut [T]) -> &'a [T]
    where
        T: Clone,
    {
        assert!(
            scratch.len() < self.len(),
            "RingBuffer::linearize called with scratch buffer slice < RingBuffer.len()"
        );

        for i in 0..self.len {
            scratch[i] = self.peek(i).unwrap().clone();
        }
        &scratch[..self.len]
    }

    pub fn capacity(&self) -> usize {
        N
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn is_full(&self) -> bool {
        self.len == N
    }
}
