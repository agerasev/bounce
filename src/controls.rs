use bounce::DrawMode;

#[derive(Clone, Copy)]
pub enum Action {
    Add,
    Remove,
    Mode,
    Pause,
    Slow,
    Reset,
}

pub struct Controls {
    pub actions: Vec<Action>,
    pub mode: DrawMode,
    pub paused: bool,
    pub slow: bool,
    pub bodies: usize,
}
impl Default for Controls {
    fn default() -> Self {
        Self {
            actions: Vec::new(),
            mode: DrawMode::Normal,
            paused: false,
            slow: false,
            bodies: 0,
        }
    }
}
