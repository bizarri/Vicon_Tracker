#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingState {
    Idle,
    Searching,
    Tracking,
    Lost,
}

impl TrackingState {
    pub fn as_str(&self) -> &'static str {
        match self {
            TrackingState::Idle => "Idle",
            TrackingState::Searching => "Searching",
            TrackingState::Tracking => "Tracking",
            TrackingState::Lost => "Lost",
        }
    }
}

#[derive(Debug, Clone)]
pub struct TrackingStateMachine {
    state: TrackingState,
}

impl TrackingStateMachine {
    pub fn new() -> Self {
        Self {
            state: TrackingState::Idle,
        }
    }

    pub fn state(&self) -> TrackingState {
        self.state
    }

    pub fn transition(&mut self, new_state: TrackingState) -> bool {
        if self.state == new_state {
            return false;
        }
        self.state = new_state;
        true
    }
}

impl Default for TrackingStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initial_state_is_idle() {
        let sm = TrackingStateMachine::new();
        assert_eq!(sm.state(), TrackingState::Idle);
    }

    #[test]
    fn transitions_between_states() {
        let mut sm = TrackingStateMachine::new();
        assert!(sm.transition(TrackingState::Searching));
        assert_eq!(sm.state(), TrackingState::Searching);
        assert!(sm.transition(TrackingState::Tracking));
        assert_eq!(sm.state(), TrackingState::Tracking);
        assert!(sm.transition(TrackingState::Lost));
        assert_eq!(sm.state(), TrackingState::Lost);
        assert!(sm.transition(TrackingState::Tracking));
        assert_eq!(sm.state(), TrackingState::Tracking);
    }

    #[test]
    fn duplicate_transition_returns_false() {
        let mut sm = TrackingStateMachine::new();
        assert!(sm.transition(TrackingState::Searching));
        assert!(!sm.transition(TrackingState::Searching));
        assert_eq!(sm.state(), TrackingState::Searching);
    }
}
