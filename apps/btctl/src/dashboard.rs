//! Interactive read-only terminal dashboard for live navigation diagnostics.

use std::{
    collections::VecDeque,
    error::Error,
    fmt::Write as _,
    io::{self, Stdout},
    time::{Duration, Instant},
};

use crossterm::{
    event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use navigation_ipc::{
    NavigationParticipantState, NavigationParticipationSnapshot,
    NavigationTransportDiagnosticsSnapshot, ParticipationObserver, TransportDiagnosticsObserver,
};
use ratatui::{
    Frame, Terminal,
    backend::CrosstermBackend,
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Cell, Paragraph, Row, Table, Wrap},
};

pub(crate) const DEFAULT_REFRESH_MS: u64 = 250;
const MAX_LOG_LINES: usize = 256;

pub(crate) struct DashboardConfig {
    pub(crate) show_tree: bool,
    pub(crate) show_nodes: bool,
    pub(crate) show_transport: bool,
    pub(crate) interval: Duration,
    pub(crate) sample_limit: Option<u64>,
}

struct Observed<T> {
    generation: u64,
    value: T,
}

#[derive(Default)]
struct DashboardState {
    sample_index: u64,
    participation: Option<Observed<NavigationParticipationSnapshot>>,
    transport: Option<Observed<NavigationTransportDiagnosticsSnapshot>>,
    logs: VecDeque<String>,
}

impl DashboardState {
    fn update(
        &mut self,
        sample_index: u64,
        participation: Option<Observed<NavigationParticipationSnapshot>>,
        transport: Option<Observed<NavigationTransportDiagnosticsSnapshot>>,
    ) {
        if self.participation.is_some() && participation.is_none() {
            self.push_log(format!(
                "sample={sample_index} participation unavailable (missing, stale, or invalid)"
            ));
        }

        let new_logs = match transport.as_ref() {
            Some(current) => transport_log_lines(
                sample_index,
                self.transport.as_ref().map(|previous| &previous.value),
                current,
            ),
            None if self.transport.is_some() => vec![format!(
                "sample={sample_index} transport unavailable (missing, stale, or invalid)"
            )],
            None => Vec::new(),
        };
        for line in new_logs {
            self.push_log(line);
        }

        self.sample_index = sample_index;
        self.participation = participation;
        self.transport = transport;
    }

    fn push_log(&mut self, line: String) {
        if self.logs.len() == MAX_LOG_LINES {
            let _ = self.logs.pop_front();
        }
        self.logs.push_back(line);
    }
}

pub(crate) fn run(
    config: DashboardConfig,
    mut participation_observer: Option<ParticipationObserver>,
    mut transport_observer: Option<TransportDiagnosticsObserver>,
) -> Result<(), Box<dyn Error>> {
    let mut terminal = TerminalSession::enter()?;
    let mut state = DashboardState::default();
    let mut sample_index = 1_u64;

    loop {
        let participation = participation_observer
            .as_mut()
            .and_then(|observer| observer.try_observe_latest())
            .map(|sample| Observed {
                generation: sample.generation,
                value: sample.value,
            });
        let transport = transport_observer
            .as_mut()
            .and_then(|observer| observer.try_observe_latest())
            .map(|sample| Observed {
                generation: sample.generation,
                value: sample.value,
            });
        state.update(sample_index, participation, transport);
        terminal.draw(&state, &config)?;

        if config
            .sample_limit
            .is_some_and(|limit| sample_index >= limit)
        {
            return Ok(());
        }
        if wait_for_refresh(&mut terminal, &state, &config)? {
            return Ok(());
        }
        sample_index = sample_index.saturating_add(1);
    }
}

struct TerminalSession {
    terminal: Terminal<CrosstermBackend<Stdout>>,
}

impl TerminalSession {
    fn enter() -> io::Result<Self> {
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        if let Err(error) = execute!(stdout, EnterAlternateScreen) {
            let _ = disable_raw_mode();
            return Err(error);
        }
        let backend = CrosstermBackend::new(stdout);
        match Terminal::new(backend) {
            Ok(mut terminal) => {
                terminal.hide_cursor()?;
                terminal.clear()?;
                Ok(Self { terminal })
            }
            Err(error) => {
                let _ = disable_raw_mode();
                let mut stdout = io::stdout();
                let _ = execute!(stdout, LeaveAlternateScreen);
                Err(error)
            }
        }
    }

    fn draw(&mut self, state: &DashboardState, config: &DashboardConfig) -> io::Result<()> {
        self.terminal
            .draw(|frame| render_dashboard(frame, state, config))?;
        Ok(())
    }
}

impl Drop for TerminalSession {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let _ = execute!(self.terminal.backend_mut(), LeaveAlternateScreen);
        let _ = self.terminal.show_cursor();
    }
}

fn wait_for_refresh(
    terminal: &mut TerminalSession,
    state: &DashboardState,
    config: &DashboardConfig,
) -> io::Result<bool> {
    let deadline = Instant::now() + config.interval;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() || !event::poll(remaining)? {
            return Ok(false);
        }
        match event::read()? {
            Event::Key(key) if key.kind == KeyEventKind::Press && should_exit(key) => {
                return Ok(true);
            }
            Event::Resize(_, _) => terminal.draw(state, config)?,
            _ => {}
        }
    }
}

fn should_exit(key: KeyEvent) -> bool {
    matches!(key.code, KeyCode::Esc | KeyCode::Char('q'))
        || (key.code == KeyCode::Char('c') && key.modifiers.contains(KeyModifiers::CONTROL))
}

fn render_dashboard(frame: &mut Frame<'_>, state: &DashboardState, config: &DashboardConfig) {
    let [header, body, footer] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Min(3),
        Constraint::Length(1),
    ])
    .areas(frame.area());
    render_header(frame, header, state, config);
    render_sections(frame, body, state, config);
    render_footer(frame, footer, config);
}

fn render_header(
    frame: &mut Frame<'_>,
    area: Rect,
    state: &DashboardState,
    config: &DashboardConfig,
) {
    let online = required_data_available(state, config);
    let status = if online { "● LIVE" } else { "● WAITING" };
    let status_style = Style::default()
        .fg(if online { Color::Green } else { Color::Yellow })
        .add_modifier(Modifier::BOLD);
    let line = Line::from(vec![
        Span::styled(
            " btctl observe ",
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::styled(status, status_style),
        Span::raw(format!(
            "   sample={}   refresh={}ms",
            state.sample_index,
            config.interval.as_millis()
        )),
    ]);
    frame.render_widget(Paragraph::new(line), area);
}

fn required_data_available(state: &DashboardState, config: &DashboardConfig) -> bool {
    (!config.show_tree && !config.show_nodes || state.participation.is_some())
        && (!config.show_transport || state.transport.is_some())
}

fn render_sections(
    frame: &mut Frame<'_>,
    area: Rect,
    state: &DashboardState,
    config: &DashboardConfig,
) {
    let section_count = usize::from(config.show_tree)
        + usize::from(config.show_nodes)
        + usize::from(config.show_transport);
    let constraints = match section_count {
        3 => vec![
            Constraint::Length(5),
            Constraint::Length(12),
            Constraint::Min(5),
        ],
        2 => vec![Constraint::Percentage(45), Constraint::Min(5)],
        _ => vec![Constraint::Min(3)],
    };
    let areas = Layout::vertical(constraints).split(area);
    let mut next = 0;
    if config.show_tree {
        render_tree(frame, areas[next], state.participation.as_ref());
        next += 1;
    }
    if config.show_nodes {
        render_nodes(frame, areas[next], state.participation.as_ref());
        next += 1;
    }
    if config.show_transport {
        render_transport_log(frame, areas[next], state);
    }
}

fn render_tree(
    frame: &mut Frame<'_>,
    area: Rect,
    observed: Option<&Observed<NavigationParticipationSnapshot>>,
) {
    let block = Block::default()
        .title(" Behavior tree ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));
    let Some(observed) = observed else {
        frame.render_widget(
            unavailable(block, "participation snapshot unavailable"),
            area,
        );
        return;
    };
    let snapshot = &observed.value;
    let header = snapshot.header();
    let mut lines = vec![Line::from(Span::styled(
        format!(
            "generation={} sequence={} epoch={} source={}",
            observed.generation,
            header.sequence(),
            snapshot.process_epoch(),
            header.source_id()
        ),
        Style::default().fg(Color::DarkGray),
    ))];
    for participant in snapshot
        .participants()
        .iter()
        .filter(|participant| participant.id().as_str().starts_with("tree."))
    {
        let root = participant.id().as_str() == "tree.root";
        lines.push(Line::from(vec![
            Span::raw(if root { "● " } else { "  └─ " }),
            Span::styled(
                participant.id().as_str().to_owned(),
                Style::default().add_modifier(Modifier::BOLD),
            ),
            Span::raw("  "),
            Span::styled(
                participant.state().as_str().to_ascii_uppercase(),
                state_style(participant.state()),
            ),
            Span::raw(format!("  {}", participant.reason().as_str())),
        ]));
    }
    frame.render_widget(
        Paragraph::new(lines).block(block).wrap(Wrap { trim: true }),
        area,
    );
}

fn render_nodes(
    frame: &mut Frame<'_>,
    area: Rect,
    observed: Option<&Observed<NavigationParticipationSnapshot>>,
) {
    let block = Block::default()
        .title(" Node states ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let Some(observed) = observed else {
        frame.render_widget(
            unavailable(block, "participation snapshot unavailable"),
            area,
        );
        return;
    };
    let rows = observed.value.participants().iter().map(|participant| {
        Row::new(vec![
            Cell::from(participant.id().as_str().to_owned()),
            Cell::from(participant.state().as_str().to_ascii_uppercase())
                .style(state_style(participant.state())),
            Cell::from(participant.reason().as_str().to_owned()),
        ])
    });
    let header = Row::new(["NODE", "STATE", "REASON"]).style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );
    let table = Table::new(
        rows,
        [
            Constraint::Length(26),
            Constraint::Length(10),
            Constraint::Min(12),
        ],
    )
    .header(header)
    .column_spacing(1)
    .block(block);
    frame.render_widget(table, area);
}

fn render_transport_log(frame: &mut Frame<'_>, area: Rect, state: &DashboardState) {
    let generation = state
        .transport
        .as_ref()
        .map_or_else(|| "-".to_owned(), |sample| sample.generation.to_string());
    let block = Block::default()
        .title(format!(" Transport send log · generation={generation} "))
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Magenta));
    let visible = usize::from(area.height.saturating_sub(2));
    let start = state.logs.len().saturating_sub(visible);
    let lines = state.logs.iter().skip(start).map(|entry| {
        let style = if entry.contains("unavailable") {
            Style::default().fg(Color::Red)
        } else {
            Style::default().fg(Color::Gray)
        };
        Line::from(Span::styled(entry.clone(), style))
    });
    frame.render_widget(Paragraph::new(lines.collect::<Vec<_>>()).block(block), area);
}

fn unavailable<'a>(block: Block<'a>, message: &'a str) -> Paragraph<'a> {
    Paragraph::new(Line::from(Span::styled(
        message,
        Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD),
    )))
    .block(block)
}

fn state_style(state: NavigationParticipantState) -> Style {
    let color = match state {
        NavigationParticipantState::Inactive => Color::DarkGray,
        NavigationParticipantState::Pending => Color::Yellow,
        NavigationParticipantState::Active => Color::Green,
        NavigationParticipantState::Terminal => Color::Cyan,
        NavigationParticipantState::Faulted => Color::Red,
    };
    Style::default().fg(color).add_modifier(Modifier::BOLD)
}

fn render_footer(frame: &mut Frame<'_>, area: Rect, config: &DashboardConfig) {
    let line = Line::from(vec![
        Span::styled(
            " q / Esc / Ctrl+C ",
            Style::default().fg(Color::Black).bg(Color::Cyan),
        ),
        Span::raw(format!(
            " exit   read-only observer   next refresh in {} ms",
            config.interval.as_millis()
        )),
    ]);
    frame.render_widget(Paragraph::new(line), area);
}

fn transport_log_lines(
    sample_index: u64,
    previous: Option<&NavigationTransportDiagnosticsSnapshot>,
    current: &Observed<NavigationTransportDiagnosticsSnapshot>,
) -> Vec<String> {
    let reset = previous.is_some_and(|snapshot| {
        snapshot.process_epoch() != current.value.process_epoch()
            || snapshot.header().boot_id() != current.value.header().boot_id()
    });
    let previous = if reset { None } else { previous };
    let mut lines = Vec::new();
    for channel in current.value.channels() {
        let prior = previous.and_then(|snapshot| {
            snapshot
                .channels()
                .iter()
                .find(|candidate| candidate.channel_id() == channel.channel_id())
        });
        let counters = channel.counters();
        let diagnostics = channel.shm_diagnostics();
        let mut line = format!("#{sample_index} {}", channel.channel_id());
        match prior {
            None => {
                let _ = write!(
                    line,
                    " baseline pub={} con={} over={} full={} disc={}",
                    counters.published,
                    counters.consumed,
                    counters.overwritten,
                    counters.full,
                    counters.disconnected
                );
                lines.push(line);
            }
            Some(prior) => {
                let prior_counters = prior.counters();
                let prior_diagnostics = prior.shm_diagnostics();
                let mut changed = false;
                append_delta(
                    &mut line,
                    "published",
                    counters.published,
                    prior_counters.published,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "consumed",
                    counters.consumed,
                    prior_counters.consumed,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "overwritten",
                    counters.overwritten,
                    prior_counters.overwritten,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "full",
                    counters.full,
                    prior_counters.full,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "disconnected",
                    counters.disconnected,
                    prior_counters.disconnected,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "attach_rejected",
                    diagnostics.attach_rejected,
                    prior_diagnostics.attach_rejected,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "read_races",
                    diagnostics.read_races,
                    prior_diagnostics.read_races,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "decode_rejected",
                    diagnostics.decode_rejected,
                    prior_diagnostics.decode_rejected,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "reclaimed_endpoints",
                    diagnostics.reclaimed_endpoints,
                    prior_diagnostics.reclaimed_endpoints,
                    &mut changed,
                );
                append_delta(
                    &mut line,
                    "skipped_backlog",
                    diagnostics.skipped_backlog,
                    prior_diagnostics.skipped_backlog,
                    &mut changed,
                );
                if changed {
                    lines.push(line);
                }
            }
        }
    }
    lines
}

fn append_delta(line: &mut String, label: &str, current: u64, previous: u64, changed: &mut bool) {
    let delta = current.saturating_sub(previous);
    if delta > 0 {
        let _ = write!(line, " {label}=+{delta}");
        *changed = true;
    }
}

#[cfg(test)]
mod tests {
    use super::{DashboardConfig, DashboardState, append_delta, render_dashboard, should_exit};
    use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
    use ratatui::{Terminal, backend::TestBackend};
    use std::time::Duration;

    #[test]
    fn exit_keys_include_q_escape_and_control_c() {
        assert!(should_exit(KeyEvent::new(
            KeyCode::Char('q'),
            KeyModifiers::NONE
        )));
        assert!(should_exit(KeyEvent::new(KeyCode::Esc, KeyModifiers::NONE)));
        assert!(should_exit(KeyEvent::new(
            KeyCode::Char('c'),
            KeyModifiers::CONTROL
        )));
        assert!(!should_exit(KeyEvent::new(
            KeyCode::Char('c'),
            KeyModifiers::NONE
        )));
    }

    #[test]
    fn delta_rendering_omits_unchanged_counters() {
        let mut line = String::from("channel=test");
        let mut changed = false;
        append_delta(&mut line, "published", 7, 7, &mut changed);
        assert!(!changed);
        assert_eq!(line, "channel=test");
        append_delta(&mut line, "published", 9, 7, &mut changed);
        assert!(changed);
        assert_eq!(line, "channel=test published=+2");
    }

    #[test]
    fn dashboard_has_fixed_tree_node_and_transport_sections() {
        let backend = TestBackend::new(120, 32);
        let mut terminal = Terminal::new(backend).expect("test terminal");
        let config = DashboardConfig {
            show_tree: true,
            show_nodes: true,
            show_transport: true,
            interval: Duration::from_millis(250),
            sample_limit: None,
        };
        terminal
            .draw(|frame| render_dashboard(frame, &DashboardState::default(), &config))
            .expect("dashboard renders");
        let screen = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();
        assert!(screen.contains("Behavior tree"));
        assert!(screen.contains("Node states"));
        assert!(screen.contains("Transport send log"));
        assert!(screen.contains("Ctrl+C"));
    }
}
