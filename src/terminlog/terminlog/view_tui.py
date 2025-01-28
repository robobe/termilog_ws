from textual.app import App, ComposeResult, SystemCommand
from textual.widgets import Static, Footer, Button, Label
from textual.containers import VerticalScroll, HorizontalGroup, Horizontal
from typing import NamedTuple
from textual.message import Message
from enum import IntEnum
from collections import deque
from textual.binding import Binding
from textual.screen import Screen
from typing import Iterable

class LogLevel(IntEnum):
    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50


class LogItem(NamedTuple):
    time: str
    message: str
    level: int
    name: str


class LogMessage(Message):
    def __init__(self, log_item: LogItem):
        super().__init__()
        self.message = log_item

class Stopwatch(HorizontalGroup):
    """A stopwatch widget."""

    def compose(self) -> ComposeResult:
        """Create child widgets of a stopwatch."""
        yield Button("Start", id="start", variant="success")
        yield Button("Stop", id="stop", variant="error")

class ViewTUI(App):
    CSS_PATH = "tui_style.tcss"
    t = Binding(key="t", action="xxx", description="XXX")
    BINDINGS = [
        Binding(key="q", action="quit", description="Quit"),
        t,
        ("i", "info", "info"),
        ("w", "warning", "warning"),
        ("e", "error", "error")
    ]

    CSS = """
        Horizontal#footer-outer {
            height: 1;
            dock: bottom;
        }
        Horizonal#footer-inner {
            width: 75%;
        }
        Label#right-label {
            width: 25%;
            text-align: right;
        }
    """
    def __init__(
        self, driver_class=None, css_path=None, watch_css=False, ansi_color=False
    ):
        super().__init__(driver_class, css_path, watch_css, ansi_color)
        self.storage = deque(maxlen=10)
        self.filter_levels = [LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR]
        self.filter_names = {}
        self.active_filter_names = []

    def register_filter_name(self, key: str, name: str):
        self.filter_names[key] = name

    def compose(self):
        yield Static("Hello World")
        with Horizontal(id="footer-outer"):
            with Horizontal(id="footer-inner"):
                yield Footer()
            
        yield VerticalScroll(Stopwatch(), id="log_container")
        # yield Footer()

    def map_style(self, level: LogLevel):
        # cast ros Log byte to int

        if level == LogLevel.DEBUG:
            return "debug"
        elif level == LogLevel.INFO:
            return "info"
        elif level == LogLevel.WARN:
            return "warning"
        elif level == LogLevel.ERROR:
            return "error"
        elif level == LogLevel.FATAL:
            return "fatal"
        else:
            return "debug"

    def action_xxx(self):
        print("00000000000000000000000000000000000000")
        
        print(dir(VerticalScroll))
        print("1111--00000000000000000000000000000000000000")

    def on_log_message(self, event: LogMessage):
        log_container = self.query_one("#log_container")
        log_container.mount(self.build_log_message(event.message))
        

        log_container.scroll_end()


    def on_mount(self):
        footer = self.query_one("#footer-outer")
        footer.mount(Label("Hello World"))

    def build_log_message(self, log_item: LogItem):
        return Static(f"{log_item.time} - {log_item.name}: {log_item.message}", classes=self.map_style(log_item.level))
    
    def update_log(self):
        log_container = self.query_one("#log_container")
        log_container.remove_children()

        for log_item in self.storage:
            if log_item.level in self.filter_levels and log_item.name not in self.active_filter_names:
                log_container.mount(self.build_log_message(log_item))

        log_container.scroll_end()
        
    def update(self, time, level, name, message):
        level = int.from_bytes(level, byteorder="big")
        level = LogLevel(level)
        log_time = time.strftime("%Y-%m-%d %H:%M:%S")
        log_item = LogItem(log_time, message, level, name)
        self.storage.append(log_item)
        if level in self.filter_levels and name not in self.active_filter_names:
            self.post_message(LogMessage(log_item))

    def on_key(self, event):
        if event.key == "i":
            
            if LogLevel.INFO in self.filter_levels:
                self.filter_levels.remove(LogLevel.INFO)
            else:
                self.filter_levels.append(LogLevel.INFO)
        elif event.key == "w":
            self.update(LogLevel.WARN, "name", "warning")
        elif event.key == "e":
            self.update(LogLevel.ERROR, "name", "error")

        self.filter_by_name(event)
        self.update_log()

    def filter_by_name(self, event):
        if event.key in self.filter_names:
            name = self.filter_names[event.key]
            if name in self.active_filter_names:
                self.active_filter_names.remove(name)
            else:
                self.active_filter_names.append(name)
        self.update_log()
    
if __name__ == "__main__":
    app = ViewTUI()
    app.run()