from textual.app import App, ComposeResult, SystemCommand
from textual.widgets import Static, Footer, Button, SelectionList, OptionList, Input
from textual.containers import VerticalScroll, HorizontalGroup, Horizontal, Container
from textual.command import Provider, Hit, Hits, SimpleProvider, SimpleCommand
from typing import NamedTuple
from textual.message import Message
from enum import IntEnum
from collections import deque
from textual.binding import Binding
from textual.screen import Screen, ModalScreen
from rich.style import Style
from typing import Iterable, Any
from dataclasses import dataclass
from functools import partial
from textual import events
from textual import on
from textual.css.query import  NoMatches
from textual._color_constants import COLOR_NAME_TO_RGB
from pathlib import Path

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


class FilterModal(ModalScreen):
    DEFAULT_CSS = """
    InputModal {
        align: center middle;
    }

    InputModal > Container {
        width: auto;
        height: auto;
    }

    InputModal > Container > Input {
        width: 32;
    }
    """
    def __init__(self, filters, selected):
        super().__init__()
        self.filters = filters
        self.selected = selected

    def compose(self) -> ComposeResult:
        with Container():
            yield SelectionList[int](  
            id="selection_list"
            )
            yield Button("Submit")

    def _on_mount(self, event):
        obj = self.query_one(SelectionList)
        for name in self.filters:
            selected = name in self.selected
            obj.add_option((name, name, selected))
    
    def on_button_pressed(self, event: Button.Pressed) -> None:
        #    vvvvvvv dismiss the modal screen;
        self.dismiss(self.query_one(SelectionList).selected)
        # self.app.pop_screen()


class ViewTUI(App):
    CSS_PATH = "tui_style.tcss"
    t = Binding(key="t", action="xxx", description="XXX")
    BINDINGS = [
        Binding(key="q", action="quit", description="Quit"),
        t,
        ("i", "info", "info"),
        ("w", "warning", "warning"),
        ("e", "error", "error"),
        Binding(key="f", action="open_filter", description="filter"),
    ]

    # COMMANDS =  {SimpleProvider}
    def get_system_commands(self, screen: Screen) -> Iterable[SystemCommand]:
        """Add command to palettate

        """
        yield SystemCommand("clear", "clear all", self.clear_all)

    def clear_all(self):
        self.storage.clear()
        self.update_log()
        

    CSS = """
        # HorizontalGroup#footer-outer {
        #     height: 2;
        #     dock: bottom;
        # }
       
        Footer {
            background: #282c34;
            color: white;
        }
    """
    def __init__(self, nodes_name):
        super().__init__()
        self.storage = deque(maxlen=10)
        self.filter_levels = [LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR]
        self.nodes_names = nodes_name
        self.active_filter_names = [name for name in self.nodes_names]

        self.updating = True
        

    def action_open_filter(self):
        self.push_screen(FilterModal(self.nodes_names, self.active_filter_names), self.filter_modal_callback)

    def filter_modal_callback(self, result) -> None:
        """filter by name callback from module
        update active_filter and render from storage

        Args:
            result (_type_): _description_
        """
        self.notify("filter by name")
        self.active_filter_names = result
        self.update_log()



    def compose(self):
        self.footer = Footer()
        yield self.footer
        yield VerticalScroll(Stopwatch(), id="log_container")


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
        """render log message item
        for known failed if module window open (it's ok because when we close it we render all logs from storage)

        Args:
            event (LogMessage): _description_
        """
        try:
            log_container = self.query_one("#log_container")
            log_container.mount(self.build_log_message(event.message))
            log_container.scroll_end()
        except NoMatches:
            pass
        
        

        


    def on_mount(self):
        # footer = self.query_one("#footer-outer")
        # footer.mount(Label("Hello World"))
        new_binding = ()

        if new_binding not in self._bindings:
            self._bindings.bind("0", "notify('New Binding!')", "name")  # Add new binding
            for b in self._bindings:
                print(b)
            self.refresh()  # Update UI to reflect changes
            self.notify("Added new binding: 'n'")

    def build_log_message(self, log_item: LogItem):
        return Static(f"{log_item.time} - {log_item.name}: {log_item.message}", classes=self.map_style(log_item.level))
    
    def update_log(self):
        log_container = self.query_one("#log_container")
        log_container.remove_children()

        for log_item in self.storage:
            if log_item.level in self.filter_levels and log_item.name in self.active_filter_names:
                log_container.mount(self.build_log_message(log_item))

        log_container.scroll_end()
        
    def update(self, time, level, name, message):
        """update log item from ROS
        save log item as generic python struct
        save to storage 
        trigger event message to draw the item
        
        Args:
            time (_type_): _description_
            level (_type_): _description_
            name (_type_): _description_
            message (_type_): _description_
        """
        level = int.from_bytes(level, byteorder="big")
        level = LogLevel(level)
        log_time = time.strftime("%Y-%m-%d %H:%M:%S")
        log_item = LogItem(log_time, message, level, name)
        self.storage.append(log_item)
        if level in self.filter_levels and name in self.active_filter_names:
            self.post_message(LogMessage(log_item))

    

    def action_toggle_updating(self):
        self.updating = not self.updating
        desc = "Stop updating" if self.updating else "Start updating"
        self.notify(desc)
        self.bind("i", "app.info", description=desc)
        # self.screen.post_message(events.ScreenResume())
        self.footer.bindings_changed(self.screen)
        # self.refresh_bindings()

    def action_info(self):
        self.action_toggle_updating()
        if LogLevel.INFO in self.filter_levels:
            self.filter_levels.remove(LogLevel.INFO)
        else:
            self.filter_levels.append(LogLevel.INFO)
        
    def on_key(self, event):
        # if event.key == "i":
        if event.key == "s":
            self.modal.visible = True
            self.refresh_layout()
        if event.key == "w":
            self.update(LogLevel.WARN, "name", "warning")
        elif event.key == "e":
            self.update(LogLevel.ERROR, "name", "error")

        self.filter_by_key(event)
        self.update_log()

    def filter_by_ids(self, ids):
        """_summary_
        map ids to names
        add names to filters by name
        Args:
            ids (_type_): names
        """
        names = []
        for id in ids:
            name = self.nodes_names[id]
            names.append(name)
        self.active_filter_names = names

    def filter_by_key(self, event):
        if event.key in self.nodes_names:
            name = self.nodes_names[event.key]
            if name in self.active_filter_names:
                self.active_filter_names.remove(name)
            else:
                self.active_filter_names.append(name)
        self.update_log()
    
if __name__ == "__main__":
    app = ViewTUI()
    app.run()
