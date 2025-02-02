from textual.app import App, ComposeResult, SystemCommand
from textual.widgets import Static, Footer, Button, SelectionList, Label, Input
from textual.containers import VerticalScroll, HorizontalGroup, Horizontal, Container
from textual.command import Provider, Hit, Hits, SimpleProvider, SimpleCommand
from textual.events import Click
from textual.message import Message
import re
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
from rapidfuzz import fuzz
from terminlog import LogItem, LogLevel
from terminlog.input_modal import InputModal, FreeTextFilterData


LOG_LEVEL_FILTER_CLEAR = 0
FUZZY_LEVEL = 50
FREE_FILTER_FUZZY_TYPE = "fuzzy"
FREE_FILTER_REGEX_TYPE = "regex"


class LogMessage(Message):
    """message use to notify between working thread and main thread"""
    def __init__(self, log_item: LogItem):
        super().__init__()
        self.message = log_item

class ClickableStatic(Static):
    """log line

    Args:
        Static (_type_): _description_
    """
    def __init__(self, log_item:LogItem, callback, **kwargs):
        text = f"{log_item.time} - {log_item.name}: {log_item.message}"
        super().__init__(text, **kwargs)
        self.log_item = log_item
        self.callback = callback

    def on_click(self, event: Click):
        if self.callback:
            self.callback(self.log_item)
        else:
            msg = f"file: {self.log_item.file}\nline_no:{self.log_item.line}"
            self.notify(msg)


#region prompt
class PromptModal(ModalScreen[str]):
    DEFAULT_CSS = """
    PromptModal {
        align: center middle;
    }

    PromptModal > Container {
        width: auto;
        height: auto;
        border: thick $background 80%;
        background: $surface;
    }

    PromptModal > Container > Horizontal {
        width: auto;
        height: auto;
    }

    PromptModal > Container > Horizontal > Button {
        margin: 2 4;
    }
    """

    def compose(self) -> ComposeResult:
        with Container():
            yield Label("Are you sure?")
            with Horizontal():
                yield Button("Ok", variant="success", id="ok")
                yield Button("Cancel", variant="error", id="cancel")

    
    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(event.button.id == "ok")



#endregion


# region node name filter modal window
class FilterModal(ModalScreen):
    DEFAULT_CSS = """
    FilterModal {
        align: center middle;
    }

    FilterModal > Container > SelectionList {
        width: 100;
        height: auto;
    }

   
    """
    def __init__(self, filters, selected):
        super().__init__()
        self.filters = filters
        self.selected = selected

    def compose(self) -> ComposeResult:
        with Container():
            yield Label("View Selected nodes")
            yield SelectionList[int](  
            id="selection_list"
            )
            with Horizontal():
                yield Button("Filter")
                yield Button("Cancel")

    def _on_mount(self, event):
        obj = self.query_one(SelectionList)
        for name in self.filters:
            selected = name in self.selected
            obj.add_option((name, name, selected))
    
    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(self.query_one(SelectionList).selected)
# endregion node name filter modal window

class ViewTUI(App):
    CSS_PATH = "tui_style.tcss"
    #TODO: check different bingings and reorganize
    BINDINGS = [
        Binding(key="q", action="quit", description="Quit"),
        ("d", "debug", "debug"),
        ("i", "info", "info"),
        ("w", "warning", "warning"),
        ("e", "error", "error"),
        Binding(key="f", action="open_filter", description="filter by node name"),
        Binding(key="c", action="reset_filter", description="clear all filter"),
        Binding(key="z", action="free_filter", description="fuzzy filter"),
        Binding(key="r", action="real_time", description="realtime"),
    ]

    
    
    
    def __init__(self, nodes_name, queue_size=100, active_node_names_cb = None):
        super().__init__()
        self.storage = deque(maxlen=queue_size)
        self.filter_levels = LOG_LEVEL_FILTER_CLEAR#[LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR]
        self.nodes_names = nodes_name
        self.active_filter_node_names = [name for name in self.nodes_names]
        self.active_node_names_cb = active_node_names_cb
        self.fuzzy_filter = ""
        self.updating = True
        self.realtime = True
        self.free_filter_type = "fuzzy"
        
    #region palette command region
    def get_system_commands(self, screen: Screen) -> Iterable[SystemCommand]:
        """Add command to palette

        """
        yield SystemCommand("clear", "clear storage", self.clear_all)
    #endregion palette command region

    def clear_all(self):
        """
        clear storage and rerender
        """
        self.storage.clear()
        self.update_log()
        
    #region fuzzy filter
    

    def free_filter_callback(self, result: FreeTextFilterData):
        self.free_filter_type = result.filter_type
        if result.active:
            self.notify(f"{self.free_filter_type} filter: {result.filter}")
            self.fuzzy_filter = result.filter
            self.update_log()
    #endregion fuzzy filter


    def quit_callback(self, result):
        if result:
            self.app.exit()    

    #region filter by node name
    

    def filter_modal_callback(self, result) -> None:
        """filter by name callback from module
        update active_filter and render from storage

        Args:
            result (_type_): _description_
        """
        self.notify("filter by name")
        self.active_filter_node_names = result
        self.update_log()

    #endregion

    def compose(self):
        self.footer = Footer()
        yield self.footer
        yield VerticalScroll(id="log_container")


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


    def on_log_message(self, event: LogMessage):
        """render log message item
        for known failed if module window open (it's ok because when we close it we render all logs from storage)

        Args:
            event (LogMessage): _description_
        """
        try:
            self.render_logs([event.message])
            
        except NoMatches:
            pass
        
    def log_link_click_handler(self):
        self.notify("click")

    def build_log_message(self, log_item: LogItem):
        """convert log item to string and return Static widget

        Args:
            log_item (LogItem): _description_

        Returns:
            _type_: Static widget
        """
        # callback = self.log_link_click_handler
        callback = None
        return ClickableStatic(log_item, callback, classes=self.map_style(log_item.level))
    
    def update_log(self):
        """clear and render all logs from storage
        """
        log_container = self.query_one("#log_container")
        log_container.remove_children()
        self.render_logs(self.storage)

    def render_logs(self, logs) -> None:
        """render log if all filter ok

        Args:
            logs (_type_): log item

        
        """
        log_container = self.query_one("#log_container")
        def level_ok():
            if self.filter_levels == LOG_LEVEL_FILTER_CLEAR:
                return True
            else:
                return log_item.level >= self.filter_levels
        
        def node_name_ok():
            if len(self.nodes_names) == 0:
                return True
            else:
                return log_item.name in self.active_filter_node_names
        
        def reg_filter_ok():
            regex = re.compile(self.fuzzy_filter, re.IGNORECASE)
            message_ok = regex.search(log_item.message) is not None
            name_ok = regex.search(log_item.name) is not None
            #TODO: regex on file name
            return name_ok or message_ok

        def fuzzy_filter_ok():
            message_ok = fuzz.ratio(self.fuzzy_filter, log_item.message) > FUZZY_LEVEL 
            name_ok = fuzz.ratio(self.fuzzy_filter, log_item.name) > FUZZY_LEVEL
            #TODO: fuzzy on file name
            return name_ok or message_ok

        def input_filter_ok():
            if self.fuzzy_filter == "":
                return True
            else:
                if self.free_filter_type == FREE_FILTER_FUZZY_TYPE:
                    return fuzzy_filter_ok()
                return reg_filter_ok()
                
            
        
        #storage iteration
        for log_item in logs:
            if  level_ok() and node_name_ok() and input_filter_ok():
                log_container.mount(self.build_log_message(log_item))

        if self.realtime:
            log_container.scroll_end()
        
    def update(self, time, level, name, message, file, line):
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
        # level = int.from_bytes(level, byteorder="big")
        level = LogLevel(level)
        log_time = time.strftime("%Y-%m-%d %H:%M:%S")
        log_item = LogItem(log_time, message, level, name, file, line)
        self.storage.append(log_item)
        self.post_message(LogMessage(log_item))


    #region filter by log level
    def level_filter(self, level):
        """Add or remove level from filter

        Args:
            level (_type_): _description_
        """
        desc = ""
        if level == self.filter_levels:
            self.filter_levels = LOG_LEVEL_FILTER_CLEAR
            desc = f"DEBUG level"
            
        else:
            desc = f"{level.name} level"
            self.filter_levels = level
        self.notify(desc)
        self.update_log()
    #endregion filter by log level

    # region actions
    def action_real_time(self):
        self.realtime = not self.realtime
        if self.realtime:
            self.notify("Realtime active")
            log_container = self.query_one("#log_container")
            log_container.scroll_end()

    def action_quit(self):
        self.push_screen(PromptModal(), self.quit_callback)

    def action_open_filter(self):
        node_names = self.nodes_names
        if len(self.nodes_names) == 0 and self.active_node_names_cb is not None:
            node_names = self.active_node_names_cb()
        self.notify(str(node_names))
        self.push_screen(FilterModal(node_names, self.active_filter_node_names), self.filter_modal_callback)
    def action_free_filter(self):
        self.push_screen(InputModal(self.fuzzy_filter), self.free_filter_callback)
        
    def action_reset_filter(self):
        """reset all filters"""
        self.filter_levels = LOG_LEVEL_FILTER_CLEAR#[LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR]
        self.active_filter_node_names = [name for name in self.nodes_names]
        self.fuzzy_filter = ""
        self.update_log()

    def action_debug(self):
        self.level_filter(LogLevel.DEBUG)

    def action_info(self):
        self.level_filter(LogLevel.INFO)

    def action_warning(self):
        self.level_filter(LogLevel.WARN)

    def action_error(self):
        self.level_filter(LogLevel.ERROR)
    # endregion actions
    

    def on_key(self, event):
        # if event.key == "i":
        if event.key == "s":
            self.modal.visible = True
            self.refresh_layout()
        
        # self.filter_by_key(event)
        # self.update_log()

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
        self.active_filter_node_names = names

    def filter_by_key(self, event):
        if event.key in self.nodes_names:
            name = self.nodes_names[event.key]
            if name in self.active_filter_node_names:
                self.active_filter_node_names.remove(name)
            else:
                self.active_filter_node_names.append(name)
        self.update_log()
    
    #region method to test

    def action_toggle_updating(self):
        """not use function try to toggle bindings description"""
        self.updating = not self.updating
        desc = "Stop updating" if self.updating else "Start updating"
        self.notify(desc)
        self.bind("i", "app.info", description=desc)
        # self.screen.post_message(events.ScreenResume())
        self.footer.bindings_changed(self.screen)
        # self.refresh_bindings()
    #endregion

if __name__ == "__main__":
    app = ViewTUI()
    app.run()
