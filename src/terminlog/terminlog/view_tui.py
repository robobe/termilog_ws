from textual.app import App, ComposeResult, SystemCommand
from textual.widgets import Static, Footer, Button, SelectionList, Label, TextArea
from textual.containers import VerticalScroll, Vertical, Horizontal, Container
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
from textual.css.query import NoMatches
from textual._color_constants import COLOR_NAME_TO_RGB
from rapidfuzz import fuzz
from terminlog import LogItem, LogLevel
from terminlog.input_modal import InputModal, FreeTextFilterData
from typing import NamedTuple, List
from threading import RLock
from ament_index_python.packages import get_package_share_directory
import os

LOG_LEVEL_FILTER_CLEAR = 0
FUZZY_LEVEL = 50
FREE_FILTER_FUZZY_TYPE = "fuzzy"
FREE_FILTER_REGEX_TYPE = "regex"


class LogMessage(Message):
    """message use to notify between working thread and main thread"""

    def __init__(self, log_item: LogItem) -> None:
        super().__init__()
        self.message = log_item


class ClickableStatic(Static):
    """log line
    with notify when click
    Args:
        Static (_type_): _description_
    """

    def __init__(self, log_item: LogItem, callback, **kwargs):
        text = f"[{LogLevel(log_item.level).name}] {log_item.time} - {log_item.name}: {log_item.message}"
        super().__init__(text, **kwargs)
        self.log_item = log_item
        self.callback = callback

    def on_click(self, event: Click):
        """click handler
        Run callback or notify log message file and line number

        Args:
            event (Click): _description_
        """
        if self.callback:
            self.callback(self.log_item)
        else:
            msg = f"file: {self.log_item.file}\nline_no:{self.log_item.line}"
            self.notify(msg)


# region prompt
class PromptData(NamedTuple):
    active: bool
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

    def ok_msg(self):
        return PromptData(True)
    
    
    def cancel_msg(self):
        return PromptData(False)
    
    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "ok":
            msg = self.ok_msg()
        else:
            msg = self.cancel_msg()

        self.dismiss(msg)

class AboutModal(ModalScreen[str]):
    DEFAULT_CSS = """
    AboutModal {
        align: center middle;
    }

    AboutModal > Container {
        width: auto;
        height: auto;
        border: thick $background 80%;
        background: $surface;
    }

    AboutModal > Container > Label {
        text-align: center;
        margin: 2 4;
    }

    AboutModal > Container > Button {
        margin: 2 4;
    }
    """
    def __init__(self, version):
        super().__init__()
        self.version = version

    def compose(self) -> ComposeResult:
        with Container():
            yield Label("", id="version")
            yield Button("Exit", variant="success", id="ok")
            
    def _on_mount(self, event):
        obj = self.query_one("#version")
        obj.update("Version: " + self.version)

    def ok_msg(self):
        return PromptData(True)
    
    
    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "ok":
            msg = self.ok_msg()
       
        self.dismiss(msg)

# endregion


# region filter list
class FilterListData(NamedTuple):
    active: bool
    filter: List[str]


class FilterListModal(ModalScreen):
    DEFAULT_CSS = """
    FilterListModal {
        align: center middle;
    }
    FilterListModal > Container {
        width: 60;
        height: 11;
        border: thick $background 80%;
        background: $surface;
    }

    FilterListModal > Container > Vertical {
        width: auto;
        height: auto;
        align: center middle;
    }

    FilterListModal > Container > Vertical > Label {
        text-align: center;
        text-style: bold;
        align: center middle;
        color: cyan;
        width: 100%;
    }

    FilterListModal > Container > Vertical > TextArea {
        align: center middle;
        width: 55;
        height: 5;
    }

    FilterListModal > Container > Vertical > Horizontal {
        align: center middle;
    }

    FilterListModal > Container > Vertical > Horizontal > Button {
        margin: 0 2;
    }
    """

    def __init__(self, ignore_list=[]):
        super().__init__()
        self.ignore_list = ignore_list

    def compose(self) -> ComposeResult:
        with Container():
            with Vertical():
                yield Label("Ignore list: Enter or remove lines to ignore")
                yield TextArea(id="filter_list")
                with Horizontal():
                    yield Button("Filter", variant="success", id="ok")
                    yield Button("Esc", variant="error")

    def _on_mount(self, event):
        obj = self.query_one(TextArea)
        data = "\n".join(self.ignore_list)
        obj.text = data

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "ok":
            msg = self.ok_msg()
        else:
            msg = self.cancel_msg()
        self.dismiss(msg)

    def ok_msg(self):
        data = self.query_one(TextArea).text.split("\n")
        data = [name for name in data if name.strip()]
        msg = FilterListData(True, data)
        return msg

    def cancel_msg(self):
        return FilterListData(False, [])

    def on_key(self, event):
        if event.key == "escape":
            self.dismiss(self.cancel_msg())

        if event.key == "enter":
            self.dismiss(self.ok_msg())


# endregion filter list


# region node name filter modal window
class FilterData(NamedTuple):
    active: bool
    filter: List[str]


class FilterModal(ModalScreen):
    DEFAULT_CSS = """
    FilterModal {
        align: center middle;
    }

    FilterModal > Container > Vertical > Label {
        text-align: center;
        text-style: bold;
        align: center middle;
        color: cyan;
        width: 100%;
    }

    FilterModal > Container {
        width: 60;
        height: 16;
        border: thick $background 80%;
        background: $surface;
        align: center middle;
    }

    FilterModal > Container > Vertical > SelectionList {
        align: center middle;
        width: 90%;
        height: 8;
    }

    FilterModal > Container > Vertical >Horizontal{
        align: center middle;
    }
    FilterModal > Container > Vertical > Horizontal > Button {
        margin: 0 4;

    }
    """

    def __init__(self, filters, selected):
        super().__init__()
        self.filters = filters
        self.selected = selected

    def compose(self) -> ComposeResult:
        with Container():
            with Vertical():
                yield Label("View Selected nodes")
                yield SelectionList[int](id="selection_list")
                with Horizontal():
                    yield Button("Filter", variant="success", id="ok")
                    yield Button("Esc", variant="error")

    def _on_mount(self, event):
        obj = self.query_one(SelectionList)
        for name in self.filters:
            selected = name in self.selected
            obj.add_option((name, name, selected))

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "ok":
            msg = self.ok_msg()
        else:
            msg = self.cancel_msg()
        self.dismiss(msg)

    def cancel_msg(self):
        return FilterData(False, "")

    def ok_msg(self):
        data = self.query_one(SelectionList).selected
        return FilterData(True, data)

    def on_key(self, event):
        if event.key == "escape":
            self.dismiss(self.cancel_msg())

        if event.key == "enter":
            self.dismiss(self.ok_msg())


# endregion node name filter modal window


class ViewTUI(App):
    CSS_PATH = "tui_style.tcss"
    # TODO: check different bingings and reorganize
    BINDINGS = [
        Binding(key="q", action="quit", description="Quit"),
        Binding(key="d", action="debug", description="Debug"),
        Binding(key="i", action="info", description="Info"),
        Binding(key="w", action="warning", description="Warning"),
        Binding(key="e", action="error", description="Error"),
        Binding(key="n", action="open_filter", description="Node"),
        Binding(key="c", action="reset_filter", description="Clear filters"),
        Binding(key="f", action="free_filter", description="Free filter"),
        Binding(key="r", action="real_time", description="Realtime"),
        Binding(key="l", action="filter_list", description="List"),
    ]

    def __init__(self, nodes_name, queue_size=100, active_node_names_cb=None):
        super().__init__()
        self.lock = RLock()
        self.queue_size = queue_size
        self.storage = deque(maxlen=queue_size)
        self.filter_levels = LOG_LEVEL_FILTER_CLEAR  # [LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR]
        self.nodes_names = nodes_name
        self.active_filter_node_names = [name for name in self.nodes_names]
        self.active_node_names_cb = active_node_names_cb
        self.fuzzy_filter = ""
        self.updating = True
        self.realtime = True
        self.free_filter_type = "fuzzy"
        self.row_counter = 0
        self.string_list_to_ignore = []

    # region palette command region
    def get_system_commands(self, screen: Screen) -> Iterable[SystemCommand]:
        """Add command to palette"""
        yield SystemCommand("clear", "clear storage", self.clear_all)
        yield SystemCommand("about", "about", self.about)

    # endregion palette command region

    def about(self):
        

        def get_ros2_package_version(package_name):
            try:
                package_dir = get_package_share_directory(package_name)
                package_xml = os.path.join(package_dir, 'package.xml')
                with open(package_xml, 'r') as f:
                    for line in f:
                        if '<version>' in line:
                            return line.strip().split('<version>')[1].split('</version>')[0]
            except Exception as e:
                print(f"Error: {e}")
            return None
        
        version = get_ros2_package_version("terminlog")
        self.push_screen(AboutModal(version))
        

    def clear_all(self):
        """
        clear storage and rerender
        """
        with self.lock:
            self.storage.clear()
            self.update_log()

    # region fuzzy filter

    def free_filter_callback(self, result: FreeTextFilterData):
        self.free_filter_type = result.filter_type
        if result.active:
            self.notify(f"{self.free_filter_type} filter: {result.filter}")
            self.fuzzy_filter = result.filter
            self.update_log()

    # endregion fuzzy filter

    def filter_list_modal_callback(self, result: FilterListData):

        if result.active:
            self.notify(f"Ignore filter: {result.filter}")
            self.string_list_to_ignore = result.filter
            self.update_log()

    def quit_callback(self, result: PromptData):
        if result.active:
            self.app.exit()

    # region filter by node name

    def filter_modal_callback(self, result: FilterData) -> None:
        """filter by name callback from module
        update active_filter and render from storage

        Args:
            result (_type_): _description_
        """
        if result.active:
            self.notify("filter by nodes name")
            self.active_filter_node_names = result.filter
            self.update_log()

    # endregion

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
            # clear viewer and render logs if overflow and realtime
            if self.realtime and self.row_counter > self.queue_size:
                self.row_counter = 0
                self.update_log()
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
        return ClickableStatic(
            log_item, callback, classes=self.map_style(log_item.level)
        )

    def update_log(self):
        """clear and render all logs from storage"""
        log_container = self.query_one("#log_container")
        log_container.remove_children()
        # log_container.scroll_home()
        self.render_logs(self.storage)

    def render_logs(self, logs) -> None:
        """render log if all filter ok
            main filter logic
        Args:
            logs (_type_): log item


        """
        log_container = self.query_one("#log_container")

        def ignore_list_check():
            """check if message not in ignore list"""
            if len(self.string_list_to_ignore) == 0:
                return True
            else:
                for ignore in self.string_list_to_ignore:
                    if ignore in log_item.message:
                        return False
                return True

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
            # TODO: regex on file name
            return name_ok or message_ok

        def fuzzy_filter_ok():
            message_ok = fuzz.ratio(self.fuzzy_filter, log_item.message) > FUZZY_LEVEL
            name_ok = fuzz.ratio(self.fuzzy_filter, log_item.name) > FUZZY_LEVEL
            # TODO: fuzzy on file name
            return name_ok or message_ok

        def input_filter_ok():
            if self.fuzzy_filter == "":
                return True
            else:
                if self.free_filter_type == FREE_FILTER_FUZZY_TYPE:
                    return fuzzy_filter_ok()
                return reg_filter_ok()

        # storage iteration
        with self.lock:
            for log_item in logs:
                # check / apply filter
                if (
                    level_ok()
                    and node_name_ok()
                    and input_filter_ok()
                    and ignore_list_check()
                ):
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
        self.post_message(LogMessage(log_item))
        with self.lock:
            self.storage.append(log_item)
            self.row_counter += 1

    # region filter by log level
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

    # endregion filter by log level

    # region actions
    def action_real_time(self):
        self.realtime = not self.realtime
        if self.realtime:
            self.notify("Realtime active")
            log_container = self.query_one("#log_container")
            log_container.scroll_end()
        else:
            self.notify("Realtime inactive", severity="warning")

    def action_quit(self):
        self.push_screen(PromptModal(), self.quit_callback)

    def action_open_filter(self):
        node_names = self.nodes_names
        if len(self.nodes_names) == 0 and self.active_node_names_cb is not None:
            node_names = self.active_node_names_cb()
        self.push_screen(
            FilterModal(node_names, self.active_filter_node_names),
            self.filter_modal_callback,
        )

    def action_free_filter(self):
        self.push_screen(InputModal(self.fuzzy_filter), self.free_filter_callback)

    def action_filter_list(self):
        self.push_screen(
            FilterListModal(self.string_list_to_ignore), self.filter_list_modal_callback
        )

    def action_reset_filter(self):
        """reset all filters"""
        self.filter_levels = LOG_LEVEL_FILTER_CLEAR  # [LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR]
        self.active_filter_node_names = [name for name in self.nodes_names]
        self.fuzzy_filter = ""
        self.string_list_to_ignore = []
        self.notify("Filter reset / cleared", severity="warning")
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

    # region method to test

    def action_toggle_updating(self):
        """not use function try to toggle bindings description"""
        self.updating = not self.updating
        desc = "Stop updating" if self.updating else "Start updating"
        self.notify(desc)
        self.bind("i", "app.info", description=desc)
        # self.screen.post_message(events.ScreenResume())
        self.footer.bindings_changed(self.screen)
        # self.refresh_bindings()

    # endregion


if __name__ == "__main__":
    app = ViewTUI()
    app.run()
