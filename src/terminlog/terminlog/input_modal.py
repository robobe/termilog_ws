from textual.app import ComposeResult
from textual.containers import VerticalScroll, Vertical, Horizontal, Container
from textual.screen import Screen, ModalScreen
from textual.widgets import Static, RadioButton, Button, RadioSet, Label, Input
from typing import NamedTuple

class FreeTextFilterData(NamedTuple):
    active: bool
    filter: str
    filter_type: str

# region free text search filter modal window
class InputModal(ModalScreen[str]):
    DEFAULT_CSS = """
    InputModal {
        align: center middle;
    }

    InputModal > Container {
        width: 40;
        height: 13;
        border: thick $background 50%;
        background: $surface;
    }

    InputModal > Container > Vertical > Input {
        align: center middle;
        color: cyan;
        width: 100%;
    }

    InputModal > Container > Vertical > Label {
        text-align: center;
        text-style: bold;
        align: center middle;
        color: cyan;
        width: 100%;
    }

    InputModal > Container > Vertical > RadioSet {
        layout: horizontal;
        align: center middle;
        width: 100%;
    }

    InputModal > Container > Vertical > RadioSet > RadioButton {
        margin-right: 5;
    }

    InputModal > Container > Vertical > Horizontal  {
        width: 100%;
        height: auto;
        align: center middle;
    }

    InputModal > Container > Vertical > Horizontal > Button {
        margin: 1 1 1 1;
    }
    """


    def __init__(self, filter):
        super().__init__()
        self.filter_type = "fuzzy"
        self.filter = filter

    def compose(self) -> ComposeResult:
        with Container():
            with Vertical():
                yield Label("apply string filter")
                yield RadioSet(
                    RadioButton("fuzzy", value=True),  # Pre-selected option
                    RadioButton("regex")
                )
                yield Input(value=self.filter)
                with Horizontal():
                    yield Button("Ok", variant="success", id="ok")
                    yield Button("Escape", variant="error", id="cancel")

    def on_input_submitted(self) -> None:
        self.dismiss(FreeTextFilterData(True, self.query_one(Input).value, self.filter_type))

    def on_radio_set_changed(self, event: RadioSet.Changed) -> None:
        """Handle radio button selection changes."""
        self.filter_type = event.pressed.label

    def on_key(self, event):
        if event.key == "escape":
            self.dismiss(FreeTextFilterData(False, "", ""))

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(FreeTextFilterData(event.button.id == "ok", self.query_one(Input).value, self.filter_type))

# endregion