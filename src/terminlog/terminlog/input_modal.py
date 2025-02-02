from textual.app import ComposeResult
from textual.containers import VerticalScroll, Vertical, Horizontal, Container
from textual.screen import Screen, ModalScreen
from textual.widgets import Static, Footer, Button, SelectionList, Label, Input

# region free text search filter modal window
class InputModal(ModalScreen[str]):
    DEFAULT_CSS = """
    InputModal {
        align: center middle;
    }

    InputModal > Container {
        width: auto;
        height: auto;
        border: thick $background 80%;
        background: $surface;
    }

    InputModal > Container > Vertical > Input {
        width: 32;
    }

    InputModal > Container > Vertical > Label {
        text-align: center;
        text-style: bold;
        align: center middle;
    }
    """

    def compose(self) -> ComposeResult:
        with Container():
            with Vertical():
                yield Label("apply string filter")
                yield Input()
                with Horizontal():
                    yield Button("Ok", variant="success", id="ok")
                    yield Button("Cancel", variant="error", id="cancel")

    def on_input_submitted(self) -> None:
        self.dismiss((True, self.query_one(Input).value))

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss((event.button.id == "ok", self.query_one(Input).value))

# endregion