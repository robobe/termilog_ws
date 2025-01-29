from textual.containers import Container
from textual.screen import ModalScreen
from textual.widgets import Button, Input, RichLog
from textual.app import App, ComposeResult

class InputModal(ModalScreen[str]):
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

    def compose(self) -> ComposeResult:
        with Container():
            yield Input()

    def on_input_submitted(self) -> None:
        #    vvvvvvv dismiss the modal screen;
        self.dismiss(self.query_one(Input).value)
        # with this 

class DummyApp(App[None]):
    CSS = """
    TextLog {
        height: 1fr;
    }
    """

    def compose(self) -> ComposeResult:
        yield Button("Open modal")
        yield RichLog()

    def on_button_pressed(self) -> None:
        self.push_screen(InputModal(), self.input_modal_callback)

    def input_modal_callback(self, result: str) -> None:
        """Handle the modal result by writing it to the TextLog."""
        self.query_one(RichLog).write(f"Modal result: {result}")

app = DummyApp()
if __name__ == "__main__":
    app.run()