from textual.containers import Container
from textual.screen import ModalScreen, Screen
from textual.widgets import Button, SelectionList, RichLog, OptionList
from textual.app import App, ComposeResult

class InputModal(Screen):
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
            yield SelectionList[int](  
            id="selection_list"
            )
            yield Button("Submit")

    def _on_mount(self, event):
        obj = self.query_one(SelectionList)
        obj.add_option(("Option 1", 0, True))
    
    def on_button_pressed(self, event: Button.Pressed) -> None:
        #    vvvvvvv dismiss the modal screen;
        self.dismiss(self.query_one(SelectionList).selected)
        # self.app.pop_screen()

class DummyApp(App[None]):
    CSS = """
    TextLog {
        height: 1fr;
    }
    """
    BINDINGS = [("q", "request_quit", "Quit")]

    def compose(self) -> ComposeResult:
        yield Button("Open modal")
        yield RichLog()

    def action_request_quit(self) -> None:
        self.push_screen(InputModal(), self.input_modal_callback)

    def input_modal_callback(self, result: str) -> None:
        """Handle the modal result by writing it to the TextLog."""
        self.query_one(RichLog).write(f"Modal result: {result}")


if __name__ == "__main__":
    app = DummyApp()
    app.run()