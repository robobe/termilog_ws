from typing import NamedTuple
from enum import IntEnum

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
    file: str
    line: int


