"""Byte-preserving C function extraction for control regression tests."""

import re


def c_function(data: bytes, name: str) -> bytes:
    """Extract original bytes, ignoring braces in C comments/literals."""
    text = data.decode("latin1")
    lexical = re.compile(
        r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', re.S
    )
    masked = lexical.sub(
        lambda m: "".join("\n" if c == "\n" else " " for c in m[0]), text
    )
    match = re.search(r"\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", masked)
    if not match:
        raise AssertionError("C function missing: " + name)
    start = text.rfind("\n", 0, match.start()) + 1
    depth = 1
    index = match.end()
    while depth:
        depth += (masked[index] == "{") - (masked[index] == "}")
        index += 1
    return data[start:index]
