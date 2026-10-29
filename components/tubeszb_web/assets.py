"""Assemble the self-contained page for firmware and browser tests."""

import json
import re
from pathlib import Path


def embedded_page() -> str:
    directory = Path(__file__).parent
    page = (directory / "frontend.html").read_text()
    page = page.replace(
        "/* NETWORK_VALIDATION */",
        (directory.parent / "network_settings/network_validation.js").read_text(),
    )
    page = page.replace("/* FRONTEND_SCRIPT */", (directory / "frontend.js").read_text())
    page = page.replace("<!-- BRAND_SYMBOL -->", (directory / "logo-symbol.svg").read_text())
    icons = json.loads((directory / "icons.json").read_text())
    page = re.sub(r'<i data-lucide="([^"]+)"[^>]*></i>', lambda match: icons[match[1]], page)
    license_text = (directory / "LICENSE-icons.txt").read_text()
    return page.replace("<!doctype html>", "<!doctype html>\n<!--\n" + license_text + "\n-->", 1)


if __name__ == "__main__":
    import sys

    sys.stdout.write(embedded_page())
