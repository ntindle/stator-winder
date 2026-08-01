"""Capture a compact animated README preview from a generated player.

The player is a standalone HTML artifact with an embedded GLB and review
state. This script seeks the player's existing timeline deterministically,
captures frames in headless Chromium, and encodes a looping GIF with Pillow.
It does not alter the engineering artifact or invent a new motion sequence.

Example:

    .venv/Scripts/python scripts/capture_readme_preview.py \
      --input out/review/integrated_adapter/play_integrated_candidate_raw.html \
      --output docs/media/winding-cycle-preview.gif \
      --coil 1 --camera flyer
"""

from __future__ import annotations

import argparse
import functools
import io
import tempfile
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from PIL import Image
from playwright.sync_api import sync_playwright


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, _format: str, *_args: object) -> None:
        return


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a looping GIF from a generated winder player"
    )
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--coil", type=int, default=1)
    parser.add_argument(
        "--camera",
        choices=("overview", "wire", "flyer", "stator", "front", "side", "top"),
        default="flyer",
    )
    parser.add_argument("--virtual-seconds", type=float, default=18.0)
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--frame-ms", type=int, default=90)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument(
        "--clean",
        action="store_true",
        help="hide player panels so the README preview centers the mechanism",
    )
    return parser.parse_args()


def _capture(args: argparse.Namespace) -> list[Image.Image]:
    source = args.input.resolve()
    if not source.is_file():
        raise FileNotFoundError(source)

    handler = functools.partial(_QuietHandler, directory=str(source.parent))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = (
        f"http://127.0.0.1:{server.server_port}/{source.name}"
        f"?autoplay=0&coil={args.coil}&camera={args.camera}"
    )

    frames: list[Image.Image] = []
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True)
            page = browser.new_page(
                viewport={"width": args.width, "height": args.height},
                device_scale_factor=1,
            )
            page.goto(url, wait_until="domcontentloaded", timeout=120_000)
            page.wait_for_selector("#loading", state="detached", timeout=120_000)
            page.add_style_tag(
                content=(
                    "#status-panel, #view-panel, #transport "
                    "{ display: none !important; }"
                    if args.clean
                    else """
                    #view-panel { display: none !important; }
                    #status-panel { transform: scale(.78); transform-origin: top left; }
                    #transport { transform: scale(.86); transform-origin: bottom left;
                                 width: 116.25% !important; }
                    """
                )
            )
            page.locator("#hold-coil-starts").evaluate(
                "element => { element.checked = false; }"
            )
            page.locator("#focus-coil-starts").evaluate(
                "element => { element.checked = false; }"
            )

            scrub = page.locator("#scrub")
            start = float(scrub.input_value())
            maximum = float(scrub.get_attribute("max") or start)
            span = min(args.virtual_seconds, max(0.0, maximum - start))
            if span <= 0:
                raise RuntimeError("selected player range has no animation span")

            for index in range(args.frames):
                fraction = index / max(1, args.frames - 1)
                virtual_time = start + span * fraction
                scrub.evaluate(
                    """(element, value) => {
                        element.value = String(value);
                        element.dispatchEvent(new Event('input', {bubbles: true}));
                    }""",
                    virtual_time,
                )
                page.wait_for_timeout(35)
                encoded = page.screenshot(type="jpeg", quality=82)
                frame = Image.open(io.BytesIO(encoded)).convert("RGB")
                frames.append(frame)
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
    return frames


def main() -> int:
    args = _arguments()
    if args.frames < 2:
        raise ValueError("--frames must be at least 2")
    if args.frame_ms < 20:
        raise ValueError("--frame-ms must be at least 20")

    frames = _capture(args)
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="winder-preview-"):
        frames[0].save(
            output,
            save_all=True,
            append_images=frames[1:],
            duration=args.frame_ms,
            loop=0,
            optimize=True,
            disposal=2,
        )
    size_mib = output.stat().st_size / (1024 * 1024)
    print(f"wrote {output} ({len(frames)} frames, {size_mib:.2f} MiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
