"""
Download road sign and traffic light test images from Wikimedia Commons.
Uses Commons search to find freely licensed photographs for each label.
"""

from __future__ import annotations

import json
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data"
SOURCES_FILE = DATA_DIR / "SOURCES.txt"

# One image per supported detection category (+ extra traffic-light colors)
# Optional manual Commons filename when search picks poor matches.
MANUAL_FILES: dict[str, str] = {
    "no_left_turn_sign_1.jpg": "Do not turn left sign in Oregon.jpg",
    "keep_left_sign_1.jpg": "Italian_traffic_signs_-_direzione_obbligatoria_a_sinistra.svg",
    "keep_right_sign_1.jpg": "Italian_traffic_signs_-_direzione_obbligatoria_a_destra.svg",
    "road_narrows_sign_1.jpg": "UK_traffic_sign_517.svg",
    "bicycle_crossing_sign_1.jpg": "UK_traffic_sign_950.svg",
    "ferry_sign_1.jpg": "UK_traffic_sign_824.svg",
    "railway_crossing_sign_1.jpg": "Old Fashioned Railroad Crossbuck @ Danbury Railway Museum.jpg",
}

DOWNLOADS: dict[str, str] = {
    "traffic_light_2.jpg": "red traffic light signal photograph",
    "traffic_light_3.jpg": "yellow amber traffic light photograph",
    "stop_sign_1.jpg": "stop sign road photograph",
    "speed_limit_sign_1.jpg": "speed limit road sign photograph mph",
    "no_u_turn_sign_1.jpg": "no u-turn road sign photograph",
    "no_left_turn_sign_1.jpg": "no left turn road sign photograph",
    "no_right_turn_sign_1.jpg": "do not turn right prohibition sign photograph",
    "keep_left_sign_1.jpg": "keep left mandatory road sign photograph",
    "keep_right_sign_1.jpg": "keep right mandatory road sign photograph",
    "railway_crossing_sign_1.jpg": "level crossing ahead warning sign photograph",
    "falling_rocks_sign_1.jpg": "falling rocks road sign photograph",
    "road_narrows_sign_1.jpg": "road narrows sign photograph",
    "pedestrian_crossing_sign_1.jpg": "pedestrian crossing warning sign photograph",
    "bicycle_crossing_sign_1.jpg": "bicycle crossing road sign photograph",
    "ferry_sign_1.jpg": "ferry road sign photograph",
    "animal_crossing_sign_1.jpg": "deer animal crossing road sign photograph",
    "first_aid_sign_1.jpg": "first aid post hospital sign photograph",
    "no_horn_sign_1.jpg": "no horn honking sign photograph",
    "no_entry_sign_1.jpg": "no entry road sign photograph",
    "safety_first_sign_1.jpg": "safety first yellow sign photograph",
}

API = "https://commons.wikimedia.org/w/api.php"
USER_AGENT = "CSS487-RoadSignDetector/1.0 (educational project)"
IMAGE_EXT = re.compile(r"\.(jpe?g|png|svg)$", re.I)


def request_json(url: str, retries: int = 6) -> dict:
    for attempt in range(retries):
        request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        try:
            with urllib.request.urlopen(request, timeout=45) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            if exc.code == 429 and attempt + 1 < retries:
                time.sleep(3.0 * (attempt + 1))
                continue
            raise
    raise RuntimeError("request_json exhausted retries")


def pause(seconds: float = 4.0) -> None:
    time.sleep(seconds)


def search_image_title(query: str) -> str | None:
    params = urllib.parse.urlencode(
        {
            "action": "query",
            "list": "search",
            "srsearch": f"{query} filetype:bitmap",
            "srnamespace": "6",
            "srlimit": "12",
            "format": "json",
        }
    )
    payload = request_json(f"{API}?{params}")
    pause()

    for hit in payload.get("query", {}).get("search", []):
        title = hit.get("title", "")
        if not title.startswith("File:"):
            continue
        filename = title[5:]
        if IMAGE_EXT.search(filename):
            return filename
    return None


def commons_url(filename: str, width: int = 1280) -> str | None:
    title = f"File:{filename.replace(' ', '_')}"
    params = urllib.parse.urlencode(
        {
            "action": "query",
            "titles": title,
            "prop": "imageinfo",
            "iiprop": "url|thumburl",
            "iiurlwidth": str(width),
            "format": "json",
        }
    )
    payload = request_json(f"{API}?{params}")
    pause()

    pages = payload.get("query", {}).get("pages", {})
    for page in pages.values():
        if "missing" in page:
            return None
        info = page.get("imageinfo", [])
        if info:
            entry = info[0]
            return entry.get("thumburl") or entry.get("url")
    return None


def download_file(url: str, destination: Path) -> None:
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(request, timeout=120) as response:
        destination.write_bytes(response.read())
    pause()


def main() -> int:
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    ok: list[str] = []
    skipped: list[str] = []
    failed: list[tuple[str, str]] = []
    sources: list[str] = [
        "# Test image sources (Wikimedia Commons, freely licensed)",
        "# Local naming: <sign_type>_1.jpg or traffic_light_N.jpg/png",
        "",
    ]

    for local_name, query in DOWNLOADS.items():
        destination = DATA_DIR / local_name
        if destination.exists() and destination.stat().st_size > 1024:
            skipped.append(local_name)
            sources.append(f"{local_name}\t(already present)")
            continue

        try:
            title = MANUAL_FILES.get(local_name)
            url = commons_url(title) if title else None
            if url is None:
                title = search_image_title(query)
                if title is None:
                    failed.append((local_name, f"search found nothing for: {query}"))
                    print(f"FAIL {local_name}: search miss")
                    continue
                url = commons_url(title)
            if url is None:
                failed.append((local_name, f"no URL for File:{title}"))
                print(f"FAIL {local_name}: no URL")
                continue

            download_file(url, destination)
            ok.append(local_name)
            page = f"https://commons.wikimedia.org/wiki/File:{title.replace(' ', '_')}"
            sources.append(f"{local_name}\t{page}")
            print(f"OK   {local_name} <- {title}")
        except Exception as exc:  # noqa: BLE001
            failed.append((local_name, str(exc)))
            print(f"FAIL {local_name}: {exc}")

    existing = sorted(
        p.name
        for p in DATA_DIR.iterdir()
        if p.is_file() and IMAGE_EXT.search(p.name)
    )

    sources.append("")
    sources.append(f"# Total image files: {len(existing)}")
    SOURCES_FILE.write_text("\n".join(sources) + "\n", encoding="utf-8")

    print()
    print(f"Downloaded: {len(ok)}")
    print(f"Skipped (already present): {len(skipped)}")
    print(f"Failed: {len(failed)}")
    print(f"Images in data/: {len(existing)}")
    for name in existing:
        size_kb = (DATA_DIR / name).stat().st_size // 1024
        print(f"  {name} ({size_kb} KB)")

    if failed:
        print("\nFailures:")
        for name, reason in failed:
            print(f"  {name}: {reason}")

    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
