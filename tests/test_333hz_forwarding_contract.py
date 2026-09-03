from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
MAIN_C = ROOT / "Core" / "Src" / "main.c"


def verify_can_forwarding_period_is_3ms_for_333hz_branch():
    text = MAIN_C.read_text(encoding="utf-8")
    match = re.search(r"#define\s+PACKET_PERIOD_MS\s+(\d+)U", text)
    assert match, "PACKET_PERIOD_MS definition not found"
    assert match.group(1) == "3", (
        "333 Hz firmware must forward at a 3 ms period; "
        f"found PACKET_PERIOD_MS={match.group(1)}U"
    )


if __name__ == "__main__":
    verify_can_forwarding_period_is_3ms_for_333hz_branch()
