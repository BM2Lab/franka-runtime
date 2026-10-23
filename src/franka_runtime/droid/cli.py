"""Run the mock DROID runtime service for loopback tests (D4/D5). Never connects to a robot."""
import argparse
from pathlib import Path
import signal

from contracts.droid import DroidIOSpec
from .config import DroidRuntimeConfig
from .service import DroidRuntimeService


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="droid.jointpos config (backend: mock)")
    parser.add_argument("--contract", type=Path, required=True, help="DroidIOSpec JSON allow-listed by the config")
    args = parser.parse_args()
    service = DroidRuntimeService(DroidRuntimeConfig.load(args.config), DroidIOSpec.from_json(args.contract.read_text()))
    port = service.bind()
    signal.signal(signal.SIGTERM, lambda *_: service.shutdown())
    print(f"mock DROID runtime on {service.settings['bind_host']}:{port} clock {service.clock_id}", flush=True)
    try:
        service.serve()
    except KeyboardInterrupt:
        service.shutdown()


if __name__ == "__main__":
    main()
