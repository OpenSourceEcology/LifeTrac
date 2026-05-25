"""Minimal amqtt broker on 0.0.0.0:1883 for LifeTrac smoke tests."""
import asyncio
import logging
import sys

from amqtt.broker import Broker

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

CONFIG = {
    "listeners": {
        "default": {
            "type": "tcp",
            "bind": "0.0.0.0:1883",
            "max_connections": 50,
        }
    },
    "sys_interval": 0,
    "auth": {"allow-anonymous": True},
    "topic-check": {"enabled": False},
}


async def main():
    broker = Broker(CONFIG)
    await broker.start()
    print("__BROKER_READY__", flush=True)
    while True:
        await asyncio.sleep(60)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
