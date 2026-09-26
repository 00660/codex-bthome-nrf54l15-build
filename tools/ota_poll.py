#!/usr/bin/env python3
"""
轮询唤醒窗口做 BLE OTA。

设备平时在睡，只被「10 分钟定时器」或「第 8 脚按键」唤醒，醒来后广播
120 秒。所以这里不去找设备，而是一直循环扫描，撞上窗口就连上去刷机。

刷机走 MCUboot 的安全路径：

    upload(upgrade=False)  把镜像写进非活动槽（slot 1）
    ImageStatesRead        读回槽状态，取出刚写进去那个镜像的 hash
    ImageStatesWrite       把它标记成 pending（下次启动试运行）
    ResetWrite             重启
    新固件启动后自己调 boot_write_img_confirmed() 落定

这样万一新镜像起不来，MCUboot 会自动回滚到旧固件，不会把设备刷砖。

用法：
    python tools/ota_poll.py artifacts/bthome_nrf54l15_app.signed.bin
    python tools/ota_poll.py <固件.bin> [MAC地址或设备名]
"""

from __future__ import annotations

import asyncio
import hashlib
import pathlib
import sys

from smpclient import SMPClient
from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite
from smpclient.requests.os_management import ResetWrite
from smpclient.transport.ble import SMPBLETransport

DEFAULT_TARGET = "EA:9F:FE:F3:26:5B"

# connect() 内部就是拿这个超时去扫描设备，所以它同时也是「等窗口」的时间
SCAN_TIMEOUT_S = 30.0
# 单次 SMP 请求的超时（蓝牙这带宽给宽一点）
REQUEST_TIMEOUT_S = 25.0
# 首包要擦 flash，给更长的超时
FIRST_CHUNK_TIMEOUT_S = 60.0


async def do_ota(client: SMPClient, image: bytes) -> bool:
    print("    上传镜像 ...", flush=True)
    last_reported = -10
    async for offset in client.upload(
        image,
        upgrade=False,
        first_timeout_s=FIRST_CHUNK_TIMEOUT_S,
        subsequent_timeout_s=REQUEST_TIMEOUT_S,
    ):
        pct = offset * 100 // len(image)
        if pct >= last_reported + 10:
            last_reported = pct
            print(f"      {pct}%", flush=True)
    print("    上传完成", flush=True)

    states = await client.request(ImageStatesRead())
    for img in states.images:
        print(
            f"    slot={img.slot} ver={img.version} active={img.active} "
            f"pending={img.pending} confirmed={img.confirmed} bootable={img.bootable}",
            flush=True,
        )

    # 刚上传的镜像在非活动槽里，取出它的 hash 用来标记 pending
    target = next((img for img in states.images if img.hash and not img.active), None)
    if target is None:
        print("    ★ 找不到刚写入的镜像槽，放弃这次 OTA", flush=True)
        return False

    print(f"    标记 slot={target.slot} 为 pending", flush=True)
    await client.request(ImageStatesWrite(hash=target.hash, confirm=False))

    print("    重启切到新镜像", flush=True)
    try:
        await client.request(ResetWrite(force=1))
    except Exception as exc:  # 设备会直接断连，这里报错是正常的
        print(f"    （设备已断开: {type(exc).__name__}）", flush=True)

    return True


async def run(target: str, image_path: pathlib.Path) -> int:
    image = image_path.read_bytes()
    digest = hashlib.sha256(image).hexdigest()[:16]
    print(f"固件  {image_path.name}")
    print(f"大小  {len(image)} 字节   sha256[:16]={digest}")
    print(f"目标  {target}")
    print()

    attempt = 0
    while True:
        attempt += 1
        print(f"[{attempt}] 扫描唤醒窗口（最多 {SCAN_TIMEOUT_S:.0f} 秒）...", flush=True)

        client = SMPClient(SMPBLETransport(), target, timeout_s=REQUEST_TIMEOUT_S)
        try:
            await client.connect()
        except Exception as exc:
            print(f"    还没醒 / 没扫到：{type(exc).__name__}: {exc}", flush=True)
            continue

        print("    ★ 抓到唤醒窗口，开始 OTA", flush=True)

        ok = False
        try:
            ok = await do_ota(client, image)
        except Exception as exc:
            print(f"    OTA 失败：{type(exc).__name__}: {exc}", flush=True)
        finally:
            try:
                await client.disconnect()
            except Exception:
                pass

        if ok:
            print("\n★ 上传并标记完成。设备重启后会跑新固件并自动确认。", flush=True)
            return 0

        print("    等下一个窗口重试 ...", flush=True)


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__.strip())
        return 2

    image_path = pathlib.Path(sys.argv[1])
    if not image_path.is_file():
        print(f"找不到固件文件：{image_path}")
        return 2

    target = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TARGET

    try:
        return asyncio.run(run(target, image_path))
    except KeyboardInterrupt:
        print("\n已中断")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
