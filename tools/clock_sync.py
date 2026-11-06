#!/usr/bin/env python3
"""CAN-only OpenIMU clock mapping. Standard-library core; python-can for CLI.

All common times are relative to the selected node's MCU, NOT UTC. Transport
adapters must pass a local monotonic receive time, never a remote wall clock.
"""
from __future__ import annotations
import argparse
from collections import OrderedDict, deque
from dataclasses import dataclass
import json
import math
import secrets
import statistics
import struct
import time

SYNC_ID = 0x700
ANCHOR_BASE = 0x710
VERSION = 1
START, TICK, STOP = 1, 2, 3
CAN_VALID, SAMPLE_VALID = 1, 2
ANCHOR = struct.Struct('>BBBBIIIQQQII')


def control(session: int, sequence: int, command: int = TICK) -> bytes:
    if command not in (START, TICK, STOP) or not 0 < session <= 0xffffffff:
        raise ValueError('invalid control command/session')
    return struct.pack('>4sBBHII', b'OSYN', VERSION, command, 0, session, sequence)


def forward32(new: int, old: int) -> bool:
    return 0 < ((new - old) & 0xffffffff) < 0x80000000


@dataclass(frozen=True)
class Anchor:
    node: int
    session: int
    sequence: int
    epoch: int
    flags: int
    local_sync: int
    local_sample: int
    source_sample: int
    overrun: int
    invalid: int

    @classmethod
    def decode(cls, can_id: int, payload: bytes):
        if len(payload) != ANCHOR.size:
            raise ValueError('anchor length')
        version, flags, node, reserved, session, seq, epoch, ls, lp, sp, ov, inv = ANCHOR.unpack(payload)
        if (version != VERSION or reserved or flags & ~3 or not 1 <= node <= 5
                or can_id != ANCHOR_BASE + node or not session or not epoch
                or (flags & SAMPLE_VALID and not flags & CAN_VALID)):
            raise ValueError('anchor header')
        if flags & SAMPLE_VALID and abs(lp - ls) > 1_000_000:
            raise ValueError('anchor sample too distant')
        return cls(node, session, seq, epoch, flags, ls, lp, sp, ov, inv)


class Affine:
    """Robust centered fit, bounded history and explicit freshness/quality gates."""
    def __init__(self, residual_us=100.0, max_ppm=5000.0):
        self.points = deque(maxlen=32)
        self.residual_limit = residual_us
        self.max_ppm = max_ppm
        self.model = None
        self.updated = -math.inf
        self.last_accepted = -math.inf
        self.residual_us = None

    def add(self, x: int, y: int, now: float) -> bool:
        if now - self.last_accepted > 1.0:
            self.points.clear()
            self.model = None
            self.residual_us = None
        if self.points and x <= self.points[-1][0]:
            return False
        if self.model is not None and abs(self.map(x) - y) > self.residual_limit:
            return False
        points = list(self.points) + [(x, y)]
        points = points[-32:]
        if len(points) < 3:
            self.points.append((x, y))
            self.last_accepted = now
            return True
        x0, y0 = points[0]
        centered = [(a - x0, b - y0) for a, b in points]
        slopes = [(v - b) / (u - a) for i, (a, b) in enumerate(centered)
                  for u, v in centered[i + 1:] if u > a]
        slope = statistics.median(slopes)
        offset = statistics.median(b - slope * a for a, b in centered)
        if abs(slope - 1) * 1e6 > self.max_ppm:
            return False
        inliers = [(a, b) for a, b in centered
                   if abs(b - (slope * a + offset)) <= self.residual_limit]
        if centered[-1] not in inliers:
            return False
        self.points = deque(((a + x0, b + y0) for a, b in inliers), maxlen=32)
        self.last_accepted = now
        if len(inliers) < 8 or inliers[-1][0] - inliers[0][0] < 500_000:
            return True
        # Fit only bounded differences; add the rounded delta to the integer origin.
        self.model = (x0, y0, slope, offset)
        self.residual_us = max(abs(b - (slope * a + offset)) for a, b in inliers)
        self.updated = now
        return True

    def map(self, x):
        x0, y0, slope, offset = self.model
        return y0 + round((x - x0) * slope + offset)

    def valid(self, now):
        return self.model is not None and 0 <= now - self.updated <= 1.0


class ClockMapper:
    """One host session, one explicitly selected reference. No automatic election."""
    def __init__(self, session: int, reference: int = 1):
        if not 0 < session <= 0xffffffff or not 1 <= reference <= 5:
            raise ValueError('session/reference')
        self.session = session
        self.reference = reference
        self.reference_epoch = None
        self.reference_floor = None
        self.epochs = {}
        self.sensor = {}
        self.mcu = {}
        self.latest = {}
        self.last_sequence = {}
        self.groups = OrderedDict()
        self.bad_groups = deque(maxlen=128)
        self.last_data = {}
        self.retired_epochs = {}
        self.latest_reference = None
        self.rejected = 0

    def _reset_models(self, clear_watermarks=False):
        self.sensor.clear()
        self.mcu.clear()
        self.latest.clear()
        if clear_watermarks:
            self.last_data.clear()
        self.groups.clear()
        self.latest_reference = None

    def _epoch(self, node, epoch):
        if epoch == 0 or epoch in self.retired_epochs.get(node, ()):
            return False
        old = self.epochs.get(node)
        if old == epoch:
            return True
        if old is not None:
            self.retired_epochs.setdefault(node, deque(maxlen=64)).append(old)
        if node == self.reference:
            self._reset_models(clear_watermarks=True)
            self.reference_epoch = epoch
            self.reference_floor = None
        self.sensor.pop(node, None)
        self.mcu.pop(node, None)
        self.latest.pop(node, None)
        self.last_data.pop(node, None)
        # Keep immutable old group identities for delayed CAN revocations.
        # Pairing below excludes anchors whose epoch is no longer current.
        self.epochs[node] = epoch
        return True

    def anchor(self, can_id: int, payload: bytes, now: float) -> bool:
        if not math.isfinite(now):
            return False
        try:
            a = Anchor.decode(can_id, payload)
        except (ValueError, struct.error):
            self.rejected += 1
            return False
        if a.session != self.session or a.sequence in self.bad_groups:
            return False
        group = self.groups.get(a.sequence, {})
        previous = group.get(a.node)
        retired = a.epoch in self.retired_epochs.get(a.node, ())
        if retired and (a.flags & CAN_VALID or previous is None or previous.epoch != a.epoch):
            return False
        last = self.last_sequence.get(a.node)
        if not a.flags & CAN_VALID:
            # Revoke a known group in its original epoch, or a current/new
            # control. Unrelated old epochs/sequences cannot erase live fits.
            known = previous is not None and previous.epoch == a.epoch
            recent = last is None or a.sequence == last or forward32(a.sequence, last)
            if not known and not recent:
                return False
            if not known and not self._epoch(a.node, a.epoch):
                return False
            self.bad_groups.append(a.sequence)
            self._reset_models()
            self.rejected += 1
            return False
        if previous:
            if previous != a:
                self.bad_groups.append(a.sequence)
                self._reset_models()
                self.rejected += 1
            return False
        if last is not None and not forward32(a.sequence, last):
            return False
        if not self._epoch(a.node, a.epoch):
            return False
        self.last_sequence[a.node] = a.sequence
        group = self.groups.setdefault(a.sequence, {})
        group[a.node] = a
        while len(self.groups) > 128:
            self.groups.popitem(last=False)
        if a.flags & SAMPLE_VALID:
            fit = self.sensor.setdefault(a.node, Affine(250.0, 50_000.0))
            if fit.add(a.source_sample, a.local_sample, now):
                self.latest[a.node] = (a, now)
        ref = group.get(self.reference)
        if ref is not None:
            for node, other in group.items():
                if node != self.reference and self.epochs.get(node) == other.epoch:
                    self.mcu.setdefault(node, Affine()).add(other.local_sync, ref.local_sync, now)
        if a.node == self.reference:
            if self.reference_floor is None:
                self.reference_floor = a.local_sync
            self.latest_reference = (a, now)
        return True

    def data(self, node: int, payload: bytes, now: float) -> dict:
        if not math.isfinite(now):
            raise ValueError('receive time must be finite monotonic seconds')
        if not 1 <= node <= 5 or len(payload) not in (16, 24):
            raise ValueError('data node/length')
        raw = struct.unpack('>H7h', payload[:16])
        out = dict(node=node, timestamp16=raw[0], raw=list(raw[1:]),
                   source_time_us=None, common_time_us=None, epoch=None,
                   session=self.session, reference=self.reference,
                   reference_epoch=self.reference_epoch, sync_valid=False, reason='legacy_frame')
        if len(payload) == 16:
            return out
        low, epoch = struct.unpack('>II', payload[16:])
        out.update(source_time_low32=low, epoch=epoch, reason='no_fresh_anchor')
        if (low & 0xffff) != raw[0]:
            out['reason'] = 'timestamp_mismatch'
            return out
        if not self._epoch(node, epoch):
            out['reason'] = 'retired_epoch'
            return out
        out['reference_epoch'] = self.reference_epoch
        entry = self.latest.get(node)
        if not entry or entry[0].epoch != epoch or not 0 <= now - entry[1] <= 1.0:
            return out
        anchor = entry[0]
        delta = ((low - (anchor.source_sample & 0xffffffff) + 0x80000000) & 0xffffffff) - 0x80000000
        if delta < -1_000_000 or delta > (now-entry[1])*1_050_000 + 100_000:
            out['reason'] = 'ambiguous_source_time'
            return out
        source = anchor.source_sample + delta
        if source < 0:
            return out
        out['source_time_us'] = source
        prior = self.last_data.get(node)
        if prior and source <= prior[0]:
            out['reason'] = 'out_of_order_data'
            return out
        sf = self.sensor.get(node)
        mf = self.mcu.get(node)
        ref = getattr(self, 'latest_reference', None)
        if not sf or not sf.valid(now) or not ref or not 0 <= now - ref[1] <= 1.0:
            out['reason'] = 'model_not_ready'
            return out
        local = sf.map(source)
        if node == self.reference:
            common = local
        elif mf and mf.valid(now):
            common = mf.map(local)
        else:
            out['reason'] = 'model_not_ready'
            return out
        if self.reference_floor is None or common < self.reference_floor:
            out['reason'] = 'before_reference_epoch'
            return out
        if prior and common <= prior[1]:
            out['reason'] = 'nonmonotonic_mapping'
            return out
        self.last_data[node] = (source, common)
        out.update(common_time_us=round(common), sync_valid=True, reason='valid',
                   sensor_fit_residual_us=sf.residual_us,
                   mcu_fit_residual_us=0.0 if node == self.reference else mf.residual_us)
        return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='socketcan')
    parser.add_argument('--channel', default='can0')
    parser.add_argument('--reference', type=int, choices=range(1, 6), default=1)
    parser.add_argument('--session', type=lambda x: int(x, 0), default=None)
    parser.add_argument('--listen-only', action='store_true', help='require --session; external master sends controls')
    args = parser.parse_args()
    if args.listen_only and not args.session:
        parser.error('--listen-only requires the external master --session')
    try:
        import can
    except ImportError:
        parser.error('install python-can to use the CAN transport')
    session = args.session or secrets.randbelow(0xffffffff) + 1
    engine = ClockMapper(session, args.reference)
    sequence = 0
    bus = can.Bus(interface=args.interface, channel=args.channel, fd=True)
    next_send = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if not args.listen_only and now >= next_send:
                # Periodic START permits late joining nodes, TICK maintains the lease.
                command = START if sequence % 5 == 0 else TICK
                bus.send(can.Message(arbitration_id=SYNC_ID, is_extended_id=False,
                                     is_fd=True, bitrate_switch=True,
                                     data=control(session, sequence, command)), timeout=0.05)
                sequence = (sequence + 1) & 0xffffffff
                next_send = now + 0.1
            msg = bus.recv(timeout=0.01)
            if msg is None or msg.is_error_frame or msg.is_remote_frame or msg.is_extended_id or not msg.is_fd:
                continue
            now = time.monotonic()
            if ANCHOR_BASE < msg.arbitration_id <= ANCHOR_BASE + 5:
                engine.anchor(msg.arbitration_id, bytes(msg.data), now)
            elif 1 <= msg.arbitration_id <= 5 and len(msg.data) in (16, 24):
                print(json.dumps(engine.data(msg.arbitration_id, bytes(msg.data), now)), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not args.listen_only:
                bus.send(can.Message(arbitration_id=SYNC_ID, is_extended_id=False,
                                     is_fd=True, bitrate_switch=True,
                                     data=control(session, sequence, STOP)), timeout=0.05)
        finally:
            bus.shutdown()


if __name__ == '__main__':
    main()
