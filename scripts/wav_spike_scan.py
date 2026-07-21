"""Scan a WAV (PCM16 or IEEE float32, as written by NAudio's WaveFileWriter) for its
amplitude envelope: prints per-100ms peak/RMS so a cold-start loudness spike (out-of-
place loud note) can be located in time. Parses the RIFF chunks directly because
Python's wave module rejects format 3 (IEEE float).
Temp diagnostic for the title-screen loud-note investigation."""
import struct
import sys


def read_wav(path):
    with open(path, "rb") as f:
        data = f.read()
    assert data[:4] == b"RIFF" and data[8:12] == b"WAVE", "not a WAV"
    pos = 12
    fmt = None
    while pos + 8 <= len(data):
        cid = data[pos:pos + 4]
        size = struct.unpack_from("<I", data, pos + 4)[0]
        body = pos + 8
        if cid == b"fmt ":
            tag, ch, rate, _, _, bits = struct.unpack_from("<HHIIHH", data, body)
            fmt = (tag, ch, rate, bits)
        elif cid == b"data":
            return fmt, data[body:body + size]
        pos = body + size + (size & 1)
    raise SystemExit("no data chunk")


def main(path: str, seconds: float) -> None:
    (tag, ch, rate, bits), raw = read_wav(path)
    if tag == 3 and bits == 32:
        count = len(raw) // 4
        samples = struct.unpack(f"<{count}f", raw)
    elif tag == 1 and bits == 16:
        count = len(raw) // 2
        samples = [s / 32768.0 for s in struct.unpack(f"<{count}h", raw)]
    else:
        raise SystemExit(f"unsupported format tag={tag} bits={bits}")

    frames = min(int(seconds * rate), count // ch)
    win = rate // 10  # 100 ms
    print(f"rate={rate} ch={ch} tag={tag} bits={bits} frames={frames}")
    print("time  peak   rms      bar")
    for start in range(0, frames, win):
        end = min(start + win, frames)
        peak = 0.0
        acc = 0.0
        cnt = 0
        for f in range(start, end):
            for c in range(ch):
                v = abs(samples[f * ch + c])
                if v > peak:
                    peak = v
                acc += v * v
                cnt += 1
        rms = (acc / max(cnt, 1)) ** 0.5
        bar = "#" * int(peak * 120)
        print(f"{start / rate:5.1f} {peak:.3f} {rms:.4f}  {bar}")


if __name__ == "__main__":
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 8.0)
