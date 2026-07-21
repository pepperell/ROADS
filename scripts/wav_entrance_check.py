"""Batch check: report the peak sample (and its time) in the first N seconds of each
WAV given on the command line. Companion to wav_spike_scan.py for seed sweeps of
--musictest renders (hunting intermittent cold-start loudness spikes)."""
import sys

from wav_spike_scan import read_wav


def main(seconds: float, paths: list[str]) -> None:
    for path in paths:
        (tag, ch, rate, bits), raw = read_wav(path)
        import struct
        count = len(raw) // 4
        samples = struct.unpack(f"<{count}f", raw)
        frames = min(int(seconds * rate), count // ch)
        peak = 0.0
        peak_t = 0.0
        for f in range(frames):
            for c in range(ch):
                v = abs(samples[f * ch + c])
                if v > peak:
                    peak = v
                    peak_t = f / rate
        print(f"{path}: peak={peak:.3f} at {peak_t:.2f}s")


if __name__ == "__main__":
    main(float(sys.argv[1]), sys.argv[2:])
