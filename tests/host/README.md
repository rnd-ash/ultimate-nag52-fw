# Owner arithmetic ports

Run `python3 -m unittest discover -s tests/host -v` from the repository root.
Tests compile the current production helpers and extracted call sites with
explicit host dependencies under ASan/UBSan. Packed-host alignment is excluded.
They cover arithmetic and wiring, not complete OEM control or physical hydraulics.
Existing configured curves, maps, settings and storage layouts are preserved.
