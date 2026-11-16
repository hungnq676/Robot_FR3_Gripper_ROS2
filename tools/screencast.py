#!/usr/bin/env python3
"""Record the whole screen (RViz, terminals) to WebM through GNOME Shell's own
screencast (Wayland: no ffmpeg/x11grab here). GNOME stops a screencast when its
D-Bus caller goes away, so this process stays up until Ctrl+C / SIGTERM.

Usage: screencast.py OUT.webm [--fps 15]
"""
import signal
import sys

from gi.repository import Gio, GLib


def call(bus, method, args=None):
    return bus.call_sync("org.gnome.Shell.Screencast", "/org/gnome/Shell/Screencast",
                         "org.gnome.Shell.Screencast", method, args, None,
                         Gio.DBusCallFlags.NONE, -1, None).unpack()


def main():
    out = sys.argv[1]
    fps = int(sys.argv[sys.argv.index("--fps") + 1]) if "--fps" in sys.argv else 15
    bus = Gio.bus_get_sync(Gio.BusType.SESSION)
    ok, used = call(bus, "Screencast", GLib.Variant("(sa{sv})", (out, {
        "framerate": GLib.Variant("i", fps), "draw-cursor": GLib.Variant("b", True)})))
    if not ok:
        sys.exit("screencast: GNOME tu choi quay man hinh")
    print(f"screencast: ghi {used}", flush=True)
    loop = GLib.MainLoop()
    for s in (signal.SIGINT, signal.SIGTERM):
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, s, loop.quit)
    loop.run()
    call(bus, "StopScreencast")
    print("screencast: dung", flush=True)


if __name__ == "__main__":
    main()
