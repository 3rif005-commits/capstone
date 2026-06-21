#!/usr/bin/env python3
"""Tactical-HUD intel panels for the machinima entity briefings.

Draws a war-game style info panel over a (frozen) video frame, and composites
the panels into the rendered clip: at each entity's freeze point the clip holds
and the panel slides in. Pure cv2 — runs headless, no editor needed.

Preview:   python3 machinima_panels.py preview
Composite: python3 machinima_panels.py render <in.mp4> <out.mp4>
"""
import sys
import cv2
import numpy as np

# ── Panel content (edit freely) ────────────────────────────────────────────────
PANELS = {
    'tank': dict(
        tag='OBJECTIVE', name='DEFENDED ASSET', side='left',
        rows=[('TYPE', 'Ground target (static)'),
              ('ROLE', 'The prize both sides fight for'),
              ('TECH', 'Mission objective')],
        bar=('VALUE', 1.0, 'CRITICAL'),
        accent=(60, 200, 255)),                 # amber (BGR)
    'kamikaze': dict(
        tag='UNIT 01', name='KAMIKAZE DRONE', side='left',
        rows=[('TYPE', 'X3 quadrotor'),
              ('ROLE', 'Player strike craft'),
              ('SPEED', '~9 m/s   can hover'),
              ('TECH', 'Player teleop')],
        bar=('THREAT', 0.85, 'HIGH'),
        accent=(70, 90, 240)),                  # red-orange
    'interceptor': dict(
        tag='UNIT 02', name='INTERCEPTOR', side='right',
        rows=[('TYPE', 'Autonomous fixed-wing'),
              ('ROLE', 'Air defense - hunts the kamikaze'),
              ('SPEED', '~28 m/s   turn ~28 m'),
              ('GUIDANCE', 'Augmented PN + GPS-style est.')],
        bar=('DEFENSE', 0.9, 'ACTIVE'),
        accent=(255, 200, 40)),                 # cyan
}

# Freeze points (seconds into the clip) + how long to hold on each.
# Freeze at the END of each entity's cinematic (segment ends: tank 6s,
# kamikaze 15s, interceptor 20s) so the panel caps the shot, then we cut on.
FREEZES = [('tank', 5.9, 3.5), ('kamikaze', 14.9, 3.5), ('interceptor', 19.9, 3.5)]
SLIDE = 0.5                                     # slide-in time (s)

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT2 = cv2.FONT_HERSHEY_DUPLEX
PAD = 22
W = 520                                          # panel width


def _blend(img, x0, y0, x1, y1, color, alpha):
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(img.shape[1], x1), min(img.shape[0], y1)
    if x1 <= x0 or y1 <= y0:
        return
    sub = img[y0:y1, x0:x1]
    ov = np.empty_like(sub)
    ov[:] = color
    cv2.addWeighted(ov, alpha, sub, 1 - alpha, 0, sub)


def _brackets(img, x0, y0, x1, y1, color, ln=26, th=2):
    for (cx, cy, dx, dy) in [(x0, y0, 1, 1), (x1, y0, -1, 1),
                             (x0, y1, 1, -1), (x1, y1, -1, -1)]:
        cv2.line(img, (cx, cy), (cx + dx * ln, cy), color, th)
        cv2.line(img, (cx, cy), (cx, cy + dy * ln), color, th)


def draw_panel(frame, spec, reveal=1.0):
    """Draw a panel on a copy of frame. reveal in [0,1] slides/fades it in."""
    out = frame.copy()
    acc = spec['accent']
    rows = spec['rows']
    h = PAD + 30 + 18 + 46 + 14 + len(rows) * 34 + 16 + 30 + PAD
    side = spec.get('side', 'left')
    y = frame.shape[0] - h - 46
    if side == 'right':
        x = frame.shape[1] - W - 46
        x += int((1.0 - reveal) * (W + 60))     # slide in from the right
    else:
        x = 46
        x += int(-(1.0 - reveal) * (W + 60))    # slide in from the left
    alpha_mul = max(0.0, min(1.0, reveal * 1.3))

    _blend(out, x, y, x + W, y + h, (18, 14, 10), 0.62 * alpha_mul)
    cv2.rectangle(out, (x, y), (x + W, y + h), acc, 1)
    _brackets(out, x, y, x + W, y + h, acc)

    cx = x + PAD
    cy = y + PAD + 18
    # tag chip
    tag = spec['tag']
    (tw, th0), _ = cv2.getTextSize(tag, FONT, 0.5, 1)
    cv2.rectangle(out, (cx - 4, cy - th0 - 8), (cx + tw + 10, cy + 6), acc, -1)
    cv2.putText(out, tag, (cx + 3, cy), FONT, 0.5, (15, 12, 8), 1, cv2.LINE_AA)
    # name
    cy += 46
    cv2.putText(out, spec['name'], (cx, cy), FONT2, 1.0, (245, 245, 245), 1, cv2.LINE_AA)
    # divider
    cy += 16
    cv2.line(out, (cx, cy), (x + W - PAD, cy), acc, 1)
    cv2.line(out, (cx, cy), (cx + 60, cy), acc, 3)
    # rows
    cy += 14
    for label, val in rows:
        cv2.putText(out, label, (cx, cy + 16), FONT, 0.46, acc, 1, cv2.LINE_AA)
        cv2.putText(out, val, (cx + 130, cy + 16), FONT, 0.52,
                    (235, 235, 235), 1, cv2.LINE_AA)
        cy += 34
    # bar
    blabel, frac, btext = spec['bar']
    cy += 18
    cv2.putText(out, blabel, (cx, cy), FONT, 0.46, acc, 1, cv2.LINE_AA)
    bx, bw, segs = cx + 130, 220, 14
    fill = int(round(frac * segs))
    for i in range(segs):
        sx = bx + i * (bw // segs)
        col = acc if i < fill else (70, 65, 60)
        cv2.rectangle(out, (sx, cy - 12), (sx + bw // segs - 3, cy), col, -1)
    cv2.putText(out, btext, (bx + bw + 12, cy), FONT, 0.5, (235, 235, 235), 1, cv2.LINE_AA)

    if alpha_mul < 1.0:
        cv2.addWeighted(out, alpha_mul, frame, 1 - alpha_mul, 0, out)
    return out


def preview():
    for name in PANELS:
        f = cv2.imread(f'/tmp/frz_{name}.png')
        if f is None:
            print('missing /tmp/frz_%s.png' % name)
            continue
        cv2.imwrite(f'/tmp/panel_{name}.png', draw_panel(f, PANELS[name]))
        print('wrote /tmp/panel_%s.png' % name)


def render(inp, outp):
    cap = cv2.VideoCapture(inp)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    wr = cv2.VideoWriter(outp, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    frames = [cap.read()[1] for _ in range(n)]
    cap.release()
    out_i = 0
    for i, fr in enumerate(frames):
        if fr is None:
            continue
        wr.write(fr)                            # the normal clip
        t = i / fps
        for name, ft, hold in FREEZES:
            if abs(t - ft) < 0.5 / fps:          # at the freeze point, inject hold
                held = fr.copy()
                steps = int(hold * fps)
                for s in range(steps):
                    rv = min(1.0, (s / fps) / SLIDE)
                    wr.write(draw_panel(held, PANELS[name], reveal=rv))
    wr.release()
    print('wrote', outp)


if __name__ == '__main__':
    if len(sys.argv) >= 2 and sys.argv[1] == 'render':
        render(sys.argv[2], sys.argv[3])
    else:
        preview()
