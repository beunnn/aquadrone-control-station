#!/usr/bin/env python3
"""Build the phone-friendly parameter tuning sheet (DOCX, then PDF via LibreOffice).

usage: make_param_sheet.py [--out docs]
Source of truth for the numbers: docs/steering-control-analysis.md section 4.
"""
import argparse
import os
import shutil
import subprocess

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Mm, Pt, RGBColor

TITLE = "AquaDrone – steering tuning sheet"
SUBTITLE = ("ArduRover 4.6.3, vectored thrust. Companion to docs/steering-control-analysis.md. "
            "Apply one step at a time, test, then continue. Save the current parameter file first.")

# (name, current, proposed, why)
STEPS = [
    {
        "title": "Step 1 – yaw-rate loop (straight-line nervousness)",
        "goal": "Kill the 0.43 Hz wobble. Loop has ~1 s dead time, so lower gain and let the integrator do the trim.",
        "params": [
            ("ATC_STR_RAT_P", "0.35", "0.15", "Limit cycle at 3 m/s means loop gain ≈ 1. Halving-plus gives > 6 dB margin."),
            ("ATC_STR_RAT_I", "0.02", "0.10", "Integral time P/I ≈ 1.5 s ≈ dead time. Removes the standing 1–2 deg/s error."),
            ("ATC_STR_RAT_IMAX", "0.10", "0.25", "Trim alone needs 0.02–0.05 output; room for wind/current."),
            ("ATC_STR_RAT_FF", "0.60", "0.50", "Rate overshoots 18 → 22 deg/s with P ≈ 0, so FF is high. 0.5 no longer saturates the mixer at 26 % throttle."),
            ("ATC_STR_RAT_D", "0", "0 (keep)", "Dead-time plant: D adds noise, recovers no phase."),
            ("ATC_STR_RAT_SMAX", "9", "0", "Never active at this output scale; disable."),
            ("ATC_STR_RAT_FLTT / FLTE", "2 / 2", "keep", "Optional later: 4 / 4 gains ~6° phase, more gyro noise."),
            ("ATC_STR_RAT_MAX / ACC_MAX", "18 / 6.5", "keep", "18 deg/s ≈ 80 % of measured 22 deg/s capability."),
        ],
        "pass": "≥ 60 s straight at 2.2 and at 3.0 m/s: no 0.3–0.5 Hz yaw wobble (band rms < 1.5 deg/s), steering servo peak-to-peak < 100 µs, "
                "PID_TUNING shows P and I similar size and I settling.",
        "ifnot": "Wobble still visible at top speed → P 0.10. Rate lags demand > 2 s entering a corner → FF +0.05 steps.",
    },
    {
        "title": "Step 2 – position loop (slow 45–50 s weave)",
        "goal": "Add damping to the cross-track loop (currently ζ ≈ 0.41, 51 s period).",
        "params": [
            ("PSC_VEL_P", "0.10", "0.40", "ζ ≈ 0.82, 26 s period. If it looks twitchy use 0.30 (ζ 0.71)."),
            ("PSC_POS_P", "0.15", "keep", "Keeps outer loop ≥ 10× slower than rate loop."),
            ("PSC_VEL_D", "0.02", "keep", ""),
        ],
        "pass": "After a corner the cross-track error is back below 2 m with at most one overshoot; no sustained 40–50 s swing on straights.",
        "ifnot": "Overshoot still > 1 cycle → PSC_VEL_P 0.30. Sluggish return → PSC_POS_P 0.20.",
    },
    {
        "title": "Step 3 – corner planning (sharp corners)",
        "goal": "Planned corner rate ≈ 2·WP_ACCEL / WP_SPEED. Today that is 37 deg/s against an 18 deg/s limit. Target ≈ 13 deg/s at the lowest WP_SPEED.",
        "params": [
            ("WP_ACCEL", "0.8", "0.25", "13 deg/s at 2.2 m/s, 9.5 at 3.0. Corner radius ≈ 7 m. Costs ~9 s / 10 m to reach 2.2 m/s from stop. Use 0.3 if you never run 2.2 m/s."),
            ("WP_JERK", "0.5", "0.25", "Rate onset ≈ 6.5 deg/s², matches ATC_STR_ACC_MAX."),
            ("ATC_TURN_MAX_G", "0.6", "keep", "Do NOT lower: below √2·WP_ACCEL/g the corner blend is delayed and corners get sharper."),
            ("TURN_RADIUS / WP_RADIUS", "25 / 10", "keep", "Only bound the apex distance; inactive here."),
        ],
        "pass": "90° corner: demanded rate < 15 deg/s and never clamped, servo not at 1200/1800 µs for > 0.5 s, cross-track after corner < 3 m.",
        "ifnot": "Still clamped → WP_ACCEL 0.2. Corners too wide/slow → WP_ACCEL 0.3.",
    },
    {
        "title": "Step 4 – speed loop & instrumentation",
        "goal": "Make the throttle hunting diagnosable before touching ATC_SPEED_*.",
        "params": [
            ("GCS_PID_MASK", "1", "3", "Streams the speed PID (PID_TUNING axis 4)."),
            ("CRUISE_SPEED / CRUISE_THROTTLE", "1.30 / 31 (file)", "re-learn", "Log shows ≈ 2.3 m/s @ 26 %. Use CRUISE_LEARN aux switch at WP_SPEED, or read off a steady MANUAL run."),
            ("SRn_EXTRA1 / SRn_RC_CHAN", "4 / 2 Hz", "10 / 10 Hz", "For tuning sessions only; or pull the on-board .bin log (STER, PIDS, PSCN/PSCE at 50 Hz)."),
            ("ATC_SPEED_P / I", "0.1 / 0.02", "keep for now", "Decide with the speed-PID stream in hand."),
        ],
        "pass": "Throttle no longer swings 13–36 % on straights; speed error < 0.3 m/s rms.",
        "ifnot": "",
    },
]

CHECKS = [
    "MOT_VEC_ANGLEMAX = 35 must equal the real thruster angle at SERVO1 1200 / 1800 µs.",
    "The ~1 s command-to-yaw delay is the hard limit on gain. A faster steering actuator is the only way to allow higher P and tighter corners.",
    "The saved .param file differs from the logged configuration (AHRS_ORIENTATION, COMPASS_USE2, cruise pair, WP_SPEED). Verify you are tuning the boat you logged.",
]

QUICK = [
    ("Step 1", [("ATC_STR_RAT_P", "0.15"), ("ATC_STR_RAT_I", "0.10"), ("ATC_STR_RAT_IMAX", "0.25"),
                ("ATC_STR_RAT_FF", "0.50"), ("ATC_STR_RAT_SMAX", "0")]),
    ("Step 2", [("PSC_VEL_P", "0.40")]),
    ("Step 3", [("WP_ACCEL", "0.25"), ("WP_JERK", "0.25")]),
    ("Step 4", [("GCS_PID_MASK", "3")]),
]


def shade(cell, hex_fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    tc_pr.append(shd)


def set_cell_margins(table, mm_val):
    tbl_pr = table._tbl.tblPr
    mar = OxmlElement("w:tblCellMar")
    for side in ("top", "start", "bottom", "end"):
        el = OxmlElement(f"w:{side}")
        el.set(qn("w:w"), str(int(mm_val * 56.7)))
        el.set(qn("w:type"), "dxa")
        mar.append(el)
    tbl_pr.append(mar)


def para(doc, text="", size=11, bold=False, color=None, space_after=3, italic=False):
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(space_after)
    p.paragraph_format.space_before = Pt(0)
    if text:
        r = p.add_run(text)
        r.font.size = Pt(size)
        r.bold = bold
        r.italic = italic
        if color:
            r.font.color.rgb = RGBColor.from_string(color)
    return p


def heading(doc, text, size=14, color="1F3B64", before=8):
    p = para(doc, text, size=size, bold=True, color=color, space_after=2)
    p.paragraph_format.space_before = Pt(before)
    p.paragraph_format.keep_with_next = True
    return p


def param_table(doc, params):
    t = doc.add_table(rows=0, cols=2)
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    set_cell_margins(t, 1.2)
    for name, cur, new, why in params:
        row = t.add_row()
        tr_pr = row._tr.get_or_add_trPr()
        tr_pr.append(OxmlElement("w:cantSplit"))
        c0, c1 = row.cells
        c0.width = Mm(50)
        c1.width = Mm(76)
        keep = new.startswith("keep") or new.startswith("0 (keep")
        shade(c0, "EEF2F7" if keep else "E3F2E1")
        p = c0.paragraphs[0]
        p.paragraph_format.space_after = Pt(0)
        r = p.add_run(name)
        r.bold = True
        r.font.size = Pt(9.5)
        p2 = c0.add_paragraph()
        p2.paragraph_format.space_after = Pt(0)
        r = p2.add_run(f"{cur}  →  ")
        r.font.size = Pt(10)
        r = p2.add_run(new)
        r.font.size = Pt(11)
        r.bold = not keep
        if not keep:
            r.font.color.rgb = RGBColor.from_string("1B6E1B")
        p = c1.paragraphs[0]
        p.paragraph_format.space_after = Pt(0)
        r = p.add_run(why)
        r.font.size = Pt(9.5)
    return t


def build(path_docx):
    doc = Document()
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Mm(148), Mm(210)  # A5 portrait reads well zoomed to width on a phone
    sec.left_margin = sec.right_margin = Mm(10)
    sec.top_margin = sec.bottom_margin = Mm(10)
    doc.styles["Normal"].font.name = "Liberation Sans"
    doc.styles["Normal"].font.size = Pt(11)

    para(doc, TITLE, size=18, bold=True, color="1F3B64", space_after=2)
    para(doc, SUBTITLE, size=9.5, color="555555", space_after=6, italic=True)

    heading(doc, "Diagnosis in one breath", size=12, before=2)
    for line in (
        "• Straight line: 0.43 Hz yaw limit cycle driven by P (±10 deg/s at 3 m/s, growing at 3.15 m/s).",
        "• Slow 45–50 s cross-track weave: under-damped position loop (PSC_VEL_P too low).",
        "• Corners: planner demands ~37 deg/s, boat is limited to 18 → servo on the stop, 5–6 m overshoot.",
        "• Your instincts (no D, more I, less P) are right; the two items above are the other half.",
    ):
        para(doc, line, size=10, space_after=1)

    for step in STEPS:
        heading(doc, step["title"])
        g = para(doc, step["goal"], size=10, italic=True, color="333333", space_after=3)
        g.paragraph_format.keep_with_next = True
        param_table(doc, step["params"])
        p = para(doc, "", size=10, space_after=1)
        p.paragraph_format.space_before = Pt(3)
        r = p.add_run("Pass: ")
        r.bold = True
        r.font.size = Pt(10)
        r = p.add_run(step["pass"])
        r.font.size = Pt(10)
        if step["ifnot"]:
            p = para(doc, "", size=10, space_after=1)
            r = p.add_run("If not: ")
            r.bold = True
            r.font.size = Pt(10)
            r = p.add_run(step["ifnot"])
            r.font.size = Pt(10)

    heading(doc, "Hardware / config checks")
    for c in CHECKS:
        para(doc, "• " + c, size=10, space_after=1)

    doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)
    heading(doc, "Quick copy list", before=0)
    para(doc, "Only the values that change. Paste into Mission Planner / QGC one step at a time.", size=9.5, italic=True, color="555555")
    for label, items in QUICK:
        para(doc, label, size=11, bold=True, space_after=1)
        for n, v in items:
            p = para(doc, "", size=11, space_after=0)
            p.paragraph_format.left_indent = Mm(4)
            r = p.add_run(f"{n},{v}")
            r.font.name = "Liberation Mono"
            r.font.size = Pt(11)
        para(doc, "", space_after=2)
    para(doc, "Re-learn CRUISE_SPEED / CRUISE_THROTTLE at WP_SPEED before changing ATC_SPEED_*.", size=10, italic=True)

    doc.save(path_docx)


def to_pdf(path_docx, out_dir):
    soffice = shutil.which("soffice") or shutil.which("libreoffice")
    if not soffice:
        print("LibreOffice not found; PDF not generated")
        return
    subprocess.run([soffice, "--headless", "--convert-to", "pdf", "--outdir", out_dir, path_docx],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    docx_path = os.path.join(args.out, "AquaDrone_tuning_sheet.docx")
    build(docx_path)
    to_pdf(docx_path, args.out)
    print("written:", docx_path, "and", docx_path.replace(".docx", ".pdf"))


if __name__ == "__main__":
    main()
