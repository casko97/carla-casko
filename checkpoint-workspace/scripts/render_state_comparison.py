#!/usr/bin/env python3
"""Render a schematic, top-down video from recorded CARLA state traces."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    data = json.loads(Path(args.report).read_text())
    seqs = [[row['observation'] for row in data['baseline']], data['candidate']]
    if not seqs[0] or len(seqs[0]) != len(seqs[1]):
        raise ValueError('unequal or empty sequences')
    path = Path(args.output)
    if path.exists():
        raise FileExistsError(str(path))
    positions = [item['states'][name]['location'] for seq in seqs for item in seq for name in ('expert', 'traffic')]
    min_y, max_y = min(p[1] for p in positions), max(p[1] for p in positions)
    min_x, max_x = min(p[0] for p in positions), max(p[0] for p in positions)
    scale = min(23.0, 780.0 / max(12.0, max_y - min_y + 12.0), 420.0 / max(12.0, max_x - min_x + 12.0))
    cy, cx = (min_y + max_y) / 2, (min_x + max_x) / 2

    def project(x, y):
        return int(round(480 + (y - cy) * scale)), int(round(385 - (x - cx) * scale))

    def draw_vehicle(panel, state, color, label):
        x, y, _ = state['location']
        yaw = math.radians(state['rotation'][1])
        forward = np.array([math.sin(yaw), -math.cos(yaw)])
        side = np.array([math.cos(yaw), math.sin(yaw)])
        center = np.array(project(x, y), dtype=float)
        points = np.array([center + sign1 * forward * 2.45 * scale + sign2 * side * .92 * scale
                           for sign1, sign2 in ((1, 1), (1, -1), (-1, -1), (-1, 1))], dtype=np.int32)
        cv2.fillConvexPoly(panel, points, color)
        cv2.polylines(panel, [points], True, (235, 235, 235), 2)
        cv2.arrowedLine(panel, tuple(center.astype(int)), tuple((center + forward * 1.8 * scale).astype(int)),
                        (255, 255, 255), 2, tipLength=.3)
        cv2.putText(panel, label, (int(center[0]) - 25, int(center[1]) - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .52, color, 1, cv2.LINE_AA)

    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), 20, (1920, 720))
    if not writer.isOpened():
        raise RuntimeError('cannot create output video')
    try:
        for index in range(len(seqs[0])):
            panels = []
            for seq, title in zip(seqs, ('ORIGINAL SIMULATED TRACE', 'RESTORED SAME-EPISODE TRACE')):
                panel = np.full((720, 960, 3), (28, 31, 35), dtype=np.uint8)
                for offset in range(-5, 6):
                    px, _ = project(cx, cy + offset * 5)
                    _, py = project(cx + offset * 5, cy)
                    cv2.line(panel, (px, 110), (px, 650), (48, 51, 55), 1)
                    cv2.line(panel, (0, py), (960, py), (48, 51, 55), 1)
                row = seq[index]
                for name, color, label in [('expert', (255, 150, 40), 'EGO'), ('traffic', (20, 150, 255), 'TRAFFIC')]:
                    points = [project(*item['states'][name]['location'][:2]) for item in seq[:index + 1]]
                    if len(points) > 1:
                        cv2.polylines(panel, [np.array(points, dtype=np.int32)], False, color, 2)
                    draw_vehicle(panel, row['states'][name], color, label)
                cv2.rectangle(panel, (0, 0), (960, 105), (0, 0, 0), -1)
                cv2.putText(panel, title, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
                cv2.putText(panel, 'STATE-DERIVED SCHEMATIC - NOT CAMERA FOOTAGE', (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (170, 220, 255), 1)
                cv2.putText(panel, 'frame {}/{}  t={:.2f}s  grid=5m'.format(index + 1, len(seq), (index + 1) * .05),
                            (20, 88), cv2.FONT_HERSHEY_SIMPLEX, .6, (235, 235, 235), 1)
                panels.append(panel)
            writer.write(np.hstack(panels))
    finally:
        writer.release()
    print(path, flush=True)


if __name__ == '__main__':
    main()
