"""make_markers.py: printable ArUco markers at TRUE size for the ZADP arena (robots 1-3, corners 10-13).

Numbers from link_config.json (MATLAB writes it from params/twin_params.m, P.cam): dictionary, ids, marker sizes,
quiet zone, corner positions. Needs OpenCV with cv2.aruco (the bit patterns come from OpenCV's dictionary) and
Pillow. Writes, next to this script:
    robots_A4.pdf      robot labels 1-3: marker (P.cam.markerSize) + white quiet zone + a strip with the NOSE arrow
    corners_A4.pdf     corner labels 10-13 with the arena position of each marker centre
    marker_<id>.png    one marker per file at --dpi (black square only, plus the quiet zone)
    placement.txt      where everything goes
Print the PDFs at 100 % ("actual size", no fit-to-page) and check the 50 mm scale bar with a ruler.

    python make_markers.py [--config ../link_config.json] [--dpi 600]
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), 'link_config.json')
A4_MM = (210.0, 297.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--dpi', type=int, default=600)
    ap.add_argument('--out', default=HERE)
    a = ap.parse_args()
    try:
        import cv2
        if not hasattr(cv2, 'aruco'):
            raise ImportError('cv2 has no aruco module')
    except ImportError as e:
        print('make_markers: OpenCV with cv2.aruco is needed (pip install opencv-contrib-python): %s' % e, file=sys.stderr)
        return 2
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np

    cam = json.load(open(a.config))['cam']
    dpi = a.dpi
    px = lambda mm: int(round(mm / 25.4 * dpi))
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cam['dictionary']))

    def marker_img(mid, size_mm):
        n = px(size_mm)
        if hasattr(cv2.aruco, 'generateImageMarker'):
            m = cv2.aruco.generateImageMarker(dic, int(mid), n, borderBits=1)
        else:
            m = cv2.aruco.drawMarker(dic, int(mid), n, borderBits=1)
        return Image.fromarray(m).convert('L')

    try:
        font = ImageFont.truetype('arial.ttf', px(4))
        small = ImageFont.truetype('arial.ttf', px(2.8))
    except OSError:
        font = small = ImageFont.load_default()

    def label(mid, size_mm, quiet_mm, title, lines, arrow):
        """One label: quiet zone + marker; arrow=True adds a nose strip above (outside the quiet zone)."""
        side = size_mm + 2 * quiet_mm
        strip = 12.0 if arrow else 0.0
        text_h = 4.0 * (len(lines) + 1) + 2
        W, H = px(side), px(side + strip + text_h)
        im = Image.new('L', (W, H), 255)
        d = ImageDraw.Draw(im)
        y0 = px(strip)
        im.paste(marker_img(mid, size_mm), (px(quiet_mm), y0 + px(quiet_mm)))
        d.rectangle([0, y0, W - 1, y0 + px(side) - 1], outline=160, width=max(1, px(0.2)))   # cut line
        c = px(side / 2)
        for t in (0, px(side) - px(3)):            # centre ticks on all four edges (align with the robot's centre lines)
            d.line([(c, y0 + t), (c, y0 + t + px(3))], fill=0, width=max(1, px(0.3)))
            d.line([(t, y0 + c), (t + px(3), y0 + c)], fill=0, width=max(1, px(0.3)))
        if arrow:                                  # nose arrow in the strip (outside the white quiet zone)
            d.polygon([(c, px(1)), (c - px(5), px(8)), (c + px(5), px(8))], fill=0)
            d.rectangle([c - px(1.5), px(8), c + px(1.5), px(11)], fill=0)
            d.text((c + px(8), px(3)), 'NOSE', fill=0, font=font)
        ty = y0 + px(side) + px(1)
        d.text((px(1), ty), title, fill=0, font=font)
        for k, s in enumerate(lines):
            d.text((px(1), ty + px(4.5) * (k + 1)), s, fill=0, font=small)
        return im

    def sheet(labels, name):
        W, H = px(A4_MM[0]), px(A4_MM[1])
        page = Image.new('L', (W, H), 255)
        d = ImageDraw.Draw(page)
        x, y, rowh = px(12), px(12), 0
        for im in labels:
            if x + im.width > W - px(12):
                x, y, rowh = px(12), y + rowh + px(8), 0
            page.paste(im, (x, y))
            x += im.width + px(8)
            rowh = max(rowh, im.height)
        yb = H - px(20)                            # 50 mm scale bar
        d.rectangle([px(12), yb, px(62), yb + px(3)], fill=0)
        d.text((px(66), yb - px(1)), '50 mm: print at 100 %% (actual size), check with a ruler. %s %d dpi' % (cam['dictionary'], dpi),
               fill=0, font=small)
        page.save(os.path.join(a.out, name), 'PDF', resolution=dpi)

    q = cam['quietZone']
    robots = [label(i, cam['markerSize'], q, 'ROBOT %d (marker id %d)' % (i, i),
                    ['marker %.0f mm, label %.0f mm' % (1000 * cam['markerSize'], 1000 * (cam['markerSize'] + 2 * q)),
                     'centre on the robot top (%.0f mm ahead of the axle)' % (1000 * cam['markerOffset']),
                     'arrow to the NOSE, %.0f mm above the floor' % (1000 * cam['markerHeight'])], True)
              for i in cam['robotIds']]
    corners = [label(i, cam['cornerMarkerSize'], q, 'CORNER id %d' % i,
                     ['centre at x = %.3f m, y = %.3f m (arena)' % tuple(xy),
                      'plane %.0f mm above the floor' % (1000 * cam['cornerHeight'])], False)
               for i, xy in zip(cam['cornerIds'], cam['cornerXY'])]
    sheet(robots, 'robots_A4.pdf')
    sheet(corners, 'corners_A4.pdf')
    for i in list(cam['robotIds']) + list(cam['cornerIds']):
        size = cam['markerSize'] if i in cam['robotIds'] else cam['cornerMarkerSize']
        im = Image.new('L', (px(size + 2 * q),) * 2, 255)
        im.paste(marker_img(i, size), (px(q), px(q)))
        im.save(os.path.join(a.out, 'marker_%d.png' % i), dpi=(dpi, dpi))
    with open(os.path.join(a.out, 'placement.txt'), 'w') as f:
        f.write('ZADP ArUco markers (%s), generated from link_config.json\n\n' % cam['dictionary'])
        f.write('Robots: marker id = robot number, %.0f mm black square, %.0f mm white margin.\n' % (1000 * cam['markerSize'], 1000 * q))
        f.write('  Flat on the top, marker centre on the centre of the 100 x 84 mm top (%.0f mm ahead of the axle),\n' % (1000 * cam['markerOffset']))
        f.write('  printed arrow (UP edge of the marker) toward the nose, marker plane %.0f mm above the floor.\n\n' % (1000 * cam['markerHeight']))
        f.write('Corners: centre of each marker at (arena metres, x right, y up, origin = lower-left site corner):\n')
        for i, xy in zip(cam['cornerIds'], cam['cornerXY']):
            f.write('  id %d: x = %.3f, y = %.3f\n' % (i, xy[0], xy[1]))
        f.write('  on blocks so the marker plane is %.0f mm above the floor (= the robot marker plane); orientation free.\n' % (1000 * cam['cornerHeight']))
    print('make_markers: wrote robots_A4.pdf, corners_A4.pdf, marker_*.png, placement.txt in %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
