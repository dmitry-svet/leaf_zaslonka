"""Plate with an oval-to-round 45 degree duct for a parking heater hose.

Print orientation: plate flat, tube pointing up. The outside rib (bottom)
lifts the plate off the bed by rib_out_h, so that side needs supports.
"""
import math

import cadquery as cq

# ============================================================
# PARAMETERS - Edit these to customize the model
# ============================================================
# Plate outline (X = length, Y = width), derived from the openings + margins.
edge_margin = 20.0      # mm - openings to the long edges
end_margin = 20.0       # mm - openings to the short (left/right) edges
plate_t = 3.0           # mm - plate thickness (2 or 3)
corner_r_near = 10.0    # mm - corners at the oval end
corner_r_far = 10.0     # mm - corners at the far end and on the taper

# Oval (stadium) where the tube meets the plate, outer size of the tube
oval_h = 40.0           # mm - across plate width (2 x R20)
oval_straight = 28.6    # mm - straight part between the two R20 arcs

# Tube
tube_od = 59.5          # mm - round outlet outer diameter (hose slides on)
tube_wall = 3.0         # mm - grows inward: outer size stays
rise_len = 0.0          # mm - straight oval part above the plate
bend_angle = 45.0       # deg - tilt toward +X (far end of the plate)
bend_r = 40.0           # mm - centreline bend radius (min ~36: half oval length)
outlet_len = 20.0       # mm - straight round part for the hose
bead_d = 60.5           # mm - bead at the very end of the outlet, keeps the hose clamp from sliding off
bead_w = 2.0            # mm - bead length along the tube

# Perimeter ribs
rib_in_w, rib_in_h = 3.0, 2.0     # mm - tube side
rib_out_w, rib_out_h = 3.0, 4.0   # mm - opposite side (mounts to the car body)

stiff_gap = 14.0        # mm - gap between the two cross ribs (tube side, between the openings)

# Mounting holes
hole_d = 6.0            # mm
corner_hole_inset = 10.0    # mm - two holes in the plate corners at the oval end, from both edges
ear_r = 7.0             # mm - ear radius (two ears beside the shutter, at the far end)
ear_out = 6.0           # mm - ear hole centre outside the long edge
ear_from_end = 8.0      # mm - ear hole centre from the far end
head_d = 14.0           # mm - free diameter for the screw head inside the tube-side ring
ear_blend_r = 3.0       # mm - concave blend between ear and plate

# Window + sliding shutter (tube side). Estimated from the photo: CHECK.
win_gap = 44.0          # mm - oval end to window
win_len = 90.0          # mm - window along X
win_w = 58.5            # mm - window along Y (one long side in line with the oval)
win_r = 10.0            # mm - window corner radius
win_overlap = 10.0      # mm - shutter overlap around the window
shutter_t = 3.0         # mm - shutter thickness
shutter_r = 10.0        # mm - shutter corner radius (plan view)
rail_w = 9.0            # mm - rail width at the plate
fit_clearance = 0.3     # mm - shutter to rails, per side
grip_w, grip_h = 3.0, 4.0   # mm - pull ridge on the shutter
loop_t = 3.0            # mm - loop (eye) thickness; stands along the travel, at the edge opposite the pull ridge
loop_hole_d = 4.0       # mm - hole for a wire hook
loop_wall = 3.0         # mm - material around the hole
loop_from_edge = 5.0    # mm - gap from the shutter edge to the loop

# Clamp bar: goes inside the heater opening, the middle hole screw pulls it to the plate
clamp_t = 3.0           # mm - bar thickness (length = plate width at the middle hole,
                        #      width = stiff_gap + both ribs, edge ribs = rib_in)

# Material
shrinkage = 0.0         # print shrinkage (e.g. 0.02): exported files are scaled up by 1/(1-shrinkage). Set right before printing

# Internal
n_pts = 48              # points per tube section
n_bend = 10             # sections along the bend
eps = 0.01

# ============================================================
# MODEL
# ============================================================
plate_w = oval_h + 2 * edge_margin
oval_len = oval_straight + oval_h
oval_cx = end_margin + oval_len / 2
oval_cy = plate_w / 2
plate_len = end_margin + oval_len + win_gap + win_len + end_margin
widen = win_w - oval_h                  # far part is wider on the +Y side
win_cy = edge_margin + win_w / 2
z_plate = rib_out_h                 # plate bottom
z_top = z_plate + plate_t           # plate top


def base_outline():
    x1 = end_margin + oval_len                 # slope runs from oval end to window start
    x2 = x1 + win_gap
    pts = [(0, 0), (plate_len, 0), (plate_len, plate_w + widen),
           (x2, plate_w + widen), (x1, plate_w), (0, plate_w)]
    radii = [corner_r_near, corner_r_far, corner_r_far,
             corner_r_far, corner_r_far, corner_r_near]
    wp = cq.Workplane("XY")
    n = len(pts)
    segs = []
    for i in range(n):
        p, a, b = cq.Vector(*pts[i]), cq.Vector(*pts[i - 1]), cq.Vector(*pts[(i + 1) % n])
        da, db = (a - p).normalized(), (b - p).normalized()
        half = math.acos(max(-1, min(1, da.dot(db)))) / 2
        d = radii[i] / math.tan(half)
        mid = p + (da + db).normalized() * (radii[i] / math.sin(half) - radii[i])
        segs.append((p + da * d, mid, p + db * d))
    wp = wp.moveTo(segs[0][2].x, segs[0][2].y)
    for s, m, e in segs[1:] + segs[:1]:
        wp = wp.lineTo(s.x, s.y).threePointArc((m.x, m.y), (e.x, e.y))
    return wp.close()


ear_pts = [(plate_len - ear_from_end, -ear_out), (plate_len - ear_from_end, plate_w + widen + ear_out)]
_body = base_outline().extrude(1).union(cq.Workplane("XY").pushPoints(ear_pts).circle(ear_r).extrude(1))
# offset out then in: rounds the concave ear-to-plate junctions, keeps the rest
outline_wire = (cq.Workplane("XY").add(_body.faces("<Z").wires().vals()[0]).toPending()
                .offset2D(ear_blend_r).offset2D(-ear_blend_r).val())


def outline():
    """Plate outline including the ears."""
    return cq.Workplane("XY").add(outline_wire).toPending()


def rib(width, height, z):
    outer = outline().extrude(height)
    inner = outline().offset2D(-width).extrude(height)
    return outer.cut(inner).translate((0, 0, z))


def section(origin, theta, h, s):
    """Stadium (h tall, s straight) on a plane tilted theta toward +X."""
    r, a = h / 2, s / 2
    perim = 2 * s + 2 * math.pi * r
    xdir = cq.Vector(math.cos(theta), 0, -math.sin(theta))
    ydir = cq.Vector(0, 1, 0)
    pts = []
    for k in range(n_pts):
        d = perim * k / n_pts
        if d < a:                                   # top straight, first half
            u, v = d, r
        elif d < a + math.pi * r:                   # +u arc
            ang = math.pi / 2 - (d - a) / r
            u, v = a + r * math.cos(ang), r * math.sin(ang)
        elif d < 3 * a + math.pi * r:               # bottom straight
            u, v = a - (d - a - math.pi * r), -r
        elif d < 3 * a + 2 * math.pi * r:           # -u arc
            ang = -math.pi / 2 - (d - 3 * a - math.pi * r) / r
            u, v = -a + r * math.cos(ang), r * math.sin(ang)
        else:                                       # top straight, second half
            u, v = -a + (d - 3 * a - 2 * math.pi * r), r
        pts.append(origin + xdir * u + ydir * v)
    return cq.Wire.assembleEdges([cq.Edge.makeSpline(pts, periodic=True)])


def tube(shrink, extend):
    """Loft along rise -> bend -> outlet. shrink: wall offset, extend: overshoot."""
    base = cq.Vector(oval_cx, oval_cy, z_plate)
    rise_top = z_top + rise_len
    ang = math.radians(bend_angle)
    step = bend_r * ang / n_bend                    # even spacing: no loft ripple
    n_rise = max(1, round((rise_top - base.z + extend) / step))
    stations = [(cq.Vector(oval_cx, oval_cy, rise_top - (rise_top - base.z + extend) * (1 - i / n_rise)), 0, 0.0)
                for i in range(n_rise + 1)]
    for i in range(1, n_bend + 1):
        t = i / n_bend
        th = ang * t
        pos = cq.Vector(oval_cx + bend_r * (1 - math.cos(th)), oval_cy,
                        rise_top + bend_r * math.sin(th))
        stations.append((pos, th, t))
    end_dir = cq.Vector(math.sin(ang), 0, math.cos(ang))
    bend_end = stations[-1][0]
    n_out = max(1, round((outlet_len + extend) / step))
    for i in range(1, n_out + 1):
        stations.append((bend_end + end_dir * ((outlet_len + extend) * i / n_out), ang, 1.0))
    wires = []
    for pos, th, t in stations:
        blend = t * t * (3 - 2 * t)                 # smooth oval -> round
        h = oval_h + (tube_od - oval_h) * blend - 2 * shrink
        s = oval_straight * (1 - blend)
        wires.append(section(pos, th, h, s))
    return cq.Workplane(obj=cq.Solid.makeLoft(wires))


plate = outline().extrude(plate_t).translate((0, 0, z_plate))
_ang = math.radians(bend_angle)
_dir = cq.Vector(math.sin(_ang), 0, math.cos(_ang))
_end = cq.Vector(oval_cx + bend_r * (1 - math.cos(_ang)), oval_cy,
                 z_top + rise_len + bend_r * math.sin(_ang)) + _dir * outlet_len
# bead overshoots the outlet end, then everything is trimmed flush at the end plane
bead = cq.Workplane(obj=cq.Solid.makeCylinder(bead_d / 2, bead_w + 0.5, _end - _dir * bead_w, _dir))
end_trim = cq.Workplane(obj=cq.Solid.makeCylinder(bead_d, 5, _end, _dir))
# Shutter zone: 45 degree dovetail rails (print without supports), open at the
# far end so the shutter slides out there. No tube-side rib in this zone.
win_x0 = end_margin + oval_len + win_gap
rail_x0 = win_x0 - win_overlap
shutter_w = win_w + 2 * win_overlap             # bottom (wide) face
shutter_len = win_len + 2 * win_overlap
rail_h = shutter_t + fit_clearance
rail_in = shutter_w / 2 + fit_clearance         # rail inner edge at the plate, from win_cy

open_end = (cq.Workplane("XY").box(plate_len - rail_x0 + eps, plate_w + widen + 100, rib_in_h + 2, centered=False)
            .translate((rail_x0, -50, z_top + eps)))
window = (cq.Workplane("XY").box(win_len, win_w, plate_t + 2, centered=(False, True, False))
          .edges("|Z").fillet(win_r).translate((win_x0, win_cy, z_plate - 1)))


def rail(sign):
    prof = [(sign * rail_in, 0), (sign * (rail_in + rail_w), 0),
            (sign * (rail_in + rail_w), rail_h), (sign * (rail_in - rail_h), rail_h)]
    return (cq.Workplane("YZ").polyline(prof).close().extrude(plate_len - rail_x0)
            .translate((rail_x0, win_cy, z_top))
            .intersect(outline().extrude(rail_h).translate((0, 0, z_top))))   # trim at rounded corners


stop = (cq.Workplane("XY").box(rib_in_w, 2 * rail_in, rail_h, centered=(False, True, False))
        .translate((rail_x0 - rib_in_w, win_cy, z_top)))

# Two cross ribs (same profile as the tube-side rib); the one nearer the window
# sits under the shutter stop. A mounting hole goes between them.
stiff_cx = rail_x0 - rib_in_w - stiff_gap / 2
stiff = (cq.Workplane("XY").rect(stiff_gap + 2 * rib_in_w, 2 * (plate_w + widen)).extrude(rib_in_h)
         .cut(cq.Workplane("XY").rect(stiff_gap, 2 * (plate_w + widen)).extrude(rib_in_h))
         .translate((stiff_cx, 0, 0)).intersect(outline().extrude(rib_in_h)).translate((0, 0, z_top)))

mid_w = plate_w + widen * (stiff_cx - end_margin - oval_len) / win_gap    # plate width on the slope
hole_pts = ear_pts + [(corner_hole_inset, corner_hole_inset), (corner_hole_inset, plate_w - corner_hole_inset),
                      (stiff_cx, mid_w / 2)]
holes = cq.Workplane("XY").pushPoints(hole_pts).circle(hole_d / 2).extrude(z_top + rail_h + 2).translate((0, 0, -1))

# Rings around the in-plate holes (corners at the oval end + the middle one):
# body side like the ears (plate rests on them when screwed down), tube side around the screw head
ring_pts = hole_pts[len(ear_pts):]
rings = (
    cq.Workplane("XY").pushPoints(ring_pts).circle(ear_r).circle(ear_r - rib_out_w).extrude(rib_out_h)
    .intersect(outline().extrude(rib_out_h))
    .union(cq.Workplane("XY").pushPoints(ring_pts).circle(head_d / 2 + rib_in_w).circle(head_d / 2).extrude(rib_in_h)
           .intersect(outline().extrude(rib_in_h)).translate((0, 0, z_top)))
)

half = shutter_w / 2
shutter = (
    cq.Workplane("YZ")
    .polyline([(-half, 0), (half, 0), (half - shutter_t, shutter_t), (-half + shutter_t, shutter_t)])
    .close().extrude(shutter_len)
    .union(cq.Workplane("XY").box(grip_w, shutter_w - 2 * shutter_t - 2 * rail_h, shutter_t + grip_h,
                                  centered=(False, True, False)).translate((shutter_len - grip_w, 0, 0)))
)
# Eye for a wire hook: fin along the travel, hole across. Sits near the edge
# opposite the pull ridge (mid-shutter it would break out of the thin plate)
loop_r = loop_hole_d / 2 + loop_wall
loop_x = loop_from_edge + loop_r
loop = (
    cq.Workplane("XZ").center(loop_x, shutter_t)
    .moveTo(-loop_r, -eps).lineTo(-loop_r, loop_r).threePointArc((0, 2 * loop_r), (loop_r, loop_r))
    .lineTo(loop_r, -eps).close().extrude(loop_t / 2, both=True)
    .cut(cq.Workplane("XZ").center(loop_x, shutter_t + loop_r).circle(loop_hole_d / 2)
         .extrude(loop_t, both=True))
)
shutter = shutter.intersect(
    cq.Workplane("XY").box(shutter_len, shutter_w, shutter_t + grip_h + 1, centered=(False, True, False))
    .edges("|Z").fillet(shutter_r)
).union(loop)
result = (
    plate
    .union(rib(rib_in_w, rib_in_h, z_top).cut(open_end))
    .union(rib(rib_out_w, rib_out_h, 0))
    .union(tube(0, 0)).union(bead)
    .union(rail(1)).union(rail(-1)).union(stop).union(stiff).union(rings)
    .cut(window)
    .cut(holes)
    .cut(tube(tube_wall, 1.0))
    .cut(end_trim)
)

# Clamp bar: flat strip with edge ribs along the long sides and a ring around the hole
clamp_w = stiff_gap + 2 * rib_in_w
clamp_box = cq.Workplane("XY").rect(clamp_w, mid_w).extrude(clamp_t + rib_in_h)
clamp = (
    cq.Workplane("XY").rect(clamp_w, mid_w).extrude(clamp_t)
    .union(clamp_box.cut(cq.Workplane("XY").rect(stiff_gap, mid_w + 2).extrude(clamp_t + rib_in_h)))
    .union(cq.Workplane("XY").circle(head_d / 2 + rib_in_w).circle(head_d / 2)
           .extrude(clamp_t + rib_in_h).intersect(clamp_box))
    .cut(cq.Workplane("XY").circle(hole_d / 2).extrude(clamp_t + rib_in_h))
)

# ============================================================
# EXPORT
# ============================================================
# STL: tessellate at nominal size, then scale the mesh (scaling the B-rep first
# gave a non-watertight mesh). STEP is scaled as a solid.
import trimesh

scale = 1 / (1 - shrinkage)
for part, name in ((result, "zaslonka"), (shutter, "zaslonka_shutter"),
                   (clamp, "zaslonka_clamp")):
    cq.exporters.export(part, f"{name}.stl", tolerance=0.01, angularTolerance=0.1)
    mesh = trimesh.load_mesh(f"{name}.stl")
    mesh.apply_scale(scale)
    mesh.export(f"{name}.stl")
    cq.exporters.export(cq.Workplane(obj=part.val().scale(scale)), f"{name}.step")

from mating import check_mating
# Closed position; rails must hold the shutter across (Y) and against lifting (Z)
check_mating(result, shutter.translate((rail_x0 + fit_clearance, win_cy, z_top)),
             clearance=fit_clearance, axes=("y", "z"))
bb = result.val().BoundingBox()
print(f"Exported (scaled x{scale:.4f}): {bb.xlen * scale:.1f} x {bb.ylen * scale:.1f} x {bb.zlen * scale:.1f} mm")
