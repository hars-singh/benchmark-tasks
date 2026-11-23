"""Check unloaded separation of cup and outlet colliders at the authored reset."""

from pathlib import Path
import math
import sys

from yam_arena.task_file import load


def shell_sections(scene):
    sections = set()
    for part in scene.BODY_PARTS[scene.CUPS[0]]:
        x = part.offset[0]
        if part.kind == 'tapered_panel':
            a, b, thickness, length, span = part.size
            sections.add((x-length/2, x+length/2, a, b, thickness, math.cos(span/2)))
        else:
            assert part.kind == 'cylinder' and part.axis == 'X'
            radius, length = part.size
            sections.add((x-length/2, x+length/2, 0., 0., radius, 1.))
    return sorted(sections)


def radii(section, x):
    start, end, inner_a, inner_b, thickness, cosine = section
    inner = inner_a + (inner_b-inner_a)*(x-start)/(end-start)
    return inner*cosine, inner+thickness


def check(scene):
    sections = shell_sections(scene)
    setup = {obj.name: obj.pos for obj in scene.OBJECTS}
    clearances = []
    for front, back in zip(scene.CUPS, scene.CUPS[1:]):
        pitch = setup[back][0]-setup[front][0]
        for a in sections:
            for b in sections:
                shared_start, shared_end = max(a[0], b[0]+pitch), min(a[1], b[1]+pitch)
                if shared_start >= shared_end:
                    continue
                for x in (shared_start, shared_end):
                    clearances.append(radii(a,x)[0]-radii(b,x-pitch)[1])
    assert clearances and min(clearances) > 0., f'Nested shell clearance {min(clearances):.6f} m'

    # Keep the chamfered cup wall behind the unloaded leaf edges at reset.
    first_x = setup[scene.CUPS[0]][0]
    gate_half_depth = scene.BODY_PARTS[scene.LEAVES[0]][0].size[0]/2
    gate_start = scene.LEAF_X-gate_half_depth-first_x
    gate_end = scene.LEAF_X+gate_half_depth-first_x
    gate_radii = [radii(section,x)[1] for section in sections
                  for x in (max(gate_start,section[0]), min(gate_end,section[1]))
                  if max(gate_start,section[0]) <= min(gate_end,section[1])]
    free = scene.LEAF_INNER-max(gate_radii)
    assert free > 0., f'Unloaded outlet clearance {free:.6f} m'
    print(f'Minimum nesting clearance {1000*min(clearances):.3f} mm')
    print(f'Minimum outlet clearance {1000*free:.3f} mm')


if __name__ == '__main__':
    path = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parents[1]/'scene.py'
    check(load(path.resolve()).module)
