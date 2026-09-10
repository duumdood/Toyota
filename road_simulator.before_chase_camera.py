"""IDM Road Lab: mouse-driven cut-in demo. Run with Python + ursina.

Drag the coral car (or DRAG CAR button) into the cyan lane. Longitudinal
mouse movement sets lead speed; lateral movement controls lane overlap.
R resets at any time, N toggles lighting, arrows adjust desired speed.
Python owns the mock physics until telemetry from STM32 replaces it.
"""
import math
import random


def clamp(x, low, high):
    return max(low, min(high, x))


def idm_acceleration(v, v0, s=None, dv=0.0, T=1.0, s0=4.0):
    if v0 <= 0:
        return -3.0 if v > 0 else 0.0
    free = 1.0 - (v / max(v0, 0.1)) ** 4
    desired = s0 + max(0.0, v * T + v * dv / (2 * math.sqrt(3 * 3.5)))
    return clamp(3 * (free - (0 if s is None else (desired / max(s, 2)) ** 2)), -7, 3)


class Simulation:
    def __init__(self):
        self.target = 50 / 3.6
        self.night = False
        self.reset()
        self.speed = self.target

    def reset(self):
        self.speed = 0.0
        self.acceleration = 0.0
        self.lead_speed = 30 / 3.6
        self.gap = 45.0
        self.x = -7.0
        self.present = False
        self.done = False
        self.did_brake = False
        self.settled = 0.0
        self.reason = ''

    @property
    def overlap(self):
        # Both cars are 1.8 m wide. No braking for an adjacent-lane car.
        return clamp((1.8 - abs(self.x)) / 1.8, 0, 1) if self.present else 0

    def step(self, dt, dragging=False):
        if self.done:
            return 0.0
        old = self.speed
        free = idm_acceleration(old, self.target)
        following = idm_acceleration(old, self.target, self.gap,
                                     old - self.lead_speed,
                                     1.8 if self.night else 1.0,
                                     7 if self.night else 4)
        self.acceleration = free + self.overlap * (following - free)
        self.speed = max(0, old + self.acceleration * dt)
        distance = (old + self.speed) * 0.5 * dt
        if self.present and not dragging:
            self.gap += self.lead_speed * dt - distance
        if self.overlap > 0.2 and self.acceleration < -0.35:
            self.did_brake = True
        if self.overlap > 0.5 and self.gap <= 0:
            self.done, self.reason = True, 'CONTACT / insufficient stopping distance'
        elif not dragging and self.overlap > 0.5:
            stable = self.did_brake and abs(self.speed - self.lead_speed) < 0.7 and abs(self.acceleration) < 0.4
            self.settled = self.settled + dt if stable else 0
            if self.did_brake and self.speed < 0.3:
                self.speed = 0.0
                self.done, self.reason = True, 'Vehicle stopped safely'
            elif self.settled > 1.2:
                self.done, self.reason = True, 'Deceleration complete / following speed reached'
        else:
            self.settled = 0
        return distance


def main():
    from ursina import (Ursina, Entity, Text, Button, DirectionalLight,
                        AmbientLight, Vec3, color, camera, window, scene,
                        mouse, held_keys, time)
    from ursina.shaders import basic_lighting_shader
    from panda3d.core import Point2, Point3
    from direct.showbase import ShowBaseGlobal

    app = Ursina(title='IDM / Road Lab', borderless=False, size=(1440, 900))
    window.size = (1440, 900)
    window.fps_counter.enabled = False
    window.exit_button.visible = False
    sim = Simulation()
    rng = random.Random(17)
    navy = color.rgb32(16, 26, 40)
    cyan = color.rgb32(67, 218, 224)
    coral = color.rgb32(255, 113, 89)
    muted = color.rgb32(155, 176, 192)

    def block(parent=None, **kw):
        return Entity(parent=parent or scene, model='cube', shader=basic_lighting_shader, **kw)

    ground = block(scale=(250, .3, 420), position=(0, -.3, 130), color=color.rgb32(115, 157, 133))
    block(scale=(13, .18, 380), position=(0, -.09, 130), color=color.rgb32(52, 66, 79))
    for x in (-6.6, 6.6):
        block(scale=(.35, .2, 380), position=(x, .04, 130), color=color.rgb32(195, 203, 189))
    for x in (-5.8, 1.9, 5.8):
        block(scale=(.09, .015, 380), position=(x, .015, 130), color=color.rgb32(205, 215, 203))
    # Cyan lane guides make the interaction corridor unambiguous.
    for x in (-1.9, 1.9):
        Entity(model='cube', scale=(.065, .025, 380), position=(x, .025, 130), color=cyan)
    moving = []
    for z in range(-50, 260, 7):
        moving.append(block(position=(-2.3, .025, z), scale=(.12, .02, 3), color=color.rgb32(233, 224, 183)))
    for i in range(68):
        z = rng.uniform(-50, 260)
        root = Entity(position=(rng.choice((-1, 1)) * rng.uniform(10, 42), 0, z))
        block(root, position=(0, 1, 0), scale=(.45, 2, .45), color=color.rgb32(103, 92, 74))
        for y, size in ((2.1, 2.5), (3.2, 1.8)):
            block(root, position=(0, y, 0), scale=(size, 1.8, size), rotation_y=rng.uniform(0, 90),
                  color=color.rgb32(59, rng.randint(113, 145), 108))
        moving.append(root)

    def car(tint):
        root = Entity()
        block(root, position=(0, .6, 0), scale=(1.8, .65, 3.6), color=tint)
        block(root, position=(0, 1.13, -.2), scale=(1.48, .6, 1.8), color=tint)
        block(root, position=(0, 1.18, .73), scale=(1.36, .4, .035), color=navy)
        block(root, position=(0, 1.18, -1.13), scale=(1.36, .4, .035), color=navy)
        for x in (-.755, .755):
            block(root, position=(x, 1.17, -.2), scale=(.025, .38, 1.55), color=navy)
        for x in (-.95, .95):
            for z in (-1.1, 1.1):
                block(root, position=(x, .35, z), scale=(.24, .62, .72), color=color.rgb32(25, 31, 40))
        for x in (-.57, .57):
            Entity(parent=root, model='cube', position=(x, .65, 1.81), scale=(.43, .14, .025), color=color.rgb32(254, 242, 198))
        lamps = [Entity(parent=root, model='cube', position=(x, .65, -1.81), scale=(.48, .16, .025), color=coral) for x in (-.57, .57)]
        return root, lamps

    ego, lamps = car(cyan)
    lead, _ = car(coral)
    lead.position = (-7, 0, 35)
    hitbox = Entity(parent=lead, model='cube', position=(0, .7, 0), scale=(2.5, 2, 4.5), collider='box', visible=False)
    sun = DirectionalLight(rotation=(50, -30, 0), color=color.rgb32(255, 240, 213))
    ambient = AmbientLight(color=color.rgb32(155, 170, 190))
    camera.orthographic = True
    camera.fov = 60
    camera.position = (49, 64, -28)
    camera.look_at(Vec3(0, 0, 21))
    window.color = color.rgb32(174, 201, 202)

    def panel(pos, size, tint=navy):
        return Entity(parent=camera.ui, model='quad', position=pos, z=.1, scale=size, color=tint)

    def label(text, pos, scale=1, tint=color.white, **kw):
        return Text(parent=camera.ui, text=text, position=pos, scale=scale, color=tint, **kw)

    panel((0, .425), (1.58, .135))
    label('IDM / ROAD LAB', (-.755, .468), 1.3, cyan)
    label('STM32 F411  /  SENSOR MOCK', (-.755, .424), .78, muted)
    speed = label('', (-.13, .465), 2.1)
    target = label('', (.30, .465), .95)
    mode = label('', (.30, .416), .83, cyan)
    panel((-.58, .22), (.38, .23))
    label('LIVE TELEMETRY', (-.75, .317), .85, cyan)
    telemetry = label('', (-.75, .277), .86)
    panel((0, -.405), (1.58, .17))
    hint = label('Drag the coral car into the CYAN lane. Fast or slow: you control the cut-in.', (-.75, -.344), .85, muted)

    def button(text, x, width, action=None, tint=navy):
        return Button(text=text, position=(x, -.414), scale=(width, .055), color=tint,
                      highlight_color=color.rgb32(62, 95, 111), pressed_color=cyan, on_click=action or (lambda: None))

    drag = {'active': False, 'offset': None, 'last_z': None, 'filtered': 0.0}
    banner = panel((0, .035), (1.05, .25), color.rgb32(111, 34, 37))
    headline = label('BREAK!!', (0, .095), 4, color.rgb32(255, 230, 207), origin=(0, 0))
    reason = label('', (0, -.005), .95, origin=(0, 0))
    label('R / RESET anytime  |  N / light  |  UP / DOWN target speed', (-.75, -.467), .73, muted)

    def reset():
        sim.reset()
        drag.update(active=False, offset=None, last_z=None, filtered=0.0)
        lead.position = (-7, 0, 35)

    def toggle():
        sim.night = not sim.night
        sim.target = min(sim.target, (100 if sim.night else 120) / 3.6)
        window.color = color.rgb32(22, 35, 57) if sim.night else color.rgb32(174, 201, 202)
        ground.color = color.rgb32(38, 68, 64) if sim.night else color.rgb32(115, 157, 133)
        ambient.color = color.rgb32(65, 84, 119) if sim.night else color.rgb32(155, 170, 190)
        sun.color = color.rgb32(110, 141, 187) if sim.night else color.rgb32(255, 240, 213)

    def sudden():
        if sim.done:
            reset()
        sim.present, sim.x, sim.gap, sim.lead_speed = True, 0, 22, 0
        sim.did_brake, sim.settled = False, 0
        drag['active'] = False

    down = button('- SPEED', -.675, .15, tint=color.rgb32(39, 61, 78))
    up = button('+ SPEED', -.505, .15, tint=color.rgb32(39, 61, 78))
    drag_button = button('DRAG CAR', -.295, .22, tint=color.rgb32(165, 73, 58))
    button('SUDDEN', -.075, .18, sudden, color.rgb32(165, 73, 58))
    button('DAY / NIGHT', .17, .25, toggle, color.rgb32(39, 61, 78))
    button('RESET  [R]', .49, .30, reset, color.rgb32(35, 111, 110))

    def pointer_on_road():
        # Unproject screen coordinates onto y=0 in Ursina's world space.
        base = ShowBaseGlobal.base
        near, far = Point3(), Point3()
        base.cam.node().get_lens().extrude(Point2(mouse.x * 2 / camera.aspect_ratio, mouse.y * 2), near, far)
        a = scene.getRelativePoint(base.cam, near)
        b = scene.getRelativePoint(base.cam, far)
        if abs(b.y - a.y) < 1e-6:
            return None
        t = -a.y / (b.y - a.y)
        return a + (b - a) * t

    def input_event(key):
        if key == 'r':
            reset()
        elif key == 'n':
            toggle()
        elif key == 'left mouse down' and (mouse.hovered_entity == hitbox or drag_button.hovered):
            if sim.done:
                return
            p = pointer_on_road()
            if p is not None:
                drag.update(active=True, offset=lead.position - p if not drag_button.hovered else Vec3(0, 0, 0),
                            last_z=None, filtered=sim.lead_speed)
                sim.present = True
                sim.did_brake, sim.settled = False, 0
        elif key == 'left mouse up':
            drag['active'] = False
            drag['last_z'] = None

    def update_frame():
        dt = min(max(time.dt, 0), .1)
        if drag['active'] and not mouse.left:
            drag['active'] = False
        if not sim.done:
            change = bool(held_keys['up arrow'] or (up.hovered and mouse.left)) - bool(held_keys['down arrow'] or (down.hovered and mouse.left))
            sim.target = clamp(sim.target + change * 4 * dt, 0, (100 if sim.night else 120) / 3.6)
            if drag['active']:
                p = pointer_on_road()
                if p is not None:
                    p += drag['offset']
                    sim.x = clamp(p.x, -12, 12)
                    new_gap = clamp(p.z - 3.6, 0, 110)
                    if drag['last_z'] is not None and dt > 0:
                        measured = clamp(sim.speed + (new_gap - drag['last_z']) / dt, 0, 160 / 3.6)
                        drag['filtered'] += (measured - drag['filtered']) * (1 - math.exp(-dt / .075))
                        sim.lead_speed = drag['filtered']
                    drag['last_z'] = new_gap
                    sim.gap = new_gap
            remaining, travel = dt, 0
            while remaining > 1e-8:
                tick = min(remaining, 1 / 120)
                travel += sim.step(tick, drag['active'])
                remaining -= tick
            for obj in moving:
                obj.z = (obj.z - travel + 50) % 310 - 50
            if sim.present:
                lead.position = (sim.x, 0, sim.gap + 3.6)
            if sim.present and (sim.gap > 115 or sim.gap < -10) and not drag['active']:
                sim.present = False
                lead.position = (-7, 0, 35)
        if sim.present:
            lead.position = (sim.x, 0, sim.gap + 3.6)
        banner.enabled = headline.enabled = reason.enabled = sim.done
        reason.text = sim.reason + '\nPress RESET to start another sequence'
        speed.text = f'{sim.speed * 3.6:03.0f} km/h'
        target.text = f'TARGET  {sim.target * 3.6:.0f} km/h'
        mode.text = ('NIGHT  /  LIMIT 100' if sim.night else 'DAY  /  LIMIT 120')
        status = 'COMPLETE' if sim.done else 'BRAKING' if sim.acceleration < -.35 else 'CRUISING'
        gap_text = f'{sim.gap:.1f} m' if sim.present else '--'
        telemetry.text = f'{status}\nAcceleration  {sim.acceleration:+.2f} m/s2\nGap  {gap_text}\nLead  {sim.lead_speed * 3.6:.0f} km/h\nLane overlap  {sim.overlap * 100:.0f}%'
        for lamp in lamps:
            lamp.color = color.rgb32(255, 35, 52) if sim.acceleration < -.35 or sim.done else color.rgb32(110, 43, 49)
        hint.text = ('DRAGGING / forward = faster, backward = slower; release to keep speed.' if drag['active'] else
                     'Drag coral car or DRAG CAR into cyan lane. SUDDEN inserts a stopped obstacle at 22 m.')

    Entity(input=input_event, update=update_frame)
    app.run()


if __name__ == '__main__':
    main()
