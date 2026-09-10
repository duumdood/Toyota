"""IDM Road Lab: mouse-driven cut-in demo. Run with Python + ursina.

Drag the orange car (or DRAG CAR button) into the ego car's centre lane.
Longitudinal mouse movement requests lead speed; lateral movement controls
lane overlap. Both cars' displayed speeds use the same integrated physics.
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
        if self.present:
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
                        mouse, held_keys, time, Mesh)
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

    ground = block(scale=(1400, .3, 1400), position=(0, -.3, 500), color=color.rgb32(142, 160, 126))
    block(scale=(13, .18, 1400), position=(0, -.09, 500), color=color.rgb32(52, 66, 79))
    for x in (-6.6, 6.6):
        block(scale=(.35, .2, 1400), position=(x, .04, 500), color=color.rgb32(195, 203, 189))
    for x in (-5.8, 5.8):
        block(scale=(.09, .015, 1400), position=(x, .015, 500), color=color.rgb32(205, 215, 203))
    # Real road markings; the ego car drives in the centre lane.
    moving = []
    for z in range(-50, 260, 7):
        for x in (-1.9, 1.9):
            moving.append(block(position=(x, .025, z), scale=(.12, .02, 3), color=color.rgb32(239, 236, 222)))
    for side in (-1, 1):
        block(position=(side*7.2, .75, 130), scale=(.15, .25, 380), color=color.rgb32(171, 185, 187))
        for z in range(-45, 260, 12):
            moving.append(block(position=(side*7.2, .35, z), scale=(.12, .7, .12), color=color.rgb32(112, 126, 133)))
        for z in range(-30, 260, 32):
            post = Entity(position=(side*8.4, 0, z))
            block(post, position=(0, 4, 0), scale=(.12, 8, .12), color=color.rgb32(73, 86, 93))
            block(post, position=(-side*.9, 8, 0), scale=(1.9, .12, .12), color=color.rgb32(73, 86, 93))
            block(post, position=(-side*1.8, 7.9, 0), scale=(.7, .12, .45), color=color.rgb32(255, 234, 180))
            moving.append(post)
    # Faceted distant terrain provides a horizon rather than an empty plane.
    for i in range(20):
        x = (i-10)*55
        vertices = [(-70,0,-25),(70,0,-25),(80,0,45),(-75,0,45),(0,rng.uniform(20,48),8)]
        hill = Mesh(vertices=vertices, triangles=[(0,1,4),(1,2,4),(2,3,4),(3,0,4)], mode='triangle')
        hill.generate_normals()
        Entity(model=hill, position=(x,0,rng.uniform(450,620)), color=color.rgb32(164,187,194), shader=basic_lighting_shader)
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
        Entity(parent=root, model='quad', rotation_x=90, position=(0,.01,0), scale=(2.15,3.9), color=color.rgba32(10,20,28,90), double_sided=True)
        def surface(vertices, triangles, tint):
            # Expand faces so normals remain flat and no unused vertex yields NaN.
            packed = [vertices[i] for face in triangles for i in face]
            mesh = Mesh(vertices=packed, triangles=list(range(len(packed))), mode='triangle')
            mesh.generate_normals()
            return Entity(parent=root, model=mesh, color=tint, shader=basic_lighting_shader, double_sided=True)
        # Chamfered body and raked cabin create a compact sports-sedan profile.
        ring = [(-.72,-1.8),(.72,-1.8),(.9,-1.55),(.9,1.45),(.68,1.8),(-.68,1.8),(-.9,1.45),(-.9,-1.55)]
        verts = [(x,y,z) for y in (.35,.78) for x,z in ring]
        faces = [(8,8+i,8+i+1) for i in range(1,7)]
        for i in range(8):
            j=(i+1)%8
            faces.extend([(i,j,j+8),(i,j+8,i+8)])
        surface(verts,faces,tint)
        cabin = [(-.77,.78,-1.1),(.77,.78,-1.1),(.77,.78,.95),(-.77,.78,.95),
                 (-.65,1.32,-.65),(.65,1.32,-.65),(.65,1.32,.35),(-.65,1.32,.35)]
        surface(cabin,[(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)],navy)
        surface(cabin,[(4,5,6),(4,6,7)],tint)
        for x in (-.72,.72):
            block(root, position=(x,1.02,-.23), scale=(.06,.49,.055), color=tint)
        for x in (-.95, .95):
            for z in (-1.1, 1.1):
                wheel_vertices = [(x+offset,.34+math.sin(i*math.tau/12)*.33,z+math.cos(i*math.tau/12)*.33) for offset in (-.10,.10) for i in range(12)]
                wheel_faces=[]
                for i in range(12):
                    j=(i+1)%12
                    wheel_faces.extend([(i,j,j+12),(i,j+12,i+12)])
                for start in (0,12):
                    wheel_faces.extend([(start,start+i,start+i+1) for i in range(1,11)])
                surface(wheel_vertices,wheel_faces,color.rgb32(21,26,32))
                block(root, position=(x+(.105 if x>0 else -.105), .34, z), scale=(.015,.28,.28), rotation_x=45, color=color.rgb32(170,180,185))
        block(root, position=(0,.38,-1.82), scale=(1.65,.14,.12), color=navy)
        block(root, position=(0,.57,-1.83), scale=(.45,.17,.025), color=color.rgb32(227,230,223))
        block(root, position=(0,.82,-1.55), scale=(1.55,.05,.22), color=tint)
        for x in (-.94,.94):
            block(root, position=(x,1,.55), scale=(.22,.14,.25), color=tint)
        for x in (-.57, .57):
            Entity(parent=root, model='cube', position=(x, .65, 1.81), scale=(.43, .14, .025), color=color.rgb32(254, 242, 198))
        lamps = [Entity(parent=root, model='cube', position=(x, .65, -1.81), scale=(.48, .16, .025), color=coral) for x in (-.57, .57)]
        return root, lamps

    ego, lamps = car(cyan)
    lead, _ = car(coral)
    lead.position = (-3.8, 0, 35)
    hitbox = Entity(parent=lead, model='cube', position=(0, .7, 0), scale=(2.5, 2, 4.5), collider='box', visible=False)
    sun = DirectionalLight(rotation=(50, -30, 0), color=color.rgb32(255, 240, 213))
    ambient = AmbientLight(color=color.rgb32(155, 170, 190))
    camera.orthographic = False
    camera.fov = 65
    camera.position = (1.7, 4.2, -10)
    camera.look_at(Vec3(0, .8, 8))
    window.color = color.rgb32(183, 208, 222)

    def panel(pos, size, tint=navy):
        return Entity(parent=camera.ui, model='quad', position=pos, z=.1, scale=size, color=tint)

    def label(text, pos, scale=1, tint=color.white, **kw):
        return Text(parent=camera.ui, text=text, position=pos, scale=scale, color=tint, **kw)

    brand = label('ROAD / LAB', (-.75,.455), .95, navy)
    speed_shadow = label('', (.002,.387), 4.2, navy, origin=(0,0), z=.01)
    speed = label('', (0,.39), 4.2, origin=(0,0), z=-.01)
    unit = label('K M / H', (0,.323), .7, navy, origin=(0,0))
    target = label('', (-.75,-.338), .9)
    mode = label('', (.74,.455), .8, navy, origin=(.5,0))
    telemetry = label('', (0,-.31), .9, origin=(0,0))
    lead_tag = label('', (0,0), .85, origin=(0,0), background=True)
    hint = label('', (0,-.47), .75, origin=(0,0))

    def button(text, x, width, action=None, tint=navy):
        return Button(text=text, position=(x, -.414), scale=(width, .044), color=tint,
                      highlight_color=color.rgb32(62, 95, 111), pressed_color=cyan, on_click=action or (lambda: None))

    drag = {'active': False, 'offset': None, 'last_z': None, 'filtered': 0.0}
    banner = panel((0, .025), (.88, .23), color.rgba32(16,26,40,225))
    headline = label('BREAK!!', (0, .095), 4, color.rgb32(255, 230, 207), origin=(0, 0))
    reason = label('', (0, -.005), .95, origin=(0, 0))

    def reset():
        sim.reset()
        drag.update(active=False, offset=None, last_z=None, filtered=0.0)
        lead.position = (-3.8, 0, 35)

    def toggle():
        sim.night = not sim.night
        sim.target = min(sim.target, (100 if sim.night else 120) / 3.6)
        window.color = color.rgb32(22, 35, 57) if sim.night else color.rgb32(183, 208, 222)
        ground.color = color.rgb32(38, 68, 64) if sim.night else color.rgb32(142,160,126)
        ambient.color = color.rgb32(65, 84, 119) if sim.night else color.rgb32(155, 170, 190)
        sun.color = color.rgb32(110, 141, 187) if sim.night else color.rgb32(255, 240, 213)
        for text in (brand, mode, unit):
            text.color = color.white if sim.night else navy

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
        if t <= 0:
            return None  # Mouse above the horizon has no road intersection.
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
                if not sim.present:
                    sim.gap, sim.x = lead.z - 3.6, lead.x
                drag.update(active=True, offset=lead.position - p,
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
                    if dt > 0:
                        # Follow the pointer using a speed command. The displayed
                        # lead speed is exactly the speed integrated by physics;
                        # no teleporting to a position inconsistent with that speed.
                        requested = clamp(sim.speed + (new_gap - sim.gap) / .10, 0, 160 / 3.6)
                        sim.lead_speed += (requested - sim.lead_speed) * (1 - math.exp(-dt / .06))
                    drag['last_z'] = new_gap
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
                lead.position = (-3.8, 0, 35)
        if sim.present:
            lead.position = (sim.x, 0, sim.gap + 3.6)
        banner.enabled = headline.enabled = reason.enabled = sim.done
        reason.text = sim.reason + '\nPress RESET to start another sequence'
        speed.text = speed_shadow.text = f'{sim.speed * 3.6:.0f}'
        target.text = f'CRUISE  {sim.target * 3.6:.0f} km/h'
        mode.text = ('NIGHT  /  LIMIT 100' if sim.night else 'DAY  /  LIMIT 120')
        status = 'COMPLETE' if sim.done else 'BRAKING' if sim.acceleration < -.35 else 'CRUISING'
        gap_text = f'{sim.gap:.1f} m' if sim.present else '--'
        telemetry.text = 'BRAKING' if status == 'BRAKING' and not sim.done else ''
        # Project the lead's actual position; its speed tag follows the car.
        base = ShowBaseGlobal.base
        projected = Point2()
        car_point = base.cam.getRelativePoint(scene, Point3(lead.x, 2.6, lead.z))
        on_screen = base.cam.node().get_lens().project(car_point, projected)
        lead_tag.enabled = bool(on_screen and abs(projected.x)<.94 and abs(projected.y)<.8 and not sim.done)
        lead_tag.position = (projected.x*camera.aspect_ratio/2, projected.y/2, -.02)
        lead_tag.text = f'{sim.lead_speed*3.6:.0f} km/h  /  {sim.gap:.0f} m' if sim.present else 'DRAG TO MERGE'
        for lamp in lamps:
            lamp.color = color.rgb32(255, 35, 52) if sim.acceleration < -.35 or sim.done else color.rgb32(110, 43, 49)
        hint.text = ('Drag forward / back to set lead speed. Release to coast.' if drag['active'] else
                     'Drag orange car to merge   /   R reset   /   N lights   /   Arrows cruise speed')

    Entity(input=input_event, update=update_frame)
    app.run()


if __name__ == '__main__':
    main()
