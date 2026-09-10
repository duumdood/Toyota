"""PENCIL / DRIVE — a US-100 distance-input mock, rendered as line art.

Run: python road_simulator.py (requires ursina).
Vertical slider is the sole obstacle input. Top detent = no return.
Mock calibration: 1 sensor cm = 0.5 simulated road metres (configurable).
Lead velocity is ESTIMATED from ego velocity + filtered distance derivative.
No real US-100 driver or serial connection is enabled in this mock.
"""
import math
import random

SENSOR_CM_TO_METRES = .5
MIN_GAP, MAX_GAP = 2.0, 100.0


def clamp(x, low, high):
    return max(low, min(high, x))


def idm_acceleration(v, v0, s=None, dv=0.0, T=1.0, s0=4.0):
    if v0 <= 0:
        return -3.0 if v > 0 else 0.0
    desired = s0 + max(0.0, v*T + v*dv/(2*math.sqrt(3*3.5)))
    interaction = 0 if s is None else (desired/max(s, 2))**2
    return clamp(3*(1-(v/max(v0,.1))**4-interaction), -7, 3)


class Simulation:
    def __init__(self):
        self.target = 50/3.6
        self.night = False
        self.reset()
        self.speed = self.target

    def reset(self):
        self.speed = self.acceleration = 0.0
        self.gap = MAX_GAP
        self.lead_speed = None
        self.range_rate = 0.0
        self.previous_gap = None
        self.sample_age = 0.0
        self.present = self.done = self.did_brake = False
        self.settled = 0.0
        self.reason = ''
        self.odometer = 0.0

    def sample_distance(self, gap, dt):
        """Sensor boundary: range in simulated metres; None means no return.

        Never differentiate the first sample or a discontinuous re-acquisition.
        A signed velocity estimate is retained: negative means approaching ego,
        not a fictitious stationary car. The range remains authoritative.
        """
        if self.done or dt <= 0:
            return
        if gap is None:
            self.present = False
            self.previous_gap = None
            self.lead_speed = None
            self.range_rate = self.sample_age = self.settled = 0.0
            self.did_brake = False
            return
        gap = clamp(gap, MIN_GAP, MAX_GAP)
        first = self.previous_gap is None or dt > .3
        rate = 0 if first else (gap-self.previous_gap)/dt
        # Slider jumps emulate a new object/return; do not invent huge speeds.
        discontinuity = abs(rate) > 80
        if first or discontinuity:
            self.range_rate = self.sample_age = 0.0
            self.lead_speed = None
            self.settled = 0
        else:
            self.sample_age += dt
            self.range_rate += (rate-self.range_rate)*(1-math.exp(-dt/.14))
            self.lead_speed = self.speed+self.range_rate if self.sample_age >= .12 else None
        self.present, self.gap, self.previous_gap = True, gap, gap

    def step(self, dt, manipulating=False):
        if self.done or dt <= 0:
            return 0.0
        old = self.speed
        dv = -self.range_rate if self.lead_speed is not None else old
        self.acceleration = idm_acceleration(old, self.target,
            self.gap if self.present else None, dv,
            1.8 if self.night else 1, 7 if self.night else 4)
        self.speed = max(0, old+self.acceleration*dt)
        travel = (old+self.speed)*.5*dt
        self.odometer += travel
        # Do not integrate gap: the sensor/slider owns measured distance.
        if self.lead_speed is not None:
            self.lead_speed = self.speed+self.range_rate
        if self.present and self.acceleration < -.35:
            self.did_brake = True
        stable = (self.present and self.did_brake and not manipulating
                  and self.lead_speed is not None and abs(self.range_rate)<.5
                  and abs(self.acceleration)<.4)
        self.settled = self.settled+dt if stable else 0
        if self.present and self.did_brake and self.speed < .3:
            self.speed = 0
            self.done, self.reason = True, 'STOP COMPLETE'
        elif self.settled > 1.2:
            self.done, self.reason = True, 'DECELERATION COMPLETE'
        return travel


def main():
    from ursina import (Ursina, Entity, Text, Button, Vec3, color, camera,
                        window, scene, mouse, held_keys, time, Mesh, Shader)
    from panda3d.core import Point2, Point3, loadPrcFileData, AntialiasAttrib
    from direct.showbase import ShowBaseGlobal

    loadPrcFileData('', 'framebuffer-multisample 1\nmultisamples 4')
    app = Ursina(title='PENCIL / DRIVE', borderless=False, size=(1440,900))
    scene.setAntialias(AntialiasAttrib.MAuto)
    window.fps_counter.enabled = False
    window.exit_button.visible = False
    sim = Simulation()
    rng = random.Random(17)
    def grey(v, alpha=1):
        return color.rgba(v,v,v,alpha)
    ink, paper = grey(.085), grey(.975)
    navy, coral = ink, grey(.25)
    window.color = paper

    sketch_shader = Shader(name='pencil_hatching', language=Shader.GLSL,
        vertex="""
#version 140
uniform mat4 p3d_ModelViewProjectionMatrix;
uniform mat4 p3d_ModelMatrix;
in vec4 p3d_Vertex;
in vec3 p3d_Normal;
out vec3 normal;
void main() {
    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
    normal = normalize(mat3(p3d_ModelMatrix)*p3d_Normal);
}
""", fragment="""
#version 140
uniform vec4 p3d_ColorScale;
in vec3 normal;
out vec4 fragColor;
void main() {
    float shade=1.0-abs(dot(normalize(normal),normalize(vec3(-.4,.85,.3))));
    float diagonal=mod(gl_FragCoord.x+gl_FragCoord.y,7.0);
    float crossline=mod(gl_FragCoord.x-gl_FragCoord.y,9.0);
    float pencil=(shade>.22 && diagonal<.8 ? .26 : 0.0);
    pencil += (shade>.72 && crossline<.7 ? .18 : 0.0);
    float grain=fract(sin(dot(floor(gl_FragCoord.xy),vec2(12.9898,78.233)))*43758.5453);
    float base=p3d_ColorScale.r;
    float tone=clamp(base-shade*.07-pencil+(grain-.5)*.024, .04, 1.0);
    fragColor=vec4(vec3(tone),p3d_ColorScale.a);
}
""")
    def strokes(vertices, parent=None, tint=ink, width=1):
        pairs=[(i,i+1) for i in range(0,len(vertices)-1,2)]
        return Entity(parent=parent or scene, model=Mesh(vertices=vertices, triangles=pairs, mode='line', thickness=width), color=tint)

    cube_points = [(-.5,-.5,-.5),(.5,-.5,-.5),(.5,.5,-.5),(-.5,.5,-.5),
                   (-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)]
    cube_edges=[(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    def block(parent=None, outline=True, **kw):
        obj=Entity(parent=parent or scene, model='cube', shader=sketch_shader, **kw)
        if outline:
            strokes([cube_points[i] for edge in cube_edges for i in edge],obj)
        return obj

    def outline_surface(parent, vertices, triangles):
        edges={}
        for face in triangles:
            a,b,c=[Vec3(*vertices[i]) for i in face]
            n=(b-a).cross(c-a).normalized()
            for i,j in ((face[0],face[1]),(face[1],face[2]),(face[2],face[0])):
                edges.setdefault(tuple(sorted((i,j))),[]).append(n)
        points=[]
        for (a,b), normals in edges.items():
            if len(normals)==1 or any(abs(normals[0].dot(n))<.98 for n in normals[1:]):
                points.extend([vertices[a],vertices[b]])
        strokes(points,parent,width=1.4)

    # Road repeats in arc-length-like world coordinates; objects follow the same
    # centreline/tangent so the lead stays attached to the curved lane.
    def bend(s):
        return 30*math.sin(s/125)
    def heading(s):
        return math.atan(.24*math.cos(s/125))
    def pose(s, lateral=0):
        h=heading(sim.odometer)
        hs=heading(s)
        dx=bend(s)-bend(sim.odometer)+lateral*math.cos(hs)
        dz=s-sim.odometer-lateral*math.sin(hs)
        return Vec3(dx*math.cos(h)-dz*math.sin(h),0,dx*math.sin(h)+dz*math.cos(h)), math.degrees(hs-h)

    Entity(model='plane', scale=3000, y=-.08, color=paper)
    road_segments=[]
    for i in range(90):
        root=Entity()
        Entity(parent=root,model='plane',scale=(12,1,6.12),y=-.025,color=grey(.96))
        for x in (-6,-2,2,6):
            strokes([(x,.006,-3),(x,.006,3 if abs(x)==6 else .5)],root,width=1.35)
        for x in (-6.4,6.4):
            strokes([(x,.07,-3),(x,.07,3),(x,.52,-3),(x,.52,3),(x,.07,0),(x,.52,0)],root)
        road_segments.append(root)

    buildings=[]
    for i in range(42):
        root=Entity()
        width=rng.uniform(4,9)
        height=rng.uniform(9,32)
        depth=rng.uniform(4,9)
        block(root, position=(0,height/2,0), scale=(width,height,depth), color=paper)
        lines=[]
        for y in range(2,int(height),3):
            lines.extend([(-width/2,y,-depth/2-.01),(width/2,y,-depth/2-.01)])
        # Pencil hatching on one side of the architectural studies.
        for y in range(0,int(height)-2,2):
            lines.extend([(width/2+.01,y,-depth/2),(width/2+.01,y+2,depth/2)])
        strokes(lines,root,tint=grey(.4))
        if i%4==0:
            strokes([(0,height,0),(0,height+5,0)],root)
        buildings.append((root,(-1 if i%2 else 1)*rng.uniform(16,48)))

    def car(tint):
        root = Entity()
        Entity(parent=root, model='quad', rotation_x=90, position=(0,.01,0), scale=(2.15,3.9), color=grey(.1,.12), double_sided=True)
        def surface(vertices, triangles, tint):
            # Expand faces so normals remain flat and no unused vertex yields NaN.
            packed = [vertices[i] for face in triangles for i in face]
            mesh = Mesh(vertices=packed, triangles=list(range(len(packed))), mode='triangle')
            mesh.generate_normals()
            obj = Entity(parent=root, model=mesh, color=tint, shader=sketch_shader, double_sided=True)
            outline_surface(root, vertices, triangles)
            return obj
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
                surface(wheel_vertices,wheel_faces,grey(0.103))
                block(root, position=(x+(.105 if x>0 else -.105), .34, z), scale=(.015,.28,.28), rotation_x=45, color=grey(0.699))
        block(root, position=(0,.38,-1.82), scale=(1.65,.14,.12), color=navy)
        block(root, position=(0,.57,-1.83), scale=(.45,.17,.025), color=grey(0.889))
        block(root, position=(0,.82,-1.55), scale=(1.55,.05,.22), color=tint)
        for x in (-.94,.94):
            block(root, position=(x,1,.55), scale=(.22,.14,.25), color=tint)
        for x in (-.57, .57):
            Entity(parent=root, model='cube', position=(x, .65, 1.81), scale=(.43, .14, .025), color=grey(0.907))
        lamps = [Entity(parent=root, model='cube', position=(x, .65, -1.81), scale=(.48, .16, .025), color=coral) for x in (-.57, .57)]
        return root, lamps


    ego, lamps = car(paper)
    lead, _ = car(grey(.7))
    camera.orthographic=False
    camera.fov=65
    camera.position=(1.6,4.1,-9.5)
    camera.look_at(Vec3(0,.9,9))

    def label(text, pos, scale=1, tint=ink, **kw):
        return Text(parent=camera.ui,text=text,position=pos,scale=scale,color=tint,**kw)
    def rule(x,y,w):
        Entity(parent=camera.ui,model='quad',position=(x,y,.03),scale=(w,.0013),color=ink)
    def card(pos,size,tint=paper):
        Entity(parent=camera.ui,model='quad',position=(pos[0]+.004,pos[1]-.005,.08),scale=size,color=grey(.25))
        Entity(parent=camera.ui,model='quad',position=(*pos,.04),scale=(size[0]+.003,size[1]+.003),color=ink)
        return Entity(parent=camera.ui,model='quad',position=(*pos,.02),scale=size,color=tint)

    label('PENCIL / DRIVE',(-.75,.456),1.05)
    label('01   /   DISTANCE STUDY',(-.75,.42),.62,grey(.4))
    rule(-.63,.401,.24)
    speed=label('',(0,.39),4.3,origin=(0,0))
    label('K M / H',(0,.32),.67,origin=(0,0))
    mode=label('',(.74,.456),.7,origin=(.5,0))
    target=label('',(-.75,-.34),.85)
    lead_tag=label('',(0,0),.78,origin=(0,0))
    lead_badge=Entity(parent=camera.ui,model='quad',scale=(.30,.031),color=paper,z=-.005)
    telemetry=label('',(0,-.31),.8,origin=(0,0))
    hint=label('SLIDE TO CHANGE DISTANCE  /  R RESET  /  ARROWS CRUISE',(0,-.471),.63,origin=(0,0))

    slider={'value':1.0,'active':False}
    card((.68,.015),(.20,.66))
    label('US-100',(.68,.31),.88,origin=(0,0))
    label('RANGE MOCK',(.68,.277),.57,grey(.4),origin=(0,0))
    slider_x, slider_bottom, slider_top=.68,-.22,.22
    rail=Button(parent=camera.ui,model='quad',position=(slider_x,0,-.015),scale=(.085,.49),color=grey(1,0),
                highlight_color=grey(1,0),pressed_color=grey(1,0),on_click=lambda:None)
    Entity(parent=camera.ui,model='quad',position=(slider_x,0,-.02),scale=(.002,.44),color=ink)
    for i in range(11):
        y=slider_bottom+i*.044
        Entity(parent=camera.ui,model='quad',position=(slider_x,y,-.025),scale=(.027 if i%5==0 else .012,.001),color=ink)
    thumb=Entity(parent=camera.ui,model='quad',position=(slider_x,slider_top,-.04),scale=(.025,.025),rotation_z=45,color=ink)
    label('FAR',(.735,.222),.53,grey(.4),origin=(0,0))
    label('NEAR',(.735,-.22),.53,grey(.4),origin=(0,0))
    range_text=label('',(.68,-.272),.77,origin=(0,0))
    sensor_text=label('',(.68,-.30),.52,grey(.4),origin=(0,0))
    label('MOCK SCALE  1 cm : 0.5 m',(.68,-.347),.48,grey(.4),origin=(0,0))

    def reset():
        sim.reset()
        slider.update(value=1.0,active=False)
    def toggle():
        sim.night=not sim.night
        sim.target=min(sim.target,(100 if sim.night else 120)/3.6)

    controls=[]
    def button(text,x,width,action=None,solid=False):
        Entity(parent=camera.ui,model='quad',position=(x+.003,-.417,.025),scale=(width,.049),color=grey(.25))
        Entity(parent=camera.ui,model='quad',position=(x,-.412,.01),scale=(width+.003,.049),color=ink)
        btn=Button(text=text,position=(x,-.412,-.01),scale=(width,.046),color=ink if solid else paper,
                   highlight_color=grey(.18) if solid else grey(.88),pressed_color=grey(.6),
                   on_click=action or (lambda:None))
        btn.text_entity.color=paper if solid else ink
        btn.text_entity.scale *= .8
        controls.append(btn)
        return btn
    down=button('- SPEED',-.62,.18)
    up=button('+ SPEED',-.41,.18)
    button('DAY / NIGHT',.20,.23,toggle)
    button('RESET  [R]',.49,.24,reset,solid=True)

    banner=Entity(parent=camera.ui,model='quad',position=(0,.04,-.10),scale=(.84,.21),color=paper)
    banner_border=Entity(parent=camera.ui,model='quad',position=(0,.04,-.09),scale=(.846,.216),color=ink)
    headline=label('BREAK!!',(0,.088),3.8,origin=(0,0),z=-.12)
    reason=label('',(0,-.005),.76,origin=(0,0),z=-.12)

    def set_slider_from_mouse():
        slider['value']=clamp((mouse.y-slider_bottom)/(slider_top-slider_bottom),0,1)
    def input_event(key):
        if key=='r':
            reset()
        elif key=='n':
            toggle()
        elif key=='left mouse down' and rail.hovered and not sim.done:
            slider['active']=True
            set_slider_from_mouse()
        elif key=='left mouse up':
            slider['active']=False

    def update_frame():
        raw_dt=max(time.dt,0)
        dt=min(raw_dt,.1)
        if not mouse.left:
            slider['active']=False
        if not sim.done:
            if slider['active']:
                set_slider_from_mouse()
            change=bool(held_keys['up arrow'] or (up.hovered and mouse.left))-bool(held_keys['down arrow'] or (down.hovered and mouse.left))
            sim.target=clamp(sim.target+change*4*dt,0,(100 if sim.night else 120)/3.6)
            u=slider['value']
            gap=None if u>=.98 else MIN_GAP+(MAX_GAP-MIN_GAP)*min(u/.96,1)
            sim.sample_distance(gap,raw_dt)
            remaining=dt
            while remaining>1e-8:
                tick=min(remaining,1/120)
                sim.step(tick,slider['active'])
                remaining-=tick
        # Deterministic recycling: no endpoint in the road, no cumulative drift.
        base_segment=math.floor(sim.odometer/6)-5
        for i,obj in enumerate(road_segments):
            p,h=pose((base_segment+i)*6)
            obj.position=p
            obj.rotation_y=h
        city_base=math.floor(sim.odometer/22)-3
        for i,(obj,lateral) in enumerate(buildings):
            p,h=pose((city_base+i)*22,lateral)
            obj.position=p
            obj.rotation_y=h
        lead.enabled=sim.present
        if sim.present:
            p,h=pose(sim.odometer+sim.gap+3.6)
            lead.position=p
            lead.rotation_y=h
        thumb.y=slider_bottom+slider['value']*(slider_top-slider_bottom)
        speed.text=f'{sim.speed*3.6:.0f}'
        target.text=f'CRUISE / {sim.target*3.6:.0f} km/h'
        mode.text=('NIGHT / 100' if sim.night else 'DAY / 120')
        range_text.text=f'{sim.gap:.1f} m' if sim.present else 'NO TARGET'
        sensor_text.text=f'{sim.gap/SENSOR_CM_TO_METRES:.0f} cm mock' if sim.present else 'top = no return'
        telemetry.text='BRAKING' if sim.acceleration<-.35 and not sim.done else ''
        base=ShowBaseGlobal.base
        projected=Point2()
        visible=base.cam.node().get_lens().project(base.cam.getRelativePoint(scene,Point3(lead.x,2.5,lead.z)),projected)
        lead_tag.enabled=bool(sim.present and not sim.done and visible and abs(projected.x)<.68 and abs(projected.y)<.75)
        lead_tag.position=(projected.x*camera.aspect_ratio/2,projected.y/2,-.015)
        lead_badge.enabled=lead_tag.enabled
        lead_badge.position=(lead_tag.x,lead_tag.y,-.005)
        estimate='EST --' if sim.lead_speed is None else f'EST {sim.lead_speed*3.6:.0f} km/h'
        lead_tag.text=f'{estimate}  /  {sim.gap:.1f} m'
        for lamp in lamps:
            lamp.color=ink if sim.acceleration<-.35 else grey(.65)
        banner.enabled=banner_border.enabled=headline.enabled=reason.enabled=sim.done
        reason.text=sim.reason+'\nR / RESET TO CONTINUE'

    Entity(input=input_event,update=update_frame)
    app.run()


if __name__=='__main__':
    main()
