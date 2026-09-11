"""PENCIL / DRIVE — board-only viewer; run with --port COM5.
Requires: pip install ursina pyserial
Keep road_board_io.py next to this file. No controls or IDM run on the PC.
All values arrive from stm32_road firmware; see stm32_road/README.md.
"""
import argparse
import math
import random
from road_board_io import BoardView, SerialReceiver, SERIAL_PORT, SERIAL_BAUD

def clamp(x, low, high):
    return max(low, min(high, x))

def main(receiver):
    from ursina import (Ursina, Entity, Text, Vec3, color, camera,
                        window, scene, time, Mesh, Shader)
    from panda3d.core import Point2, Point3, loadPrcFileData, AntialiasAttrib
    from direct.showbase import ShowBaseGlobal

    loadPrcFileData('', 'framebuffer-multisample 1\nmultisamples 4')
    app = Ursina(title='PENCIL / DRIVE — BOARD LIVE', borderless=False, size=(1440,900), development_mode=False)
    scene.setAntialias(AntialiasAttrib.MAuto)
    window.fps_counter.enabled = False
    window.exit_button.visible = False
    sim = BoardView()
    rng = random.Random(17)
    def grey(v, alpha=1):
        return color.rgba(v,v,v,alpha)
    ink, paper = grey(.085), grey(.975)
    navy, coral = ink, grey(.25)
    day_sky=color.rgb32(249,247,240)
    night_sky=color.rgb32(46,53,68)
    window.color = day_sky
    scene.set_shader_input('night_mix',0.0)

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
uniform float night_mix;
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
    vec3 day=vec3(tone)*vec3(1.0,.994,.975);
    vec3 night=vec3(tone)*vec3(.65,.69,.76)+vec3(.025,.035,.055);
    fragColor=vec4(mix(day,night,night_mix),p3d_ColorScale.a);
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

    ground=Entity(model='plane', scale=3000, y=-.08, color=day_sky)
    road_surfaces=[]
    road_segments=[]
    for i in range(90):
        root=Entity()
        road_surfaces.append(Entity(parent=root,model='plane',scale=(12,1,6.12),y=-.025,color=color.rgb32(239,238,231)))
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
        # Layered contact shadows give the paper city a little depth.
        for k in range(3):
            Entity(parent=root,model='quad',rotation_x=90,position=(.7+k*.18,-.065+k*.001,.5),
                   scale=(width+1.8-k*.5,depth+1.5-k*.4),color=grey(.12,.025),double_sided=True)
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

    brand=label('PENCIL / DRIVE',(-.75,.456),1.05)
    subtitle=label('02   /   STM32 LIVE',(-.75,.42),.62,grey(.4))
    rule(-.63,.401,.24)
    speed=label('',(0,.39),4.3,origin=(0,0))
    unit=label('K M / H',(0,.32),.67,origin=(0,0))
    mode=label('',(.74,.456),.7,origin=(.5,0))
    target=label('',(-.75,-.34),.85)
    lead_tag=label('',(0,0),.78,origin=(0,0))
    lead_badge=Entity(parent=camera.ui,model='quad',scale=(.30,.031),color=paper,z=-.005)
    telemetry=label('',(-.75,-.377),.70)
    hint=label('PHYSICAL BUTTONS ONLY  /  SPEED +  SPEED -  RESET',(0,-.471),.63,origin=(0,0))
    card((.68,.03),(.20,.40))
    label('BOARD LIVE',(.68,.202),.80,origin=(0,0))
    label('US-100 / LDR',(.68,.165),.56,grey(.4),origin=(0,0))
    range_text=label('--',(.68,.10),.9,origin=(0,0))
    sensor_text=label('NO DATA',(.68,.058),.53,grey(.4),origin=(0,0))
    ldr_text=label('ADC --',(.68,-.015),.65,origin=(0,0))
    range_state=label('WAITING',(.68,-.065),.51,grey(.4),origin=(0,0))
    label('READ ONLY',(.68,-.13),.52,grey(.4),origin=(0,0))
    scale_note=label('PHYSICS / STM32 ONLY',(.68,-.196),.48,grey(.4),origin=(0,0))
    link_status=label('WAITING FOR BOARD',(0,-.413),.84,origin=(0,0),z=-.05)
    link_back=Entity(parent=camera.ui,model='quad',position=(0,-.411,-.04),scale=(.65,.045),color=paper)

    # One reusable toast: repeated toggles restart it, never stack animations.
    toast={'age':4.0}
    popup=Entity(parent=camera.ui,enabled=False)
    toast_shadow=Entity(parent=popup,model='quad',position=(.005,.018,-.16),scale=(.54,.264),color=grey(0,.15))
    toast_border=Entity(parent=popup,model='quad',position=(0,.025,-.17),scale=(.542,.262),color=color.rgb32(147,163,188))
    toast_body=Entity(parent=popup,model='quad',position=(0,.025,-.18),scale=(.536,.256),color=color.rgb32(249,250,252))
    toast_eyebrow=Text(parent=popup,text='NIGHT DRIVE',position=(0,.122,-.20),origin=(0,0),scale=.73,color=color.rgb32(89,106,132))
    toast_title=Text(parent=popup,text='SPEED LIMIT',position=(0,.079,-.20),origin=(0,0),scale=.95,color=ink)
    toast_number=Text(parent=popup,text='100',position=(0,.011,-.20),origin=(0,0),scale=3.3,color=ink)
    toast_unit=Text(parent=popup,text='km/h  /  cruise limit adjusted',position=(0,-.067,-.20),origin=(0,0),scale=.65,color=color.rgb32(89,106,132))
    toast_parts=[(e,tuple(e.color)) for e in (toast_shadow,toast_border,toast_body,toast_eyebrow,toast_title,toast_number,toast_unit)]

    def update_theme():
        toast['age']=0.0 if sim.night else 4.0
        window.color=night_sky if sim.night else day_sky
        ground.color=color.rgb32(72,80,96) if sim.night else day_sky
        scene.set_shader_input('night_mix',float(sim.night))
        for surface in road_surfaces:
            surface.color=color.rgb32(151,158,173) if sim.night else color.rgb32(239,238,231)
        for text in (brand,subtitle,speed,unit,mode,target,telemetry,hint,scale_note):
            text.color=color.rgb32(242,244,249) if sim.night else ink

    banner=Entity(parent=camera.ui,model='quad',position=(0,.04,-.10),scale=(.84,.21),color=paper)
    banner_border=Entity(parent=camera.ui,model='quad',position=(0,.04,-.09),scale=(.846,.216),color=ink)
    headline=label('BREAK!!',(0,.088),3.8,origin=(0,0),z=-.12)
    reason=label('',(0,-.005),.76,origin=(0,0),z=-.12)

    def update_frame():
        raw_dt=max(time.dt,0)
        previous_night=sim.night
        previous_reset=sim.packet.reset_id if sim.packet else None
        sim.refresh(receiver)
        if sim.night != previous_night:
            update_theme()
        if sim.packet and previous_reset is not None and sim.packet.reset_id != previous_reset:
            toast['age']=4.0
        toast['age']+=raw_dt
        age=toast['age']
        alpha=min(1,age/.18)*clamp((3.4-age)/.9,0,1)
        popup.enabled=bool(sim.connected and sim.night and not sim.done and not sim.packet.fault and alpha>0)
        popup.y=.008*(1-min(age/.18,1))
        for obj,base_color in toast_parts:
            obj.color=color.rgba(*base_color[:3],base_color[3]*alpha)
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
        fresh=sim.connected
        speed.text=f'{sim.speed*3.6:.0f}' if fresh else '--'
        target.text=f'CRUISE / {sim.target*3.6:.0f} km/h' if fresh else 'CRUISE / --'
        mode.text=('NIGHT / 100' if sim.night else 'DAY / 120') if fresh else 'MODE / --'
        range_text.text=(f'{sim.packet.gap_m:.1f} m' if sim.present else 'NO TARGET') if fresh else '--'
        sensor_text.text=(f'{sim.packet.distance_mm/10:.1f} cm measured' if sim.packet.distance_mm is not None else 'NO ECHO') if fresh else 'NO DATA'
        ldr_text.text=f'ADC {sim.packet.ldr_adc}' if fresh else 'ADC --'
        range_state.text={0:'VALID ECHO',1:'NO RETURN',2:'CHECK SENSOR'}[sim.packet.range_status] if fresh else 'WAITING'
        telemetry.text='BRAKING' if fresh and sim.braking and not sim.done else ''
        link_status.text=sim.status
        link_status.color=ink
        base=ShowBaseGlobal.base
        projected=Point2()
        visible=base.cam.node().get_lens().project(base.cam.getRelativePoint(scene,Point3(lead.x,2.5,lead.z)),projected)
        lead_tag.enabled=bool(sim.connected and sim.present and not sim.done and visible and abs(projected.x)<.68 and abs(projected.y)<.75)
        lead_tag.position=(projected.x*camera.aspect_ratio/2,projected.y/2,-.015)
        lead_badge.enabled=lead_tag.enabled
        lead_badge.position=(lead_tag.x,lead_tag.y,-.005)
        estimate='EST --' if sim.lead_speed is None else f'EST {sim.lead_speed*3.6:.0f} km/h'
        lead_tag.text=f'{estimate}  /  {sim.gap:.1f} m'
        for lamp in lamps:
            lamp.color=ink if sim.braking else grey(.65)
        banner.enabled=banner_border.enabled=headline.enabled=reason.enabled=bool(sim.done and sim.connected)
        reason.text=sim.reason+'\nPRESS PHYSICAL RESET ON BOARD'

    Entity(update=update_frame)
    app.run()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='STM32 read-only road viewer')
    parser.add_argument('--port',default=SERIAL_PORT,help='ST-LINK COM port, e.g. COM5')
    parser.add_argument('--list-ports',action='store_true')
    args=parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        parser.exit(1,'Install dependencies: python -m pip install ursina pyserial\n')
    if args.list_ports or not args.port:
        for port in list_ports.comports():
            print(f'{port.device}: {port.description}')
        if not args.list_ports:
            parser.exit(2,'Choose the board port: python road_simulator_board.py --port COM5\n')
    else:
        receiver=SerialReceiver(args.port,SERIAL_BAUD)
        receiver.start()
        try:
            main(receiver)
        finally:
            receiver.close()
