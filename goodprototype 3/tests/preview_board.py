"""Offscreen visual check only. Test packets never appear in production UI."""
import dataclasses
import sys
import time as clock
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'.runtime')]
import ursina
from panda3d.core import loadPrcFileData, Filename
from road_board_io import BoardTelemetry
loadPrcFileData('', 'audio-library-name null\nwin-size 1440 900')

class Receiver:
    p=None
    live=False
    received=None
    def snapshot(self):
        if self.live:
            return self.p,self.received,'BOARD CONNECTED',0
        return self.p,None,'SERIAL DISCONNECTED',0
    def set(self,**kw):
        self.p=dataclasses.replace(self.p,**kw)
        self.received=clock.monotonic()
        self.live=True
receiver=Receiver()
original=ursina.Ursina
def factory(**kw):
    kw['development_mode']=False
    app=original(window_type='offscreen',**kw)
    def run():
        ursina.camera.ui_lens.set_film_size(20*ursina.window.aspect_ratio,20)
        control=next(e for e in ursina.scene.entities if getattr(getattr(e,'update',None),'__name__','')=='update_frame')
        env=dict(zip(control.update.__code__.co_freevars,[c.cell_contents for c in control.update.__closure__]))
        def tick(n=1):
            ursina.time.dt=1/60
            for _ in range(n):
                if receiver.live:
                    receiver.received=clock.monotonic()
                control.update()
        def capture(name):
            for _ in range(3):app.graphicsEngine.renderFrame()
            assert app.win.saveScreenshot(Filename.fromOsSpecific(str(ROOT/name)))
        tick()
        assert env['speed'].text=='--'
        assert not [e for e in ursina.scene.entities if isinstance(e,ursina.Button)
                    and any(word in str(getattr(e,'text','')).upper() for word in ('SPEED','RESET','DAY / NIGHT','SUDDEN','DRAG'))]
        receiver.p=BoardTelemetry(1,100,20,25,-2,0,25,15,False,True,False,False,1600,500,0,0,0)
        receiver.set()
        tick(5)
        assert env['speed'].text=='72' and env['telemetry'].text=='BRAKING'
        capture('preview_board_day.png')
        receiver.set(accel_blocked=True)
        tick(); assert env['accel_notice'].enabled
        capture('preview_board_accel_blocked.png')
        receiver.set(fault=True)
        tick(); assert not env['accel_notice'].enabled
        receiver.set(fault=False,accel_blocked=False)
        tick(); assert not env['accel_notice'].enabled
        receiver.set(night=True,ldr_adc=3900,board_ms=150)
        tick(20)
        assert env['popup'].enabled
        receiver.set(accel_blocked=True)
        tick(); assert not env['accel_notice'].enabled
        capture('preview_board_night.png')
        tick(210);assert not env['popup'].enabled
        assert env['accel_notice'].enabled
        receiver.set(done=True,done_reason=1,speed_mps=0,braking=False,board_ms=200)
        tick();assert env['banner'].enabled
        assert env['result_text'].enabled
        assert 'ego 0.0 km/h' in env['result_text'].text
        assert not env['accel_notice'].enabled
        capture('preview_board_stop.png')
        for n in range(4):
            receiver.set(sequence=10+n,board_ms=300+n*50,reset_id=1,
                         done=False,done_reason=0,speed_mps=60/3.6,
                         gap_m=5,distance_mm=100)
            tick()
        assert env['headline'].text=='CRASH!' and not env['physics_warn'].enabled
        assert not env['accel_notice'].enabled
        assert env['result_text'].text=='Speed at crash: 60.0 km/h'
        frozen_result=env['sim'].result_packet
        receiver.set(sequence=19,board_ms=480,speed_mps=12,gap_m=10)
        tick(); assert env['sim'].result_packet is frozen_result
        assert env['result_text'].text=='Speed at crash: 60.0 km/h'
        assert not receiver.p.done and receiver.p.speed_mps==12
        capture('preview_board_crash.png')
        receiver.set(sequence=20,done=False,done_reason=0,reset_id=2,board_ms=500)
        tick();assert not env['banner'].enabled
        assert env['sim'].result_packet is None
        receiver.live=False
        before=env['sim'].odometer
        tick(60)
        assert env['sim'].odometer==before and env['speed'].text=='--'
        assert not env['accel_notice'].enabled
        capture('preview_board_disconnected.png')
        print('PASS: board-only UI, day/night popup, brake, physical-reset packet, disconnected freeze')
        app.destroy()
    app.run=run
    return app
ursina.Ursina=factory
import road_simulator_board
road_simulator_board.main(receiver)
