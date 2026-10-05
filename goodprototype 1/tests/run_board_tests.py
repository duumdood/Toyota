"""Compile/run actual C with ARM GCC + Unicorn, then verify Python contract.
Usage: python tests/run_board_tests.py --gcc /path/to/arm-none-eabi-gcc.exe
Test dependencies: pip install unicorn (plus project pyserial/ursina for UI).
No serial port is opened and no board is flashed by this test.
"""
import argparse
import dataclasses
from pathlib import Path
import struct
import subprocess
import sys
import zlib
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'.runtime')]
from road_board_io import parse_frame, LineDecoder, BoardView, SerialReceiver


def run_c(gcc):
    import unicorn as u
    from unicorn import arm_const as a
    build=ROOT/'tmp'/'board_tests'
    build.mkdir(parents=True,exist_ok=True)
    elf=build/'core_tests.elf'
    subprocess.run([gcc,'-std=c11','-Wall','-Wextra','-Werror','-Wconversion','-Os',
        '-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard',
        '-I',str(ROOT/'stm32_road'),'-nostartfiles','--specs=nosys.specs',
        '-Wl,-Ttext=0x08000000,-Tdata=0x20000000,-e,run_tests',
        str(ROOT/'tests'/'board_core_tests.c'),str(ROOT/'stm32_road'/'road_app.c'),
        str(ROOT/'stm32_road'/'road_protocol.c'),'-o',str(elf)],check=True)
    prefix=str(Path(gcc).parent/'arm-none-eabi-')
    binary=build/'core_tests.bin'
    subprocess.run([prefix+'objcopy.exe','-O','binary','--only-section=.text','--only-section=.rodata',str(elf),str(binary)],check=True)
    output=subprocess.check_output([prefix+'nm.exe','-n',str(elf)],text=True)
    symbols={parts[2]:int(parts[0],16) for line in output.splitlines() if len(parts:=line.split())==3}
    machine=u.Uc(u.UC_ARCH_ARM,u.UC_MODE_THUMB)
    machine.ctl_set_cpu_model(a.UC_CPU_ARM_CORTEX_M4)
    machine.mem_map(0x08000000,0x100000)
    machine.mem_map(0x20000000,0x20000)
    machine.mem_write(0x08000000,binary.read_bytes())
    machine.reg_write(a.UC_ARM_REG_C1_C0_2,0x00F00000)
    machine.reg_write(a.UC_ARM_REG_FPEXC,0x40000000)
    machine.reg_write(a.UC_ARM_REG_SP,0x2001FFF0)
    machine.reg_write(a.UC_ARM_REG_LR,0x080FF001)
    machine.emu_start(symbols['run_tests']|1,0x080FF000,count=20000000)
    assert machine.reg_read(a.UC_ARM_REG_PC)==0x080FF000,'C instruction budget exhausted'
    result=machine.reg_read(a.UC_ARM_REG_R0)
    assert result==0,f'C assertion failed at tests/board_core_tests.c:{result}'
    size=struct.unpack('<I',machine.mem_read(symbols['g_test_frame_size'],4))[0]
    frame=bytes(machine.mem_read(symbols['g_test_frame'],size))
    print('PASS: actual ARM C IDM, tap/hold, both buttons, LDR hysteresis/dwell, night cap, stop/reset, stale sensors, derivative, encoder')
    return frame


def python_checks(frame):
    p=parse_frame(frame)
    assert (p.sequence,p.speed_mps,p.target_mps,p.gap_m,p.lead_speed_mps,p.ldr_adc)==(42,20,25,20,12,3900)
    assert p.night and p.braking and not p.done
    for cut in range(1,len(frame)):
        d=LineDecoder()
        assert not d.feed(frame[:cut])
        assert d.feed(frame[cut:])==[p]
    d=LineDecoder()
    assert d.feed(b'x'*10000+b'\n'+frame)==[p]
    assert len(d.buffer)==0
    corrupt=frame.replace(b'RD1',b'RD2')
    assert not d.feed(corrupt)
    assert d.feed(frame+frame)==[p,p]
    def wire(body):
        return body+b'*'+f'{zlib.crc32(body):08X}'.encode()+b'\n'
    invalids=[b'RD2,1',frame.split(b'*')[0].replace(b',3900,',b',5000,'),frame.split(b'*')[0].replace(b',20000,25000,',b',NaN,25000,')]
    for body in invalids:
        try: parse_frame(wire(body))
        except ValueError: pass
        else: raise AssertionError(body)
    class Receiver:
        packet=p
        received=10.0
        def snapshot(self):return self.packet,self.received,'CONNECTED',0
    r=Receiver(); view=BoardView()
    view.refresh(r,10.0)
    assert view.odometer==123 and view.speed==20 and view.night
    r.packet=dataclasses.replace(p,sequence=43,board_ms=12395,odometer_m=124)
    r.received=10.05
    view.refresh(r,10.075)
    assert 123<view.odometer<124
    view.refresh(r,10.15); assert view.odometer==124
    view.refresh(r,11.0); assert not view.connected and view.odometer==124
    r.packet=dataclasses.replace(p,reset_id=3,odometer_m=0,speed_mps=0)
    r.received=12.0
    view.refresh(r,12.0); assert view.odometer==0 and view.speed==0
    assert not hasattr(view,'step') and not hasattr(view,'sample_distance')
    print('PASS: C -> Python CRC/units, split frames, corrupt/oversized recovery, render-only interpolation, stale link freeze, board reset')

    # Exercise receiver context/open setup and duplicate handling without ever
    # touching a real COM device. Any attempted PC write fails this test.
    import serial
    receiver=SerialReceiver('TEST_ONLY')
    chunks=[frame[:10],frame[10:],frame,corrupt]
    class Link:
        in_waiting=4096
        def __init__(self,port,baudrate,timeout):
            assert port is None and baudrate==115200 and timeout==.1
            self.dtr=self.rts=True
        def __enter__(self):
            assert self.port=='TEST_ONLY' and not self.dtr and not self.rts
            return self
        def __exit__(self,*args):return False
        def read(self,size):
            if chunks:return chunks.pop(0)
            receiver.stop_event.set()
            return b''
        def write(self,data):raise AssertionError('PC must never send control data')
    with patch.object(serial,'Serial',Link):
        receiver._run()
    packet,received,message,invalid=receiver.snapshot()
    assert packet==p and received is not None and invalid==1
    assert message=='BOARD CONNECTED'
    receiver.close()
    print('PASS: read-only serial receiver setup, fragmented input, duplicates, invalid frames, clean stop (fake transport only)')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--gcc',required=True)
    args=parser.parse_args()
    python_checks(run_c(args.gcc))
