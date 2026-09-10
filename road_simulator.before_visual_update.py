"""
Road Simulator - Auto Brake Demo (เฟส Mock ทดสอบก่อนต่อบอร์ดจริง)
==================================================================
จำลองรถวิ่งบนถนนสไตล์ low-poly ด้วย Ursina Engine (pip install ursina)

เฟสนี้ Python คำนวณสมการ IDM (Intelligent Driver Model) เอง
เพื่อทดสอบพฤติกรรมการเบรกให้ลื่นก่อน แล้วค่อยย้ายการคำนวณนี้
ไปทำบน STM32F411 ทีหลัง (ตอนนั้น Python จะแค่รับค่าความเร็ว/สถานะ
ผ่าน UART มาแสดงผลอย่างเดียว ไม่ต้องคำนวณเองแล้ว)

ควบคุม:
  - ปุ่ม/ลูกศรขึ้น-ลง : เพิ่ม/ลดความเร็วเป้าหมาย (กดค้างได้)
  - ปุ่ม Insert Car   : จำลองรถแทรกเข้ามาข้างหน้า
  - ปุ่ม Day / Night  : สลับโหมดกลางวัน-กลางคืน
  - ปุ่ม Reset        : รีเซ็ต (กดได้เฉพาะตอนขึ้น BRAKE เท่านั้น)
"""

from ursina import *
import math
import random

app = Ursina()

# ---------------------------------------------------------------
# ค่าคงที่ทางฟิสิกส์ (พารามิเตอร์ของสมการ IDM)
# ---------------------------------------------------------------
A_MAX = 3.0            # ความเร่งสูงสุด (m/s^2)
B_COMFORT = 3.5         # ความหน่วงที่สบาย / comfortable deceleration (m/s^2)
DELTA = 4               # เลขยกกำลังของพจน์ free-road
S_MIN = 2.0             # ระยะขั้นต่ำ กันหารด้วยค่าใกล้ศูนย์ (m)

T_DAY = 1.0             # safe time headway ตอนกลางวัน (s)
T_NIGHT = 1.8           # safe time headway ตอนกลางคืน (s) - เผื่อมากขึ้น
S0_DAY = 4.0            # minimum gap ตอนกลางวัน (m)
S0_NIGHT = 7.0          # minimum gap ตอนกลางคืน (m)

V0_MAX_DAY = 120 / 3.6      # จำกัดความเร็วสูงสุดตอนกลางวัน (m/s) = 120 km/h
V0_MAX_NIGHT = 100 / 3.6    # จำกัดความเร็วสูงสุดตอนกลางคืน (m/s) = 100 km/h
V0_STEP = 4.0                # อัตราเพิ่ม/ลดความเร็วเป้าหมายต่อวินาทีตอนกดค้าง (m/s ต่อ s)

V_LEAD_DEFAULT = 30 / 3.6    # ความเร็วของรถที่แทรกเข้ามา (m/s) คงที่
S_INITIAL = 90.0              # ระยะเริ่มต้นตอนรถแทรกเข้ามา (m)
BRAKE_V_THRESHOLD = 0.3        # ต่ำกว่านี้ถือว่าหยุดสนิทแล้ว (m/s)

SKY_DAY = color.rgb(0.53, 0.81, 0.92)
SKY_NIGHT = color.rgb(0.04, 0.05, 0.12)
GROUND_DAY = color.rgb(0.24, 0.55, 0.24)
GROUND_NIGHT = color.rgb(0.06, 0.14, 0.06)

# ---------------------------------------------------------------
# สถานะของระบบ (จะถูกย้ายไปอยู่บน MCU ในเฟสถัดไป)
# ---------------------------------------------------------------
state = {
    'v_ego': 50 / 3.6,       # ความเร็วปัจจุบันของรถเรา (m/s) เริ่มที่ 50 km/h
    'v0': 50 / 3.6,           # ความเร็วเป้าหมายที่ผู้ใช้ตั้ง (m/s)
    'is_night': False,
    'obstacle_active': False,
    's': 0.0,                  # ระยะห่างปัจจุบันจากรถที่แทรกเข้ามา (m)
    'v_lead': 0.0,
    'sequence': 'RUNNING',     # 'RUNNING' หรือ 'BRAKE'
}


def current_v0_max():
    return V0_MAX_NIGHT if state['is_night'] else V0_MAX_DAY


def current_T():
    return T_NIGHT if state['is_night'] else T_DAY


def current_s0():
    return S0_NIGHT if state['is_night'] else S0_DAY


# ---------------------------------------------------------------
# ฉาก: พื้นดิน ถนน รถเรา
# ---------------------------------------------------------------
ground = Entity(model='plane', scale=(200, 1, 1000), position=(0, -0.01, 400),
                 color=GROUND_DAY, texture='white_cube', texture_scale=(40, 200))

road = Entity(model='cube', scale=(10, 0.1, 1000), position=(0, 0, 400),
              color=color.rgb(0.16, 0.16, 0.16))

# เส้นแบ่งเลนกลางถนน (ตกแต่งเล็กน้อยให้ดูมีการเคลื่อนไหว)
for i in range(60):
    Entity(model='cube', scale=(0.3, 0.11, 3), position=(0, 0.01, i * 16),
           color=color.yellow)

ego_car = Entity(model='cube', scale=(1.6, 1.0, 3.0), position=(0, 0.5, 0),
                  color=color.azure)

# ---------------------------------------------------------------
# ต้นไม้สไตล์ low-poly (loop ไปเรื่อยๆ)
# ---------------------------------------------------------------
TREE_COUNT = 40
trees = []


def make_tree(z):
    side = random.choice([-1, 1])
    x = side * random.uniform(4.5, 8.0)
    trunk = Entity(model='cube', scale=(0.3, 1.2, 0.3), color=color.rgb(0.35, 0.24, 0.12),
                    position=(x, 0.6, z))
    top = Entity(model='cube', scale=(1.4, 1.4, 1.4), color=color.rgb(0.12, 0.43, 0.16),
                  position=(x, 1.6, z))
    return {'trunk': trunk, 'top': top, 'z': z, 'x': x}


for i in range(TREE_COUNT):
    trees.append(make_tree(random.uniform(5, 300)))


def respawn_tree(t):
    t['z'] += 300
    side = random.choice([-1, 1])
    t['x'] = side * random.uniform(4.5, 8.0)
    t['trunk'].position = (t['x'], 0.6, t['z'])
    t['top'].position = (t['x'], 1.6, t['z'])


# ---------------------------------------------------------------
# รถที่แทรกเข้ามา (ซ่อนไว้ก่อน จนกว่าจะกด Insert Car)
# ---------------------------------------------------------------
obstacle_car = Entity(model='cube', scale=(1.6, 1.0, 3.0), position=(-6, 0.5, 0),
                       color=color.red, enabled=False)

# ---------------------------------------------------------------
# กล้อง (fixed chase-cam มองตามหลังรถเราตลอด)
# ---------------------------------------------------------------
camera.position = (0, 3.2, -7)
camera.rotation_x = 12
window.color = SKY_DAY

# ---------------------------------------------------------------
# HUD
# ---------------------------------------------------------------
speed_text = Text(text='0 km/h', parent=camera.ui, position=(0, 0.44), origin=(0, 0),
                   scale=2.5, color=color.white)
mode_text = Text(text='DAY  |  limit 120 km/h', parent=camera.ui, position=(0, 0.36),
                  origin=(0, 0), scale=1.3, color=color.yellow)
brake_text = Text(text='BRAKE', parent=camera.ui, position=(0, 0), origin=(0, 0),
                   scale=6, color=color.red, enabled=False)


# ---------------------------------------------------------------
# ปุ่มควบคุม (mock UX แทนที่บอร์ดจริงในเฟสนี้)
# ---------------------------------------------------------------
def insert_car():
    if state['obstacle_active'] or state['sequence'] == 'BRAKE':
        return
    state['obstacle_active'] = True
    state['s'] = S_INITIAL
    state['v_lead'] = V_LEAD_DEFAULT
    obstacle_car.enabled = True
    obstacle_car.x = -6
    obstacle_car.z = state['s']
    obstacle_car.animate_x(0, duration=1.0)


def do_reset():
    if state['sequence'] != 'BRAKE':
        return
    state['sequence'] = 'RUNNING'
    state['v_ego'] = 0.0
    state['obstacle_active'] = False
    obstacle_car.enabled = False
    brake_text.enabled = False


def toggle_day_night():
    state['is_night'] = not state['is_night']
    window.color = SKY_NIGHT if state['is_night'] else SKY_DAY
    ground.color = GROUND_NIGHT if state['is_night'] else GROUND_DAY
    state['v0'] = min(state['v0'], current_v0_max())
    limit_kmh = round(current_v0_max() * 3.6)
    mode_label = 'NIGHT' if state['is_night'] else 'DAY'
    mode_text.text = f"{mode_label}  |  limit {limit_kmh} km/h"


btn_y = -0.44
speed_down_btn = Button(text='- Speed', color=color.gray, scale=(0.14, 0.06),
                         position=(-0.34, btn_y))
speed_up_btn = Button(text='+ Speed', color=color.gray, scale=(0.14, 0.06),
                       position=(-0.17, btn_y))
insert_btn = Button(text='Insert Car', color=color.orange, scale=(0.16, 0.06),
                     position=(0, btn_y), on_click=insert_car)
daynight_btn = Button(text='Day / Night', color=color.blue, scale=(0.16, 0.06),
                       position=(0.19, btn_y), on_click=toggle_day_night)
reset_btn = Button(text='Reset', color=color.violet, scale=(0.14, 0.06),
                    position=(0.36, btn_y), on_click=do_reset)


# ---------------------------------------------------------------
# สมการ IDM (Intelligent Driver Model)
#   a = A_MAX * [1 - (v/v0)^delta - (s*/s)^2]
#   s* = s0 + v*T + (v*dv) / (2*sqrt(A_MAX*B_COMFORT))
# ---------------------------------------------------------------
def idm_acceleration(v, v0, s=None, dv=0.0, T=1.0, s0=4.0):
    v0 = max(v0, 0.1)
    free_term = 1.0 - (v / v0) ** DELTA

    if s is None:
        # ไม่มีสิ่งกีดขวาง -> ใช้แค่พจน์ free-road (อยากไปให้ถึง v0)
        return A_MAX * free_term

    s = max(s, S_MIN)
    s_star = s0 + v * T + (v * dv) / (2.0 * math.sqrt(A_MAX * B_COMFORT))
    s_star = max(s_star, 0.0)
    interaction_term = (s_star / s) ** 2
    return A_MAX * (free_term - interaction_term)


# ---------------------------------------------------------------
# Loop หลัก (Ursina เรียก update() ให้เองทุกเฟรมอัตโนมัติ)
# ---------------------------------------------------------------
def update():
    dt = time.dt

    if state['sequence'] == 'RUNNING':
        # --- รับ input ปุ่ม/คีย์บอร์ด (รองรับกดค้าง) ---
        holding_up = held_keys['up arrow'] or (speed_up_btn.hovered and mouse.left)
        holding_down = held_keys['down arrow'] or (speed_down_btn.hovered and mouse.left)
        if holding_up:
            state['v0'] = min(state['v0'] + V0_STEP * dt, current_v0_max())
        if holding_down:
            state['v0'] = max(state['v0'] - V0_STEP * dt, 0.0)

        # --- คำนวณความเร่งด้วยสมการ IDM ---
        if state['obstacle_active']:
            dv = state['v_ego'] - state['v_lead']
            a = idm_acceleration(state['v_ego'], state['v0'], s=state['s'],
                                  dv=dv, T=current_T(), s0=current_s0())
        else:
            a = idm_acceleration(state['v_ego'], state['v0'])

        a = max(-B_COMFORT * 2.0, min(A_MAX, a))       # clamp กันค่าเหวี่ยงเกินจริง
        state['v_ego'] = max(0.0, state['v_ego'] + a * dt)  # integrate ความเร็ว, กันติดลบ

        # --- อัพเดตระยะห่างจากรถที่แทรกเข้ามา (ตาม closing rate จริง) ---
        if state['obstacle_active']:
            state['s'] -= (state['v_ego'] - state['v_lead']) * dt
            state['s'] = max(state['s'], S_MIN)
            obstacle_car.z = state['s']

        # --- โลกเลื่อนเข้าหากล้องตามความเร็วรถเรา (ทำให้ดูเหมือนรถเราวิ่งไปเรื่อยๆ) ---
        for t in trees:
            t['z'] -= state['v_ego'] * dt
            t['trunk'].z = t['z']
            t['top'].z = t['z']
            if t['z'] < -10:
                respawn_tree(t)

        # --- เช็คว่าต้องจบ sequence เข้าสถานะ BRAKE หรือยัง ---
        if state['obstacle_active'] and state['v_ego'] <= BRAKE_V_THRESHOLD:
            state['sequence'] = 'BRAKE'
            brake_text.enabled = True

    # --- อัพเดต HUD เสมอ ไม่ว่าจะ state ไหน ---
    speed_text.text = f"{round(state['v_ego'] * 3.6)} km/h"


app.run()
