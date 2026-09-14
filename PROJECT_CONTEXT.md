# Project Context — STM32F411 Final Project (Embedded Systems)

> ไฟล์นี้สรุปทุกการตัดสินใจจากแชทวางแผนโปรเจกต์ (Claude.ai) เพื่อใช้เป็น context
> ตอนย้ายไปทำงานต่อใน Claude Code / VSCode ให้แปะไฟล์นี้ไว้ในโฟลเดอร์โปรเจกต์
> (หรือใช้เป็น `CLAUDE.md`) แล้วให้ Claude Code อ่านก่อนเริ่มงาน

---

## 1. โจทย์วิชาและเกณฑ์การประเมิน

**โจทย์**: ออกแบบและพัฒนา Application บนบอร์ด STM32 ภายใน 1 เดือน เลือกหัวข้อได้อิสระ

### Component requirement (บังคับ)

| # | Component | เงื่อนไข |
|---|---|---|
| 1 | GPIO | ใช้เป็น Input หรือ Output |
| 2 | UART | ใช้ Interrupt หรือ DMA เท่านั้น ห้ามใช้ Polling |
| 3 | ADC | ใช้ Interrupt หรือ DMA เท่านั้น ห้ามใช้ Polling |
| 4 | External Interrupt | อย่างน้อย 1 จุด |
| 5 | Additional Peripheral | เลือกอย่างน้อย 1: Watchdog, Timer, I2C, SPI, I2S, CRC |

### เกณฑ์การประเมิน

1. Component ข้างบนครบ
2. **MISRA-C** อย่างน้อย 22 ข้อ (ยังไม่ได้ลงรายละเอียด ต้องทำตอนเขียนโค้ดจริง)
3. **Software Structure** — ต้องแยก Application กับ Driver layer อย่างชัดเจน
4. **Innovate/Creativity** — ความน่าสนใจ ความแปลกใหม่ ความคิดสร้างสรรค์

### Week 1 deliverables (ต้องส่ง)
ชื่อโปรเจกต์, Features, Block diagram, เหตุผลที่เลือก (เขียนสั้นกระชับ), Timeline

### ข้อควรระวังจากโน้ตอาจารย์
- ห้าม Polling เด็ดขาด ต้องใช้ Interrupt
- เลือก component ให้เหมาะสมกับงาน ไม่ยัดเข้าไปเกินจำเป็น ต้องคิดด้วยว่า interrupt ใช้ตรงไหนสมเหตุสมผล
- คำนึงเรื่อง security ด้วย (ถ้าเกี่ยวข้อง)
- **อาจารย์เน้นความซับซ้อน (complexity) ไม่ใช่ปริมาณฟีเจอร์** — โปรเจกต์ที่ดูฉลาดแต่ implementation เบาเกินไปจะไม่ผ่านเกณฑ์นี้
- ไม่อยากเห็นบั๊กเยอะ/ซับซ้อนเกินจำเป็นจนควบคุมไม่ได้

---

## 2. Hardware ที่มีอยู่แล้ว (ไม่ต้องซื้อ)

### บอร์ดหลัก
**STM32F411RE (Nucleo-F411RE)** — Cortex-M4F @ 100MHz, มี FPU, DSP instructions, Flash 512KB, SRAM 128KB
- **ไม่มี DAC ในตัว**, **ไม่มี DCMI** (camera interface) — ยืนยันแล้วว่าไม่มีในชิปรุ่นนี้ (มีเฉพาะ F407/F429/F7 ขึ้นไป)
- มี USB OTG FS, I2S, ADC 12-bit เร็ว, Timer หลายตัว, DMA หลาย channel, CRC hardware, ST-LINK/V2-1 ในตัว

### Training Shield 1 (Toyota Tsusho Nexty) — ต่อบน Nucleo แล้ว

| อุปกรณ์ | Pin | ประเภท |
|---|---|---|
| Photoresistor (LDR) | A1 / PA1 | Analog (ADC) |
| BH1750 (วัดความสว่าง lux) | I2C header | Digital (I2C) |
| AHT10 (อุณหภูมิ+ความชื้น) | I2C header | Digital (I2C) |
| Potentiometer | PA4 | Analog (ADC) — **ปัจจุบันไม่ได้ใช้ในดีไซน์ล่าสุด** (ดูหัวข้อ 5) |
| LED สีน้ำเงิน | D13 / PA5 | GPIO/PWM |
| LED สีแดง | D12 / PA6 | GPIO/PWM |
| LED สีเหลือง | D11 / PA7 | GPIO/PWM |
| LED สีเขียว | D10 / PB6 [PC9] | GPIO/PWM |
| ปุ่มกด x4 | D2/PA10, D3/PB3, D4/PB5, D5/PB4 | GPIO/EXTI |
| 7-Segment (1 หลัก) | D7/PA8, D6/PB10, D8/PA9 (ผ่าน 3-bit decoder) | Output |

### 37-in-1 Sensor Kit
มีเซนเซอร์หลากหลาย (ส่วนใหญ่เป็น digital comparator ธรรมดา) — **"Avoid" (IR proximity) ในกล่องนี้เป็น digital ON/OFF เท่านั้น ไม่เหมาะกับโปรเจกต์นี้เพราะต้องการระยะต่อเนื่อง**

### เครื่องมือ
STM32CubeIDE, CoolTerm, serialterminal.com

---

## 3. Hardware ที่ต้องซื้อเพิ่ม

| อุปกรณ์ | เหตุผล | ราคาประมาณ |
|---|---|---|
| **HC-SR04 (Ultrasonic sensor)** | วัดระยะต่อเนื่องจริง ไม่มีในบอร์ด/กล่องที่มี | 35-60 บาท |

**ไม่ต้องซื้อ servo/motor** — ตัดออกแล้ว (ดูหัวข้อ 4) การเลี้ยว/เบรกทั้งหมดแสดงผลใน PC simulator ไม่ใช่ actuator จริงบนบอร์ด

---

## 4. Concept โปรเจกต์ (เวอร์ชันล่าสุดที่ตกลงแล้ว)

**ชื่อแนวคิด**: ระบบเบรกอัตโนมัติแบบปรับตาม IDM (Intelligent Driver Model) พร้อม PC 3D simulator

**หลักการ**: บอร์ด STM32 อ่านค่าจากเซนเซอร์จริง (ระยะวัตถุจาก ultrasonic, ความสว่างจาก LDR) และปุ่มควบคุมความเร็ว คำนวณพฤติกรรมการเร่ง/เบรกด้วยสมการ IDM แล้วส่งผลลัพธ์ไปแสดงเป็นภาพรถขับบนถนน 3D สไตล์ low-poly บนจอ PC ผ่าน UART

### วิวัฒนาการของไอเดีย (สรุปสั้นๆ เพื่อไม่ให้เสนอซ้ำ)
ลองมาหลายทาง: Optical Theremin (ตัดเพราะ logic เบาไป, แค่ linear mapping), เกมต่างๆ (Nim, Minimax, ฯลฯ — เป็นทางเลือกสำรอง ไม่ได้เลือก), IMU sensor fusion (ทางเลือกสำรอง), ระบบเลี่ยงสิ่งกีดขวาง 4 ทิศทาง+เลี้ยวอัตโนมัติ (ตัดออกเพราะ ultrasonic 4 ตัวยิงพร้อมกันมีปัญหา echo cross-talk + scope ใหญ่เกินไปใน 1 เดือน) → **สุดท้ายลงตัวที่ single-sensor Auto-Brake ด้วย IDM equation** เพราะสมดุลระหว่างความซับซ้อนทางวิชาการกับความเป็นไปได้ดีที่สุด

### ทำไมใช้ IDM แทน threshold/TTC ธรรมดา
IDM (Treiber, Hennecke, Helbing 2000) เป็นโมเดล car-following ที่ใช้จริงในงานวิจัยจราจรและระบบ ACC จริง ไม่ใช่ heuristic ที่คิดเอง — ให้ความน่าเชื่อถือทางวิชาการสูงกว่า if-else/threshold ธรรมดามาก และมีความซับซ้อนทางคณิตศาสตร์แท้จริง (ODE integration, สองพจน์แข่งกัน)

---

## 5. Requirement mapping (เวอร์ชันสุดท้าย — ครบทุกข้อแล้ว)

| Requirement | Peripheral | รายละเอียด |
|---|---|---|
| GPIO | ปุ่ม 3 ปุ่ม (input) | Speed+, Speed-, Reset |
| UART (DMA) | ส่ง telemetry ไป PC | ส่งทางเดียว (board→PC): speed, brake state, ฯลฯ |
| ADC (DMA) | **LDR** (ไม่ใช่ potentiometer) | วัดความสว่าง → day/night mode |
| EXTI ≥1 | ปุ่ม 3 ปุ่ม | เกินพอ |
| Additional Peripheral | **Timer** (2 หน้าที่) | (1) Input Capture วัด echo pulse จาก HC-SR04, (2) Timer แยกสำหรับ repeat-increment ตอนกดค้างปุ่มความเร็ว |

**หมายเหตุสำคัญ**: Potentiometer (PA4) เคยถูกเสนอให้ทำหน้าที่ต่างๆ หลายรอบ (ตั้ง v0, ตั้งความไวการเบรก) แต่ **ในดีไซน์สุดท้ายไม่ได้ใช้ potentiometer เลย** — ผู้ใช้เปลี่ยนมาใช้ปุ่มกดสำหรับตั้งความเร็วแทน และ ADC requirement ถูกเติมเต็มด้วย LDR แทน ถ้า Claude Code เห็นโค้ดเก่าที่อ้างถึง potentiometer ให้รู้ว่าเป็นแนวคิดที่ถูกแทนที่แล้ว

---

## 6. สมการ IDM (Intelligent Driver Model)

อ้างอิง: Treiber, Hennecke, Helbing (2000)

```
a = a_max * [ 1 - (v/v0)^δ - (s*/s)² ]

s* = s0 + v*T + (v*Δv) / (2*sqrt(a_max*b))
```

| ตัวแปร | ความหมาย |
|---|---|
| `v` | ความเร็วปัจจุบันของรถเรา (state, integrate สะสมทุก tick: `v += a*dt`) |
| `v0` | ความเร็วที่ต้องการวิ่ง (ตั้งจากปุ่ม Speed+/-, ถูก cap ตาม day/night) |
| `s` | ระยะห่างจากวัตถุ/รถข้างหน้า (จาก ultrasonic) |
| `Δv` | อัตราการเข้าใกล้ = `v - v_lead` (v_lead = ความเร็วของรถ/วัตถุข้างหน้า) |
| `s*` | ระยะที่ "ต้องการ" ในสถานการณ์นั้น (ไม่คงที่) |
| `a_max, b, T, s0, δ` | ค่าคงที่ปรับพฤติกรรม |

### ค่าพารามิเตอร์ที่ใช้ใน mock (Python) ตอนนี้

```
A_MAX = 3.0            # m/s^2
B_COMFORT = 3.5        # m/s^2
DELTA = 4
S_MIN = 2.0            # m (กันหารด้วยค่าใกล้ศูนย์)

T_DAY = 1.0 s      T_NIGHT = 1.8 s
S0_DAY = 4.0 m     S0_NIGHT = 7.0 m

V0_MAX_DAY = 120 km/h     V0_MAX_NIGHT = 100 km/h
V0_STEP = 4.0 m/s ต่อวินาทีตอนกดค้าง

V_LEAD_DEFAULT = 30 km/h   # ความเร็วรถที่แทรกเข้ามา (คงที่ ไม่ใช่หยุดนิ่ง)
S_INITIAL = 90 m           # ระยะเริ่มต้นตอนแทรกรถ
BRAKE_V_THRESHOLD = 0.3 m/s
```

### ข้อควรระวัง (well-posedness) — ต้อง clamp เสมอ

| ความเสี่ยง | วิธีป้องกัน |
|---|---|
| `v` ติดลบ | `v = max(v, 0)` |
| หารด้วย `s` ใกล้ 0 | `s = max(s, S_MIN)` |
| `a` แกว่งเกินจริง | `a = clamp(a, -B_COMFORT*2, A_MAX)` |

**คุณสมบัติที่ดีของ IDM**: ถ้า `v0` ลดลงกะทันหัน (เช่น เข้าโหมดกลางคืน) ระบบจะชะลอให้เองอัตโนมัติจากพจน์ `1-(v/v0)^δ` ที่ติดลบเมื่อ `v > v0` — ไม่ต้องเขียน logic บังคับชะลอแยก

### Scaling ระยะจริง (ยังไม่ได้ implement ในเฟสนี้)
เมื่อจะต่อ HC-SR04 จริง ระยะที่วัดได้จะสั้นกว่าระยะที่อยากจำลอง (HC-SR04 วัดได้จริงไม่เกิน ~4m) ต้อง**scale ค่าที่วัดได้จริงให้เป็นระยะที่ใหญ่ขึ้นในโลกจำลอง** (เช่น "10cm จริง = 1m จำลอง") — อัตราส่วนที่แน่นอนยังไม่ได้ตกลง ต้องทดสอบตอนต่อฮาร์ดแวร์จริง

---

## 7. State design

ระบบมีแค่ 2 state จริง: **RUNNING** และ **BRAKE**

- **RUNNING**: ทำงานปกติ คำนวณ IDM ทุก tick, รับ input ปุ่ม, โลกเลื่อนตามความเร็ว
- **BRAKE**: เข้าสถานะนี้เมื่อ **มีสิ่งกีดขวางอยู่ (`obstacle_active=True`) และ `v` ลดลงจนต่ำกว่า `BRAKE_V_THRESHOLD`** เท่านั้น — ถ้าผู้ใช้กด Speed- เองจนความเร็วเป็น 0 โดยไม่มีสิ่งกีดขวาง **ไม่นับเป็น BRAKE** (แยกเคสนี้ไว้ชัดเจนแล้ว)
- ตอน BRAKE: freeze ทุกอย่าง (โลกไม่เลื่อน, ปุ่มความเร็วไม่ทำงาน), แสดงข้อความ "BRAKE" กลางจอ, รอกด **Reset** เท่านั้น (Reset ใช้ได้เฉพาะตอน BRAKE)
- Reset: `v` กลับเป็น 0, ลบสิ่งกีดขวางออก, กลับสถานะ RUNNING — **`v0` (ความเร็วเป้าหมาย) ไม่ถูกรีเซ็ต** ทำให้รถเร่งกลับไปที่ความเร็วเดิมอัตโนมัติหลัง reset (IDM free-road term ทำงานเอง)

**หมายเหตุ**: มีการออกแบบ state machine แบบซับซ้อนกว่านี้ในช่วงต้น (ACC off / Free road / Following พร้อมปุ่มเปิด-ปิด ACC) จากตอนที่ยังพิจารณาไอเดีย 4-ทิศ — **เวอร์ชันนั้นถูกแทนที่แล้ว** ดีไซน์ปัจจุบันไม่มีปุ่มเปิด-ปิดระบบแยก ระบบทำงานตลอดเวลา

### พฤติกรรมปุ่ม (ตัดสินใจแล้ว)
- **Speed+/Speed-**: แบบ**กดค้าง** (hold) ไม่ใช่กดย้ำทีละครั้ง — เลือกเพราะทนต่อปัญหา button bounce ได้ดีกว่า (ไม่ต้องนับ edge แม่นเป๊ะ)
- **Reset**: gate ด้วยซอฟต์แวร์ ทำงานเฉพาะตอน `sequence == BRAKE`

### พฤติกรรม Day/Night (LDR)
มืด → เข้าโหมดกลางคืน → ส่งผล 2 อย่าง: (1) ลด `v0_max` cap, (2) เพิ่ม `T`/`s0` ทำให้เริ่มเบรกไวขึ้น/ไกลขึ้น — ไม่มีปุ่ม calibrate แยก ใช้ threshold คงที่ที่ปรับจากการทดสอบจริงก่อน demo

---

## 8. Software Architecture (แผนสำหรับโค้ด embedded C)

```
Driver layer (ติดต่อ peripheral ตรงๆ, ไม่มี logic ตัดสินใจ):
  hcsr04_driver.c/h    — Timer Input Capture อ่าน echo pulse
  ldr_driver.c/h       — ADC + DMA อ่านความสว่าง
  buttons_driver.c/h   — EXTI + Timer สำหรับ repeat ตอนกดค้าง
  uart_driver.c/h      — DMA TX ส่ง telemetry

Application layer (logic/คำนวณทั้งหมดอยู่ที่นี่):
  idm_model.c/h        — สมการ IDM, state ของ v, clamp
  sequence_state.c/h   — RUNNING/BRAKE state, reset gating
```

**ยังไม่ได้เขียนโค้ด C จริง** — มีแค่ mock ฝั่ง Python (ดูหัวข้อ 9)

**MISRA-C**: ยังไม่ได้ลงรายละเอียดกฎ ต้องทำตอนเขียนโค้ดจริง (เป้าหมาย ≥22 ข้อ)

---

## 9. PC Simulator (สถานะปัจจุบัน — Mock phase)

### เทคโนโลยีที่เลือก
**Python + Ursina Engine** (`pip install ursina`) — 3D game engine บน Panda3D, API หน้าตาคล้าย Unity (Entity-based), เหมาะกับ low-poly style, ติดตั้งง่าย

เหตุผลที่เลือก Python เหนือ HTML/Web Serial API: อ่าน UART ง่ายกว่ามากด้วย `pyserial`, ไม่ต้องกังวล browser permission

**Monitor**: จอเดียว (เคย misunderstand ว่าเป็น dual monitor — แก้ไขแล้ว เป็นแค่จอสำหรับ simulator ตัวเดียว)

### สถาปัตยกรรมปัจจุบัน — Mock phase (สำคัญ)
**ตอนนี้ Python คำนวณ IDM เอง** (ตามที่ตกลงไว้) เพื่อทดสอบพฤติกรรมภาพก่อน — **แผนคือย้ายการคำนวณ IDM ไปที่ MCU ทีหลัง** เมื่อนั้น Python จะแค่รับค่า (speed, brake state) ผ่าน UART มาแสดงผล ไม่คำนวณเองแล้ว

### เทคนิคการ render
รถเราอยู่นิ่งที่ตำแหน่ง origin เสมอ (fixed chase-cam) — **โลกทั้งหมด (ต้นไม้, รถที่แทรกเข้ามา) เคลื่อนที่เข้าหากล้องแทน** ด้วยอัตราเท่าความเร็วปัจจุบันของรถเรา ต้นไม้ที่เลื่อนผ่านไปข้างหลังจะถูก respawn ไปข้างหน้าใหม่ (loop) ถนน/พื้นเป็น plane ขนาดใหญ่คงที่ ไม่ต้อง loop

### รถที่แทรกเข้ามา (obstacle/lead car)
มีความเร็วของตัวเอง (`V_LEAD_DEFAULT`) ไม่ใช่หยุดนิ่งสนิท — ตามที่ตัดสินใจไว้ ระยะ `s` ลดลงตาม `(v_ego - v_lead)*dt` ทุกเฟรม ใช้ค่า `s` นี้เป็นทั้งค่าฟิสิกส์และตำแหน่ง render ของรถที่แทรกเข้ามาโดยตรง

### 4 ปุ่ม Mock UI (แทนที่ input จากบอร์ดจริงชั่วคราว)
1. **Insert Car** — รถแทรกเข้าเลนแบบ smooth (animate slide), ตั้ง `s = S_INITIAL`, `v_lead = V_LEAD_DEFAULT`
2. **Reset** — ใช้ได้เฉพาะตอน BRAKE
3. **Day/Night** — สลับโหมด เปลี่ยนสีฉากด้วย
4. **Speed +/-** — กดค้างเพิ่ม/ลด `v0` (ผูกกับลูกศรขึ้น-ลงบนคีย์บอร์ดด้วย)

### HUD
ความเร็ว (km/h) บนสุดจอเสมอ, label โหมด day/night + speed limit, ข้อความ "BRAKE" กลางจอ (แสดงเฉพาะตอน BRAKE state)

### ไฟล์ที่ส่งมอบแล้ว
`road_simulator.py` — เช็คแล้ว: syntax ผ่าน (`py_compile`), ติดตั้ง ursina สำเร็จ, ชื่อ API ทุกตัวมีอยู่จริง (`Entity`, `Button`, `Text`, `color.rgb`, `animate_x`, `held_keys`, `mouse`, `camera`, `window` — เช็คแบบ import ไม่ error)
**ยังไม่เช็ค**: การรันแสดงผลจริง (ไม่มีจอ/GPU ใน sandbox ที่ใช้พัฒนา) — **ผู้ใช้ต้องรันทดสอบเองบนเครื่องจริงเป็นครั้งแรก**

จุดเสี่ยงที่อาจต้องปรับหลังรันจริง: ตำแหน่ง/มุมกล้อง, ตำแหน่งปุ่มที่ขอบจอ

---

## 10. สิ่งที่ยังไม่ได้ทำ (Next steps)

1. รัน `road_simulator.py` บนเครื่องจริง ทดสอบ/tune พฤติกรรม IDM ให้ดูดีก่อน (ปรับค่าพารามิเตอร์ถ้าจำเป็น)
2. เขียนโค้ด C ฝั่ง STM32 ตามโครง Driver/Application layer ในหัวข้อ 8
3. ย้ายสมการ IDM จาก Python ไปเป็น C บน MCU (`idm_model.c`)
4. ต่อ HC-SR04 จริง วัด echo ด้วย Timer Input Capture, คิดอัตราส่วน scaling ระยะ
5. แก้ Python simulator ให้เปลี่ยนจากคำนวณ IDM เอง เป็นแค่รับค่าผ่าน UART (pyserial) มาแสดงผลแทน
6. ตัดสินใจ/implement MISRA-C compliance (≥22 ข้อ) ในโค้ด C
7. เตรียมงาน Week 1: ชื่อโปรเจกต์อย่างเป็นทางการ, features list, block diagram, เหตุผลการเลือก, timeline
8. Calibrate threshold ของ LDR (มืด/สว่าง) จากสภาพห้องจริงก่อน demo

---

## 11. คะแนนประเมินล่าสุด (ก่อนเริ่ม PC simulator)

| ด้าน | คะแนน |
|---|---|
| ความซับซ้อน | 8/10 (IDM + LDR ปรับ parameter แบบ context-aware) |
| Feasibility 1 เดือน | 8/10 |
| ความแข็งแรง | 8/10 (ไม่มีกลไกเสี่ยง ตัด servo ออกแล้ว) |
