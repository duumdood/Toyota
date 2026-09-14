# Road simulator — รับค่าจาก STM32 เท่านั้น

ไฟล์ชุดนี้แยกจาก `road_simulator.py` เวอร์ชันจำลองเดิม ไม่มีการแก้ไฟล์เดิม

## ใครทำหน้าที่อะไร

บอร์ดอ่านปุ่ม/LDR/US-100 → คำนวณ IDM, ความเร็ว, ระยะในโลกจำลอง, ความเร็วรถหน้า, day/night และ sequence → ส่ง UART → Python แสดงผล

- `../road_simulator_board.py`: ภาพ 3D แบบเดิม แต่ไม่มี slider, ปุ่มขับ หรือ keyboard shortcut ขับรถ
- `../road_board_io.py`: รับ UART, ตรวจรูปแบบ/CRC และแปลงหน่วยเพื่อแสดงผล ไม่มี IDM หรือการตัดสินกลางคืนใน Python
- `road_config.h`: ค่าปรับเทียบทั้งหมด เช่น threshold LDR, ขั้นความเร็ว, scaling ระยะ
- `road_app.h/.c`: ตัวแปร input/output และ application logic/IDM ที่ทำงานบนบอร์ด
- `road_board.h/.c`: driver GPIO/EXTI, ADC interrupt, timer capture/PWM, UART DMA
- `road_protocol.h/.c`: แปลงผลลัพธ์เป็น telemetry ที่ Python อ่านได้
- `main.c`: entry point สำหรับโปรเจกต์ CMSIS บน STM32F411RE

Python ทำเพียงแปลงหน่วยและ interpolate **ระหว่างตำแหน่งที่บอร์ดส่งมาจริง** เพื่อให้ภาพลื่น ไม่ integrate ความเร็วหรือเดาการเคลื่อนที่หลังข้อมูลขาด

## ตัวแปร input ที่ต้องรู้

ฝั่งบอร์ดมี `volatile RoadInputs g_road_inputs` ให้ดูใน debugger; driver เป็นผู้เขียน ห้ามใช้แทนช่องป้อนค่าเองในการใช้งานจริง

| ตัวแปรใน `g_road_inputs` | แหล่งข้อมูล / ความหมาย |
|---|---|
| `speed_up_pressed` | ปุ่ม PA10/D2 หลัง debounce, true ตลอดที่กดค้าง |
| `speed_down_pressed` | ปุ่ม PB3/D3 หลัง debounce |
| `reset_pressed` | ปุ่ม PB4/D5 หลัง debounce; ปุ่ม Reset แยกจาก 2 ปุ่มความเร็ว |
| `ldr_adc` | ADC1 channel 1, PA1/A1, ค่า raw 12-bit 0–4095 ไม่ใช่ lux |
| `ldr_ready`, `ldr_ms` | เคยอ่านค่าแล้วหรือยัง / เวลาที่ conversion เสร็จ |
| `distance_mm` | ระยะจริงจาก US-100 หน่วย mm ก่อนขยายเป็นระยะบนถนน |
| `range_status` | 0 = echo ถูกต้อง, 1 = ไม่พบ echo, 2 = ผิดปกติ |
| `range_ready`, `range_ms`, `range_sequence` | ความพร้อม, เวลา และหมายเลขตัวอย่างระยะ |

ผลลัพธ์อยู่ใน `g_road_output`: `speed_mps`, `target_mps`, `acceleration_mps2`, `odometer_m`, `gap_m`, `lead_speed_mps`, `night`, `braking`, `done`, `fault`, `reset_id`

ฝั่ง Python ตัวรับที่ระบุชื่อชัดเจนคือ `BoardTelemetry` ใน `road_board_io.py` เช่น `speed_mps`, `gap_m`, `night`, `ldr_adc`, `distance_mm` โดย **ค่าทุกตัวมาจากบอร์ด** ไม่ได้ตั้งค่าความเร็ว/กลางคืนจาก Python

## ขาต่อเริ่มต้น — ตรวจรุ่น shield ก่อนต่อ

| หน้าที่ | STM32F411RE | การตั้งค่า |
|---|---|---|
| เพิ่มความเร็ว | PA10 / D2 | Input pull-up, EXTI10 ทั้งสองขอบ |
| ลดความเร็ว | PB3 / D3 | Input pull-up, EXTI3 ทั้งสองขอบ |
| Reset sequence | PB4 / D5 | Input pull-up, EXTI4 ทั้งสองขอบ |
| LDR | PA1 / A1 | Analog ADC1_IN1 |
| LED แสดงกลางคืน | PA5 / D13 | GPIO output |
| US-100 ECHO | **PC6 บน Morpho** | AF2, TIM3_CH1 rising + CH2 indirect falling |
| US-100 TRIG | **PC8 บน Morpho** | AF2, TIM3_CH3 PWM 12 µs ทุก 60 ms |
| UART ไป PC | PA2 / USART2_TX | AF7 → ST-LINK Virtual COM, DMA1 Stream6 Channel4 |

US-100 ตัวที่ทดสอบใช้โหมด **ถอด jumper ด้านหลัง** (Trig/Echo), จ่าย **5 V ที่ขา VCC** และต่อ GND ร่วมกับบอร์ด PC6 เป็นขา digital แบบ FT และโค้ดปิด internal pull ไว้ ห้ามส่งแรงดัน 5 V เข้าขา analog LDR หรือขา 3V3

เลือก PC6/PC8 เพื่อไม่ใช้ PA0 ซึ่งเป็น NTC และ PC7 ซึ่งใช้กับ 7-segment บน shield ตามรูป lecture สำหรับ F411RE ตรวจ silk screen และ continuity ของ shield รุ่นจริง โดยเฉพาะข้อความ pin ทางเลือกของบอร์ดรุ่นอื่นในวงเล็บ ไม่ต้องต่ออะไรเข้าขา PA3 เพื่อสั่งงานจาก PC เพราะระบบนี้ส่งข้อมูลทางเดียว

ใช้ debugger แบบ **SWD**; ไม่เปิด SWO/JTAG ที่ PB3/PB4 ซึ่งใช้เป็นปุ่มแล้ว การเลือกขานี้อิงชื่อขา MCU ไม่อิงเลขตำแหน่ง header ที่อาจต่างรุ่น

## ปุ่มแตะ / ค้าง / Reset

- Debounce 25 ms: EXTI จับขอบ แล้ว timer ยืนยันระดับหลังนิ่งครบเวลา ไม่รอด้วย delay loop
- แตะเพิ่ม/ลด: 0.5 m/s หรือ 1.8 km/h ต่อครั้ง
- ค้าง: หลัง 350 ms เพิ่ม/ลดต่อเนื่อง 4 m/s ต่อวินาที (14.4 km/h ต่อวินาที)
- กดสองปุ่มความเร็วพร้อมกัน: เป็นกลาง ไม่เพิ่มหรือลด
- Reset กดได้ทุกเมื่อ; นับครั้งเดียวต่อการกด ไม่ reset ซ้ำระหว่างค้าง
- Reset คืนความเร็วรถเป็นศูนย์และเริ่ม sequence ใหม่ แต่เก็บ cruise target และสถานะ LDR
- **Reset ไม่ทำให้วัตถุจริงหายไป** หาก US-100 ยังวัดใกล้มาก รถอาจเบรก/จบ sequence อีก ต้องขยับวัตถุออกก่อนทดลองใหม่
- ตอน sequence จบ ปุ่มความเร็วถูกพักไว้จน Reset; ADC, sensor, UART และ LDR ยังทำงาน

## LDR แบบ “มืดจัดจึงกลางคืน”

ใช้การตัดสินแบบเปิด/ปิด ไม่มีการค่อย ๆ เปลี่ยนโหมดตามระดับแสง

ค่าเริ่มต้นใน `road_config.h`:

```c
#define ROAD_LDR_DARK_IS_HIGH 1U
#define ROAD_NIGHT_ENTER_ADC  3800U
#define ROAD_NIGHT_EXIT_ADC   3500U
#define ROAD_NIGHT_ENTER_MS   400U
#define ROAD_NIGHT_EXIT_MS    200U
```

วงจรใน lecture ADC หน้า PDF 42 วาง LDR ลง GND จึงคาดว่า **ยิ่งมืด ค่า ADC ยิ่งสูง** ค่า 3800 ต้องสูงต่อเนื่อง 400 ms จึงเป็นกลางคืน กลับกลางวันเมื่อค่า ≤3500 ต่อเนื่อง 200 ms ช่วง 3500–3800 จะรักษาโหมดเดิม ไม่กระพริบ

**3800/3500 เป็นค่าเริ่มต้นเพื่อทดลอง ไม่ใช่ threshold ที่ยืนยันจากบอร์ดจริงแล้ว** ให้ปรับดังนี้:

1. ดูเลข `ADC` ในจอหรือ `g_road_inputs.ldr_adc` ใน debugger ตอนเปิดรับแสงห้องปกติ
2. ใช้นิ้ว/วัสดุทึบแสงปิด LDR จนมืดมาก แล้วจดช่วงค่าที่อ่านได้
3. ตั้ง `ROAD_NIGHT_ENTER_ADC` ใกล้ช่วงที่ปิดมืด แต่ไม่สูงเกินค่าที่เซนเซอร์ทำได้; `ROAD_NIGHT_EXIT_ADC` ต้องต่ำกว่า entry และสูงกว่าค่าตอนแสงห้องปกติ เพื่อให้กลับกลางวันได้
4. ถ้าปิดแล้ว ADC ลดลง ให้เปลี่ยน `ROAD_LDR_DARK_IS_HIGH` เป็น `0U`; threshold จะเทียบกับ `4095-ldr_adc` แทน
5. ทดสอบว่าวางตามปกติแล้วไม่เข้ากลางคืน และค่าใกล้จุดเปลี่ยนไม่สลับถี่

ไม่คำนวณ lux เพราะไม่มีการปรับเทียบเชิงแสง เมื่อเข้า night บอร์ด cap target ที่ 100 km/h และใช้ IDM headway 1.8 s / minimum desired gap 7 m ส่วนกลางวัน 120 km/h, 1 s / 4 m รถชะลอตาม IDM ไม่กระโดดลดความเร็วปัจจุบันทันที Python แสดง popup SPEED LIMIT 100 รวม fade ประมาณ 3.4 วินาที

## ระยะและรถข้างหน้า

ค่าเริ่มต้น `ROAD_METRES_PER_SENSOR_MM = 0.05f` หมายถึง 1 cm จริง = 0.5 m บนถนนเหมือน mock เดิม ระยะในโลกถูกจำกัด 2–100 m ปรับสเกลที่บอร์ด ไม่ใช่ Python

บอร์ดประมาณ `lead_speed = ego_speed + d(gap)/dt` จากตัวอย่างระยะทุก 60 ms พร้อมกรองอนุพันธ์; ครั้งแรก/การกระโดดไกลยังไม่อ้างว่ารู้ความเร็ว จะแสดง `EST --` ชั่วคราว ค่าติดลบแปลว่าเคลื่อนเข้าหารถเรา ไม่ได้บังคับให้เป็นศูนย์

ถ้าถือวัตถุคงระยะอยู่ ความเร็วที่ตีความในโลกจำลองของรถหน้าจะใกล้เคียงรถเรา นี่คือการจำลอง **relative distance** ไม่ใช่การวัดความเร็วรถจริงด้วยเซนเซอร์เดี่ยว

## ใส่ C ใน STM32CubeIDE

1. สร้าง **โปรเจกต์ใหม่** สำหรับ STM32F411RET6 / NUCLEO-F411RE ภาษา C แบบ Empty/CMSIS เพื่อไม่ทับ lab เดิม
2. เก็บ startup assembly ของ F411RE, `system_stm32f4xx.c`, CMSIS device/core headers และ linker script FLASH 512 KB / RAM 128 KB ของ ST ไว้
3. ใช้ `main.c` ชุดนี้แทน main ที่สร้างมา เพิ่ม `.c/.h` อีกทั้งหมดในโฟลเดอร์นี้เข้า build และเพิ่มโฟลเดอร์ใน include path
4. Define `STM32F411xE`; CPU Cortex-M4, `-mthumb`, FPU `fpv4-sp-d16`, float ABI `hard`, C11 ใช้ตัวเลือกเดียวกันทุกไฟล์
5. ใช้ clock reset **HSI 16 MHz**, AHB/APB1/APB2 หาร 1 ไม่เรียก `SystemClock_Config()`, `HAL_Init()` หรือ `MX_*` ซ้ำกับ driver นี้
6. อย่าให้มี handler ชื่อเดียวกันซ้ำใน `stm32f4xx_it.c`: `EXTI3_IRQHandler`, `EXTI4_IRQHandler`, `EXTI15_10_IRQHandler`, `ADC_IRQHandler`, `TIM3_IRQHandler`, `TIM4_IRQHandler`, `DMA1_Stream6_IRQHandler` ให้ใช้ implementation ใน `road_board.c` เพียงชุดเดียว เก็บ fault handlers ของ startup/project ได้
7. Build ก่อน flash ตรวจขาต่อและโหมด US-100 แล้วจึง flash โดยผู้ใช้ การทดสอบในงานนี้ **ไม่ได้แฟลชบอร์ด**

`main()` นอนด้วย `__WFI()`; TIM4 interrupt ทุก 1 ms จัด debounce และเรียก application ทุก 10 ms, ADC EOC interrupt เก็บ LDR, TIM3 ใช้ hardware PWM/capture วัด US-100, UART ส่งด้วย DMA ทุก 50 ms ไม่มี polling รอ EOC/TXE/echo และไม่มี blocking delay ใน control loop

หากนำไปผสมกับโปรเจกต์ HAL เดิม ต้องปรับ ownership ของ peripheral, clock และ IRQ ให้ตรงกันก่อน ไม่ใช่คัดลอก `main.c` ทับแล้วเรียก initialization ทั้งสองชุด

## เปิด Python

รันจากโฟลเดอร์ `codes` โดยมี `road_simulator_board.py` และ `road_board_io.py` อยู่ด้วยกัน:

```powershell
python -m pip install ursina pyserial
python road_simulator_board.py --list-ports
python road_simulator_board.py --port COM5
```

เปลี่ยน COM5 เป็น ST-LINK Virtual COM ของเครื่องจริง หรือกำหนด `SERIAL_PORT` ใน `road_board_io.py` ไม่เลือก port อัตโนมัติเพื่อเลี่ยงการเปิดอุปกรณ์อื่น ปิด CoolTerm/Serial Monitor ที่จับพอร์ตเดียวกันก่อน

เมื่อยังไม่มีข้อมูล จะแสดง WAITING FOR BOARD และ `--`; หากข้อมูลขาดเกิน 0.5 s จะหยุดเลื่อนโลกและแจ้ง timeout/disconnected มีการลองเชื่อมใหม่ ไม่ fallback ไปสร้างค่า mock

## UART contract (C encoder ↔ Python parser)

115200 baud, 8 data bits, no parity, 1 stop bit, no flow control, ประมาณ 20 frames/s; ส่งบอร์ด → PC ทางเดียว

```text
RD1,seq,ms,v_mmps,target_mmps,a_mmps2,odo_mm,gap_mm,lead_mmps,flags,ldr_adc,distance_mm,reset_id,reason,range_status*CRC32\n
```

- ทุก field หลัง RD1 เป็นเลขจำนวนเต็ม; หน่วยความเร็ว mm/s, acceleration mm/s², ระยะ mm
- `gap_mm=-1` = ไม่มีเป้าหมาย; `distance_mm=-1` = ไม่มีระยะจริงที่ใช้ได้
- `flags` bit0 night, bit1 present, bit2 lead estimate valid, bit3 braking, bit4 sequence done, bit5 sensor fault
- `reason`: 0 running, 1 stop complete, 2 deceleration complete; `range_status`: 0 valid echo, 1 no return, 2 invalid/not ready
- CRC32 IEEE reflected polynomial `0xEDB88320`, initial/final xor `0xFFFFFFFF`, 8 hex digits; คิดเฉพาะ ASCII ตั้งแต่ R ถึง field สุดท้ายก่อน `*` ตรงกับ `zlib.crc32(body)`
- ห้ามแทรก debug printf กลาง frame; Python ทิ้ง frame ที่เสีย/ผิด version/เกินขอบเขต และไม่ใช้ packet ซ้ำเพื่อยืดเวลาความสดของข้อมูล

## ขอบเขตการทดสอบและข้อจำกัด

- มีการ compile C ด้วย ARM GCC ของ CubeIDE และ CMSIS headers จริง พร้อม warnings เป็น errors
- `../tests/board_core_tests.c` ทดสอบ C จริงบน ARM emulator: IDM, tap/hold, LDR dwell/hysteresis, cap, stop/settle/reset, stale sensor, อนุพันธ์ และ encoder
- `../tests/run_board_tests.py` รับ frame จาก C ที่รันจริงไปตรวจ parser/CRC/หน่วย และการรับทีละส่วน/ข้อมูลเสีย/timeout
- `../tests/preview_board.py` ตรวจภาพแบบ offscreen ด้วย packet สำหรับทดสอบเท่านั้น ไม่เปิด serial จริงและไม่เปิดช่อง mock ในโปรแกรมใช้งาน
- **ยังไม่ยืนยันบนฮาร์ดแวร์**: polarity/ช่วง ADC จริง, echo timing/noise, pin routing ของ shield, ความแม่น clock, ISR latency, DMA/USB และ baud error ต้อง bench-test ก่อน demo
- เมื่อ ADC/ระยะ stale หรือ echo invalid ระบบจะ freeze simulation และส่ง fault ไม่เร่งต่อจากข้อมูลที่ผิด
- **ไม่พบ echo ไม่สามารถแยกได้ว่าไม่มีวัตถุหรือสาย sensor หลุด** โหมดนี้รายงาน NO RETURN และตีความว่าไม่มีเป้าหมายสำหรับ demo; ต้องตรวจการต่อ/ทดสอบด้วยวัตถุที่รู้ระยะก่อนเริ่ม ส่วน ADC ที่ค้างแต่ยังได้ conversion ก็ไม่สามารถวินิจฉัยสายขาดได้ครบ
- ไม่ใช่ซอฟต์แวร์ควบคุมเบรก/รถจริง และยังไม่ได้ผ่าน MISRA checker หรือการรับรอง MISRA-C 22 ข้อ ใช้ fixed-size buffers, explicit types และแยก layers เพื่อเตรียมตรวจต่อ ไม่กล่าวอ้าง compliance ที่ยังไม่ได้ทดสอบ

## อิง lecture และเอกสาร

- `academy source/0100_GPIO.pdf`: Lab GPIO input pull-up และการตั้ง register; PB4 pressed = LOW ตาม PDF หน้า 51/54
- `academy source/0300_UART_noTimer.pdf`: USART2, PA2 AF7, BRR ที่ 16 MHz/115200; เปลี่ยนส่วนรอส่งแบบ polling เป็น DMA ตาม requirement โปรเจกต์
- `academy source/0400_ADC.pdf`: analog input, channel sequence และวงจร LDR (PDF หน้า 42)
- `academy source/0500_Interrupts.pdf`: CMSIS, NVIC, ชื่อ IRQ handler, ADC EOC และ EXTI; นำไปแยก driver/application ไม่ใส่ logic ลง main busy loop
- ระวัง typo ในตาราง ADC lecture: analog mode ต้องเป็น `11`, single conversion ใช้ SQR1.L=`0` (N−1), SWSTART อยู่ CR2; โค้ดนี้ใช้ค่าตาม peripheral definition ไม่คัดลอก typo
- [ST STM32F411 datasheet, alternate functions PC6/PC8 และข้อมูล MCU](https://www.st.com/resource/en/datasheet/stm32f411re.pdf)
- [ST RM0383 reference manual](https://www.st.com/resource/en/reference_manual/dm00119316.pdf)
- [ST official CMSIS device headers](https://github.com/STMicroelectronics/cmsis-device-f4)
- [US-100 Trig/Echo mode และ jumper](https://learn.adafruit.com/ultrasonic-sonar-distance-sensors/python-circuitpython)

เอกสาร context เดิมกล่าวถึง HC-SR04 และ Reset เฉพาะตอน BRAKE แต่คำขอล่าสุดใช้ US-100 และ Reset ได้ตลอด ชุดนี้ยึดคำขอล่าสุด ไม่ย้อนกลับไปใช้ข้อกำหนดเก่า
