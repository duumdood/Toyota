# MISRA-C Checksheet — IDM Auto-Brake (STM32F411RE)

โฟลเดอร์นี้คือ source เวอร์ชันที่แก้ให้ตรง Guideline Checksheet 22 ข้อของบริษัท
ไฟล์ต้นฉบับยังอยู่ที่ `../stm32_road/` ไม่ถูกแก้ไข

**พฤติกรรมเหมือนเดิมทุกอย่าง** — แก้เฉพาะรูปแบบโค้ด ไม่ได้เปลี่ยน logic
(ยืนยันด้วยการทดสอบด้านล่าง)

## ไฟล์ในโฟลเดอร์

| ไฟล์ | สถานะ |
|---|---|
| `main.c` | แก้ |
| `road_config.h` | แก้ — เพิ่ม named constant ทั้งหมดของ application |
| `road_app.h` | แก้ — แยก declaration ทีละบรรทัด (field เดิม ลำดับเดิม) |
| `road_app.c` | แก้ + แตก `RoadApp_Tick` เป็นฟังก์ชันย่อย (one role = one function) |
| `road_board.c` | แก้ + แตก `RoadBoard_Init` เป็น `init_*` ตาม peripheral |
| `road_protocol.c` | แก้ |
| `road_board.h`, `road_protocol.h` | ไม่ได้แก้ (ผ่านอยู่แล้ว) ใส่ไว้ให้ครบชุด copy ไปใช้ได้ทันที |

## ตาราง 22 ข้อ

| # | Rule | ผล | หลักฐานในโค้ด |
|---|---|---|---|
| 1 | ห้ามใช้ comment `//` | ✅ | ทุกไฟล์ใช้ `/* */` เท่านั้น |
| 2 | `{` `}` ห้ามอยู่บรรทัดเดียวกับ statement | ✅ | เดิม 38 จุด เช่น `if (...) { return 0U; }` → แยกบรรทัดทั้งหมด (ยกเว้น `} else {` ตาม exception) |
| 3 | เว้นวรรค 1 ช่องรอบ operator | ✅ | เช่น `gap-app->previous_gap` → `gap - app->previous_gap`, `i<3U` → `i < BUTTON_COUNT` |
| 4 | หนึ่ง assignment ต่อหนึ่งบรรทัด | ✅ | เช่น `done = braking = did_brake = false;` → 3 บรรทัด (`road_app.c` `reset_sequence`), `TIM3->EGR = ...; TIM3->SR = 0U;` → แยก |
| 5 | ห้าม magic number | ✅ | ค่าคาลิเบรตอยู่ใน `road_config.h` (เช่น `ROAD_IDM_A_MAX`, `ROAD_RATE_FILTER_TAU_S`), ขา/บิต register อยู่ต้น `road_board.c` (เช่น `PIN_SPEED_UP`, `OC_MODE_PWM1`, `SOUND_SPEED_M_S`), flag/CRC อยู่ต้น `road_protocol.c` (`FLAG_NIGHT`, `CRC_POLY_REFLECTED`) |
| 6 | หลีกเลี่ยง ternary `?:` | ✅ | เดิม 37 จุด → 0 จุด ใช้ `if/else` หรือ `#if` (เช่น `pin_pressed`, `update_light`) |
| 7 | ห้าม unreachable code | ✅ | ไม่มีโค้ดหลัง `return`; `main.c` แยก fail-trap ไว้ใน `if` |
| 8 | ห้าม object ที่ไม่ได้ใช้ | ✅ | คอมไพล์ `-Wall -Wextra -Werror` ไม่มี unused warning |
| 9 | ห้ามใช้ octal | ✅ | ไม่มีเลขขึ้นต้นด้วย 0 (นอกจาก `0U`) |
| 10 | ค่าคงที่ unsigned ต้องมี `U` | ✅ | ทุก unsigned constant มี `U` เช่น `ROAD_ECHO_PERIOD_US 60000U` |
| 11 | function prototype พร้อมชื่อ parameter | ✅ | ทุก static function มี prototype ด้านบนไฟล์, ไม่มี parameter ใช้ `(void)` |
| 12 | ห้ามอ่านตัวแปร local ก่อนกำหนดค่า | ✅ | ตัวแปร local ทุกตัว init ตอนประกาศ เช่น `float result = 0.0f;` |
| 13 | ชนิดข้อมูลต้องเหมาะกับ operator | ✅ | `bool` ใช้กับ `!` `&&` `\|\|` เท่านั้น, ไม่เอา bool ไปคำนวณ (flags สร้างด้วย `if` ใน `build_flags`), ตัวเลขฐานสิบใช้ lookup table แทน `'0' + n`, shift ใช้ unsigned |
| 14 | วงเล็บแสดงลำดับ operator | ✅ | เช่น `((rate - app->range_rate) * dt) / (ROAD_RATE_FILTER_TAU_S + dt)`, `(speed * headway) + ((speed * closing) / ...)` |
| 15 | ห้ามเทียบ float ด้วย `==`/`!=` | ✅ | ใช้เฉพาะ `<` `>` `<=` `>=` (ใน firmware ไม่มี float `==`) |
| 16 | loop counter ห้ามเป็น float | ✅ | loop ทุกตัวใช้ `uint32_t`/`size_t` |
| 17 | เงื่อนไข if/while ต้องเป็น boolean | ✅ | เช่น `if ((status & TIM_SR_UIF) != 0U)`, `while (rest != 0U)` |
| 18 | body ของ if/else/for/while ต้องมี `{ }` | ✅ | ทุก body มีปีกกา |
| 19 | if / else if ต้องปิดด้วย else | ✅ | ทุก `if` มี `else { /* ... */ }` พร้อมเหตุผล |
| 20 | ทุก case ใน switch ต้องจบด้วย break | ✅ | `debounce()` ใน `road_board.c` |
| 21 | switch ต้องมี default | ✅ | `debounce()` มี `default:` |
| 22 | default อยู่แรกหรือสุดท้าย | ✅ | `default:` เป็น label สุดท้าย |

> หมายเหตุ Rule 20–22: เดิมโค้ดไม่มี switch เลย เวอร์ชันนี้เปลี่ยนการเลือกปุ่มใน
> `debounce()` จาก if/else if หลายชั้นเป็น `switch (i)` ซึ่งเหมาะกับงานนี้โดยธรรมชาติ
> (index ปุ่ม 0/1/2) จึงแสดงให้เห็นทั้ง 3 ข้อได้จริง

## การยืนยันว่าพฤติกรรมไม่เปลี่ยน

1. **คอมไพล์ ARM GCC** `-std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow` กับ CMSIS STM32F411xE → 0 warning
2. **Unit test เดิม** (`tests/run_board_tests.py`, ARM + Unicorn emulator) → PASS ทั้ง 3 ชุด
3. **Differential test ฝั่ง application** — รันโค้ดเดิมกับโค้ดใหม่คู่กัน 12 ล้าน tick
   (สุ่มปุ่ม, LDR, ระยะ, sensor หลุด/ค้าง, reset) เทียบทุก field ของ `RoadApp`
   และทุก byte ของเฟรม UART → **เหมือนกัน bit-by-bit**
4. **Differential test ฝั่ง driver** — emulate firmware ทั้งสองเวอร์ชันบน Cortex-M4
   (Unicorn) เรียก `RoadBoard_Init` และทุก IRQ handler (EXTI, ADC, TIM2/3/4, DMA)
   ประมาณ 58,000 ครั้ง เทียบค่า register ทุกตัวของ GPIO/RCC/EXTI/ADC/USART/DMA/TIM/NVIC
   ระดับขา output และตัวแปร global → **เหมือนกันทุกขั้น**

สิ่งที่ยังต้องทดสอบเองบนบอร์ดจริง: build ใน CubeIDE แล้ว flash ดูว่าทำงานเหมือนเดิม
(ปุ่ม, LDR, US-100, LED, 7-segment, UART ไป simulator)

## วิธีใช้ใน CubeIDE

copy ไฟล์ `.c` / `.h` ทั้ง 8 ไฟล์ในโฟลเดอร์นี้ไปแทนไฟล์เดิมในโปรเจกต์ CubeIDE
แล้ว Clean → Build → Flash
อย่าเพิ่มโฟลเดอร์นี้เข้า source path พร้อมกับ `stm32_road/` เพราะจะเกิด duplicate symbol
