# arduinoY
Arduino Projects

---

## 16-bit CPU Emulator (`16bit_cpu_emulator/`)

An Arduino Uno sketch that drives a panel of **16 LEDs** to emulate a
16-bit computer — great for teaching computer architecture in the classroom.

### What it demonstrates

| Concept | How it is shown |
|---|---|
| Binary representation | LEDs light up to show each 16-bit value |
| Instruction set architecture | 16 opcodes, printed name + hex on Serial Monitor |
| Fetch–decode–execute cycle | One step per tick; single-step button lets students pause and advance manually |
| ALU operations | ADD, SUB, AND, OR, XOR, NOT, SHL, SHR |
| Flags | Zero (ZF) and Carry (CF) updated after every ALU instruction |
| Branching | JMP, JZ (branch if zero), JNZ (branch if not zero) |
| Memory | STORE / LOAD_M round-trip through a 256-word RAM |

---

### Bill of materials

| Qty | Part |
|---|---|
| 1 | Arduino Uno (or compatible) |
| 2 | 74HC595 8-bit shift register |
| 16 | LED (any colour) |
| 16 | 220 Ω resistor |
| 3 | Momentary push-button |
| — | Breadboard + jumper wires |

---

### Wiring

#### Shift-register chain (Arduino → IC1 → IC2)

```
Arduino D11 ──► IC1 pin 14  SER    (serial data in)
Arduino D13 ──► IC1 pin 11  SRCLK  (shift clock)
Arduino D10 ──► IC1 pin 12  RCLK   (latch / storage clock)

IC1 pin  9  ──► IC2 pin 14          (daisy-chain serial out → serial in)
IC1 pin 11  ──► IC2 pin 11          (shared shift clock)
IC1 pin 12  ──► IC2 pin 12          (shared latch clock)

IC1 & IC2  pin 16 ──► 5 V
IC1 & IC2  pin  8 ──► GND
IC1 & IC2  pin 13 ──► GND  (OE, active-low → always enabled)
IC1 & IC2  pin 10 ──► 5 V  (MR, active-low → never reset)
```

#### LED panel

```
IC1  Q7 Q6 Q5 Q4 Q3 Q2 Q1 Q0  → LED anodes for bus bits [15:8]
IC2  Q7 Q6 Q5 Q4 Q3 Q2 Q1 Q0  → LED anodes for bus bits  [7:0]

Each LED cathode → 220 Ω → GND

Physical layout (left = most significant):
  [ 15  14  13  12 | 11  10   9   8 |  7   6   5   4 |  3   2   1   0 ]
```

#### Control buttons (active-LOW, Arduino internal pull-ups)

| Button | Arduino pin | Function |
|---|---|---|
| STEP | D2 | Advance one instruction (while paused) |
| RUN / PAUSE | D3 | Toggle continuous / single-step mode |
| RESET | D4 | Restart demo from address 0 |

---

### Instruction set architecture

Each instruction is one **16-bit word**: `[15:12]` = 4-bit opcode, `[11:0]` = 12-bit operand.

| Opcode | Mnemonic | Operation |
|---|---|---|
| `0x0` | `NOP` | No operation |
| `0x1` | `LOAD #n` | `ACC ← n` (12-bit immediate) |
| `0x2` | `ADD  #n` | `ACC ← ACC + n` (sets carry on overflow) |
| `0x3` | `SUB  #n` | `ACC ← ACC − n` (sets carry on borrow) |
| `0x4` | `AND  #n` | `ACC ← ACC & n` |
| `0x5` | `OR   #n` | `ACC ← ACC \| n` |
| `0x6` | `XOR  #n` | `ACC ← ACC ^ n` |
| `0x7` | `NOT` | `ACC ← ~ACC` |
| `0x8` | `SHL  #n` | `ACC ← ACC << n` (logical, result truncated to 16 bits) |
| `0x9` | `SHR  #n` | `ACC ← ACC >> n` (logical / unsigned) |
| `0xA` | `JMP  #a` | `PC ← a` (unconditional) |
| `0xB` | `JZ   #a` | `PC ← a` if Zero flag set |
| `0xC` | `JNZ  #a` | `PC ← a` if Zero flag clear |
| `0xD` | `STORE #a` | `RAM[a] ← ACC` |
| `0xE` | `LOAD_M #a` | `ACC ← RAM[a]` |
| `0xF` | `OUT  #f` | Display `ACC` on LEDs; HALT if `f == 0xFF` |

**Flags** (updated after ALU instructions and `LOAD`; NOT updated by `LOAD_M`, `STORE`, jumps, or `OUT`):

- **ZF** (Zero Flag) — set when `ACC == 0`
- **CF** (Carry Flag) — set on `ADD` overflow or `SUB` borrow

---

### Built-in demo program

The sketch ships with a 62-instruction demo that runs automatically on power-up:

| Section | Addresses | What students see |
|---|---|---|
| Bit-shift walk | 0–31 | Single lit LED walks from bit 0 → bit 15 |
| Boolean ops | 32–41 | AND, OR, XOR, NOT on visible bit patterns |
| Alternating | 42–47 | `0xAAAA` (1010…) ↔ `0x5555` (0101…) |
| Memory round-trip | 48–53 | ACC cleared, then restored from RAM |
| Countdown | 54–58 | Counts 8 → 0, JNZ loop, zero flag fires |
| All-on + HALT | 59–61 | All 16 LEDs light up, then LEDs blink |

The Serial Monitor (9600 baud) prints every instruction with the format:

```
PC=0x37  OUT   0x000  ACC=0x0005 [0000 0000 0000 0101]  ZF=0  CF=0
```

---

### Loading and running

1. Open `16bit_cpu_emulator/16bit_cpu_emulator.ino` in the Arduino IDE.
2. Select **Board → Arduino Uno** and the correct **Port**.
3. Click **Upload**.
4. Open the **Serial Monitor** at **9600 baud**.
5. Watch the LEDs and Serial Monitor — the demo starts immediately.

Use the buttons to pause, single-step through instructions, or reset.

---

### Writing your own programs

Replace (or append to) the `DEMO_PROGRAM[]` array in the sketch.  Use the
`INSTR(opcode, operand)` macro:

```cpp
INSTR(OP_LOAD, 42),    // LOAD #42   → ACC = 42
INSTR(OP_ADD,  10),    // ADD  #10   → ACC = 52
INSTR(OP_OUT,  0),     // OUT        → display 52 on LEDs
INSTR(OP_OUT,  OUT_HALT), // OUT + HALT
```

Operands are **12-bit unsigned integers** (0–4095).  For jump targets use the
decimal or hex instruction address within `DEMO_PROGRAM[]`.
