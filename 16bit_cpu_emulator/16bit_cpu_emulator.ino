/*
 * ============================================================
 *  16-bit CPU Emulator – Arduino Uno + 16-LED Panel
 * ============================================================
 *
 *  PURPOSE
 *  -------
 *  This sketch turns an Arduino Uno into a visual 16-bit
 *  computer simulator for classroom use.  A panel of 16 LEDs
 *  driven by two daisy-chained 74HC595 shift registers shows
 *  the value placed on the emulated 16-bit data bus by every
 *  OUT instruction.  The Serial Monitor (9600 baud) mirrors
 *  the full CPU state so students can follow along on a
 *  projected screen.
 *
 * ============================================================
 *  BILL OF MATERIALS
 * ============================================================
 *
 *  • 1 × Arduino Uno (or compatible)
 *  • 2 × 74HC595 8-bit serial-in / parallel-out shift register
 *  • 16 × LED (any colour)
 *  • 16 × 220 Ω resistor (one per LED)
 *  • 3 × momentary push-button (STEP / RUN-PAUSE / RESET)
 *  • breadboard + jumper wires
 *
 * ============================================================
 *  WIRING
 * ============================================================
 *
 *  Shift-register chain
 *  --------------------
 *   Arduino D11 ──► IC1 pin 14  SER   (serial data in)
 *   Arduino D13 ──► IC1 pin 11  SRCLK (shift clock)
 *   Arduino D10 ──► IC1 pin 12  RCLK  (latch / storage clock)
 *
 *   IC1 pin  9  ──► IC2 pin 14  (daisy-chain: serial out → serial in)
 *   IC1 pin 11  ──► IC2 pin 11  (share shift clock)
 *   IC1 pin 12  ──► IC2 pin 12  (share latch clock)
 *
 *   IC1 & IC2 pin 16 ──► 5 V
 *   IC1 & IC2 pin  8 ──► GND
 *   IC1 & IC2 pin 13 ──► GND  (OE active-low: always enabled)
 *   IC1 & IC2 pin 10 ──► 5 V  (MR active-low: never reset)
 *
 *  LED panel
 *  ---------
 *   IC1 Q7-Q0  → LED anodes for bus bits [15:8]  (most significant byte)
 *   IC2 Q7-Q0  → LED anodes for bus bits  [7:0]  (least significant byte)
 *   Each LED cathode through a 220 Ω resistor to GND.
 *
 *   Physical layout (left = MSB):
 *   [ 15 14 13 12 | 11 10  9  8 | 7  6  5  4 | 3  2  1  0 ]
 *
 *  Control buttons (active-LOW, Arduino internal pull-ups used)
 *  -------------------------------------------------------------
 *   Arduino D2  ──► STEP button   (advance one instruction while paused)
 *   Arduino D3  ──► RUN / PAUSE   (toggle continuous / single-step mode)
 *   Arduino D4  ──► RESET         (restart demo from address 0)
 *
 * ============================================================
 *  INSTRUCTION SET ARCHITECTURE  (16-bit instruction word)
 * ============================================================
 *
 *  Encoding:  bits[15:12] = 4-bit OPCODE  |  bits[11:0] = 12-bit OPERAND
 *
 *  Opcode  Mnemonic    Operation
 *  ------  ----------  -----------------------------------------------
 *  0x0     NOP         No operation
 *  0x1     LOAD  #n    ACC ← n          (n = 12-bit immediate, 0–4095)
 *  0x2     ADD   #n    ACC ← ACC + n    (sets carry flag on overflow)
 *  0x3     SUB   #n    ACC ← ACC − n    (sets carry flag on borrow)
 *  0x4     AND   #n    ACC ← ACC & n
 *  0x5     OR    #n    ACC ← ACC | n
 *  0x6     XOR   #n    ACC ← ACC ^ n
 *  0x7     NOT         ACC ← ~ACC       (operand ignored)
 *  0x8     SHL   #n    ACC ← ACC << n   (n = 0–15, result truncated to 16 bits)
 *  0x9     SHR   #n    ACC ← ACC >> n   (n = 0–15, logical / unsigned shift)
 *  0xA     JMP   #a    PC  ← a          (unconditional jump to address a)
 *  0xB     JZ    #a    PC  ← a  if ZERO flag is set
 *  0xC     JNZ   #a    PC  ← a  if ZERO flag is clear
 *  0xD     STORE #a    RAM[a] ← ACC     (a = 0–255, word-addressed)
 *  0xE     LOAD_M #a   ACC ← RAM[a]     (load from data RAM)
 *  0xF     OUT   #f    Display ACC on LED panel; HALT if f == 0x0FF
 *
 *  Flags updated after every ALU instruction (not after jumps / OUT):
 *    ZERO  (ZF) – set when ACC == 0
 *    CARRY (CF) – set on ADD overflow or SUB borrow
 *
 * ============================================================
 *  DEMO PROGRAM  (built in, loaded at startup)
 * ============================================================
 *  Section 1 (addr  0–31): Bit-shift left — watch the lit LED
 *                           walk from bit 0 to bit 15
 *  Section 2 (addr 32–41): Boolean operations on the ACC
 *  Section 3 (addr 42–47): Alternating 1010… and 0101… patterns
 *  Section 4 (addr 48–53): Data-RAM store / load round-trip
 *  Section 5 (addr 54–58): Countdown 8→0 using SUB + JNZ loop
 *  Section 6 (addr 59–61): All-LEDs-on (0xFFFF), then HALT
 *
 * ============================================================
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Hardware pin assignments
// ---------------------------------------------------------------------------
static const uint8_t PIN_DATA  = 11;   // 74HC595 SER
static const uint8_t PIN_CLOCK = 13;   // 74HC595 SRCLK
static const uint8_t PIN_LATCH = 10;   // 74HC595 RCLK

static const uint8_t PIN_BTN_STEP  = 2;  // single-step
static const uint8_t PIN_BTN_RUN   = 3;  // run / pause toggle
static const uint8_t PIN_BTN_RESET = 4;  // reset

// ---------------------------------------------------------------------------
// Emulator parameters
// ---------------------------------------------------------------------------
static const uint16_t ROM_SIZE       = 256;  // max instruction words
static const uint16_t RAM_SIZE       = 256;  // data RAM words
static const uint16_t STEP_DELAY_MS  = 600;  // ms between steps in run mode
static const uint16_t DEBOUNCE_MS    = 50;   // button debounce window

// ---------------------------------------------------------------------------
// Instruction opcodes (upper 4 bits of a 16-bit word)
// ---------------------------------------------------------------------------
enum Opcode : uint8_t {
    OP_NOP    = 0x0,
    OP_LOAD   = 0x1,
    OP_ADD    = 0x2,
    OP_SUB    = 0x3,
    OP_AND    = 0x4,
    OP_OR     = 0x5,
    OP_XOR    = 0x6,
    OP_NOT    = 0x7,
    OP_SHL    = 0x8,
    OP_SHR    = 0x9,
    OP_JMP    = 0xA,
    OP_JZ     = 0xB,
    OP_JNZ    = 0xC,
    OP_STORE  = 0xD,
    OP_LOAD_M = 0xE,
    OP_OUT    = 0xF
};

// Pack opcode + 12-bit operand into one instruction word
#define INSTR(op, val)  ((uint16_t)(((uint8_t)(op) << 12u) | ((uint16_t)(val) & 0x0FFFu)))

// Operand passed to OUT that signals end-of-program / HALT
#define OUT_HALT  0x0FF

// ---------------------------------------------------------------------------
// Demo program  (stored in flash to spare precious SRAM)
//
// Jump-target addresses as hex:
//   Section 5 loop top = addr 55 = 0x37
// ---------------------------------------------------------------------------
static const uint16_t DEMO_PROGRAM[] PROGMEM = {

    /* ── Section 1: Bit-shift left – walking LED ─────────────────────────── */
    /* 00 */ INSTR(OP_LOAD, 0x001),   // ACC = 0x0001
    /* 01 */ INSTR(OP_OUT,  0x000),   // display ACC
    /* 02 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0002
    /* 03 */ INSTR(OP_OUT,  0x000),
    /* 04 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0004
    /* 05 */ INSTR(OP_OUT,  0x000),
    /* 06 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0008
    /* 07 */ INSTR(OP_OUT,  0x000),
    /* 08 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0010
    /* 09 */ INSTR(OP_OUT,  0x000),
    /* 10 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0020
    /* 11 */ INSTR(OP_OUT,  0x000),
    /* 12 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0040
    /* 13 */ INSTR(OP_OUT,  0x000),
    /* 14 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0080
    /* 15 */ INSTR(OP_OUT,  0x000),
    /* 16 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0100
    /* 17 */ INSTR(OP_OUT,  0x000),
    /* 18 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0200
    /* 19 */ INSTR(OP_OUT,  0x000),
    /* 20 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0400
    /* 21 */ INSTR(OP_OUT,  0x000),
    /* 22 */ INSTR(OP_SHL,  0x001),   // ACC = 0x0800
    /* 23 */ INSTR(OP_OUT,  0x000),
    /* 24 */ INSTR(OP_SHL,  0x001),   // ACC = 0x1000
    /* 25 */ INSTR(OP_OUT,  0x000),
    /* 26 */ INSTR(OP_SHL,  0x001),   // ACC = 0x2000
    /* 27 */ INSTR(OP_OUT,  0x000),
    /* 28 */ INSTR(OP_SHL,  0x001),   // ACC = 0x4000
    /* 29 */ INSTR(OP_OUT,  0x000),
    /* 30 */ INSTR(OP_SHL,  0x001),   // ACC = 0x8000  (bit 15 set)
    /* 31 */ INSTR(OP_OUT,  0x000),

    /* ── Section 2: Boolean operations ──────────────────────────────────── */
    /* 32 */ INSTR(OP_LOAD, 0xFF0),   // ACC = 0x0FF0
    /* 33 */ INSTR(OP_OUT,  0x000),   // display 0x0FF0
    /* 34 */ INSTR(OP_AND,  0x0F0),   // ACC = 0x00F0  (mask away upper nibble)
    /* 35 */ INSTR(OP_OUT,  0x000),   // display 0x00F0
    /* 36 */ INSTR(OP_OR,   0xF0F),   // ACC = 0x0FFF  (set bits 11-8 and 3-0)
    /* 37 */ INSTR(OP_OUT,  0x000),   // display 0x0FFF
    /* 38 */ INSTR(OP_XOR,  0xF0F),   // ACC = 0x00F0  (flip those bits back)
    /* 39 */ INSTR(OP_OUT,  0x000),   // display 0x00F0
    /* 40 */ INSTR(OP_NOT,  0x000),   // ACC = 0xFF0F  (bitwise invert all 16 bits)
    /* 41 */ INSTR(OP_OUT,  0x000),   // display 0xFF0F

    /* ── Section 3: Alternating 1010/0101 patterns ───────────────────────── */
    /*
     * Build 0xAAAA (1010 1010 1010 1010):
     *   LOAD 0xAAA  → ACC = 0x0AAA = 0000 1010 1010 1010
     *   SHL  4      → ACC = 0xAAA0 = 1010 1010 1010 0000
     *   OR   0x00A  → ACC = 0xAAAA = 1010 1010 1010 1010
     */
    /* 42 */ INSTR(OP_LOAD, 0xAAA),   // ACC = 0x0AAA
    /* 43 */ INSTR(OP_SHL,  0x004),   // ACC = 0xAAA0
    /* 44 */ INSTR(OP_OR,   0x00A),   // ACC = 0xAAAA
    /* 45 */ INSTR(OP_OUT,  0x000),   // display 0xAAAA  (1010 1010 1010 1010)
    /* 46 */ INSTR(OP_NOT,  0x000),   // ACC = 0x5555   (0101 0101 0101 0101)
    /* 47 */ INSTR(OP_OUT,  0x000),   // display 0x5555

    /* ── Section 4: Data-RAM store / load round-trip ─────────────────────── */
    /* 48 */ INSTR(OP_LOAD,   0xBEE), // ACC = 0x0BEE
    /* 49 */ INSTR(OP_STORE,  0x010), // RAM[16] = 0x0BEE
    /* 50 */ INSTR(OP_LOAD,   0x000), // ACC = 0x0000  (clear accumulator)
    /* 51 */ INSTR(OP_OUT,    0x000), // display 0x0000  (shows ACC was cleared)
    /* 52 */ INSTR(OP_LOAD_M, 0x010), // ACC = RAM[16]  = 0x0BEE  (reload)
    /* 53 */ INSTR(OP_OUT,    0x000), // display 0x0BEE

    /* ── Section 5: Countdown 8→0 using SUB + JNZ loop ─────────────────── */
    /*
     * addr 54 = 0x36, addr 55 = 0x37
     * Loop: display ACC (count), subtract 1, repeat until zero
     */
    /* 54 */ INSTR(OP_LOAD, 0x008),   // ACC = 8
    /* 55 */ INSTR(OP_OUT,  0x000),   // display current count  (loop top)
    /* 56 */ INSTR(OP_SUB,  0x001),   // ACC -= 1
    /* 57 */ INSTR(OP_JNZ,  0x037),   // if ACC != 0 jump back to addr 55 (0x37)
    /* 58 */ INSTR(OP_OUT,  0x000),   // display 0x0000  (countdown complete)

    /* ── Section 6: All LEDs on, then HALT ───────────────────────────────── */
    /*
     * Build 0xFFFF:
     *   LOAD 0x000 → ACC = 0x0000
     *   NOT        → ACC = 0xFFFF
     *   OUT 0x0FF  → display 0xFFFF, then HALT (OUT_HALT flag)
     */
    /* 59 */ INSTR(OP_LOAD, 0x000),   // ACC = 0x0000
    /* 60 */ INSTR(OP_NOT,  0x000),   // ACC = 0xFFFF  (all bits set)
    /* 61 */ INSTR(OP_OUT,  OUT_HALT),// display 0xFFFF, then HALT
};

static const uint16_t DEMO_LEN =
    (uint16_t)(sizeof(DEMO_PROGRAM) / sizeof(DEMO_PROGRAM[0]));

// ---------------------------------------------------------------------------
// CPU state
// ---------------------------------------------------------------------------
struct CPU {
    uint16_t acc;     // accumulator
    uint16_t pc;      // program counter
    bool     zf;      // zero flag
    bool     cf;      // carry flag (ADD overflow / SUB borrow)
    bool     halted;
};

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
static uint16_t rom[ROM_SIZE];
static uint16_t ram[RAM_SIZE];
static CPU      cpu;
static bool     running = true;   // true = continuous, false = single-step

// ---------------------------------------------------------------------------
// Shift-register display driver
//
// Sends 16 bits MSB-first to the daisy-chained 74HC595s and latches the
// outputs.  The first byte sent emerges at IC1's Q7-Q0 (bus bits 15-8);
// the second byte at IC2's Q7-Q0 (bus bits 7-0).
// ---------------------------------------------------------------------------
static void displayValue(uint16_t value) {
    digitalWrite(PIN_LATCH, LOW);
    shiftOut(PIN_DATA, PIN_CLOCK, MSBFIRST, (uint8_t)(value >> 8));
    shiftOut(PIN_DATA, PIN_CLOCK, MSBFIRST, (uint8_t)(value & 0xFFu));
    digitalWrite(PIN_LATCH, HIGH);
}

// ---------------------------------------------------------------------------
// Serial logging helpers
// ---------------------------------------------------------------------------

static void printBinary16(uint16_t v) {
    for (int8_t i = 15; i >= 0; i--) {
        Serial.print((v >> (uint8_t)i) & 1u);
        if (i > 0 && ((uint8_t)i % 4u) == 0u) Serial.print(' ');
    }
}

static void printCpuState(uint8_t op, uint16_t arg) {
    static const char* const NAMES[16] = {
        "NOP   ", "LOAD  ", "ADD   ", "SUB   ",
        "AND   ", "OR    ", "XOR   ", "NOT   ",
        "SHL   ", "SHR   ", "JMP   ", "JZ    ",
        "JNZ   ", "STORE ", "LOAD_M", "OUT   "
    };

    // PC (already incremented, so subtract 1 to show the fetched address)
    Serial.print(F("PC=0x"));
    uint16_t fetchedAt = cpu.pc - 1u;
    if (fetchedAt < 0x10u) Serial.print('0');
    Serial.print(fetchedAt, HEX);

    Serial.print(F("  "));
    Serial.print(NAMES[op & 0xFu]);
    Serial.print(F(" 0x"));
    if (arg < 0x100u) Serial.print('0');
    if (arg < 0x010u) Serial.print('0');
    Serial.print(arg, HEX);

    Serial.print(F("  ACC=0x"));
    if (cpu.acc < 0x1000u) Serial.print('0');
    if (cpu.acc < 0x100u)  Serial.print('0');
    if (cpu.acc < 0x010u)  Serial.print('0');
    Serial.print(cpu.acc, HEX);

    Serial.print(F(" ["));
    printBinary16(cpu.acc);
    Serial.print(F("]"));

    Serial.print(F("  ZF="));
    Serial.print(cpu.zf ? '1' : '0');
    Serial.print(F("  CF="));
    Serial.println(cpu.cf ? '1' : '0');
}

// ---------------------------------------------------------------------------
// CPU reset
// ---------------------------------------------------------------------------
static void resetCPU() {
    cpu.acc    = 0;
    cpu.pc     = 0;
    cpu.zf     = false;
    cpu.cf     = false;
    cpu.halted = false;
    memset(ram, 0, sizeof(ram));
    displayValue(0x0000u);

    Serial.println();
    Serial.println(F("[RESET] CPU reset – starting demo program."));
    Serial.println(F("────────────────────────────────────────────────────────────────────"));
    Serial.println(F(" PC    INSTR        ARG    ACC (hex)  [15..12 11..8 7..4 3..0]  ZF CF"));
    Serial.println(F("────────────────────────────────────────────────────────────────────"));
}

// ---------------------------------------------------------------------------
// Fetch – decode – execute one instruction
// ---------------------------------------------------------------------------
static void stepCPU() {
    if (cpu.halted) return;

    if (cpu.pc >= ROM_SIZE) {
        cpu.halted = true;
        Serial.println(F("[HALT] PC exceeded ROM size."));
        return;
    }

    uint16_t instr = rom[cpu.pc++];
    uint8_t  op    = (uint8_t)(instr >> 12u);
    uint16_t arg   = instr & 0x0FFFu;
    uint32_t tmp;

    switch (op) {
        case OP_NOP:
            break;

        case OP_LOAD:
            cpu.acc = arg;
            break;

        case OP_ADD:
            tmp     = (uint32_t)cpu.acc + (uint32_t)arg;
            cpu.cf  = (tmp > 0xFFFFu);
            cpu.acc = (uint16_t)(tmp & 0xFFFFu);
            break;

        case OP_SUB:
            cpu.cf  = (arg > cpu.acc);
            cpu.acc = (uint16_t)(cpu.acc - arg);
            break;

        case OP_AND:
            cpu.acc &= arg;
            break;

        case OP_OR:
            cpu.acc |= arg;
            break;

        case OP_XOR:
            cpu.acc ^= arg;
            break;

        case OP_NOT:
            cpu.acc = (uint16_t)(~cpu.acc);
            break;

        case OP_SHL:
            cpu.acc = (uint16_t)(cpu.acc << (arg & 0xFu));
            break;

        case OP_SHR:
            cpu.acc = (uint16_t)(cpu.acc >> (arg & 0xFu));
            break;

        case OP_JMP:
            cpu.pc = (uint16_t)(arg & (ROM_SIZE - 1u));
            break;

        case OP_JZ:
            if (cpu.zf) {
                cpu.pc = (uint16_t)(arg & (ROM_SIZE - 1u));
            }
            break;

        case OP_JNZ:
            if (!cpu.zf) {
                cpu.pc = (uint16_t)(arg & (ROM_SIZE - 1u));
            }
            break;

        case OP_STORE:
            ram[arg & (RAM_SIZE - 1u)] = cpu.acc;
            break;

        case OP_LOAD_M:
            cpu.acc = ram[arg & (RAM_SIZE - 1u)];
            break;

        case OP_OUT:
            displayValue(cpu.acc);
            // Print OUT result prominently
            Serial.print(F("*** OUT  0x"));
            if (cpu.acc < 0x1000u) Serial.print('0');
            if (cpu.acc < 0x100u)  Serial.print('0');
            if (cpu.acc < 0x010u)  Serial.print('0');
            Serial.print(cpu.acc, HEX);
            Serial.print(F("  ["));
            printBinary16(cpu.acc);
            Serial.println(F("]  → LEDs updated"));

            if (arg == (uint16_t)OUT_HALT) {
                cpu.halted = true;
                Serial.println(F("\n[HALT] End of demo. Press RESET to replay."));
            }
            break;
    }

    // Update flags after every ALU instruction
    // (flags are NOT updated by jumps, OUT, or STORE/LOAD_M)
    bool updatesFlags = (op != OP_JMP  && op != OP_JZ   &&
                         op != OP_JNZ  && op != OP_OUT  &&
                         op != OP_STORE);
    if (updatesFlags) {
        cpu.zf = (cpu.acc == 0);
    }

    // Print non-OUT instructions in a quieter log line
    if (op != OP_OUT) {
        printCpuState(op, arg);
    }
}

// ---------------------------------------------------------------------------
// Button edge detection (falling edge = pressed)
// Each call checks ONE button identified by its Arduino pin.
// Returns true exactly once per physical press.
// ---------------------------------------------------------------------------
static bool buttonEdge(uint8_t pin, bool* prevState, uint32_t* lastTime) {
    bool cur = (digitalRead(pin) == LOW);
    uint32_t now = millis();
    bool fell = false;
    if (cur && !(*prevState) && (now - *lastTime) > DEBOUNCE_MS) {
        fell = true;
        *lastTime = now;
    }
    *prevState = cur;
    return fell;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    pinMode(PIN_DATA,  OUTPUT);
    pinMode(PIN_CLOCK, OUTPUT);
    pinMode(PIN_LATCH, OUTPUT);

    pinMode(PIN_BTN_STEP,  INPUT_PULLUP);
    pinMode(PIN_BTN_RUN,   INPUT_PULLUP);
    pinMode(PIN_BTN_RESET, INPUT_PULLUP);

    Serial.begin(9600);

    // Copy demo from flash (PROGMEM) into RAM-based ROM array
    memset(rom, 0, sizeof(rom));
    uint16_t len = (DEMO_LEN < ROM_SIZE) ? DEMO_LEN : ROM_SIZE;
    for (uint16_t i = 0; i < len; i++) {
        rom[i] = pgm_read_word(&DEMO_PROGRAM[i]);
    }

    Serial.println();
    Serial.println(F("======================================================"));
    Serial.println(F("  16-bit CPU Emulator  –  Arduino Uno + 16-LED Panel  "));
    Serial.println(F("======================================================"));
    Serial.println(F("  D2 = STEP    D3 = RUN / PAUSE    D4 = RESET         "));
    Serial.println(F("======================================================"));

    resetCPU();
    running = true;
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    // -- Button state (persists between calls via static locals) -------------
    static bool     prevStep  = false;
    static bool     prevRun   = false;
    static bool     prevReset = false;
    static uint32_t tStep     = 0;
    static uint32_t tRun      = 0;
    static uint32_t tReset    = 0;

    bool stepPressed  = buttonEdge(PIN_BTN_STEP,  &prevStep,  &tStep);
    bool runPressed   = buttonEdge(PIN_BTN_RUN,   &prevRun,   &tRun);
    bool resetPressed = buttonEdge(PIN_BTN_RESET, &prevReset, &tReset);

    // RUN / PAUSE toggle
    if (runPressed) {
        running = !running;
        Serial.print(F("[MODE] "));
        Serial.println(running ? F("RUNNING") : F("PAUSED – use STEP button"));
    }

    // RESET
    if (resetPressed) {
        resetCPU();
        running = true;
    }

    // -- Execution -----------------------------------------------------------
    if (cpu.halted) {
        // Blink all LEDs at 1 Hz to signal end-of-program
        static uint32_t blinkT     = 0;
        static bool     blinkState = false;
        if ((millis() - blinkT) >= 500u) {
            blinkT     = millis();
            blinkState = !blinkState;
            displayValue(blinkState ? 0xFFFFu : 0x0000u);
        }
        return;
    }

    if (running) {
        static uint32_t lastStepT = 0;
        if ((millis() - lastStepT) >= STEP_DELAY_MS) {
            lastStepT = millis();
            stepCPU();
        }
    } else if (stepPressed) {
        stepCPU();
    }
}
