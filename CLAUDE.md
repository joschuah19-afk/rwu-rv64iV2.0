# SystemVerilog Design Guidelines

These rules apply to ALL `.sv` files in this project.
Claude MUST follow them exactly when writing or modifying SystemVerilog.

---

## 1. Declaration Order within a Module

All signal and variable declarations come **immediately after the port list**,
before any `assign`, `always_comb`, `always_ff`, or `generate` block.
Never declare a signal next to the logic that drives it.

**Required order inside a module:**

```
module foo ( ... ports ... );

  // 1. imports / typedefs / enums / localparams
  // 2. ALL signal declarations   ← complete declaration block here
  // 3. assign statements
  // 4. always_comb blocks
  // 5. always_ff blocks
  // 6. generate blocks
  // 7. module instantiations

endmodule
```

This rule eliminates xsim warning VRFC 10-3380 ("identifier used before its declaration").

---

## 2. Naming Convention

| Category                    | Suffix | Example              |
|-----------------------------|--------|----------------------|
| Registered signal (FF)      | `_r`   | `tx_flag_r`          |
| Combinatorial / wire signal | `_s`   | `grant_d_s`          |
| Port input                  | `_i`   | `clk_i`, `rst_i`     |
| Port output                 | `_o`   | `data_o`             |
| Port inout                  | `_io`  | `gpio_io`            |
| FSM current state           | `_state_s`     | `arb_state_s`    |
| FSM next state              | `_nextstate_s` | `arb_nextstate_s`|
| FSM enum state item         | `_ST`  | `IDLE_ST`, `GRANT_D_ST` |

**Dropped pattern**: `_reg_s` — do not use. Replace with `_r`.

---

## 3. FSM Style — Explicit 3-Block (reference: `asMemArb.sv`)

Every FSM uses exactly three named blocks in this fixed order:

### 3a. Block 1 — Delay (state register)

```systemverilog
// FSM block: delay
always_ff @(posedge clk_i, posedge rst_i)
begin
  if (rst_i)
    foo_state_s <= IDLE_ST;
  else
    foo_state_s <= foo_nextstate_s;
end
```

- Generates `foo_state_s` (the current state).
- Contains **nothing** except reset and the `state ← nextstate` assignment.

### 3b. Block 2 — Input logic (next-state / CLC)

```systemverilog
// FSM block: input logic
always_comb
begin
  foo_nextstate_s = foo_state_s;   // self-arc default
  case (foo_state_s)
    IDLE_ST: if (req_i) foo_nextstate_s = ACTIVE_ST;
    ...
    default: foo_nextstate_s = IDLE_ST;
  endcase
end
```

- Generates **exactly one** signal: `foo_nextstate_s`.
- No other signals are computed here.
- Default assignment before the `case` prevents latches.
- Uses blocking assignments (`=`).

### 3c. Block 3 — Output logic

Can be combinatorial (`assign` or `always_comb`) or registered (`always_ff`).

**Moore output** (depends only on `state_s` — safe):
```systemverilog
assign busy_o = (foo_state_s != IDLE_ST);
```

**Mealy output** (depends on `state_s` AND an input — dangerous, must be marked):
```systemverilog
// MEALY: grant_d_s depends on state AND dcache_axi4.arvalid (glitch risk)
assign grant_d_s = (arb_state_s == GRANT_D_ST)
                 | (arb_state_s == IDLE_ST & dcache_axi4.arvalid);
```

Mealy outputs **must** carry a `// MEALY:` comment explaining the input dependency
and why a registered output is not used instead.

---

## 4. always_comb / always_ff Assignment Types

- `always_comb`: blocking assignments (`=`) **only**
- `always_ff`:   non-blocking assignments (`<=`) **only**

Never mix assignment types within one block.

---

## 5. Comments — Only "Why", Never "What"

Do not add comments that restate what the code already says.
Only add a comment when the **reason** is non-obvious:
a hidden constraint, a hardware quirk, a workaround, or a Mealy hazard (see above).

---

## 6. Module and File Naming Prefix

Every module and its corresponding `.sv` file carries a two-letter designer prefix
that identifies authorship:

| Prefix | Author                          |
|--------|---------------------------------|
| `as_`  | Andreas Siggelkow (project lead)|
| `cc_`  | Claude Code (AI-generated code) |

Examples: `as_qspi.sv`, `cc_fifo_sync.sv`.

When Claude writes a **new** module it uses the `cc_` prefix.
When Claude **modifies** an existing `as_` module it keeps the original prefix.

## 7. Simulation / Tool Target

- Primary simulator: **xsim (Vivado)**
- All VRFC 10-3380 warnings must be absent (enforced by rule 1)
- No `initial` blocks in synthesisable RTL (testbenches only)
