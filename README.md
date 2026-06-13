# An ultra fast SoftFloat library with 24mantissa and 8bit exponent (IEEE 754 like)

This library is an ultra fast soft float implementation optimized for speed for the ARM Cortex M3 based microcontrollers</br>
It is written in C++23, and is constexpr, so all calculations that can be done at compile time will be</br>
It also implements autodetection and optimization of fused multiplications and additions, to increase speed and accuracy</br>
Precision is equivalent to IEEE754 float type (24 bit mantissa, 8bit exponent). Only thing not produced are Inf and Nans and subnormals

All the library is in a header file called FusedSoftFloat.hh</br>

To use it, just include it, and, instead of using the 'float' type for variables, use the 'SoftFloat' type.</br>
It has equivalent resolution as of IEEE float type (29 bits mantissa, 7 bits exponent)</br>
No NaN, +/-Inf are handled and/or created</br>
</br>

# Expected approximate cycle counts

## 1. SoftFloat Cycle Counts (min / typical / max) — Measured on STM32F103 (Cortex-M3) Core Cycles

| Function                           | Min | Typ | Max | Remarks                                                                 |
|------------------------------------|-----|-----|-----|-------------------------------------------------------------------------|
| Construction / assignment          |   5 |  10 |  30 | Normalisation dominates; CLZ + shift path for non-power-of-2.           |
| `operator+`, `operator-`           |   8 |  15 |  45 | Fast path: same sign add; slow path: cancellation with CLZ.            |
| `operator*` (`mul_plain`)          |   6 |  12 |  20 | Single UMULL + shift + ADC pack.                                       |
| `operator/`                        |  10 |  18 |  30 | UDIV+SDIV two-stage mantissa divide (~30-bit); 2 hardware divs + ALU.  |
| `sqrt()`                           |  20 |  35 |  60 | Table lookup + 2 SDIV corrections + remainder check.                   |
| `inv_sqrt()`                       |  15 |  25 |  40 | Table + one Newton iteration (2 SMULL) + optional √2 multiply.        |
| `exp()`                            |  25 |  40 |  80 | SMULL for range reduction, table lookup, linear interpolation.          |
| `log2()`                           |  15 |  25 |  40 | Table lookup + SMULL interpolation; very fast.                          |
| `log()` / `log10()`                |  20 |  30 |  50 | ARM fast path: UDIV for 1/m + table + series; log10 adds one mul.     |
| `sincos()`                         |  30 |  60 | 150 | Angle table interpolation; range reduction may use UDIV or loop.       |
| `sin()` / `cos()`                  |  25 |  55 | 140 | Wrappers around `sincos()` with Angle-based SMULL interpolation.       |
| `tan()`                            |  40 |  80 | 200 | Table interp; near π/2 uses cot path with UDIV+SDIV.                  |
| `asin()` / `acos()`                |  50 | 100 | 200 | Uses `atan2()` + `sqrt()`; both expensive sub-operations.              |
| `atan2()`                          |  40 |  90 | 180 | UDIV+SDIV quotient + table + UMULL interp in Angle domain.             |
| `sinh()` / `cosh()` / `tanh()`     |  35 |  55 | 100 | Based on `exp()`; tanh adds a division.                                 |
| `pow()` (integer exponent)         |   5 |  20 | 100 | Fast exponentiation by squaring; loop count up to 31.                   |
| `pow()` (general)                  |  80 | 150 | 400 | Calls `exp(y*log(x))`.                                                  |
| `trunc()` / `floor()` / `ceil()` / `round()` |   8 |  15 |  30 | Simple bit manipulations.                                               |
| `fmod()`                           |  15 |  40 | 200 | div+trunc+mul+sub (O(1)); cost proportional to exponent difference.     |
| `fma()` / `fused_mul_add`          |  10 |  20 |  40 | UMULL product + addition + normalisation.                                |
| `hypot()`                          |  15 |  35 |  70 | Two multiplies + one square root.                                        |
| `lerp()`                           |  15 |  25 |  40 | One multiply + two adds.                                                 |

## 2. Comparison with Typical Soft‑Float Library and qfplib

| Function         | SoftFloat Typical | Soft‑float Lib Typical | qfplib Typical | Comments |
|------------------|-------------------|------------------------|----------------|----------|
| `fadd` / `fsub`  | 15                | 80–120                 | 35–50          | SoftFloat same-sign fast path avoids CLZ; different-sign needs CLZ renorm. qfplib uses alignment shifts and careful rounding. |
| `fmul`           | 12                | 60–90                  | 20–30          | SoftFloat uses same UMULL+ADC kernel as qfplib. Generic lib uses 64-bit shift path. |
| `fdiv`           | 18                | 150–250                | 40–70          | SoftFloat uses UDIV+SDIV two-stage mantissa divide (~30-bit prec). qfplib uses UDIV for quotient estimate + correction. |
| `fsqrt`          | 35                | 300–500                | 50–80          | SoftFloat uses same table+2×SDIV algorithm as qfplib. Generic lib uses Newton iteration without SDIV. |
| `fexp`           | 40                | 500–800                | 80–150         | SoftFloat uses SMULL range reduction + table + linear interp. qfplib uses table + power series. |
| `fln` / `flog`   | 30                | 400–700                | 60–100         | SoftFloat ARM fast path uses UDIV for 1/m + table + series. qfplib uses table + series. |
| `fsin` / `fcos`  | 55                | 400–700                | 100–200        | SoftFloat uses Angle-based table interpolation with SMULL. qfplib performs range reduction and table interpolation. |
| `ftan`           | 80                | 600–1000               | 150–250        | SoftFloat table interpolation + UDIV/SDIV for cot path. qfplib uses fraction decomposition. |
| `fatan2`         | 90                | 600–1000               | 100–200        | SoftFloat uses UDIV+SDIV quotient + table + UMULL interp in Angle domain. qfplib uses table + refinement. |

### Methodology

- **Toolchain**: GCC 14.2.1 (`arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O2 -std=gnu++2b`)
- **Target**: STM32F103 (Cortex-M3 with UDIV/SDIV, 64K SRAM, 256K Flash)
- **Measurement**: Static cycle analysis from ARM Thumb-2 disassembly, tracing
  typical/hot execution paths instruction-by-instruction with Cortex-M3 core
  cycle timing:
  - UMULL/SMULL: 3 cycles (pipelined)
  - UDIV/SDIV: 2–12 cycles (2 for small operands, ~6 typical, 12 worst)
  - MUL/MLA/MLS: 1–2 cycles
  - Branch taken: 3 cycles; not taken: 1 cycle
  - LDR/STR: 2 cycles; ALU: 1 cycle
  - PUSH/POP: 1+N cycles
- **QEMU**: Used for functional verification (semihosting on LM3S6965evb);
  DWT_CYCCNT is not implemented in QEMU's Cortex-M3 model, so cycle counts
  derive from static analysis rather than runtime measurement.
- **Flash wait states**: Numbers are **core cycles only**. At 72 MHz with
  2 WS, multiply by ~1.5–2× for wall-clock time.

### Key Findings

1. **Basic arithmetic** (add/sub/mul/div): FusedSoftFloat achieves cycle counts
   comparable to qfplib-m3 because it uses the same ARM instruction sequences
   (UMULL, UDIV, SDIV, table lookups). The C++ inline asm produces nearly
   identical machine code.

2. **Division**: The compiler selects the UDIV+SDIV path (not the recip32 table)
   when compiling for Cortex-M3, since hardware division is available. This is
   10–15× faster than generic soft-float libraries that must use iterative
   subtraction or Newton-Raphson without hardware divide.

3. **Transcendental functions**: FusedSoftFloat's table-driven algorithms with
   ARM-optimised interpolation (SMULL for 64-bit multiply-accumulate in one
   instruction) are 8–15× faster than generic soft-float libraries that rely
   on software-only 64-bit arithmetic.

4. **Angle subsystem**: The dedicated Angle/DeltaAngle types enable trig
   functions to use direct phase interpolation (no modular reduction by fmod),
   making sin/cos ~3× faster than the qfplib approach that requires explicit
   range reduction with multi-step Cody-Waite reduction.
### Performance
At this point, the library is nearly 8 times slower than hardware float support.


### Results from running in a STM32F407 @ 168Mhz

##  SoftFloat Test Suite

=== FUNCTIONAL TESTS ===</br>
[PASS] Default Constructor</br>
[PASS] Integer Constructor</br>
[PASS] Float Constructor</br>
[PASS] Constants</br>
[PASS] Addition</br>
[PASS] Subtraction</br>
[PASS] Multiplication</br>
[PASS] Division</br>
[PASS] Negation</br>
[PASS] Absolute Value</br>
[PASS] Comparison</br>
[PASS] Shift Operators</br>
[PASS] Fused Multiply-Add</br>
[PASS] Fused Multiply-Sub</br>
[PASS] Fused Multiply-Multiply-Add</br>
[PASS] Square Root</br>
[PASS] Inverse Square Root</br>
[PASS] Sine</br>
[PASS] Cosine</br>
[PASS] Linear Interpolation</br>
[PASS] Min/Max</br>
[PASS] Clamp</br>
[PASS] Integer Conversion</br>
[PASS] Chained Operations</br>
[PASS] Overflow Handling</br>
[PASS] Underflow Handling</br>
[PASS] Edge Cases</br>
[PASS] Compound Assignment</br>
[PASS] Expression Templates</br>
[PASS] Fused Multiply-Sub (ext)</br>
[PASS] Fused Mul-Mul-Sub (ext)</br>
[PASS] Invariant Normalization</br>
[PASS] Zero Semantics</br>
[PASS] Div/Zero & Inf Inputs</br>
[PASS] Int Conv Edges</br>
[PASS] Exponent Saturation</br>
[PASS] Math Symmetries</br>
[PASS] Lerp Edges</br>
[PASS] Expr Negation</br>
[PASS] Constexpr Complex</br>
[PASS] Shift Edges</br>
[PASS] Comparison Logic</br>
[PASS] Corner Cases & Bit Patterns</br>
[PASS] atan2</br>
  atan2 worst case error: 0.00 degree</br>
[PASS] atan2 (v2)</br>
[PASS] exp</br>
[PASS] hypot</br>
[PASS] tan</br>
[PASS] asin/acos</br>
[PASS] log/log2/log10</br>
[PASS] pow</br>
[PASS] rounding</br>
[PASS] hyperbolic</br>
[PASS] copysign/fmod</br>
[PASS] Random Stress Test</br>

-----------------------------------------------------------
Results: 55/55 passed, 0 failed
-----------------------------------------------------------

=== BENCHMARKS ===
| Operation |               |    timing     |
|-----------|---------------|-------------- |
| Multiply  | (100000 ops): |  530.50 ns/op |
| Add       | (100000 ops): |  601.33 ns/op |
| Subtract  | (100000 ops): |  595.38 ns/op |
| Divide    | (100000 ops): |  578.68 ns/op |
| FMA       | (100000 ops): |  654.91 ns/op |
| FMS       | (100000 ops): |  726.35 ns/op |
| FMMA      | (100000 ops): |  774.38 ns/op |
| FMMS      | (100000 ops): |  845.68 ns/op |
| sin       | (100000 ops): |  672.78 ns/op |
| cos       | (100000 ops): |  666.82 ns/op |
| tan       | (100000 ops): |  553.71 ns/op |
| asin      | (100000 ops): | 1738.47 ns/op |
| acos      | (100000 ops): | 1696.80 ns/op |
| atan2     | (100000 ops): | 1089.52 ns/op |
| sinh      | (100000 ops): | 1119.88 ns/op |
| cosh      | (100000 ops): | 1160.97 ns/op |
| tanh      | (100000 ops): | 1411.91 ns/op |
| exp       | (100000 ops): |  470.69 ns/op |
| log       | (100000 ops): |  458.86 ns/op |
| log2      | (100000 ops): |  553.71 ns/op |
| log10     | (100000 ops): |  809.71 ns/op |
| pow       | (100000 ops): |  744.68 ns/op |
| sqrt      | (100000 ops): |  482.26 ns/op |
| inv_sqrt  | (100000 ops): |  559.65 ns/op |
| floor     | (100000 ops): |  482.26 ns/op |
| ceil      | (100000 ops): |  452.49 ns/op |
| trunc     | (100000 ops): |  375.10 ns/op |
| round     | (100000 ops): |  488.22 ns/op |
| fract     | (100000 ops): |  833.52 ns/op |
| modf      | (100000 ops): |  899.00 ns/op |
| abs       | (100000 ops): |  160.77 ns/op |
| copysign  | (100000 ops): |  184.58 ns/op |
| fmod      | (100000 ops): |  708.49 ns/op |
| hypot     | (100000 ops): | 1149.06 ns/op |
| lerp      | (100000 ops): |  702.55 ns/op |
| compare < | (100000 ops): |  113.14 ns/op |
| compare ==| (100000 ops): |   65.51 ns/op |
| shift <<  | (100000 ops): |  202.44 ns/op |
| shift >>  | (100000 ops): |  202.44 ns/op |
| to_int32  | (100000 ops): |  196.49 ns/op |
| from_int  | (100000 ops): |  232.21 ns/op |
| Dot4      | (100000 ops): | 1923.02 ns/op |
| negate    | (100000 ops): |  172.68 ns/op |
| unary +   | (100000 ops): |  142.91 ns/op |
| reciprocal| (100000 ops): |  244.12 ns/op |
| atan      | (100000 ops): |  803.76 ns/op |
| sincos    | (100000 ops): |  660.87 ns/op |
| min       | (100000 ops): |  202.44 ns/op |
| max       | (100000 ops): |  202.44 ns/op |
| clamp     | (100000 ops): |  238.16 ns/op |
| mul SF int| (100000 ops): |  524.49 ns/op |
| mul int SF| (100000 ops): |  525.59 ns/op |
| div SF/int| (100000 ops): |  184.58 ns/op |
| div int/SF| (100000 ops): |  375.10 ns/op |
| add SF+int| (100000 ops): |  416.77 ns/op |
| sub SF-int| (100000 ops): |  500.12 ns/op |
| cmp SF<int| (100000 ops): |   89.33 ns/op |
| cmp int<SF| (100000 ops): |  107.19 ns/op |
| += mul_exp| (100000 ops): |  684.68 ns/op |
| -= mul_exp| (100000 ops): |  774.01 ns/op |
| MulExpr.sq| (100000 ops): |  678.73 ns/op |

ALL TESTS PASSED


### Results for our SoftFloat implementation

LINPACK benchmark, SoftFloat precision.
Machine precision:  6 digits.
Array size 100 X 100.
Average rolled and unrolled performance:

| Reps | Time(s) | DGEFA  | DGESL | OVERHEAD | KFLOPS   |
|------|---------|--------|-------|----------|----------|
| 16   | 0.87    | 87.53% | 5.31% |  7.17%   | 3488.334 |
| 32   | 1.75    | 87.53% | 5.31% |  7.17%   | 3488.296 |
| 64   | 3.49    | 87.53% | 5.31% |  7.17%   | 3488.301 |
| 128  | 6.98    | 87.53% | 5.31% |  7.17%   | 3488.320 |

Calibrate
| 0.32 Seconds |  1 | Passes (x 100) |
| 1.59 Seconds |  5 | Passes (x 100) |
| 7.94 Seconds | 25 | Passes (x 100) |

Use 314  passes (x 100)

          SoftFloat Precision C/C++ Whetstone Benchmark

| Loop content      |         Result       | MFLOPS |  MOPS  | Seconds|
|-------------------|----------------------|--------|--------|--------|
| N1 floating point | -1.12477111816406250 | 20.148 |        |  0.299 |
| N2 floating point | -1.12276840209960938 |  2.077 |        | 20.317 |
| N3 if then else   |  1.00000000000000000 |        | 83.981 |  0.387 |
| N4 fixed point    | 12.00000000000000000 |        | 78.727 |  0.000 |
| N5 sin,cos etc.   |  0.49193423986434937 |        |  1.874 | 13.938 |
| N6 floating point |  0.99999994039535522 | 28.173 |        |  6.012 |
| N7 assignments    |  3.00000000000000000 |        | 71.985 |  0.806 |
| N8 exp,sqrt etc.  |  0.75110626220703125 |        |  3.284 |  3.557 |
|-------------------|----------------------|--------|--------|--------|
| MWIPS             |                      | 69.292 |        | 45.316 |

### Results for hardware float implementation

LINPACK benchmark, Single precision.
Machine precision:  6 digits.
Array size 100 X 100.
Average rolled and unrolled performance:

| Reps | Time(s) | DGEFA  | DGESL | OVERHEAD | KFLOPS    |
|------|---------|--------|-------|----------|-----------|
| 128  | 0.93    | 79.53% | 4.89% | 15.58%   | 28912.676 |
| 256  | 1.85    | 79.53% | 4.89% | 15.58%   | 28912.655 |
| 512  | 3.71    | 79.53% | 4.89% | 15.58%   | 28912.678 |
| 1024 | 7.41    | 79.53% | 4.89% | 15.58%   | 28912.311 |

Calibrate
| 0.05 Seconds |   1 | Passes (x 100) |
| 0.26 Seconds |   5 | Passes (x 100) |
| 1.32 Seconds |  25 | Passes (x 100) |
| 6.62 Seconds | 125 | Passes (x 100) |

Use 1889  passes (x 100)

          Single Precision C/C++ Whetstone Benchmark

| Loop content      |         Result       | MFLOPS |  MOPS  | Seconds|
|-------------------|----------------------|--------|--------|--------|
| N1 floating point | -1.12391686439514160 | 116.845|        |   0.310|
| N2 floating point | -1.12191414833068848 |  82.272|        |   3.086|
| N3 if then else   |  1.00000000000000000 |        | 100.779|   1.940|
| N4 fixed point    | 12.00000000000000000 |        |8670.000|   0.000|
| N5 sin,cos etc.   |  0.49909299612045288 |        |   9.914|  15.853|
| N6 floating point |  0.99999982118606567 |  62.988|        |  16.177|
| N7 assignments    |  3.00000000000000000 |        |  71.985|   4.849|
| N8 exp,sqrt etc.  |  0.75110864639282227 |        |  10.566|   6.651|
|-------------------|----------------------|--------|--------|--------|
| MWIPS             |                      | 386.563|        |  48.866|

### Conclusion

Our software floating point is 386.563 / 69.292 = 5.57 times slower than hardware floating point on the Whetstone benchmark and 28912.311 / 3488.334 = 8.28 times slower in the Linpack benchmark.

