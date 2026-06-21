/*
 * FusedSoftFloat library/header
 * 
 * Copyright (c) 2026 Eduardo José Tagle (ejtagle@hotmail.com)
 * 
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/** @file FusedSoftFloat.hh
 *  @brief SoftFloat optimised for Cortex-M3, ARMv7-M
 *
 *  Packed representation (32 bits):
 *    sign[31], exponent[30:23] biased by 127, mantissa[22:0].
 *    exp == 0 is canonical zero; exp 1..255 are normal values (255 is finite).
 *    The internal unpack helper exposes the original Q29 working mantissa to
 *    the existing algorithms; raw_mantissa_top() exposes the requested
 *    top-aligned implicit-1 mantissa with eight low guard bits.
 */
#pragma once
#include <cstdint>

// If defined, will force inline everything, otherwise, only inlines hot path
#undef MAXIMUM_SPEED

#ifndef INT_MAX
#   define INT_MAX 2147483647
#endif
#ifndef INT_MIN
#   define INT_MIN (-INT_MAX - 1)
#endif

#ifndef __arm__
#define SF_INT_EQUALS_INT32
#endif

// =========================================================================
// Platform detection
// =========================================================================
#if defined(__GNUC__) || defined(__clang__)
# if 1
#   define SF_INLINE    __attribute__((always_inline)) inline
#   define SF_HOT       __attribute__((hot))
#   define SF_FLATTEN   __attribute__((flatten))
#   define SF_PURE      __attribute__((pure))
#   define SF_CONST     __attribute__((const))
#   define LIKELY(x)    __builtin_expect(!!(x), 1)
#   define UNLIKELY(x)  __builtin_expect(!!(x), 0)
#   define IS_CONST(x)  __builtin_constant_p(x)
# else
#   define SF_INLINE    
#   define SF_HOT       
#   define SF_FLATTEN   
#   define SF_PURE      __attribute__((pure))
#   define SF_CONST     __attribute__((const))
#   define LIKELY(x)    __builtin_expect(!!(x), 1)
#   define UNLIKELY(x)  __builtin_expect(!!(x), 0)
#   define IS_CONST(x)  __builtin_constant_p(x)
# endif
#else
#   define SF_INLINE    inline
#   define SF_HOT
#   define SF_FLATTEN
#   define SF_PURE
#   define SF_CONST
#   define LIKELY(x)    (x)
#   define UNLIKELY(x)  (x)
#   define IS_CONST(x)  0
#endif


// =========================================================================
// Consteval detection (no extra #include needed — GCC/Clang built-in)
// =========================================================================
#if defined(__GNUC__) || defined(__clang__)
#   define SF_IS_CONSTEVAL() __builtin_is_constant_evaluated()
#else
#   define SF_IS_CONSTEVAL() false
#endif

class SoftFloat;
struct IntFractPair;
class DeltaAngle;
class Angle;


// =========================================================================
// FixedQ30 - Q30 fixed point
// =========================================================================
class FixedQ30 {
	friend class SoftFloat;
	friend class Angle;
private:
	int32_t raw_;

	[[nodiscard]] static constexpr int32_t float_to_raw(float v) noexcept {
		// Q30 stores value = raw / 2^30.  It can represent approximately
		// [-2.0, +2.0), which comfortably covers sin/cos and normalized factors.
		constexpr float scale = 1073741824.0f; // 2^30

		if (!(v == v)) return 0; // NaN -> 0, keeps construction deterministic
		if (v >= 2.0f) return INT32_MAX;
		if (v <= -2.0f) return INT32_MIN;

		const float scaled = v * scale;
		const float rounded = (scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f);
		return static_cast<int32_t>(rounded);
	}

	[[nodiscard]] static constexpr int32_t float_to_raw(SoftFloat v) noexcept;
	explicit constexpr FixedQ30(int32_t v) noexcept : raw_(v) {}
public:

	constexpr FixedQ30() noexcept : raw_(0) {}
	explicit constexpr FixedQ30(float v) noexcept : raw_(float_to_raw(v)) {}
	explicit constexpr FixedQ30(SoftFloat v) noexcept;

	[[nodiscard]] constexpr SoftFloat to_softfloat() const noexcept;

	[[nodiscard]] constexpr int32_t get_raw() const noexcept { return raw_; }
	[[nodiscard]] static constexpr FixedQ30 from_raw(int32_t raw) noexcept {
		return FixedQ30(raw);
	}

	[[nodiscard]] constexpr bool is_zero() const noexcept { return raw_ == 0; }

	// Exact comparisons between Q30 values.
	[[nodiscard]] constexpr bool operator==(FixedQ30 rhs) const noexcept { return raw_ == rhs.raw_; }
	[[nodiscard]] constexpr bool operator!=(FixedQ30 rhs) const noexcept { return raw_ != rhs.raw_; }
	[[nodiscard]] constexpr bool operator< (FixedQ30 rhs) const noexcept { return raw_ < rhs.raw_; }
	[[nodiscard]] constexpr bool operator> (FixedQ30 rhs) const noexcept { return raw_ > rhs.raw_; }
	[[nodiscard]] constexpr bool operator<=(FixedQ30 rhs) const noexcept { return raw_ <= rhs.raw_; }
	[[nodiscard]] constexpr bool operator>=(FixedQ30 rhs) const noexcept { return raw_ >= rhs.raw_; }

	// Convenience comparisons against float thresholds.  The float is quantized
	// to the same Q30 representation used by the constructor, avoiding a
	// FixedQ30 -> SoftFloat conversion in hot paths.
	[[nodiscard]] constexpr bool operator==(float rhs) const noexcept { return raw_ == float_to_raw(rhs); }
	[[nodiscard]] constexpr bool operator!=(float rhs) const noexcept { return raw_ != float_to_raw(rhs); }
	[[nodiscard]] constexpr bool operator< (float rhs) const noexcept { return raw_ < float_to_raw(rhs); }
	[[nodiscard]] constexpr bool operator> (float rhs) const noexcept { return raw_ > float_to_raw(rhs); }
	[[nodiscard]] constexpr bool operator<=(float rhs) const noexcept { return raw_ <= float_to_raw(rhs); }
	[[nodiscard]] constexpr bool operator>=(float rhs) const noexcept { return raw_ >= float_to_raw(rhs); }

	friend constexpr bool operator==(float lhs, FixedQ30 rhs) noexcept { return FixedQ30(lhs) == rhs; }
	friend constexpr bool operator!=(float lhs, FixedQ30 rhs) noexcept { return FixedQ30(lhs) != rhs; }
	friend constexpr bool operator< (float lhs, FixedQ30 rhs) noexcept { return FixedQ30(lhs) < rhs; }
	friend constexpr bool operator> (float lhs, FixedQ30 rhs) noexcept { return FixedQ30(lhs) > rhs; }
	friend constexpr bool operator<=(float lhs, FixedQ30 rhs) noexcept { return FixedQ30(lhs) <= rhs; }
	friend constexpr bool operator>=(float lhs, FixedQ30 rhs) noexcept { return FixedQ30(lhs) >= rhs; }

	constexpr FixedQ30& operator+=(FixedQ30 rhs) noexcept { raw_ += rhs.raw_; return *this; }
	constexpr FixedQ30& operator-=(FixedQ30 rhs) noexcept { raw_ -= rhs.raw_; return *this; }

	// ------------------------------------------------------------------
	// Clamp — constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr FixedQ30 clamp(FixedQ30 lo, FixedQ30 hi) const noexcept {
		if (*this < lo)
			return lo;
		if (*this > hi)
			return hi;
		return *this;
	}
	[[nodiscard]] constexpr SF_INLINE FixedQ30 clamp(float lo, FixedQ30 hi) const noexcept { return clamp(FixedQ30(lo), hi); }
	[[nodiscard]] constexpr SF_INLINE FixedQ30 clamp(FixedQ30 lo, float hi) const noexcept { return clamp(lo, FixedQ30(hi)); }
	[[nodiscard]] constexpr SF_INLINE FixedQ30 clamp(float lo, float hi) const noexcept { return clamp(FixedQ30(lo), FixedQ30(hi)); }

	[[nodiscard]] constexpr float to_float() const noexcept;

	// Implicit decay to SoftFloat (so FixedQ30 "just works" in most places)
	[[nodiscard]] constexpr operator SoftFloat() const noexcept;

	[[nodiscard]] constexpr FixedQ30 operator-() const noexcept {
		return FixedQ30(-raw_);
	}

	[[nodiscard]] constexpr friend int32_t operator*(FixedQ30 lhs, int32_t rhs) noexcept {
		// Q30 * int32 -> int32, with saturation.  The raw multiplication is
		// exact (Q30 fits in 32 bits), but the result may overflow int32.
		int64_t product = (static_cast<int64_t>(lhs.raw_) * static_cast<int64_t>(rhs)) >> 30;
		if (product > INT32_MAX) return INT32_MAX;
		if (product < INT32_MIN) return INT32_MIN;
		return static_cast<int32_t>(product);
	}
	[[nodiscard]] constexpr friend int32_t operator*(int32_t lhs, FixedQ30 rhs) noexcept {
		// Q30 * int32 -> int32, with saturation.  The raw multiplication is
		// exact (Q30 fits in 32 bits), but the result may overflow int32.
		int64_t product = (static_cast<int64_t>(lhs) * static_cast<int64_t>(rhs.raw_)) >> 30;
		if (product > INT32_MAX) return INT32_MAX;
		if (product < INT32_MIN) return INT32_MIN;
		return static_cast<int32_t>(product);
	}

	[[nodiscard]] static constexpr FixedQ30 zero() noexcept { return FixedQ30(int32_t(0)); }
	[[nodiscard]] static constexpr FixedQ30 one()  noexcept { return FixedQ30(1.0f); }
};

struct SinCosPair { FixedQ30 sin; FixedQ30 cos; };

// =========================================================================
// DeltaAngle - difference between angles (int32_t, 2^32 units = 2π radians)
// Range: [-π, π)
// =========================================================================
class DeltaAngle {
	friend class SoftFloat;
	friend class Angle;
private:
	using raw_t = int32_t;
	static constexpr raw_t RAW_HALF_PI = 0x40000000u;
	static constexpr raw_t RAW_QUARTER_PI = 0x20000000u;

private:
	explicit constexpr DeltaAngle(raw_t raw_) noexcept : raw_(raw_) {}

public:
	constexpr DeltaAngle() noexcept : raw_(0) {}

	[[nodiscard]] constexpr raw_t get_raw() const noexcept { return raw_; }
	[[nodiscard]] static constexpr DeltaAngle from_raw(raw_t raw) noexcept {
		return DeltaAngle(raw);
	}

	// Conversion FROM SoftFloat (treated as signed radians delta; reduced to [-π, π))
	constexpr DeltaAngle(SoftFloat d) noexcept;

	// Conversion TO SoftFloat (returns signed radians delta in [-π, π))
	[[nodiscard]] constexpr operator SoftFloat() const noexcept;

	// Multiplication by SoftFloat
	constexpr DeltaAngle operator*(SoftFloat s) const noexcept;

	// Multiplication by float
	constexpr DeltaAngle operator*(float s) const noexcept;

	// Multiplication by integer
	constexpr DeltaAngle operator*(int32_t k) const noexcept {
		return DeltaAngle(raw_ * k);
	}
#ifndef SF_INT_EQUALS_INT32
	constexpr DeltaAngle operator*(int k) const noexcept {
		return DeltaAngle(raw_ * int32_t(k));
	}
#endif

	friend constexpr DeltaAngle operator*(SoftFloat s, DeltaAngle d) noexcept;

	// Absolute value (returns non-negative DeltaAngle in [0, π))
	[[nodiscard]] constexpr DeltaAngle abs() const noexcept;

	// Unary minus (negation; safe for INT32_MIN boundary)
	constexpr DeltaAngle operator-() const noexcept;

	// Comparisons with other DeltaAngle (direct on raw for exactness/speed, constexpr)
	constexpr bool operator==(DeltaAngle rhs) const noexcept { return raw_ == rhs.raw_; }
	constexpr bool operator!=(DeltaAngle rhs) const noexcept { return raw_ != rhs.raw_; }
	constexpr bool operator< (DeltaAngle rhs) const noexcept { return raw_ < rhs.raw_; }
	constexpr bool operator> (DeltaAngle rhs) const noexcept { return raw_ > rhs.raw_; }
	constexpr bool operator<=(DeltaAngle rhs) const noexcept { return raw_ <= rhs.raw_; }
	constexpr bool operator>=(DeltaAngle rhs) const noexcept { return raw_ >= rhs.raw_; }

	// Comparisons with float (one direction as members; reverses as free funcs below)
	// Implemented via SoftFloat conversion (constexpr, consistent with SoftFloat mixed compares)
	[[nodiscard]] constexpr bool operator==(float rhs) const noexcept;
	[[nodiscard]] constexpr bool operator!=(float rhs) const noexcept;
	[[nodiscard]] constexpr bool operator< (float rhs) const noexcept;
	[[nodiscard]] constexpr bool operator> (float rhs) const noexcept;
	[[nodiscard]] constexpr bool operator<=(float rhs) const noexcept;
	[[nodiscard]] constexpr bool operator>=(float rhs) const noexcept;

	// Arithmetic with other DeltaAngle (result reduced to [-π, π))
	friend constexpr DeltaAngle operator+(DeltaAngle a, DeltaAngle b) noexcept;
	friend constexpr DeltaAngle operator-(DeltaAngle a, DeltaAngle b) noexcept;

	// Ratio of two angular deltas (dimensionless scalar; raw units cancel exactly)
	friend constexpr SoftFloat operator/(DeltaAngle a, DeltaAngle b) noexcept;

	// ------------------------------------------------------------------
	// Clamp — constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr DeltaAngle clamp(DeltaAngle lo, DeltaAngle hi) const noexcept {
		if (*this < lo)
			return lo;
		if (*this > hi)
			return hi;
		return *this;
	}
	[[nodiscard]] constexpr SF_INLINE DeltaAngle clamp(float lo, DeltaAngle hi) const noexcept { return clamp(DeltaAngle(lo), hi); }
	[[nodiscard]] constexpr SF_INLINE DeltaAngle clamp(DeltaAngle lo, float hi) const noexcept { return clamp(lo, DeltaAngle(hi)); }
	[[nodiscard]] constexpr SF_INLINE DeltaAngle clamp(float lo, float hi) const noexcept { return clamp(DeltaAngle(lo), DeltaAngle(hi)); }

	[[nodiscard]] constexpr float to_float() const noexcept;

	// Named constants
	[[nodiscard]] static constexpr DeltaAngle zero()      noexcept { return DeltaAngle(0U); }
	[[nodiscard]] static constexpr DeltaAngle max_value() noexcept { return DeltaAngle(INT32_MAX); }
	[[nodiscard]] static constexpr DeltaAngle pi()        noexcept { return DeltaAngle(INT32_MAX); }
	[[nodiscard]] static constexpr DeltaAngle minus_pi()  noexcept { return DeltaAngle(INT32_MIN); }
	[[nodiscard]] static constexpr DeltaAngle half_pi()   noexcept { return DeltaAngle(RAW_HALF_PI); }
	[[nodiscard]] static constexpr DeltaAngle quarter_pi()noexcept { return DeltaAngle(RAW_QUARTER_PI); }

private:
	raw_t raw_;
};

// =========================================================================
// Angle - modular angle type (uint32_t, 2^32 units = 2π radians)
// =========================================================================
class Angle {
	friend class SoftFloat;
	friend class DeltaAngle;
private:
	using raw_t = uint32_t;
	static constexpr raw_t RAW_PI = 0x80000000u;
	static constexpr raw_t RAW_HALF_PI = 0x40000000u;
	static constexpr raw_t RAW_QUARTER_PI = 0x20000000u;

private:
	raw_t raw_;

private:
	explicit constexpr Angle(raw_t raw) noexcept : raw_(raw) {}

public:

	// Default + raw construction
	constexpr Angle() noexcept : raw_(0) {}

	// Implicit conversion FROM SoftFloat (treated as radians)
	constexpr Angle(SoftFloat r) noexcept;

	// Implicit conversion TO SoftFloat (returns radians)
	[[nodiscard]] constexpr operator SoftFloat() const noexcept;

	[[nodiscard]] constexpr raw_t get_raw() const noexcept { return raw_; }
	[[nodiscard]] static constexpr Angle from_raw(raw_t raw) noexcept {
		return Angle(raw);
	}

	// Arithmetic (wraps automatically)
	constexpr Angle operator+(Angle rhs) const noexcept { return Angle(raw_ + rhs.raw_); }
	constexpr DeltaAngle operator-(Angle rhs) const noexcept { return DeltaAngle(static_cast<int32_t>(raw_ - rhs.raw_)); }

	// Support angle + (angular_velocity * dt)
	constexpr Angle operator+(SoftFloat x) const noexcept;
	constexpr Angle operator-() const noexcept { return Angle(static_cast<raw_t>(-raw_)); }

	// Support Angle +/- DeltaAngle
	constexpr Angle operator+(DeltaAngle d) const noexcept { return Angle(raw_ + static_cast<uint32_t>(d.raw_)); }
	constexpr Angle operator-(DeltaAngle d) const noexcept { return Angle(raw_ - static_cast<uint32_t>(d.raw_)); }

	constexpr Angle& operator+=(Angle rhs) noexcept { raw_ += rhs.raw_; return *this; }
	constexpr Angle& operator-=(Angle rhs) noexcept { raw_ -= rhs.raw_; return *this; }

	// Support angle += angular_velocity * dt  (SoftFloat in radians)
	constexpr Angle& operator+=(SoftFloat x) noexcept;
	constexpr Angle& operator-=(SoftFloat x) noexcept;

	// Support angle += DeltaAngle
	constexpr Angle& operator+=(DeltaAngle d) noexcept { raw_ += static_cast<uint32_t>(d.raw_); return *this; }
	constexpr Angle& operator-=(DeltaAngle d) noexcept { raw_ -= static_cast<uint32_t>(d.raw_); return *this; }

	constexpr Angle operator*(int32_t k) const noexcept {
		return Angle(raw_ * static_cast<raw_t>(k));
	}
	constexpr Angle operator*(uint32_t k) const noexcept {
		return Angle(raw_ * k);
	}
#ifndef SF_INT_EQUALS_INT32
	constexpr Angle operator*(int k) const noexcept {
		return Angle(raw_ * int32_t(k));
	}
#endif

	// Trigonometric functions on Angle — return FixedQ30 for maximum speed
	[[nodiscard]] constexpr FixedQ30 sin() const noexcept;
	[[nodiscard]] constexpr FixedQ30 cos() const noexcept;
	[[nodiscard]] constexpr SoftFloat tan() const noexcept;
	[[nodiscard]] constexpr SinCosPair sincos() const noexcept;

	// Direct float/radian access (delegates to SoftFloat conversion for constexpr)
	[[nodiscard]] constexpr float to_float() const noexcept;
	[[nodiscard]] constexpr explicit operator float() const noexcept;
	[[nodiscard]] constexpr bool is_zero() const noexcept { return raw_ == 0; }

	// Comparisons with other Angle (direct on raw for exactness/speed, constexpr)
	constexpr bool operator==(Angle rhs) const noexcept { return raw_ == rhs.raw_; }
	constexpr bool operator!=(Angle rhs) const noexcept { return raw_ != rhs.raw_; }
	constexpr bool operator< (Angle rhs) const noexcept { return raw_ < rhs.raw_; }
	constexpr bool operator> (Angle rhs) const noexcept { return raw_ > rhs.raw_; }
	constexpr bool operator<=(Angle rhs) const noexcept { return raw_ <= rhs.raw_; }
	constexpr bool operator>=(Angle rhs) const noexcept { return raw_ >= rhs.raw_; }


	// Named constants
	[[nodiscard]] static constexpr Angle zero()      noexcept { return Angle(0U); }
	[[nodiscard]] static constexpr Angle pi()        noexcept { return Angle(RAW_PI); }
	[[nodiscard]] static constexpr Angle half_pi()   noexcept { return Angle(RAW_HALF_PI); }
	[[nodiscard]] static constexpr Angle quarter_pi()noexcept { return Angle(RAW_QUARTER_PI); }
};

// =========================================================================
// SoftFloat class
// =========================================================================
class SoftFloat {
	friend class FixedQ30;
	friend class DeltaAngle;
	friend class Angle;
	friend constexpr SoftFloat operator/(DeltaAngle a, DeltaAngle b) noexcept;
	friend constexpr SoftFloat operator*(SoftFloat a, FixedQ30 b) noexcept;
	friend constexpr SoftFloat operator*(FixedQ30 a, SoftFloat b) noexcept;
public:
	// ------------------------------------------------------------------
	// Nested types
	// ------------------------------------------------------------------
	struct MulExpr;

private:
	// ------------------------------------------------------------------
	// Normalization invariants
	// ------------------------------------------------------------------
	static constexpr uint32_t MANT_MIN      = 0x20000000u; // 2^29
	static constexpr uint32_t MANT_MAX      = 0x3FFFFFFFu; // 2^30 - 1
	static constexpr uint32_t MANT_OVERFLOW = 0x40000000u; // 2^30
	static constexpr uint32_t MANT_TOP_TWO  = 0x60000000u; // bits 30:29 mask
	static constexpr int32_t  MANT_BITS     = 29;
	static constexpr int32_t  EXP_MIN       = 1 - (127 + MANT_BITS);   // exp field 1
	static constexpr int32_t  EXP_MAX       = 255 - (127 + MANT_BITS); // exp field 255 is normal
	static constexpr int32_t  EXP_BIAS      = 127 + MANT_BITS; // Q29 bridge bias: 156

public:
	// ------------------------------------------------------------------
	// Bit-cast helper (C++20 or fallback)
	// ------------------------------------------------------------------
	template<typename To, typename From>
	[[nodiscard]] static constexpr SF_INLINE To bitcast(From v) noexcept
	{
		static_assert(sizeof(To) == sizeof(From), "bitcast: size mismatch");
#if defined(__has_builtin)
#  if __has_builtin(__builtin_bit_cast)
		return __builtin_bit_cast(To, v);
#  else
		To r; __builtin_memcpy(&r, &v, sizeof(To)); return r;
#  endif
#else
		To r; __builtin_memcpy(&r, &v, sizeof(To)); return r;
#endif
	}

private:
	// ------------------------------------------------------------------
	// Cortex-M3 primitives
	// ------------------------------------------------------------------
	[[nodiscard]] static constexpr SF_CONST SF_INLINE int clz(uint32_t x) noexcept
	{
		return __builtin_clz(x);
	}
	[[nodiscard]] static constexpr SF_CONST SF_INLINE int clz64(uint64_t x) noexcept
	{
		uint32_t hi = static_cast<uint32_t>(x >> 32);
		return hi ? clz(hi) : (32 + clz(static_cast<uint32_t>(x)));
	}

	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint32_t abs32(int32_t m) noexcept
	{
		uint32_t mask = static_cast<uint32_t>(m >> 31);
		return (static_cast<uint32_t>(m) ^ mask) - mask;
	}

	// sat_exp / sat_exp_fast removed — all exponent clamping is done inside
	// pack_normalized_bits (the single point where the biased exponent is
	// range-checked against [1, 255]).  The old wrappers were identity
	// functions that just added a call + return overhead on every hot path.

	static constexpr SF_INLINE void normalise_fast(int32_t& m, int32_t& e) noexcept;

private:
	// ------------------------------------------------------------------
	// Data: one packed word.  For algorithm compatibility, get_mantissa()
	// returns the original Q29 working mantissa; raw_mantissa_top() returns
	// the requested top-aligned Q31 mantissa ((bits << 8) | implicit one).
	// ------------------------------------------------------------------
	uint32_t bits;

	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint32_t field_sign(uint32_t v) noexcept { return v & 0x80000000u; }
	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint32_t field_exp (uint32_t v) noexcept { return (v >> 23) & 0xFFu; }
	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint32_t field_mant(uint32_t v) noexcept { return v & 0x007FFFFFu; }

	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint32_t pack_fields(uint32_t s, uint32_t be, uint32_t mf) noexcept {
		return (s & 0x80000000u) | ((be & 0xFFu) << 23) | (mf & 0x007FFFFFu);
	}

	// Fast unpack helpers for the packed representation.  Call mant_from_bits()
	// only when the biased exponent is non-zero.
	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint32_t mant24_from_bits(uint32_t v) noexcept {
		return 0x00800000u | (v & 0x007FFFFFu);
	}
	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint32_t mant_from_bits(uint32_t v) noexcept {
		return 0x20000000u | ((v & 0x007FFFFFu) << 6);
	}
	[[nodiscard]] static constexpr SF_CONST SF_INLINE int32_t exp_from_biased(uint32_t be) noexcept {
		return static_cast<int32_t>(be) - EXP_BIAS;
	}
	[[nodiscard]] static constexpr SF_CONST SF_INLINE uint16_t sign_from_bits(uint32_t v) noexcept {
		return static_cast<uint16_t>(v >> 31);
	}
	[[nodiscard]] static constexpr SF_INLINE uint32_t mul24_to_q29(uint32_t a, uint32_t b) noexcept {
#if defined(__arm__)
		if (!SF_IS_CONSTEVAL()) {
			uint32_t lo, hi;
			__asm__("umull %0, %1, %2, %3"
				: "=&r"(lo), "=&r"(hi)
				: "r"(a), "r"(b));
			return (hi << 15) | (lo >> 17);
		}
#endif
		return static_cast<uint32_t>((static_cast<uint64_t>(a) * static_cast<uint64_t>(b)) >> 17);
	}

	[[nodiscard]] static constexpr SF_INLINE uint32_t pack_normalized_bits(uint32_t m, int32_t e, uint16_t neg = 0) noexcept {
		if (UNLIKELY(m == 0)) return 0u;

#if defined(__arm__)
		if (!SF_IS_CONSTEVAL()) {
			uint32_t s = static_cast<uint32_t>(neg) << 31;
			int32_t be_m1 = e + (EXP_BIAS - 1);
			
                        // Combined range check: unsigned comparison catches both
                        // be_m1 < 0 (underflow) and be_m1 > 254 (overflow) in one branch.
                        if (UNLIKELY(static_cast<uint32_t>(be_m1) > 254u)) {
                                if (be_m1 < 0) return 0u;
                                return s | 0x7FFFFFFFu;
                        }

			uint32_t out;
			__asm__(
			    "lsrs  %[out], %[m], #6\n\t"              // Q29 -> mant24, C = bit 5
			    "adc.w %[out], %[out], %[be_m1], lsl #23\n\t"
			    : [out] "=&r"(out)
			    : [m] "r"(m),[be_m1] "r"(be_m1)
			    : "cc");

			// If be == 255 and rounding overflowed, ADC can carry into bit 31.
			// Clamp to your finite-saturation encoding.
			if (UNLIKELY(out & 0x80000000u))
				out = 0x7FFFFFFFu;

			return s | out;
		}
#endif

		// Standard portable path (and consteval path)
		m += 0x20u;
		uint32_t overflowed = m >> 30;
		m >>= overflowed;
		e += static_cast<int32_t>(overflowed);

		int32_t be = e + EXP_BIAS;
		uint32_t s = static_cast<uint32_t>(neg) << 31;
		if (UNLIKELY(be <= 0)) return 0u;
		if (UNLIKELY(be > 255)) return s | 0x7FFFFFFFu;
		return pack_fields(s, static_cast<uint32_t>(be), (m >> 6) & 0x007FFFFFu);
	}

	[[nodiscard]] static constexpr SF_INLINE uint32_t pack_int32_bits(int32_t v) noexcept {
		uint32_t uv = static_cast<uint32_t>(v);
		uint32_t neg = uv >> 31;
		uint32_t mask = 0u - neg;
		uint32_t a = (uv ^ mask) - mask;
		if (UNLIKELY(a == 0u)) return 0u;

		int top = 31 - clz(a);
		uint32_t mant24;
		uint32_t be = static_cast<uint32_t>(top + 127);
		if (top <= 23) {
			mant24 = a << (23 - top);
		} else {
			int sh = top - 23;
			mant24 = a >> sh;
			uint32_t rem = a & ((1u << sh) - 1u);
			uint32_t half = 1u << (sh - 1);
			mant24 += (rem > half) | ((rem == half) & (mant24 & 1u));
			if (UNLIKELY(mant24 >= 0x01000000u)) {
				mant24 >>= 1;
				be += 1;
			}
		}
		return (neg << 31) | (be << 23) | (mant24 & 0x007FFFFFu);
	}

	[[nodiscard]] static constexpr SF_INLINE uint32_t pack_components_bits(uint32_t m, int32_t e, uint16_t neg = 0) noexcept {
		if (UNLIKELY(m == 0)) return 0u;

		// Single normalization pass
		if (UNLIKELY(m >= MANT_OVERFLOW)) {
			m >>= 1;
			e += 1;
		}
		if (UNLIKELY(m < MANT_MIN)) {
			int sh = clz(m) - 2;
			m <<= sh;
			e -= sh;
		}

		// Round and check for carry
		m += 0x20u;
		uint32_t overflowed = m >> 30;
		m >>= overflowed;
		e += static_cast<int32_t>(overflowed);

		int32_t be = e + EXP_BIAS;
		uint32_t s = static_cast<uint32_t>(neg) << 31;
		if (UNLIKELY(be <= 0)) return 0u;
		if (UNLIKELY(be > 255)) return s | 0x7FFFFFFFu;
		return pack_fields(s, static_cast<uint32_t>(be), (m >> 6) & 0x007FFFFFu);
	}

	constexpr SF_INLINE void set_components(uint32_t m, int32_t e, uint16_t neg = 0) noexcept {
		bits = pack_components_bits(m, e, neg);
	}
	constexpr SF_INLINE void set_zero() noexcept { bits = 0u; }

private:
	[[nodiscard]] constexpr SF_CONST SF_INLINE uint32_t raw_bits() const noexcept { return bits; }
	[[nodiscard]] static constexpr SF_INLINE SoftFloat from_bits(uint32_t b) noexcept {
		SoftFloat r;
		uint32_t be = field_exp(b);
		r.bits = b & (0u - static_cast<uint32_t>(be != 0u)); // canonicalise exp=0 to zero; exp=255 is normal
		return r;
	}

	[[nodiscard]] constexpr SF_CONST SF_INLINE uint32_t raw_mantissa_top() const noexcept {
		uint32_t be = field_exp(bits);
		return be ? (0x80000000u | (field_mant(bits) << 8)) : 0u;
	}
	[[nodiscard]] constexpr SF_CONST SF_INLINE uint32_t get_mantissa() const noexcept {
		uint32_t be = field_exp(bits);
		return be ? mant_from_bits(bits) : 0u; // Q29 bridge for the existing algorithms
	}
	[[nodiscard]] constexpr SF_CONST SF_INLINE int32_t get_exponent() const noexcept {
		uint32_t be = field_exp(bits);
		return be ? exp_from_biased(be) : 0;
	}
	[[nodiscard]] constexpr SF_CONST SF_INLINE uint16_t get_negative() const noexcept {
		return sign_from_bits(bits); // zero is canonical, so its sign bit is clear
	}

private:
	// Build SoftFloat from already-normalized Q29 mantissa (caller guarantees
	// invariants).  pack_normalized_bits handles exponent clamping.
	[[nodiscard]] static constexpr SF_INLINE SoftFloat from_raw_normalized(uint32_t m, int32_t e, uint16_t neg = 0) noexcept {
		SoftFloat r; r.bits = pack_normalized_bits(m, e, neg); return r;
	}
	[[nodiscard]] static constexpr SF_INLINE SoftFloat from_bits_unchecked(uint32_t b) noexcept {
		SoftFloat r; r.bits = b; return r;
	}
	// Build a SoftFloat from an unsigned 64-bit mantissa times a binary scale.
	// Used by angle conversions to avoid multiple SoftFloat mul/div operations.
	[[nodiscard]] static constexpr SF_INLINE SoftFloat from_u64_scaled(uint64_t p, int32_t e, uint16_t neg = 0) noexcept {
		if (UNLIKELY(p == 0)) return zero();
		int top = 63 - clz64(p);
		int sh = top - MANT_BITS;
		uint32_t m;
		if (LIKELY(sh > 0)) {
			uint64_t rounded = (p + (uint64_t(1) << (sh - 1))) >> sh;
			if (UNLIKELY(rounded >= MANT_OVERFLOW)) {
				rounded >>= 1;
				e += sh + 1;
			} else {
				e += sh;
			}
			m = static_cast<uint32_t>(rounded);
		} else {
			m = static_cast<uint32_t>(p << (-sh));
			e += sh;
		}
		return from_raw_normalized(m, e, neg);
	}

#define mantissa get_mantissa()
#define exponent get_exponent()
#define negative get_negative()

	// finish_addsub (signed wrapper) removed — never called.  All callers
	// use finish_addsub_u directly with unsigned magnitude + separate sign.

#ifdef MAXIMUM_SPEED

	[[nodiscard]] static constexpr SF_INLINE SoftFloat finish_addsub_u(uint32_t rm, int32_t re, uint16_t neg) noexcept {
		if (UNLIKELY(rm == 0)) return zero();

		uint32_t ov = rm >> 30;
		rm >>= ov;
		re  += static_cast<int32_t>(ov);

		int lz = clz(rm);
		int sh = lz - 2;
		uint32_t under_rm = rm << sh;
		int32_t  under_re = re - sh;

		// branchless selection: need_shift = (no overflow) && (top two bits != MANT_MIN)
		uint32_t need_shift = (ov == 0) & ((rm & MANT_TOP_TWO) != MANT_MIN);
		rm = need_shift ? under_rm : rm;
		re = need_shift ? under_re : re;

		return from_raw_normalized(rm, re, neg);
	}

	// Subtraction finalizer for |large|-|small|.  Unlike the generic
	// finish_addsub_u(), the magnitude difference can never overflow above the
	// Q29 interval; the common case is already normalized and only needs the
	// final pack/round.  This mirrors qfplib's split subtract path, where the
	// expensive CLZ normalization is only taken after real cancellation.
	[[nodiscard]] static constexpr SF_INLINE SoftFloat finish_sub_mag(uint32_t rm, int32_t re, uint16_t neg) noexcept {
		if (UNLIKELY(rm == 0)) return zero();
#if defined(__arm__)
		if (!SF_IS_CONSTEVAL()) {
			uint32_t lz;
			__asm__("clz %0, %1" : "=r"(lz) : "r"(rm));
			int32_t shift = static_cast<int32_t>(lz) - 2;
			return from_raw_normalized(rm << shift, re - shift, neg);
		}
#endif
		if (LIKELY(rm >= MANT_MIN)) return from_raw_normalized(rm, re, neg);
		int sh = clz(rm) - 2;
		return from_raw_normalized(rm << sh, re - sh, neg);
	}
	
#else

	// -----------------------------------------------------------------
	// Hot/cold split for the add/subtract finalisers.
	//
	// The common case (result mantissa already normalised in [2^29, 2^30))
	// is kept inline — it only needs the rounding + pack.  The rare case
	// (cancellation shrank the result below 2^29, requiring a CLZ-based
	// renormalise) is delegated to renorm_slow(), a single noinline+cold
	// function.  This implements the requested strategy: inline the hot
	// path, forbid inlining of the cold path, saving FLASH (the CLZ
	// renormalise code is emitted once, not duplicated at every call site)
	// at the cost of slightly slower execution only when real cancellation
	// occurs.
	// -----------------------------------------------------------------
	[[gnu::noinline]] static constexpr
	SoftFloat renorm_slow(uint32_t rm, int32_t re, uint16_t neg) noexcept {
		int sh = clz(rm) - 2;
		return from_raw_normalized(rm << sh, re - sh, neg);
	}

	[[nodiscard]] static constexpr SF_INLINE
	SoftFloat finish_addsub_u(uint32_t rm, int32_t re, uint16_t neg) noexcept {
		if (UNLIKELY(rm == 0)) return zero();

		uint32_t ov = rm >> 30;
		rm >>= ov;
		re  += static_cast<int32_t>(ov);

		// HOT (inlined): after the carry-out shift the mantissa is already
		// normalised — straight to pack, skipping the CLZ entirely.
		if (LIKELY(ov || (rm & MANT_TOP_TWO) == MANT_MIN))
			return from_raw_normalized(rm, re, neg);

		// COLD (out-of-line): genuine cancellation.
		return renorm_slow(rm, re, neg);
	}

	// Subtraction finalizer for |large|-|small|.  The magnitude difference
	// can never overflow above the Q29 interval; the common case is already
	// normalised and only needs the final pack/round.
	[[nodiscard]] static constexpr SF_INLINE
	SoftFloat finish_sub_mag(uint32_t rm, int32_t re, uint16_t neg) noexcept {
		if (UNLIKELY(rm == 0)) return zero();
		// HOT (inlined): no cancellation.
		if (LIKELY(rm >= MANT_MIN)) return from_raw_normalized(rm, re, neg);
		// COLD (out-of-line): cancellation shrank the result below 2^29.
		return renorm_slow(rm, re, neg);
	}
	
#endif

	// Same-sign addition finalizer.  When two normalized Q29 mantissas
	// (each in [2^29, 2^30)) are added, the sum is always in [2^30, 2^31): it
	// can never be too small, so it needs at most a single 1-bit down-shift and
	// NEVER a CLZ renormalize.  This mirrors qfplib's carry-driven add path and
	// avoids the general finish_addsub_u (clz + branchless select) on the hot
	// same-sign add/FMA path.
	[[nodiscard]] static constexpr SF_INLINE SoftFloat finish_add_samesign(uint32_t rm, int32_t re, uint16_t neg) noexcept {
		uint32_t ov = rm >> 30;           // 0 or 1 (sum < 2^31)
		rm >>= ov;
		re += static_cast<int32_t>(ov);
                // rm is guaranteed >= MANT_MIN (2^29) here, so we can skip
                // the m == 0 check that pack_normalized_bits does.
                // Inline the pack to save 2 instructions on the hot path.
#if defined(__arm__)
                if (!SF_IS_CONSTEVAL()) {
                        uint32_t s = static_cast<uint32_t>(neg) << 31;
                        int32_t be_m1 = re + (EXP_BIAS - 1);
                        // Combined range check: unsigned comparison catches both
                        // be_m1 < 0 (underflow) and be_m1 > 254 (overflow) in one
                        // branch instead of two.
                        if (UNLIKELY(static_cast<uint32_t>(be_m1) > 254u)) {
                                if (be_m1 < 0) return zero();
                                return from_bits_unchecked(s | 0x7FFFFFFFu);
                        }
                        uint32_t out;
                        __asm__(
                            "lsrs  %[out], %[m], #6\n\t"
                            "adc.w %[out], %[out], %[be_m1], lsl #23\n\t"
                            : [out] "=&r"(out)
                            : [m] "r"(rm), [be_m1] "r"(be_m1)
                            : "cc");
                        if (UNLIKELY(out & 0x80000000u))
                                out = 0x7FFFFFFFu;
                        return from_bits_unchecked(s | out);
                }
#endif
		return from_raw_normalized(rm, re, neg);
	}

	[[nodiscard]] static constexpr uint64_t isqrt64(uint64_t n) noexcept {
		if (n < 2) return n;
		uint64_t lo = 1, hi = n >> 1;
		if (hi > 0xFFFFFFFFULL) hi = 0xFFFFFFFFULL;
		while (lo <= hi) {
			uint64_t mid = lo + ((hi - lo) >> 1);
			uint64_t sq = mid * mid;
			if (sq == n) return mid;
			if (sq < n) lo = mid + 1;
			else hi = mid - 1;
		}
		return hi;
	}

	[[nodiscard]] static constexpr SF_INLINE SF_FLATTEN
	SoftFloat mul_plain(SoftFloat a, SoftFloat b) noexcept
	{
		const uint32_t ab = a.bits;
		const uint32_t bb = b.bits;
		const uint32_t abe = field_exp(ab);
		const uint32_t bbe = field_exp(bb);
		if (UNLIKELY((abe == 0u) | (bbe == 0u))) return zero();

#if defined(__arm__)
		// Cortex-M3 fast path: qfplib-m3's fmul kernel.  One UMULL, then a single
		// ADDS reconstructs the full product x*y from (x-1)*y + y; its carry flag
		// selects the [1,2) vs [2,4) case, and the round bit (carry-out of the
		// final LSR) is folded into the exponent+mantissa pack by an ADC.  This is
		// the real-hardware-carry version of the rounding (portable C++ cannot
		// express the flag dependencies), verified bit-identical to the portable
		// path over 8M random products.  Denormal result -> +0 (canonical),
		// overflow / rounding carry into bit31 -> max finite 0x7FFFFFFF.
		if (!SF_IS_CONSTEVAL()) {
			uint32_t r0 = ab, r1 = bb, r2, r3, r12;
			__asm__(
				"eor   %[r2], %[r0], %[r1]\n\t"
				"and   %[r12], %[r2], #0x80000000\n\t"      // result sign
				"ubfx  %[r3], %[r1], #23, #8\n\t"           // y exponent
				"ubfx  %[r2], %[r0], #23, #8\n\t"           // x exponent
				"add   %[r3], %[r3], %[r2]\n\t"             // exp + 254
				"lsls  %[r0], %[r0], #9\n\t"                // x Q32, implicit 1 absent
				"lsls  %[r1], %[r1], #8\n\t"                // y Q31 ...
				"orr   %[r1], %[r1], #0x80000000\n\t"       // ... implicit 1 present
				"umull %[r2], %[r0], %[r0], %[r1]\n\t"      // (x-1)*y
				"adds  %[r0], %[r0], %[r1]\n\t"             // x*y ; C => result in [2,4)
				"bcc   1f\n\t"
				"subs  %[r3], %[r3], #126\n\t"              // [2,4): implicit 1 present
				"ble   3f\n\t"                              // underflow -> 0
				"cmp   %[r3], #255\n\t"
				"bgt   4f\n\t"                              // exp > 255 -> saturate
				"lsrs  %[r0], %[r0], #9\n\t"                // C = round bit
				"adc   %[r0], %[r0], %[r3], lsl #23\n\t"    // round + pack exp
				"b     5f\n\t"
				"1:\n\t"
				"subs  %[r3], %[r3], #128\n\t"              // [1,2): implicit 1 folds via add
				"blt   3f\n\t"                              // underflow -> 0  (note: < not <=)
				"cmp   %[r3], #255\n\t"
				"bgt   4f\n\t"
				"lsrs  %[r0], %[r0], #8\n\t"
				"adc   %[r0], %[r0], %[r3], lsl #23\n\t"
				"b     5f\n\t"
				"4:\n\t"
				"mvn   %[r0], #0x80000000\n\t"              // 0x7FFFFFFF (max finite)
				"b     6f\n\t"
				"3:\n\t"
				"movs  %[r0], #0\n\t"
				"movs  %[r12], #0\n\t"                      // canonical +0 (clear sign)
				"b     6f\n\t"
				"5:\n\t"
				"tst   %[r0], #0x80000000\n\t"              // rounding carried into exp 256?
				"it    ne\n\t"
				"mvnne %[r0], #0x80000000\n\t"              // -> saturate
				"6:\n\t"
				"orr   %[r0], %[r0], %[r12]\n\t"            // apply sign
				: [r0] "+r"(r0), [r1] "+r"(r1), [r2] "=&r"(r2), [r3] "=&r"(r3), [r12] "=&r"(r12)
				: : "cc");
			SoftFloat r; r.bits = r0; return r;
		}
#endif

		// Portable / consteval path: direct IEEE-field multiply, round to nearest
		// even.  Bit-identical to the asm fast path above.
		const uint32_t ma = mant24_from_bits(ab);
		const uint32_t mb = mant24_from_bits(bb);
		const uint64_t prod = static_cast<uint64_t>(ma) * static_cast<uint64_t>(mb);

		int32_t be = static_cast<int32_t>(abe) + static_cast<int32_t>(bbe) - 127;
		uint32_t mant24;
		if (LIKELY(prod & (1ull << 47))) {
			mant24 = static_cast<uint32_t>((prod + ((1ull << 23) - 1u) + ((prod >> 24) & 1u)) >> 24);
			be += 1;
		} else {
			mant24 = static_cast<uint32_t>((prod + ((1ull << 22) - 1u) + ((prod >> 23) & 1u)) >> 23);
		}
		if (UNLIKELY(mant24 >= 0x01000000u)) {
			mant24 >>= 1;
			be += 1;
		}

		SoftFloat r;
		uint32_t sign = (ab ^ bb) & 0x80000000u;
		if (UNLIKELY(be <= 0)) { r.bits = 0u; return r; }
		if (UNLIKELY(be > 255)) { r.bits = sign | 0x7FFFFFFFu; return r; }
		r.bits = sign | (static_cast<uint32_t>(be) << 23) | (mant24 & 0x007FFFFFu);
		return r;
	}

	constexpr SF_HOT void from_float(float f) noexcept {
		uint32_t b = bitcast<uint32_t>(f);
		uint32_t be = field_exp(b);
		if (UNLIKELY(be == 0u)) { set_zero(); return; } // zero/subnormal -> canonical zero
		if (UNLIKELY(be == 0xFFu)) { bits = (b & 0x80000000u) | 0x7FFFFFFFu; return; }
		// For finite normal IEEE inputs the packed representation is bit-identical.
		bits = b;
	}

	// =========================================================================
	// sqrt approximation tables (Q7).  Generated independently from the
	// midpoint formulae below, not hand-copied from qfplib:
	//   even exponent: min(round(sqrt(2 + (i+0.5)/64)  * 128), 255)
	//   odd  exponent:     round(sqrt(1 + (i+0.5)/128) * 128)
	// They seed the Cortex-M3 direct-sqrt path; the final remainder correction
	// below brings the result to correctly-rounded IEEE/Fused float precision.
	// =========================================================================
	static constexpr uint8_t SQRT_EVEN_Q7[128] = {
		181, 182, 183, 183, 184, 185, 186, 186, 187, 188, 188, 189, 190, 190, 191, 192,
		192, 193, 194, 194, 195, 196, 196, 197, 198, 198, 199, 200, 200, 201, 201, 202,
		203, 203, 204, 205, 205, 206, 206, 207, 208, 208, 209, 210, 210, 211, 211, 212,
		213, 213, 214, 214, 215, 216, 216, 217, 217, 218, 219, 219, 220, 220, 221, 221,
		222, 223, 223, 224, 224, 225, 225, 226, 227, 227, 228, 228, 229, 229, 230, 230,
		231, 232, 232, 233, 233, 234, 234, 235, 235, 236, 237, 237, 238, 238, 239, 239,
		240, 240, 241, 241, 242, 242, 243, 243, 244, 244, 245, 246, 246, 247, 247, 248,
		248, 249, 249, 250, 250, 251, 251, 252, 252, 253, 253, 254, 254, 255, 255, 255
	};
	static constexpr uint8_t SQRT_ODD_Q7[128] = {
		128, 129, 129, 130, 130, 131, 131, 132, 132, 133, 133, 134, 134, 135, 135, 136,
		136, 136, 137, 137, 138, 138, 139, 139, 140, 140, 141, 141, 142, 142, 142, 143,
		143, 144, 144, 145, 145, 146, 146, 146, 147, 147, 148, 148, 149, 149, 149, 150,
		150, 151, 151, 152, 152, 152, 153, 153, 154, 154, 155, 155, 155, 156, 156, 157,
		157, 157, 158, 158, 159, 159, 159, 160, 160, 161, 161, 161, 162, 162, 163, 163,
		163, 164, 164, 165, 165, 165, 166, 166, 166, 167, 167, 168, 168, 168, 169, 169,
		170, 170, 170, 171, 171, 171, 172, 172, 173, 173, 173, 174, 174, 174, 175, 175,
		175, 176, 176, 177, 177, 177, 178, 178, 178, 179, 179, 179, 180, 180, 180, 181
	};

	// exp stage-2 tables, independently generated from:
	//   EXP_E_Q8[i]      = round(exp(i/128) * 256)
	//   EXP_LN_E_Q29[i]  = round(log(EXP_E_Q8[i]/256) * 2^29)
	static constexpr uint16_t EXP_E_Q8[90] = {
		0x0100, 0x0102, 0x0104, 0x0106, 0x0108, 0x010A, 0x010C, 0x010E, 0x0111, 0x0113,
		0x0115, 0x0117, 0x0119, 0x011B, 0x011E, 0x0120, 0x0122, 0x0124, 0x0127, 0x0129,
		0x012B, 0x012E, 0x0130, 0x0132, 0x0135, 0x0137, 0x013A, 0x013C, 0x013F, 0x0141,
		0x0144, 0x0146, 0x0149, 0x014B, 0x014E, 0x0151, 0x0153, 0x0156, 0x0158, 0x015B,
		0x015E, 0x0161, 0x0163, 0x0166, 0x0169, 0x016C, 0x016F, 0x0172, 0x0174, 0x0177,
		0x017A, 0x017D, 0x0180, 0x0183, 0x0186, 0x0189, 0x018D, 0x0190, 0x0193, 0x0196,
		0x0199, 0x019C, 0x01A0, 0x01A3, 0x01A6, 0x01A9, 0x01AD, 0x01B0, 0x01B3, 0x01B7,
		0x01BA, 0x01BE, 0x01C1, 0x01C5, 0x01C8, 0x01CC, 0x01D0, 0x01D3, 0x01D7, 0x01DB,
		0x01DE, 0x01E2, 0x01E6, 0x01EA, 0x01ED, 0x01F1, 0x01F5, 0x01F9, 0x01FD, 0x0201
	};
	static constexpr int32_t EXP_LN_E_Q29[90] = {
		0x00000000, 0x003FC055, 0x007F02A3, 0x00BDC8D8, 0x00FC14D8, 0x0139E87C,
		0x0177458F, 0x01B42DD7, 0x020EB307, 0x024A7EC6, 0x0285DB98, 0x02C0CB15,
		0x02FB4ECE, 0x03356849, 0x038BCA92, 0x03C4E0EE, 0x03FD9226, 0x0435DF9F,
		0x04899BC6, 0x04C0F5C6, 0x04F7F0AC, 0x0549B979, 0x057FCC1C, 0x05B583FA,
		0x06057028, 0x063A4A38, 0x0688EF08, 0x06BCF254, 0x070A5A15, 0x073D8D54,
		0x0789C1DC, 0x07BC2B73, 0x08073623, 0x0838DC30, 0x0882C5FD, 0x08CC0697,
		0x08FC7FCF, 0x0944AD0A, 0x0974715D, 0x09BB9331, 0x0A021843, 0x0A480340,
		0x0A764B99, 0x0ABB3B8C, 0x0AFF9838, 0x0B43640F, 0x0B86A171, 0x0BC952B0,
		0x0BF57C1E, 0x0C3748CD, 0x0C788F44, 0x0CB951A1, 0x0CF991F6, 0x0D39524B,
		0x0D789499, 0x0DB75ACF, 0x0E0A4FF9, 0x0E47FBE4, 0x0E8531D7, 0x0EC1F393,
		0x0EFE42CD, 0x0F3A2131, 0x0F8947B0, 0x0FC4251B, 0x0FFE9704, 0x10389EF0,
		0x10855C88, 0x10BE72E4, 0x10F7241D, 0x11422025, 0x1179EABC, 0x11C3B81F,
		0x11FAA34C, 0x12434B6F, 0x12795E13, 0x12C0E9ED, 0x1307D733, 0x133CA2BA,
		0x138280FE, 0x13C7C800, 0x13FB5B85, 0x143F9FE3, 0x148353D2, 0x14C679B0,
		0x14F87A3F, 0x153AAD06, 0x157C57F3, 0x15BD7D31, 0x15FE1EDB, 0x163E3F00
	};

	// -ln(1+i/128) in Q31, generated independently from that formula.
	static constexpr uint32_t LN_NEG_Q31[129] = {
		0x00000000, 0xFF00FEAD, 0xFE03F575, 0xFD08DC9F, 0xFC0FAC9E, 0xFB185E12,
		0xFA22E9C2, 0xF92F48A4, 0xF83D73D0, 0xF74D6489, 0xF65F1435, 0xF5727C60,
		0xF48796BB, 0xF39E5D15, 0xF2B6C963, 0xF1D0D5B8, 0xF0EC7C49, 0xF009B767,
		0xEF288183, 0xEE48D52B, 0xED6AAD08, 0xEC8E03E0, 0xEBB2D493, 0xEAD91A1C,
		0xEA00CF8F, 0xE929F019, 0xE85476FE, 0xE7805F9B, 0xE6ADA563, 0xE5DC43E0,
		0xE50C36B1, 0xE43D798C, 0xE3700838, 0xE2A3DE95, 0xE1D8F892, 0xE10F5234,
		0xE046E793, 0xDF7FB4D7, 0xDEB9B63B, 0xDDF4E80D, 0xDD3146A9, 0xDC6ECE7D,
		0xDBAD7C08, 0xDAED4BD8, 0xDA2E3A8A, 0xD97044CB, 0xD8B36755, 0xD7F79EF3,
		0xD73CE87C, 0xD68340D5, 0xD5CAA4F2, 0xD51311D1, 0xD45C8481, 0xD3A6FA1B,
		0xD2F26FC3, 0xD23EE2AC, 0xD18C5013, 0xD0DAB540, 0xD02A0F89, 0xCF7A5C4C,
		0xCECB98F2, 0xCE1DC2F2, 0xCD70D7C8, 0xCCC4D4FE, 0xCC19B827, 0xCB6F7EDE,
		0xCAC626CB, 0xCA1DAD9B, 0xC9761108, 0xC8CF4ED3, 0xC82964C5, 0xC78450B0,
		0xC6E01071, 0xC63CA1E9, 0xC59A0304, 0xC4F831B5, 0xC4572BF6, 0xC3B6EFCB,
		0xC3177B3C, 0xC278CC5B, 0xC1DAE141, 0xC13DB80B, 0xC0A14EE1, 0xC005A3EF,
		0xBF6AB569, 0xBED08189, 0xBE370690, 0xBD9E42C3, 0xBD06346F, 0xBC6ED9E9,
		0xBBD83187, 0xBB4239AA, 0xBAACF0B4, 0xBA185511, 0xB984652E, 0xB8F11F82,
		0xB85E8286, 0xB7CC8CBA, 0xB73B3CA1, 0xB6AA90C6, 0xB61A87B6, 0xB58B2005,
		0xB4FC584B, 0xB46E2F24, 0xB3E0A333, 0xB353B31C, 0xB2C75D8A, 0xB23BA12B,
		0xB1B07CB3, 0xB125EED8, 0xB09BF656, 0xB01291ED, 0xAF89C05F, 0xAF018074,
		0xAE79D0F8, 0xADF2B0B8, 0xAD6C1E89, 0xACE61941, 0xAC609FB9, 0xABDBB0D0,
		0xAB574B67, 0xAAD36E62, 0xAA5018A9, 0xA9CD4929, 0xA94AFECF, 0xA8C9388E,
		0xA847F55B, 0xA7C7342F, 0xA746F404
	};

	// tan(i*pi/(4*1024)) in Q30 for Angle::tan table interpolation.
	static constexpr uint32_t TAN_PI4_Q30[1025] = {
		0x00000000u, 0x000C90FEu, 0x001921FDu, 0x0025B2FDu, 0x00324401u, 0x003ED509u, 0x004B6615u, 0x0057F727u,
		0x00648840u, 0x00711961u, 0x007DAA8Au, 0x008A3BBDu, 0x0096CCFBu, 0x00A35E44u, 0x00AFEF9Au, 0x00BC80FEu,
		0x00C91270u, 0x00D5A3F2u, 0x00E23584u, 0x00EEC727u, 0x00FB58DDu, 0x0107EAA7u, 0x01147C84u, 0x01210E77u,
		0x012DA081u, 0x013A32A1u, 0x0146C4DAu, 0x0153572Cu, 0x015FE998u, 0x016C7C20u, 0x01790EC3u, 0x0185A184u,
		0x01923462u, 0x019EC760u, 0x01AB5A7Eu, 0x01B7EDBCu, 0x01C4811Du, 0x01D114A0u, 0x01DDA848u, 0x01EA3C14u,
		0x01F6D006u, 0x0203641Fu, 0x020FF85Fu, 0x021C8CC9u, 0x0229215Cu, 0x0235B61Au, 0x02424B03u, 0x024EE019u,
		0x025B755Du, 0x02680AD0u, 0x0274A071u, 0x02813644u, 0x028DCC48u, 0x029A627Eu, 0x02A6F8E8u, 0x02B38F87u,
		0x02C0265Bu, 0x02CCBD65u, 0x02D954A6u, 0x02E5EC20u, 0x02F283D4u, 0x02FF1BC1u, 0x030BB3EAu, 0x03184C4Fu,
		0x0324E4F1u, 0x03317DD2u, 0x033E16F2u, 0x034AB051u, 0x035749F2u, 0x0363E3D6u, 0x03707DFCu, 0x037D1866u,
		0x0389B316u, 0x03964E0Bu, 0x03A2E948u, 0x03AF84CCu, 0x03BC209Au, 0x03C8BCB1u, 0x03D55914u, 0x03E1F5C2u,
		0x03EE92BDu, 0x03FB3006u, 0x0407CD9Fu, 0x04146B87u, 0x042109BFu, 0x042DA84Au, 0x043A4728u, 0x0446E659u,
		0x045385DFu, 0x046025BBu, 0x046CC5EEu, 0x04796678u, 0x0486075Bu, 0x0492A898u, 0x049F4A30u, 0x04ABEC23u,
		0x04B88E73u, 0x04C53121u, 0x04D1D42Du, 0x04DE779Au, 0x04EB1B66u, 0x04F7BF95u, 0x05046426u, 0x0511091Bu,
		0x051DAE74u, 0x052A5433u, 0x0536FA59u, 0x0543A0E6u, 0x055047DCu, 0x055CEF3Bu, 0x05699705u, 0x05763F3Bu,
		0x0582E7DDu, 0x058F90EDu, 0x059C3A6Cu, 0x05A8E45Au, 0x05B58EB8u, 0x05C23989u, 0x05CEE4CCu, 0x05DB9082u,
		0x05E83CADu, 0x05F4E94Eu, 0x06019665u, 0x060E43F4u, 0x061AF1FBu, 0x0627A07Cu, 0x06344F78u, 0x0640FEEFu,
		0x064DAEE3u, 0x065A5F55u, 0x06671045u, 0x0673C1B5u, 0x068073A5u, 0x068D2617u, 0x0699D90Cu, 0x06A68C85u,
		0x06B34082u, 0x06BFF505u, 0x06CCAA0Eu, 0x06D95FA0u, 0x06E615BAu, 0x06F2CC5Eu, 0x06FF838Cu, 0x070C3B47u,
		0x0718F38Eu, 0x0725AC63u, 0x073265C7u, 0x073F1FBBu, 0x074BDA3Fu, 0x07589556u, 0x076550FFu, 0x07720D3Cu,
		0x077ECA0Fu, 0x078B8777u, 0x07984576u, 0x07A5040Eu, 0x07B1C33Eu, 0x07BE8309u, 0x07CB436Eu, 0x07D80470u,
		0x07E4C60Fu, 0x07F1884Cu, 0x07FE4B28u, 0x080B0EA4u, 0x0817D2C2u, 0x08249782u, 0x08315CE6u, 0x083E22EDu,
		0x084AE99Bu, 0x0857B0EEu, 0x086478E9u, 0x0871418Du, 0x087E0ADAu, 0x088AD4D2u, 0x08979F75u, 0x08A46AC5u,
		0x08B136C3u, 0x08BE036Fu, 0x08CAD0CCu, 0x08D79ED9u, 0x08E46D98u, 0x08F13D09u, 0x08FE0D2Fu, 0x090ADE0Au,
		0x0917AF9Bu, 0x092481E3u, 0x093154E3u, 0x093E289Cu, 0x094AFD10u, 0x0957D23Fu, 0x0964A82Au, 0x09717ED3u,
		0x097E563Au, 0x098B2E60u, 0x09980748u, 0x09A4E0F0u, 0x09B1BB5Cu, 0x09BE968Bu, 0x09CB727Fu, 0x09D84F39u,
		0x09E52CBAu, 0x09F20B03u, 0x09FEEA15u, 0x0A0BC9F1u, 0x0A18AA98u, 0x0A258C0Cu, 0x0A326E4Du, 0x0A3F515Cu,
		0x0A4C353Au, 0x0A5919EAu, 0x0A65FF6Au, 0x0A72E5BEu, 0x0A7FCCE5u, 0x0A8CB4E1u, 0x0A999DB3u, 0x0AA6875Cu,
		0x0AB371DDu, 0x0AC05D37u, 0x0ACD496Bu, 0x0ADA367Bu, 0x0AE72467u, 0x0AF41330u, 0x0B0102D8u, 0x0B0DF360u,
		0x0B1AE4C8u, 0x0B27D713u, 0x0B34CA40u, 0x0B41BE51u, 0x0B4EB347u, 0x0B5BA923u, 0x0B689FE7u, 0x0B759792u,
		0x0B829028u, 0x0B8F89A8u, 0x0B9C8413u, 0x0BA97F6Bu, 0x0BB67BB2u, 0x0BC378E7u, 0x0BD0770Cu, 0x0BDD7622u,
		0x0BEA762Au, 0x0BF77726u, 0x0C047917u, 0x0C117BFDu, 0x0C1E7FD9u, 0x0C2B84AEu, 0x0C388A7Bu, 0x0C459142u,
		0x0C529905u, 0x0C5FA1C4u, 0x0C6CAB80u, 0x0C79B63Au, 0x0C86C1F5u, 0x0C93CEB0u, 0x0CA0DC6Cu, 0x0CADEB2Cu,
		0x0CBAFAF0u, 0x0CC80BB9u, 0x0CD51D89u, 0x0CE23060u, 0x0CEF443Fu, 0x0CFC5929u, 0x0D096F1Du, 0x0D16861Du,
		0x0D239E2Bu, 0x0D30B746u, 0x0D3DD171u, 0x0D4AECADu, 0x0D5808FAu, 0x0D65265Bu, 0x0D7244CFu, 0x0D7F6458u,
		0x0D8C84F8u, 0x0D99A6AFu, 0x0DA6C97Fu, 0x0DB3ED68u, 0x0DC1126Cu, 0x0DCE388Du, 0x0DDB5FCAu, 0x0DE88826u,
		0x0DF5B1A1u, 0x0E02DC3Du, 0x0E1007FBu, 0x0E1D34DCu, 0x0E2A62E1u, 0x0E37920Bu, 0x0E44C25Cu, 0x0E51F3D4u,
		0x0E5F2676u, 0x0E6C5A41u, 0x0E798F37u, 0x0E86C55Au, 0x0E93FCAAu, 0x0EA13529u, 0x0EAE6ED8u, 0x0EBBA9B8u,
		0x0EC8E5CAu, 0x0ED6230Fu, 0x0EE3618Au, 0x0EF0A139u, 0x0EFDE220u, 0x0F0B243Fu, 0x0F186798u, 0x0F25AC2Bu,
		0x0F32F1F9u, 0x0F403905u, 0x0F4D814Fu, 0x0F5ACAD8u, 0x0F6815A1u, 0x0F7561ADu, 0x0F82AEFBu, 0x0F8FFD8Du,
		0x0F9D4D65u, 0x0FAA9E83u, 0x0FB7F0EAu, 0x0FC54499u, 0x0FD29992u, 0x0FDFEFD7u, 0x0FED4768u, 0x0FFAA047u,
		0x1007FA76u, 0x101555F4u, 0x1022B2C4u, 0x103010E7u, 0x103D705Eu, 0x104AD12Au, 0x1058334Cu, 0x106596C7u,
		0x1072FB9Au, 0x108061C7u, 0x108DC94Fu, 0x109B3235u, 0x10A89C78u, 0x10B6081Au, 0x10C3751Cu, 0x10D0E380u,
		0x10DE5347u, 0x10EBC472u, 0x10F93703u, 0x1106AAFAu, 0x11142059u, 0x11219721u, 0x112F0F53u, 0x113C88F1u,
		0x114A03FCu, 0x11578075u, 0x1164FE5Eu, 0x11727DB7u, 0x117FFE83u, 0x118D80C1u, 0x119B0474u, 0x11A8899Du,
		0x11B6103Du, 0x11C39856u, 0x11D121E8u, 0x11DEACF5u, 0x11EC397Eu, 0x11F9C785u, 0x1207570Au, 0x1214E810u,
		0x12227A97u, 0x12300EA1u, 0x123DA42Fu, 0x124B3B42u, 0x1258D3DCu, 0x12666DFDu, 0x127409A8u, 0x1281A6DEu,
		0x128F459Fu, 0x129CE5EDu, 0x12AA87CAu, 0x12B82B37u, 0x12C5D035u, 0x12D376C5u, 0x12E11EE9u, 0x12EEC8A2u,
		0x12FC73F2u, 0x130A20D9u, 0x1317CF59u, 0x13257F74u, 0x1333312Bu, 0x1340E47Eu, 0x134E9970u, 0x135C5002u,
		0x136A0835u, 0x1377C20Bu, 0x13857D84u, 0x13933AA3u, 0x13A0F968u, 0x13AEB9D5u, 0x13BC7BEBu, 0x13CA3FACu,
		0x13D80519u, 0x13E5CC33u, 0x13F394FCu, 0x14015F75u, 0x140F2BA0u, 0x141CF97Du, 0x142AC90Fu, 0x14389A56u,
		0x14466D54u, 0x1454420Bu, 0x1462187Bu, 0x146FF0A7u, 0x147DCA8Fu, 0x148BA635u, 0x1499839Au, 0x14A762C0u,
		0x14B543A9u, 0x14C32654u, 0x14D10AC5u, 0x14DEF0FCu, 0x14ECD8FBu, 0x14FAC2C4u, 0x1508AE56u, 0x15169BB5u,
		0x15248AE1u, 0x15327BDDu, 0x15406EA8u, 0x154E6345u, 0x155C59B6u, 0x156A51FBu, 0x15784C16u, 0x15864808u,
		0x159445D4u, 0x15A2457Au, 0x15B046FCu, 0x15BE4A5Bu, 0x15CC4F98u, 0x15DA56B6u, 0x15E85FB6u, 0x15F66A98u,
		0x16047760u, 0x1612860Du, 0x162096A2u, 0x162EA920u, 0x163CBD88u, 0x164AD3DCu, 0x1658EC1Eu, 0x1667064Eu,
		0x1675226Fu, 0x16834082u, 0x16916088u, 0x169F8283u, 0x16ADA675u, 0x16BBCC5Eu, 0x16C9F440u, 0x16D81E1Eu,
		0x16E649F8u, 0x16F477D0u, 0x1702A7A7u, 0x1710D97Fu, 0x171F0D59u, 0x172D4338u, 0x173B7B1Cu, 0x1749B507u,
		0x1757F0FAu, 0x17662EF8u, 0x17746F01u, 0x1782B118u, 0x1790F53Du, 0x179F3B72u, 0x17AD83B9u, 0x17BBCE14u,
		0x17CA1A83u, 0x17D86909u, 0x17E6B9A7u, 0x17F50C5Fu, 0x18036131u, 0x1811B821u, 0x1820112Eu, 0x182E6C5Cu,
		0x183CC9ABu, 0x184B291Du, 0x18598AB4u, 0x1867EE71u, 0x18765456u, 0x1884BC64u, 0x1893269Du, 0x18A19303u,
		0x18B00197u, 0x18BE725Bu, 0x18CCE550u, 0x18DB5A78u, 0x18E9D1D5u, 0x18F84B68u, 0x1906C734u, 0x19154538u,
		0x1923C578u, 0x193247F5u, 0x1940CCB0u, 0x194F53ABu, 0x195DDCE8u, 0x196C6869u, 0x197AF62Eu, 0x1989863Au,
		0x1998188Fu, 0x19A6AD2Du, 0x19B54417u, 0x19C3DD4Eu, 0x19D278D5u, 0x19E116ACu, 0x19EFB6D5u, 0x19FE5953u,
		0x1A0CFE26u, 0x1A1BA551u, 0x1A2A4ED5u, 0x1A38FAB3u, 0x1A47A8EEu, 0x1A565988u, 0x1A650C81u, 0x1A73C1DCu,
		0x1A82799Au, 0x1A9133BDu, 0x1A9FF047u, 0x1AAEAF3Au, 0x1ABD7097u, 0x1ACC345Fu, 0x1ADAFA96u, 0x1AE9C33Cu,
		0x1AF88E53u, 0x1B075BDDu, 0x1B162BDCu, 0x1B24FE51u, 0x1B33D33Eu, 0x1B42AAA5u, 0x1B518488u, 0x1B6060E9u,
		0x1B6F3FC9u, 0x1B7E212Au, 0x1B8D050Du, 0x1B9BEB76u, 0x1BAAD464u, 0x1BB9BFDBu, 0x1BC8ADDCu, 0x1BD79E69u,
		0x1BE69183u, 0x1BF5872Du, 0x1C047F67u, 0x1C137A35u, 0x1C227798u, 0x1C317792u, 0x1C407A24u, 0x1C4F7F50u,
		0x1C5E8718u, 0x1C6D917Fu, 0x1C7C9E85u, 0x1C8BAE2Du, 0x1C9AC079u, 0x1CA9D56Au, 0x1CB8ED02u, 0x1CC80744u,
		0x1CD72430u, 0x1CE643CAu, 0x1CF56612u, 0x1D048B0Bu, 0x1D13B2B6u, 0x1D22DD16u, 0x1D320A2Cu, 0x1D4139FAu,
		0x1D506C83u, 0x1D5FA1C7u, 0x1D6ED9CAu, 0x1D7E148Cu, 0x1D8D5210u, 0x1D9C9258u, 0x1DABD565u, 0x1DBB1B3Au,
		0x1DCA63D9u, 0x1DD9AF43u, 0x1DE8FD7Au, 0x1DF84E81u, 0x1E07A259u, 0x1E16F904u, 0x1E265285u, 0x1E35AEDCu,
		0x1E450E0Du, 0x1E547019u, 0x1E63D502u, 0x1E733CCBu, 0x1E82A774u, 0x1E921501u, 0x1EA18573u, 0x1EB0F8CBu,
		0x1EC06F0Du, 0x1ECFE83Bu, 0x1EDF6455u, 0x1EEEE35Eu, 0x1EFE6559u, 0x1F0DEA47u, 0x1F1D722Bu, 0x1F2CFD05u,
		0x1F3C8ADAu, 0x1F4C1BA9u, 0x1F5BAF76u, 0x1F6B4643u, 0x1F7AE011u, 0x1F8A7CE3u, 0x1F9A1CBBu, 0x1FA9BF9Bu,
		0x1FB96585u, 0x1FC90E7Bu, 0x1FD8BA7Fu, 0x1FE86993u, 0x1FF81BBAu, 0x2007D0F5u, 0x20178947u, 0x202744B1u,
		0x20370337u, 0x2046C4D9u, 0x2056899Bu, 0x2066517Eu, 0x20761C84u, 0x2085EAB0u, 0x2095BC04u, 0x20A59081u,
		0x20B5682Bu, 0x20C54303u, 0x20D5210Cu, 0x20E50247u, 0x20F4E6B7u, 0x2104CE5Eu, 0x2114B93Eu, 0x2124A75Au,
		0x213498B3u, 0x21448D4Du, 0x21548528u, 0x21648048u, 0x21747EAEu, 0x2184805Eu, 0x21948558u, 0x21A48D9Fu,
		0x21B49937u, 0x21C4A820u, 0x21D4BA5Du, 0x21E4CFF0u, 0x21F4E8DDu, 0x22050524u, 0x221524C8u, 0x222547CCu,
		0x22356E32u, 0x224597FCu, 0x2255C52Du, 0x2265F5C6u, 0x227629CBu, 0x2286613Cu, 0x22969C1Eu, 0x22A6DA72u,
		0x22B71C3Bu, 0x22C7617Au, 0x22D7AA33u, 0x22E7F667u, 0x22F84619u, 0x2308994Bu, 0x2318F001u, 0x23294A3Bu,
		0x2339A7FDu, 0x234A0949u, 0x235A6E21u, 0x236AD688u, 0x237B4281u, 0x238BB20Du, 0x239C252Fu, 0x23AC9BE9u,
		0x23BD163Fu, 0x23CD9432u, 0x23DE15C4u, 0x23EE9AFAu, 0x23FF23D4u, 0x240FB055u, 0x24204080u, 0x2430D458u,
		0x24416BDEu, 0x24520716u, 0x2462A601u, 0x247348A3u, 0x2483EEFEu, 0x24949914u, 0x24A546E8u, 0x24B5F87Cu,
		0x24C6ADD4u, 0x24D766F1u, 0x24E823D7u, 0x24F8E487u, 0x2509A905u, 0x251A7152u, 0x252B3D73u, 0x253C0D68u,
		0x254CE135u, 0x255DB8DCu, 0x256E9461u, 0x257F73C5u, 0x2590570Bu, 0x25A13E36u, 0x25B22949u, 0x25C31846u,
		0x25D40B30u, 0x25E5020Au, 0x25F5FCD5u, 0x2606FB96u, 0x2617FE4Eu, 0x26290501u, 0x263A0FB1u, 0x264B1E60u,
		0x265C3112u, 0x266D47C9u, 0x267E6288u, 0x268F8152u, 0x26A0A42Au, 0x26B1CB11u, 0x26C2F60Cu, 0x26D4251Cu,
		0x26E55845u, 0x26F68F89u, 0x2707CAECu, 0x27190A6Fu, 0x272A4E16u, 0x273B95E4u, 0x274CE1DBu, 0x275E31FFu,
		0x276F8651u, 0x2780DED6u, 0x27923B8Fu, 0x27A39C80u, 0x27B501ACu, 0x27C66B15u, 0x27D7D8BFu, 0x27E94AACu,
		0x27FAC0DFu, 0x280C3B5Bu, 0x281DBA24u, 0x282F3D3Bu, 0x2840C4A4u, 0x28525062u, 0x2863E078u, 0x287574E9u,
		0x28870DB7u, 0x2898AAE6u, 0x28AA4C7Au, 0x28BBF273u, 0x28CD9CD7u, 0x28DF4BA7u, 0x28F0FEE7u, 0x2902B69Bu,
		0x291472C4u, 0x29263366u, 0x2937F884u, 0x2949C221u, 0x295B9040u, 0x296D62E5u, 0x297F3A11u, 0x299115CAu,
		0x29A2F611u, 0x29B4DAEAu, 0x29C6C457u, 0x29D8B25Du, 0x29EAA4FEu, 0x29FC9C3Du, 0x2A0E981Du, 0x2A2098A3u,
		0x2A329DD0u, 0x2A44A7A8u, 0x2A56B62Eu, 0x2A68C966u, 0x2A7AE153u, 0x2A8CFDF8u, 0x2A9F1F57u, 0x2AB14576u,
		0x2AC37055u, 0x2AD59FFAu, 0x2AE7D468u, 0x2AFA0DA0u, 0x2B0C4BA8u, 0x2B1E8E82u, 0x2B30D631u, 0x2B4322B9u,
		0x2B55741Du, 0x2B67CA61u, 0x2B7A2588u, 0x2B8C8595u, 0x2B9EEA8Bu, 0x2BB1546Fu, 0x2BC3C343u, 0x2BD6370Bu,
		0x2BE8AFCAu, 0x2BFB2D84u, 0x2C0DB03Cu, 0x2C2037F5u, 0x2C32C4B4u, 0x2C45567Bu, 0x2C57ED4Eu, 0x2C6A8931u,
		0x2C7D2A27u, 0x2C8FD033u, 0x2CA27B59u, 0x2CB52B9Du, 0x2CC7E102u, 0x2CDA9B8Cu, 0x2CED5B3Eu, 0x2D00201Cu,
		0x2D12EA29u, 0x2D25B969u, 0x2D388DE0u, 0x2D4B6791u, 0x2D5E4680u, 0x2D712AB1u, 0x2D841427u, 0x2D9702E6u,
		0x2DA9F6F1u, 0x2DBCF04Du, 0x2DCFEEFCu, 0x2DE2F303u, 0x2DF5FC66u, 0x2E090B27u, 0x2E1C1F4Cu, 0x2E2F38D7u,
		0x2E4257CCu, 0x2E557C30u, 0x2E68A606u, 0x2E7BD551u, 0x2E8F0A16u, 0x2EA24458u, 0x2EB5841Cu, 0x2EC8C965u,
		0x2EDC1436u, 0x2EEF6495u, 0x2F02BA84u, 0x2F161608u, 0x2F297724u, 0x2F3CDDDDu, 0x2F504A37u, 0x2F63BC34u,
		0x2F7733DAu, 0x2F8AB12Du, 0x2F9E342Fu, 0x2FB1BCE6u, 0x2FC54B55u, 0x2FD8DF81u, 0x2FEC796Cu, 0x3000191Du,
		0x3013BE95u, 0x302769DAu, 0x303B1AF0u, 0x304ED1DBu, 0x30628E9Eu, 0x3076513Fu, 0x308A19C0u, 0x309DE827u,
		0x30B1BC77u, 0x30C596B5u, 0x30D976E4u, 0x30ED5D0Au, 0x3101492Au, 0x31153B48u, 0x31293369u, 0x313D3191u,
		0x315135C4u, 0x31654007u, 0x3179505Du, 0x318D66CCu, 0x31A18357u, 0x31B5A603u, 0x31C9CED4u, 0x31DDFDCFu,
		0x31F232F8u, 0x32066E53u, 0x321AAFE5u, 0x322EF7B2u, 0x324345BEu, 0x32579A0Fu, 0x326BF4A8u, 0x3280558Fu,
		0x3294BCC7u, 0x32A92A55u, 0x32BD9E3Du, 0x32D21885u, 0x32E69930u, 0x32FB2044u, 0x330FADC5u, 0x332441B7u,
		0x3338DC1Fu, 0x334D7D02u, 0x33622464u, 0x3376D24Au, 0x338B86B9u, 0x33A041B6u, 0x33B50344u, 0x33C9CB6Au,
		0x33DE9A2Bu, 0x33F36F8Du, 0x34084B93u, 0x341D2E44u, 0x343217A4u, 0x344707B7u, 0x345BFE82u, 0x3470FC0Bu,
		0x34860056u, 0x349B0B68u, 0x34B01D46u, 0x34C535F4u, 0x34DA5579u, 0x34EF7BD9u, 0x3504A918u, 0x3519DD3Cu,
		0x352F1849u, 0x35445A46u, 0x3559A336u, 0x356EF31Fu, 0x35844A06u, 0x3599A7F0u, 0x35AF0CE2u, 0x35C478E2u,
		0x35D9EBF3u, 0x35EF661Du, 0x3604E763u, 0x361A6FCBu, 0x362FFF5Au, 0x36459616u, 0x365B3403u, 0x3670D928u,
		0x36868588u, 0x369C392Au, 0x36B1F413u, 0x36C7B648u, 0x36DD7FCEu, 0x36F350ACu, 0x370928E5u, 0x371F0881u,
		0x3734EF83u, 0x374ADDF2u, 0x3760D3D3u, 0x3776D12Cu, 0x378CD602u, 0x37A2E25Au, 0x37B8F63Bu, 0x37CF11A9u,
		0x37E534ABu, 0x37FB5F45u, 0x3811917Fu, 0x3827CB5Cu, 0x383E0CE3u, 0x3854561Au, 0x386AA705u, 0x3880FFACu,
		0x38976014u, 0x38ADC842u, 0x38C4383Cu, 0x38DAB009u, 0x38F12FADu, 0x3907B72Fu, 0x391E4695u, 0x3934DDE4u,
		0x394B7D22u, 0x39622456u, 0x3978D386u, 0x398F8AB6u, 0x39A649EEu, 0x39BD1133u, 0x39D3E08Bu, 0x39EAB7FDu,
		0x3A01978Eu, 0x3A187F45u, 0x3A2F6F28u, 0x3A46673Cu, 0x3A5D6789u, 0x3A747013u, 0x3A8B80E2u, 0x3AA299FCu,
		0x3AB9BB67u, 0x3AD0E528u, 0x3AE81748u, 0x3AFF51CBu, 0x3B1694B8u, 0x3B2DE016u, 0x3B4533EBu, 0x3B5C903Du,
		0x3B73F513u, 0x3B8B6273u, 0x3BA2D865u, 0x3BBA56EDu, 0x3BD1DE13u, 0x3BE96DDEu, 0x3C010653u, 0x3C18A77Au,
		0x3C305159u, 0x3C4803F8u, 0x3C5FBF5Bu, 0x3C77838Bu, 0x3C8F508Eu, 0x3CA7266Bu, 0x3CBF0528u, 0x3CD6ECCDu,
		0x3CEEDD60u, 0x3D06D6E7u, 0x3D1ED96Bu, 0x3D36E4F2u, 0x3D4EF982u, 0x3D671723u, 0x3D7F3DDCu, 0x3D976DB3u,
		0x3DAFA6B0u, 0x3DC7E8DAu, 0x3DE03438u, 0x3DF888D1u, 0x3E10E6ACu, 0x3E294DD1u, 0x3E41BE46u, 0x3E5A3812u,
		0x3E72BB3Eu, 0x3E8B47D0u, 0x3EA3DDD0u, 0x3EBC7D45u, 0x3ED52636u, 0x3EEDD8AAu, 0x3F0694AAu, 0x3F1F5A3Cu,
		0x3F382969u, 0x3F510237u, 0x3F69E4AEu, 0x3F82D0D5u, 0x3F9BC6B5u, 0x3FB4C655u, 0x3FCDCFBCu, 0x3FE6E2F3u,
		0x40000000u
	};

	// =========================================================================
	// INV_SQRT_Q29 table definition
	// =========================================================================
	static constexpr int32_t INV_SQRT_Q29[257] = {
		0x20000000,	0x1FF00BF6,	0x1FE02FB0,	0x1FD06AF4,
		0x1FC0BD88,	0x1FB12733,	0x1FA1A7BB,	0x1F923EEA,
		0x1F82EC88,	0x1F73B05F,	0x1F648A3A,	0x1F5579E4,
		0x1F467F28,	0x1F3799D3,	0x1F28C9B3,	0x1F1A0E95,
		0x1F0B6849,	0x1EFCD69C,	0x1EEE595E,	0x1EDFF061,
		0x1ED19B76,	0x1EC35A6D,	0x1EB52D18,	0x1EA7134C,
		0x1E990CDB,	0x1E8B1998,	0x1E7D3959,	0x1E6F6BF2,
		0x1E61B139,	0x1E540903,	0x1E467328,	0x1E38EF7F,
		0x1E2B7DDE,	0x1E1E1E1E,	0x1E10D017,	0x1E0393A3,
		0x1DF6689B,	0x1DE94ED9,	0x1DDC4637,	0x1DCF4E8F,
		0x1DC267BE,	0x1DB5919F,	0x1DA8CC0E,	0x1D9C16E8,
		0x1D8F7209,	0x1D82DD4F,	0x1D765897,	0x1D69E3C0,
		0x1D5D7EA9,	0x1D512930,	0x1D44E334,	0x1D38AC96,
		0x1D2C8535,	0x1D206CF1,	0x1D1463AC,	0x1D086947,
		0x1CFC7DA3,	0x1CF0A0A2,	0x1CE4D225,	0x1CD91210,
		0x1CCD6046,	0x1CC1BCA9,	0x1CB6271C,	0x1CAA9F85,
		0x1C9F25C5,	0x1C93B9C4,	0x1C885B63,	0x1C7D0A8A,
		0x1C71C71C,	0x1C669100,	0x1C5B681B,	0x1C504C54,
		0x1C453D91,	0x1C3A3BB8,	0x1C2F46B0,	0x1C245E61,
		0x1C198AB3,	0x1C0EB38C,	0x1C03F0D5,	0x1BF93A75,
		0x1BEE9057,	0x1BE3F261,	0x1BD9607E,	0x1BCEDA96,
		0x1BC46093,	0x1BB9F25E,	0x1BAF8FE1,	0x1BA53907,
		0x1B9AEDBA,	0x1B90ADE4,	0x1B867970,	0x1B7C504A,
		0x1B72325B,	0x1B681F91,	0x1B5E17D5,	0x1B541B15,
		0x1B4A293C,	0x1B404236,	0x1B3665F0,	0x1B2C9457,
		0x1B22CD57,	0x1B1910DD,	0x1B0F5ED6,	0x1B05B730,
		0x1AFC19D8,	0x1AF286BC,	0x1AE8FDCB,	0x1ADF7EF1,
		0x1AD60A1D,	0x1ACC9F3E,	0x1AC33E42,	0x1AB9E718,
		0x1AB099AE,	0x1AA755F5,	0x1A9E1BDB,	0x1A94EB4F,
		0x1A8BC441,	0x1A82A6A2,	0x1A79925F,	0x1A70876B,
		0x1A6785B4,	0x1A5E8D2B,	0x1A559DC1,	0x1A4CB766,
		0x1A43DA0B,	0x1A3B05A0,	0x1A323A17,	0x1A297761,
		0x1A20BD70,	0x1A180C34,	0x1A0F639F,	0x1A06C3A3,
		0x19FE2C31,	0x19F59D3C,	0x19ED16B6,	0x19E49890,
		0x19DC22BE,	0x19D3B531,	0x19CB4FDD,	0x19C2F2B3,
		0x19BA9DA7,	0x19B250AB,	0x19AA0BB3,	0x19A1CEB1,
		0x19999999,	0x19916D5F,	0x198946F5,	0x19812950,
		0x19791363,	0x19710521,	0x1968FE80,	0x1960FF72,
		0x195907EB,	0x195117E1,	0x19492F47,	0x19414E12,
		0x19397436,	0x1931A1A8,	0x1929D65D,	0x19221249,
		0x191A5561,	0x19129F9B,	0x190AF0EA,	0x19034946,
		0x18FBA8A1,	0x18F40EF4,	0x18EC7C31,	0x18E4F050,
		0x18DD6B45,	0x18D5ED07,	0x18CE758B,	0x18C704C6,
		0x18BF9AB0,	0x18B8373E,	0x18B0DA66,	0x18A9841E,
		0x18A2345D,	0x189AEB18,	0x1893A847,	0x188C6BE0,
		0x188535D9,	0x187E0629,	0x1876DCCF,	0x186FB9AA,
		0x18689CC8,	0x18618618,	0x185A7592,	0x18536B2D,
		0x184C66DF,	0x184568A0,	0x183E7067,	0x18377E2C,
		0x183091E6,	0x1829AB8D,	0x1822CB18,	0x181BF07E,
		0x18151BB8,	0x180E4CBD,	0x18078386,	0x1800C009,
		0x17FA023F,	0x17F34A20,	0x17EC97A4,	0x17E5EAC3,
		0x17DF4375,	0x17D8A1B3,	0x17D20575,	0x17CB6EB3,
		0x17C4DD66,	0x17BE5186,	0x17B7CB0C,	0x17B149F0,
		0x17AACE2B,	0x17A457B5,	0x179DE689,	0x17977A9D,
		0x179113EB,	0x178AB26D,	0x1784561B,	0x177EFEED,
		0x177AACDE,	0x17715FE6,	0x176B17FF,	0x1764D521,
		0x175E9746,	0x17585E68,	0x17522A7F,	0x174BFB85,
		0x1745D174,	0x173FAC45,	0x17398BF2,	0x17337073,
		0x172D59C4,	0x172747DD,	0x172139B9,	0x171B3251,
		0x17152E9F,	0x170F2F9D,	0x17093544,	0x17033F90,
		0x16FD4E79,	0x16F761FA,	0x16F17A0D,	0x16EB96AC,
		0x16E5B7D1,	0x16DFDD77,	0x16DA0797,	0x16D4362D,
		0x16CE6932,	0x16C8A0A0,	0x16C2DC73,	0x16BD1CA4,
		0x16B7612F,	0x16B1AA0D,	0x16ABF739,	0x16A648AE,
		0x16A09E66,
	};

        // Hoisted from atan2(), exp(), log2() to fix C++20 constexpr-evaluation rules.
        static constexpr uint32_t ATAN_RAW_TAB[258] = {
                0x00000000u, 0x0028BE53u, 0x00517C55u, 0x007A39B4u, 0x00A2F61Eu, 0x00CBB143u, 0x00F46AD1u, 0x011D2276u,
                0x0145D7E1u, 0x016E8AC2u, 0x01973AC8u, 0x01BFE7A1u, 0x01E890FDu, 0x0211368Bu, 0x0239D7FCu, 0x026274FEu,
                0x028B0D43u, 0x02B3A07Au, 0x02DC2E54u, 0x0304B681u, 0x032D38B4u, 0x0355B49Cu, 0x037E29EBu, 0x03A69855u,
                0x03CEFF8Au, 0x03F75F3Du, 0x041FB721u, 0x044806EAu, 0x04704E4Bu, 0x04988CF8u, 0x04C0C2A5u, 0x04E8EF07u,
                0x051111D4u, 0x05392AC1u, 0x05613984u, 0x05893DD4u, 0x05B13767u, 0x05D925F6u, 0x06010937u, 0x0628E0E5u,
                0x0650ACB7u, 0x06786C67u, 0x06A01FAFu, 0x06C7C649u, 0x06EF5FF2u, 0x0716EC63u, 0x073E6B5Bu, 0x0765DC95u,
                0x078D3FCFu, 0x07B494C6u, 0x07DBDB3Au, 0x080312EAu, 0x082A3B95u, 0x085154FCu, 0x08785EDFu, 0x089F5902u,
                0x08C64325u, 0x08ED1D0Du, 0x0913E67Cu, 0x093A9F37u, 0x09614704u, 0x0987DDA7u, 0x09AE62E7u, 0x09D4D68Bu,
                0x09FB385Bu, 0x0A218820u, 0x0A47C5A2u, 0x0A6DF0ACu, 0x0A940907u, 0x0ABA0E80u, 0x0AE000E2u, 0x0B05DFFAu,
                0x0B2BAB95u, 0x0B516382u, 0x0B770790u, 0x0B9C978Du, 0x0BC2134Cu, 0x0BE77A9Bu, 0x0C0CCD4Fu, 0x0C320B38u,
                0x0C57342Bu, 0x0C7C47FBu, 0x0CA1467Du, 0x0CC62F87u, 0x0CEB02EFu, 0x0D0FC08Du, 0x0D346837u, 0x0D58F9C7u,
                0x0D7D7515u, 0x0DA1D9FCu, 0x0DC62856u, 0x0DEA6000u, 0x0E0E80D4u, 0x0E328AB1u, 0x0E567D73u, 0x0E7A58FAu,
                0x0E9E1D24u, 0x0EC1C9D1u, 0x0EE55EE3u, 0x0F08DC39u, 0x0F2C41B7u, 0x0F4F8F3Fu, 0x0F72C4B4u, 0x0F95E1FBu,
                0x0FB8E6F9u, 0x0FDBD394u, 0x0FFEA7B1u, 0x10216337u, 0x1044060Fu, 0x10669021u, 0x10890156u, 0x10AB5998u,
                0x10CD98D1u, 0x10EFBEEDu, 0x1111CBD6u, 0x1133BF7Au, 0x115599C7u, 0x11775AA8u, 0x1199020Eu, 0x11BA8FE7u,
                0x11DC0423u, 0x11FD5EB3u, 0x121E9F86u, 0x123FC690u, 0x1260D3C2u, 0x1281C70Fu, 0x12A2A06Au, 0x12C35FC8u,
                0x12E4051Eu, 0x13049060u, 0x13250184u, 0x13455882u, 0x1365954Fu, 0x1385B7E4u, 0x13A5C038u, 0x13C5AE45u,
                0x13E58204u, 0x14053B6Eu, 0x1424DA7Eu, 0x14445F2Eu, 0x1463C97Au, 0x1483195Fu, 0x14A24ED8u, 0x14C169E2u,
                0x14E06A7Bu, 0x14FF50A0u, 0x151E1C51u, 0x153CCD8Cu, 0x155B6450u, 0x1579E09Eu, 0x15984275u, 0x15B689D7u,
                0x15D4B6C5u, 0x15F2C93Fu, 0x1610C149u, 0x162E9EE6u, 0x164C6217u, 0x166A0AE0u, 0x16879946u, 0x16A50D4Cu,
                0x16C266F7u, 0x16DFA64Cu, 0x16FCCB50u, 0x1719D60Au, 0x1736C67Fu, 0x17539CB6u, 0x177058B6u, 0x178CFA85u,
                0x17A9822Du, 0x17C5EFB4u, 0x17E24323u, 0x17FE7C82u, 0x181A9BDBu, 0x1836A137u, 0x18528C9Fu, 0x186E5E1Du,
                0x188A15BCu, 0x18A5B386u, 0x18C13785u, 0x18DCA1C6u, 0x18F7F252u, 0x19132937u, 0x192E4680u, 0x19494A38u,
                0x1964346Eu, 0x197F052Cu, 0x1999BC81u, 0x19B45A79u, 0x19CEDF22u, 0x19E94A8Au, 0x1A039CBEu, 0x1A1DD5CDu,
                0x1A37F5C5u, 0x1A51FCB4u, 0x1A6BEAAAu, 0x1A85BFB5u, 0x1A9F7BE5u, 0x1AB91F49u, 0x1AD2A9F0u, 0x1AEC1BEBu,
                0x1B057548u, 0x1B1EB61Au, 0x1B37DE6Fu, 0x1B50EE58u, 0x1B69E5E6u, 0x1B82C529u, 0x1B9B8C33u, 0x1BB43B15u,
                0x1BCCD1E0u, 0x1BE550A5u, 0x1BFDB776u, 0x1C160664u, 0x1C2E3D81u, 0x1C465CE0u, 0x1C5E6492u, 0x1C7654A9u,
                0x1C8E2D38u, 0x1CA5EE52u, 0x1CBD9807u, 0x1CD52A6Cu, 0x1CECA593u, 0x1D04098Fu, 0x1D1B5672u, 0x1D328C4Fu,
                0x1D49AB3Bu, 0x1D60B347u, 0x1D77A487u, 0x1D8E7F0Fu, 0x1DA542F1u, 0x1DBBF042u, 0x1DD28714u, 0x1DE9077Cu,
                0x1DFF718Cu, 0x1E15C55Au, 0x1E2C02F8u, 0x1E422A7Au, 0x1E583BF4u, 0x1E6E377Bu, 0x1E841D21u, 0x1E99ECFCu,
                0x1EAFA71Fu, 0x1EC54B9Eu, 0x1EDADA8Du, 0x1EF05401u, 0x1F05B80Eu, 0x1F1B06C8u, 0x1F304043u, 0x1F456493u,
                0x1F5A73CDu, 0x1F6F6E05u, 0x1F84534Fu, 0x1F9923C0u, 0x1FADDF6Bu, 0x1FC28667u, 0x1FD718C6u, 0x1FEB969Du,
                0x20000000u, 0x20000000u
        };

        static constexpr int32_t EXP_MANT[257] = {
                0x20000000, 0x201635F5, 0x202C7B54, 0x2042D028,
                0x2059347D, 0x206FA85C, 0x20862BD1, 0x209CBEE6,
                0x20B361A6, 0x20CA141C, 0x20E0D654, 0x20F7A857,
                0x210E8A31, 0x21257BED, 0x213C7D96, 0x21538F36,
                0x216AB0DA, 0x2181E28C, 0x21992457, 0x21B07646,
                0x21C7D866, 0x21DF4AC0, 0x21F6CD60, 0x220E6052,
                0x222603A0, 0x223DB757, 0x22557B81, 0x226D502A,
                0x2285355D, 0x229D2B27, 0x22B53191, 0x22CD48A9,
                0x22E57079, 0x22FDA90D, 0x2315F271, 0x232E4CB0,
                0x2346B7D7, 0x235F33F0, 0x2377C108, 0x23905F2A,
                0x23A90E63, 0x23C1CEBD, 0x23DAA046, 0x23F38308,
                0x240C7711, 0x24257C6B, 0x243E9323, 0x2457BB45,
                0x2470F4DD, 0x248A3FF7, 0x24A39C9F, 0x24BD0AE2,
                0x24D68ACC, 0x24F01C68, 0x2509BFC4, 0x252374EB,
                0x253D3BEA, 0x255714CE, 0x2570FFA2, 0x258AFC73,
                0x25A50B4E, 0x25BF2C3F, 0x25D95F52, 0x25F3A495,
                0x260DFC14, 0x262865DC, 0x2642E1F9, 0x265D7077,
                0x26781165, 0x2692C4CE, 0x26AD8ABF, 0x26C86346,
                0x26E34E6E, 0x26FE4C46, 0x27195CDA, 0x27348037,
                0x274FB66A, 0x276AFF80, 0x27865B86, 0x27A1CA8A,
                0x27BD4C98, 0x27D8E1BE, 0x27F48A09, 0x28104587,
                0x282C1444, 0x2847F64E, 0x2863EBB3, 0x287FF47F,
                0x289C10C1, 0x28B84085, 0x28D483DA, 0x28F0DACD,
                0x290D456C, 0x2929C3C3, 0x294655E2, 0x2962FBD5,
                0x297FB5AA, 0x299C8370, 0x29B96534, 0x29D65B04,
                0x29F364ED, 0x2A1082FF, 0x2A2DB546, 0x2A4AFBD0,
                0x2A6856AD, 0x2A85C5EA, 0x2AA34995, 0x2AC0E1BC,
                0x2ADE8E6D, 0x2AFC4FB8, 0x2B1A25A9, 0x2B381050,
                0x2B560FBB, 0x2B7423F7, 0x2B924D15, 0x2BB08B21,
                0x2BCEDE2B, 0x2BED4642, 0x2C0BC373, 0x2C2A55CE,
                0x2C48FD60, 0x2C67BA3A, 0x2C868C6A, 0x2CA573FD,
                0x2CC47105, 0x2CE3838E, 0x2D02ABA9, 0x2D21E963,
                0x2D413CCD, 0x2D60A5F5, 0x2D8024EA, 0x2D9FB9BC,
                0x2DBF6479, 0x2DDF2531, 0x2DFEFBF3, 0x2E1EE8CE,
                0x2E3EEBD2, 0x2E5F050E, 0x2E7F3491, 0x2E9F7A6C,
                0x2EBFD6AD, 0x2EE04963, 0x2F00D2A0, 0x2F217271,
                0x2F4228E8, 0x2F62F613, 0x2F83DA02, 0x2FA4D4C6,
                0x2FC5E66E, 0x2FE70F09, 0x30084EA8, 0x3029A55C,
                0x304B1333, 0x306C983D, 0x308E348C, 0x30AFE82F,
                0x30D1B337, 0x30F395B2, 0x31158FB3, 0x3137A149,
                0x3159CA84, 0x317C0B76, 0x319E642D, 0x31C0D4BC,
                0x31E35D32, 0x3205FDA0, 0x3228B617, 0x324B86A7,
                0x326E6F62, 0x32917057, 0x32B48998, 0x32D7BB35,
                0x32FB0540, 0x331E67C9, 0x3341E2E2, 0x3365769B,
                0x33892305, 0x33ACE833, 0x33D0C634, 0x33F4BD1A,
                0x3418CCF7, 0x343CF5DB, 0x346137D9, 0x34859301,
                0x34AA0764, 0x34CE9516, 0x34F33C26, 0x3517FCA8,
                0x353CD6AB, 0x3561CA42, 0x3586D780, 0x35ABFE74,
                0x35D13F33, 0x35F699CC, 0x361C0E53, 0x36419CD9,
                0x36674571, 0x368D082B, 0x36B2E51C, 0x36D8DC54,
                0x36FEEDE6, 0x372519E4, 0x374B6061, 0x3771C16F,
                0x37983D21, 0x37BED388, 0x37E584B8, 0x380C50C3,
                0x383337BB, 0x385A39B4, 0x388156C0, 0x38A88EF2,
                0x38CFE25D, 0x38F75113, 0x391EDB28, 0x394680AF,
                0x396E41BA, 0x39961E5D, 0x39BE16AB, 0x39E62AB7,
                0x3A0E5A94, 0x3A36A656, 0x3A5F0E10, 0x3A8791D6,
                0x3AB031BA, 0x3AD8EDD1, 0x3B01C62E, 0x3B2ABAE4,
                0x3B53CC08, 0x3B7CF9AC, 0x3BA643E6, 0x3BCFAAC8,
                0x3BF92E67, 0x3C22CED6, 0x3C4C8C2A, 0x3C766676,
                0x3CA05DCF, 0x3CCA7249, 0x3CF4A3F8, 0x3D1EF2F0,
                0x3D495F45, 0x3D73E90D, 0x3D9E905B, 0x3DC95544,
                0x3DF437DD, 0x3E1F3839, 0x3E4A566F, 0x3E759292,
                0x3EA0ECB7, 0x3ECC64F3, 0x3EF7FB5B, 0x3F23B004,
                0x3F4F8303, 0x3F7B746D, 0x3FA78457, 0x3FD3B2D6,
                0x20000000
        };

        static constexpr int32_t LOG2_Q30[257] = {
                0x00000000, 0x005C2711, 0x00B7F285, 0x01136311,
                0x016E7968, 0x01C9363B, 0x02239A3A, 0x027DA612,
                0x02D75A6E, 0x0330B7F8, 0x0389BF57, 0x03E27130,
                0x043ACE27, 0x0492D6DF, 0x04EA8BF7, 0x0541EE0D,
                0x0598FDBE, 0x05EFBBA5, 0x0646285B, 0x069C4477,
                0x06F21090, 0x07478D38, 0x079CBB04, 0x07F19A83,
                0x08462C46, 0x089A70DA, 0x08EE68CB, 0x094214A5,
                0x099574F1, 0x09E88A36, 0x0A3B54FC, 0x0A8DD5C8,
                0x0AE00D1C, 0x0B31FB7D, 0x0B83A16A, 0x0BD4FF63,
                0x0C2615E8, 0x0C76E574, 0x0CC76E83, 0x0D17B191,
                0x0D67AF16, 0x0DB7678B, 0x0E06DB66, 0x0E560B1E,
                0x0EA4F726, 0x0EF39FF1, 0x0F4205F3, 0x0F90299C,
                0x0FDE0B5C, 0x102BABA2, 0x107908DB, 0x10C62975,
                0x111307DA, 0x115FA676, 0x11AC05B2, 0x11F825F6,
                0x124407AB, 0x128FAB35, 0x12DB10FC, 0x13263963,
                0x13712ACE, 0x13BBD3A0, 0x1406463B, 0x14507CFE,
                0x149A784B, 0x14E43880, 0x152DBDFC, 0x1577091B,
                0x15C01A39, 0x1608F1B4, 0x16518FE4, 0x1699F524,
                0x16E221CD, 0x172A1637, 0x1771D2BA, 0x17B957AC,
                0x1800A563, 0x1847BC33, 0x188E9C72, 0x18D54673,
                0x191BBA89, 0x1961F905, 0x19A80239, 0x19EDD675,
                0x1A33760A, 0x1A78E146, 0x1ABE1879, 0x1B031BEF,
                0x1B47EBF7, 0x1B8C88DB, 0x1BD0F2E9, 0x1C152A6C,
                0x1C592FAD, 0x1C9D02F6, 0x1CE0A492, 0x1D2414C8,
                0x1D6753E0, 0x1DAA6222, 0x1DED3FD4, 0x1E2FED3D,
                0x1E726AA1, 0x1EB4B847, 0x1EF6D673, 0x1F38C567,
                0x1F7A8568, 0x1FBC16B9, 0x1FFD799A, 0x203EAE4E,
                0x207FB517, 0x20C08E33, 0x210139E4, 0x2141B869,
                0x21820A01, 0x21C22EEA, 0x22022762, 0x2241F3A7,
                0x228193F5, 0x22C10889, 0x2300519E, 0x233F6F71,
                0x237E623D, 0x23BD2A3B, 0x23FBC7A6, 0x243A3AB7,
                0x247883A8, 0x24B6A2B1, 0x24F4980B, 0x253263EC,
                0x2570068E, 0x25AD8026, 0x25EAD0EB, 0x2627F914,
                0x2664F8D5, 0x26A1D064, 0x26DE7FF6, 0x271B07C0,
                0x275767F5, 0x2793A0C9, 0x27CFB26F, 0x280B9D1A,
                0x284760FD, 0x2882FE49, 0x28BE7531, 0x28F9C5E5,
                0x2934F097, 0x296FF577, 0x29AAD4B6, 0x29E58E83,
                0x2A20230E, 0x2A5A9285, 0x2A94DD19, 0x2ACF02F7,
                0x2B09044D, 0x2B42E149, 0x2B7C9A19, 0x2BB62EEA,
                0x2BEF9FE8, 0x2C28ED40, 0x2C62171E, 0x2C9B1DAE,
                0x2CD4011C, 0x2D0CC192, 0x2D455F3C, 0x2D7DDA44,
                0x2DB632D4, 0x2DEE6917, 0x2E267D36, 0x2E5E6F5A,
                0x2E963FAC, 0x2ECDEE56, 0x2F057B7F, 0x2F3CE751,
                0x2F7431F2, 0x2FAB5B8B, 0x2FE26443, 0x30194C40,
                0x305013AB, 0x3086BAA9, 0x30BD4161, 0x30F3A7F8,
                0x3129EE96, 0x3160155E, 0x31961C76, 0x31CC0404,
                0x3201CC2C, 0x32377512, 0x326CFEDB, 0x32A269AB,
                0x32D7B5A5, 0x330CE2ED, 0x3341F1A7, 0x3376E1F5,
                0x33ABB3FA, 0x33E067D9, 0x3414FDB4, 0x344975AD,
                0x347DCFE7, 0x34B20C82, 0x34E62BA0, 0x351A2D62,
                0x354E11EB, 0x3581D959, 0x35B583CE, 0x35E9116A,
                0x361C824D, 0x364FD697, 0x36830E69, 0x36B629E1,
                0x36E9291E, 0x371C0C41, 0x374ED367, 0x37817EAF,
                0x37B40E39, 0x37E68222, 0x3818DA88, 0x384B178A,
                0x387D3945, 0x38AF3FD7, 0x38E12B5D, 0x3912FBF4,
                0x3944B1B9, 0x39764CC9, 0x39A7CD41, 0x39D9333D,
                0x3A0A7EDA, 0x3A3BB033, 0x3A6CC764, 0x3A9DC48A,
                0x3ACEA7C0, 0x3AFF7121, 0x3B3020C8, 0x3B60B6D1,
                0x3B913356, 0x3BC19672, 0x3BF1E041, 0x3C2210DB,
                0x3C52285C, 0x3C8226DD, 0x3CB20C79, 0x3CE1D948,
                0x3D118D66, 0x3D4128EB, 0x3D70ABF1, 0x3DA01691,
                0x3DCF68E3, 0x3DFEA301, 0x3E2DC503, 0x3E5CCF02,
                0x3E8BC117, 0x3EBA9B59, 0x3EE95DE1, 0x3F1808C7,
                0x3F469C22, 0x3F75180B, 0x3FA37C98, 0x3FD1C9E2,
                0x40000000
        };
	// Hoisted large tables as static constexpr members of SoftFloat (and Angle for sin).
	// This fixes "constexpr issues": C++20 does not allow (or has severe limitations on)
	// local `static constexpr` arrays inside `constexpr` functions when the function
	// is used in constant evaluation (atan2, exp, log2, Angle::sincos).
	// Member tables are ODR-usable in all constant expressions and work with both
	// runtime and consteval paths.
public:
	// ------------------------------------------------------------------
	// Default constructor — zero
	// ------------------------------------------------------------------
	constexpr SoftFloat() noexcept : bits{ 0 } {}

	// ------------------------------------------------------------------
	// Normalising constructors
	// ------------------------------------------------------------------
private:
	// Internal mantissa/exponent constructor.  Keep it private so outside code
	// cannot depend on the Q29 working representation.
	constexpr SF_HOT SoftFloat(int32_t m, int32_t e) noexcept : bits{ 0 }
	{
		uint32_t uv = static_cast<uint32_t>(m);
		uint16_t neg = static_cast<uint16_t>(uv >> 31);
		uint32_t mask = 0u - static_cast<uint32_t>(neg);
		uint32_t um = (uv ^ mask) - mask; // safe unsigned abs, including INT32_MIN
		set_components(um, e, neg);
	}

public:
#ifndef SF_INT_EQUALS_INT32
	constexpr SF_HOT explicit SoftFloat(int v) noexcept : bits{ pack_int32_bits(static_cast<int32_t>(v)) }
	{}
#endif

	constexpr SF_HOT explicit SoftFloat(int32_t v) noexcept : bits{ pack_int32_bits(v) }
	{}

	constexpr SF_HOT explicit SoftFloat(int16_t v) noexcept : bits{ pack_int32_bits(static_cast<int32_t>(v)) }
	{}

	constexpr SF_HOT explicit SoftFloat(float f) noexcept : bits{ 0 }
	{
		from_float(f);
	}

#ifndef SF_INT_EQUALS_INT32
	constexpr SF_HOT SoftFloat& operator=(int v) noexcept {
		bits = pack_int32_bits(static_cast<int32_t>(v));
		return *this;
	}
#endif

	constexpr SF_HOT SoftFloat& operator=(int32_t v) noexcept {
		bits = pack_int32_bits(v);
		return *this;
	}

	constexpr SF_HOT SoftFloat& operator=(int16_t v) noexcept {
		bits = pack_int32_bits(static_cast<int32_t>(v));
		return *this;
	}

	constexpr SF_HOT SoftFloat& operator=(float f) noexcept {
		from_float(f);
		return *this;
	}

	// Proxy constructor (defined after MulExpr)
	constexpr SF_HOT SoftFloat(const MulExpr& m) noexcept;

	// ------------------------------------------------------------------
	// Manual re-normalise
	//
	// Invariant: mantissa == 0  =>  zero
	//            mantissa != 0  =>  abs(mantissa) in [2^29, 2^30)
	// ------------------------------------------------------------------
	constexpr SF_HOT SF_INLINE void normalise() noexcept
	{
		set_components(mantissa, exponent, negative);
	}

	// ------------------------------------------------------------------
	// to_float — constexpr via compiler bit-cast builtin
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SF_HOT float to_float() const noexcept {
		uint32_t out = bits; // zero is already canonical, so bit-casting it is fine
		// Our exp=255 is finite; IEEE float would interpret it as Inf/NaN, so clamp
		// only at this top shelf for host conversion/debugging.
		if (UNLIKELY(field_exp(out) == 255u)) out = (out & 0x80000000u) | 0x7F7FFFFFu;
		return bitcast<float>(out);
	}

	[[nodiscard]] constexpr explicit operator float()   const noexcept { return to_float(); }

	// ------------------------------------------------------------------
	// to_int32 — truncate toward zero, constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SF_HOT int32_t to_int32() const noexcept {
		uint32_t be = field_exp(bits);
		if (UNLIKELY(be == 0u)) return 0;
		int32_t e = exp_from_biased(be);
		uint32_t sign = bits >> 31;
		if (UNLIKELY(e >= 2)) return sign ? INT32_MIN : INT32_MAX;

		uint32_t a = mant_from_bits(bits);
		if (LIKELY(e >= 0)) {
			a <<= e;
		}
		else {
			int rs = -e;
			if (UNLIKELY(rs >= 31)) return 0;
			a >>= rs;
		}
		uint32_t mask = 0u - sign;
		return static_cast<int32_t>((a ^ mask) - mask);
	}

	[[nodiscard]] constexpr explicit operator int32_t() const noexcept { return to_int32(); }

	// ------------------------------------------------------------------
	// Unary
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SoftFloat operator-() const noexcept {
		SoftFloat r;
		uint32_t mag = bits & 0x7FFFFFFFu;
		r.bits = (bits ^ 0x80000000u) & (0u - static_cast<uint32_t>(mag != 0u)); // keep zero canonical
		return r;
	}
	[[nodiscard]] constexpr SoftFloat operator+() const noexcept { return *this; }

	// ------------------------------------------------------------------
	// Binary operator declarations (defined after MulExpr)
	// ------------------------------------------------------------------
	friend constexpr SF_HOT SF_INLINE MulExpr operator*(SoftFloat a, SoftFloat b) noexcept;
	friend constexpr SF_HOT SF_INLINE MulExpr operator*(SoftFloat a, float     b) noexcept;
#ifndef SF_INT_EQUALS_INT32
	friend constexpr SF_HOT SF_INLINE MulExpr operator*(SoftFloat a, int       b) noexcept;
#endif
	friend constexpr SF_HOT SF_INLINE MulExpr operator*(SoftFloat a, int32_t   b) noexcept;
	friend constexpr SF_HOT SF_INLINE MulExpr operator*(float     a, SoftFloat b) noexcept;
#ifndef SF_INT_EQUALS_INT32
	friend constexpr SF_HOT SF_INLINE MulExpr operator*(int       a, SoftFloat b) noexcept;
#endif
	friend constexpr SF_HOT SF_INLINE MulExpr operator*(int32_t   a, SoftFloat b) noexcept;

	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
	friend SoftFloat operator+(SoftFloat a, SoftFloat b) noexcept
	{
		const uint32_t ab = a.bits;
		const uint32_t bb = b.bits;
		const uint32_t abe = field_exp(ab);
		const uint32_t bbe = field_exp(bb);
		if (UNLIKELY(abe == 0u)) return b;
		if (UNLIKELY(bbe == 0u)) return a;

		int d = static_cast<int>(abe) - static_cast<int>(bbe);
                if (UNLIKELY(static_cast<uint32_t>(d + 30) >= 60u)) {
                        if (d >= 30) return a;
                        return b; // d <= -30
                }

		uint32_t am = mant_from_bits(ab);
		uint32_t bm = mant_from_bits(bb);
		uint16_t an = sign_from_bits(ab);
		uint16_t bn = sign_from_bits(bb);
		int32_t re;

		if (d >= 0) { bm >>= d; re = exp_from_biased(abe); }
		else { am >>= -d; re = exp_from_biased(bbe); }

		if (an == bn) {
			// Same sign: unsigned add — sum is in [2^30, 2^31), so it needs at
			// most a single down-shift (no CLZ renormalize).
			return finish_add_samesign(am + bm, re, an);
		}
                // Different signs: subtract smaller from larger.
                // Use finish_sub_mag (no overflow check needed for subtraction;
                // the result can never exceed the Q29 interval, only shrink
                // from cancellation).
		if (am >= bm)
                        return finish_sub_mag(am - bm, re, an);
		else
                        return finish_sub_mag(bm - am, re, bn);
	}
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator+(SoftFloat a, float b) noexcept {
		return a + SoftFloat(b);
	}
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator+(SoftFloat a, int b) noexcept {
		return a + SoftFloat(b);
	}
#endif
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator+(SoftFloat a, int32_t b) noexcept {
		return a + SoftFloat(b);
	}
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator+(float a, SoftFloat b) noexcept {
		return SoftFloat(a) + b;
	}
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator+(int a, SoftFloat b) noexcept {
		return SoftFloat(a) + b;
	}
#endif
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator+(int32_t a, SoftFloat b) noexcept {
		return SoftFloat(a) + b;
	}

	// ------------------------------------------------------------------
	// Sub — constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
	friend SoftFloat operator-(SoftFloat a, SoftFloat b) noexcept
	{
		const uint32_t ab = a.bits;
		const uint32_t bb = b.bits;
		const uint32_t abe = field_exp(ab);
		const uint32_t bbe = field_exp(bb);

		if (UNLIKELY(bbe == 0u)) return a;
		if (UNLIKELY(abe == 0u)) return -b;

		const uint16_t an = sign_from_bits(ab);

		// Same signs: true subtraction of magnitudes.  Keep it as a direct
		// larger-magnitude-minus-smaller-magnitude operation instead of flipping
		// b and entering the fully-general operator+ dispatcher.  The explicit
		// exponent/field compare is what GCC turns into the shortest Cortex-M3 hot
		// path here (shorter than a branchless packed-magnitude select).
		if (LIKELY(((ab ^ bb) & 0x80000000u) == 0u)) {
			uint32_t lb, sb;
			uint32_t lbe, sbe;
			uint16_t neg = an;

			if ((abe > bbe) || ((abe == bbe) && (field_mant(ab) >= field_mant(bb)))) {
				lb = ab; sb = bb; lbe = abe; sbe = bbe;
			} else {
				lb = bb; sb = ab; lbe = bbe; sbe = abe;
				neg ^= 1u;
			}

			int d = static_cast<int>(lbe) - static_cast<int>(sbe);
			if (UNLIKELY(d >= 30))
				return from_bits_unchecked((lb & 0x7FFFFFFFu) | (static_cast<uint32_t>(neg) << 31));

			uint32_t lm = mant_from_bits(lb);
			uint32_t sm = mant_from_bits(sb) >> d;
			return finish_sub_mag(lm - sm, exp_from_biased(lbe), neg);
		}

		// Different signs: a - b is magnitude addition with a's sign.  This is the
		// same hot path as operator+'s same-sign addition, but written directly so
		// operator- does not drag in the entire add/sub dispatcher.
		int d = static_cast<int>(abe) - static_cast<int>(bbe);
		if (UNLIKELY(d >= 30)) return a;
		if (UNLIKELY(d <= -30))
			return from_bits_unchecked((bb & 0x7FFFFFFFu) | (static_cast<uint32_t>(an) << 31));

		uint32_t am = mant_from_bits(ab);
		uint32_t bm = mant_from_bits(bb);
		int32_t re;
		if (d >= 0) { bm >>= d;  re = exp_from_biased(abe); }
		else        { am >>= -d; re = exp_from_biased(bbe); }
		return finish_add_samesign(am + bm, re, an);
	}

	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator-(SoftFloat a, float b) noexcept {
		return a - SoftFloat(b);
	}
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator-(SoftFloat a, int b) noexcept {
		return a - SoftFloat(b);
	}
#endif
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator-(SoftFloat a, int32_t b) noexcept {
		return a - SoftFloat(b);
	}
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator-(float a, SoftFloat b) noexcept {
		return SoftFloat(a) - b;
	}
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator-(int a, SoftFloat b) noexcept {
		return SoftFloat(a) - b;
	}
#endif
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator-(int32_t a, SoftFloat b) noexcept {
		return SoftFloat(a) - b;
	}

	// =========================================================================
	// operator/ — hardware-udiv mantissa division (Cortex-M3 fast path)
	// =========================================================================
	[[nodiscard]] constexpr SF_HOT SoftFloat operator/(SoftFloat rhs) const noexcept
	{
		const uint32_t ab = bits;
		const uint32_t rb = rhs.bits;
		const uint32_t abe = field_exp(ab);
		const uint32_t rbe = field_exp(rb);
		if (UNLIKELY(rbe == 0u))
			return from_raw_normalized(MANT_MIN, EXP_MAX, sign_from_bits(ab));
		if (UNLIKELY(abe == 0u)) return zero();

		// Compile-time division by an exact power of two is just a biased-exponent
		// adjustment.  Do this only when the divisor bits are known at compile time
		// (or in constant evaluation): keeping a runtime power-of-two branch on the
		// generic hot path costs more than the UDIV/SDIV quotient path saves for
		// non-constant divisors.
		if ((SF_IS_CONSTEVAL() || IS_CONST(rb)) && UNLIKELY(field_mant(rb) == 0u)) {
			int32_t k = static_cast<int32_t>(rbe) - 127; // rhs magnitude == 2^k
			SoftFloat q = (k >= 0) ? (*this >> k) : (*this << -k);
			return sign_from_bits(rb) ? -q : q;
		}

		uint16_t neg = sign_from_bits(ab ^ rb);
		uint32_t ua  = mant_from_bits(ab);
		uint32_t ub  = mant_from_bits(rb);

		uint32_t qm;
		int32_t  qe = static_cast<int32_t>(abe) - static_cast<int32_t>(rbe) - (MANT_BITS + 1);

		// Cortex-M3 has a hardware integer divider (udiv/sdiv).  Computing the
		// Q30 mantissa quotient with two hardware divides (qfplib-m3 style:
		// udiv + mul + sdiv) is accurate to ~30 bits (<= 1 ULP at Q30,
		// far better than the 24-bit float target).
		//   qm = floor(ua * 2^30 / ub),  ua,ub in [2^29, 2^30)  =>  qm in (2^29, 2^31)
		// Stage 1: divide by a rounded 16-bit divisor approximation.
		// Stage 2: correct using the exact remainder vs the FULL divisor.
		{
			uint32_t x   = ua << 2;                      // Q31, in [2^31, 2^32)
			uint32_t yh  = (ub + (1u << 13)) >> 14;      // ~16-bit divisor approx (rounded)
			uint32_t q1  = x / yh;                        // first quotient estimate
			uint32_t r1  = (x << 14) - q1 * ub;          // exact remainder vs full ub (mod 2^32)
			int32_t  dq  = static_cast<int32_t>(r1) / static_cast<int32_t>(yh); // signed correction
			qm = (q1 << 14) + static_cast<uint32_t>(dq);
		}

		// qm = floor(ua*2^30/ub), with ua,ub in [2^29,2^30), is always
		// >= 2^29.  It may be in [2^30,2^31), so only a one-bit down-normalise
		// is needed; the old lshift-underflow guard was unreachable and cost
		// several instructions on Cortex-M3.
		uint32_t rshift = qm >> (MANT_BITS + 1);
		qm >>= rshift;
		qe += static_cast<int32_t>(rshift);

		return from_raw_normalized(qm, qe, neg);
	}

	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		SoftFloat operator/(float rhs) const noexcept {
		return *this / SoftFloat(rhs);
	}
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		SoftFloat operator/(int rhs) const noexcept {
		return *this / SoftFloat(rhs);
	}
#endif
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		SoftFloat operator/(int32_t rhs) const noexcept {
		return *this / SoftFloat(rhs);
	}
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator/(float lhs, SoftFloat rhs) noexcept {
		return SoftFloat(lhs) / rhs;
	}
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator/(int lhs, SoftFloat rhs) noexcept {
		return SoftFloat(lhs) / rhs;
	}
#endif
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		friend SoftFloat operator/(int32_t lhs, SoftFloat rhs) noexcept {
		return SoftFloat(lhs) / rhs;
	}

	// ------------------------------------------------------------------
	// reciprocal — 1/x via hardware-udiv mantissa division, O(1)
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SF_HOT SoftFloat reciprocal() const noexcept
	{
		const uint32_t ab = bits;
		const uint32_t abe = field_exp(ab);
		if (UNLIKELY(abe == 0u))
			return from_raw_normalized(MANT_MIN, EXP_MAX, sign_from_bits(ab));

		// Exact reciprocal for powers of two: new biased exponent is 127-k,
		// while the input has biased exponent 127+k, hence 254-abe.
		if (UNLIKELY(field_mant(ab) == 0u)) {
			int32_t nbe = 254 - static_cast<int32_t>(abe);
			SoftFloat r;
			if (UNLIKELY(nbe <= 0)) { r.bits = 0u; return r; }
			r.bits = field_sign(ab) | (static_cast<uint32_t>(nbe) << 23);
			return r;
		}

		uint32_t ub  = mant_from_bits(ab);

		// 1/x via the hardware-udiv mantissa divide.  This mirrors operator/
		// with the numerator being 1.0 (mantissa = 2^29, biased exponent = 127):
		//   qm = floor(2^29 * 2^30 / ub),  in (2^29, 2^31)
		//   qe = 127 - abe - (MANT_BITS + 1)
		uint32_t qm;
		int32_t  qe = 127 - static_cast<int32_t>(abe) - (MANT_BITS + 1);
		{
			constexpr uint32_t ua = MANT_MIN;            // 1.0 mantissa (2^29)
			uint32_t x   = ua << 2;                      // Q31
			uint32_t yh  = (ub + (1u << 13)) >> 14;      // 16-bit divisor approx
			uint32_t q1  = x / yh;
			uint32_t r1  = (x << 14) - q1 * ub;
			int32_t  dq  = static_cast<int32_t>(r1) / static_cast<int32_t>(yh);
			qm = (q1 << 14) + static_cast<uint32_t>(dq);
		}

		uint32_t rshift = qm >> (MANT_BITS + 1);
		qm >>= rshift;
		qe += static_cast<int32_t>(rshift);

		return from_raw_normalized(qm, qe, sign_from_bits(ab));
	}

	// ------------------------------------------------------------------
	// Power-of-2 scaling (exponent adjust only, O(1)) — constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		SoftFloat operator>>(int s) const noexcept {
		uint32_t be = field_exp(bits);
		if (UNLIKELY(be == 0u)) return zero();
		int32_t nbe = static_cast<int32_t>(be) - s;
		SoftFloat r;
		if (UNLIKELY(nbe <= 0)) { r.bits = 0u; return r; }
		if (UNLIKELY(nbe > 255)) { r.bits = (bits & 0x80000000u) | 0x7FFFFFFFu; return r; }
		r.bits = (bits & 0x807FFFFFu) | (static_cast<uint32_t>(nbe) << 23);
		return r;
	}
	constexpr SF_HOT SF_INLINE SF_FLATTEN
		SoftFloat& operator>>=(int s) noexcept {
		uint32_t be = field_exp(bits);
		if (UNLIKELY(be == 0u)) return *this;
		int32_t nbe = static_cast<int32_t>(be) - s;
		if (UNLIKELY(nbe <= 0)) { bits = 0u; return *this; }
		if (UNLIKELY(nbe > 255)) { bits = (bits & 0x80000000u) | 0x7FFFFFFFu; return *this; }
		bits = (bits & 0x807FFFFFu) | (static_cast<uint32_t>(nbe) << 23);
		return *this;
	}
	[[nodiscard]] constexpr SF_HOT SF_INLINE SF_FLATTEN
		SoftFloat operator<<(int s) const noexcept {
		uint32_t be = field_exp(bits);
		if (UNLIKELY(be == 0u)) return zero();
		int32_t nbe = static_cast<int32_t>(be) + s;
		SoftFloat r;
		if (UNLIKELY(nbe <= 0)) { r.bits = 0u; return r; }
		if (UNLIKELY(nbe > 255)) { r.bits = (bits & 0x80000000u) | 0x7FFFFFFFu; return r; }
		r.bits = (bits & 0x807FFFFFu) | (static_cast<uint32_t>(nbe) << 23);
		return r;
	}
	constexpr SF_HOT SF_INLINE SF_FLATTEN
		SoftFloat& operator<<=(int s) noexcept {
		uint32_t be = field_exp(bits);
		if (UNLIKELY(be == 0u)) return *this;
		int32_t nbe = static_cast<int32_t>(be) + s;
		if (UNLIKELY(nbe <= 0)) { bits = 0u; return *this; }
		if (UNLIKELY(nbe > 255)) {
			bits = (bits & 0x80000000u) | 0x7FFFFFFFu;
			return *this;
		}
		bits = (bits & 0x807FFFFFu) | (static_cast<uint32_t>(nbe) << 23);
		return *this;
	}

	// ------------------------------------------------------------------
	// Comparison — constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] friend constexpr bool operator==(SoftFloat a, SoftFloat b) noexcept {
		return a.bits == b.bits;
	}

#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator==(int av, SoftFloat b) noexcept { return SoftFloat(av) == b; }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator==(int32_t av, SoftFloat b) noexcept { return SoftFloat(av) == b; }
	[[nodiscard]] friend constexpr SF_INLINE bool operator==(float av, SoftFloat b) noexcept { return SoftFloat(av) == b; }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator==(SoftFloat a, int bv) noexcept { return a == SoftFloat(bv); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator==(SoftFloat a, int32_t bv) noexcept { return a == SoftFloat(bv); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator==(SoftFloat a, float bv) noexcept { return a == SoftFloat(bv); }

	[[nodiscard]] friend constexpr SF_INLINE bool operator!=(SoftFloat a, SoftFloat b) noexcept { return !(a == b); }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator!=(int a, SoftFloat b) noexcept { return !(a == b); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator!=(int32_t a, SoftFloat b) noexcept { return !(a == b); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator!=(float a, SoftFloat b) noexcept { return !(a == b); }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator!=(SoftFloat a, int b) noexcept { return !(a == b); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator!=(SoftFloat a, int32_t b) noexcept { return !(a == b); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator!=(SoftFloat a, float b) noexcept { return !(a == b); }

	[[nodiscard]] friend constexpr bool operator< (SoftFloat a, SoftFloat b) noexcept {
		// Canonical zero means the usual IEEE-style monotonic key transform works:
		// positives map above 0x80000000, negatives are bitwise inverted below it.
		uint32_t ab = a.bits;
		uint32_t bb = b.bits;
		uint32_t ak = ab ^ ((0u - (ab >> 31)) | 0x80000000u);
		uint32_t bk = bb ^ ((0u - (bb >> 31)) | 0x80000000u);
		return ak < bk;
	}

#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator< (int av, SoftFloat b) noexcept { return SoftFloat(av) < b; }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator< (int32_t av, SoftFloat b) noexcept { return SoftFloat(av) < b; }
	[[nodiscard]] friend constexpr SF_INLINE bool operator< (float av, SoftFloat b) noexcept { return SoftFloat(av) < b; }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator< (SoftFloat a, int bv)     noexcept { return a < SoftFloat(bv); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator< (SoftFloat a, int32_t bv)     noexcept { return a < SoftFloat(bv); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator< (SoftFloat a, float bv)   noexcept { return a < SoftFloat(bv); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator> (SoftFloat a, SoftFloat b) noexcept { return b < a; }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator> (int a, SoftFloat b) noexcept { return b < a; }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator> (int32_t a, SoftFloat b) noexcept { return b < a; }
	[[nodiscard]] friend constexpr SF_INLINE bool operator> (float a, SoftFloat b) noexcept { return b < a; }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator> (SoftFloat a, int b) noexcept { return b < a; }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator> (SoftFloat a, int32_t b) noexcept { return b < a; }
	[[nodiscard]] friend constexpr SF_INLINE bool operator> (SoftFloat a, float b) noexcept { return b < a; }

	[[nodiscard]] friend constexpr SF_INLINE bool operator<=(SoftFloat a, SoftFloat b) noexcept { return !(a > b); }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator<=(int a, SoftFloat b) noexcept { return !(a > b); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator<=(int32_t a, SoftFloat b) noexcept { return !(a > b); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator<=(float a, SoftFloat b) noexcept { return !(a > b); }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator<=(SoftFloat a, int b) noexcept { return !(a > b); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator<=(SoftFloat a, int32_t b) noexcept { return !(a > b); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator<=(SoftFloat a, float b) noexcept { return !(a > b); }

	[[nodiscard]] friend constexpr SF_INLINE bool operator>=(SoftFloat a, SoftFloat b) noexcept { return !(a < b); }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator>=(int a, SoftFloat b) noexcept { return !(a < b); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator>=(int32_t a, SoftFloat b) noexcept { return !(a < b); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator>=(float a, SoftFloat b) noexcept { return !(a < b); }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] friend constexpr SF_INLINE bool operator>=(SoftFloat a, int b) noexcept { return !(a < b); }
#endif
	[[nodiscard]] friend constexpr SF_INLINE bool operator>=(SoftFloat a, int32_t b) noexcept { return !(a < b); }
	[[nodiscard]] friend constexpr SF_INLINE bool operator>=(SoftFloat a, float b) noexcept { return !(a < b); }

	// ------------------------------------------------------------------
	// Utility queries — constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr bool is_zero()     const noexcept { return bits == 0u; }
	[[nodiscard]] constexpr bool is_negative() const noexcept { return (bits >> 31) != 0u; }
	[[nodiscard]] constexpr bool is_positive() const noexcept { return bits != 0u && (bits >> 31) == 0u; }
	[[nodiscard]] constexpr bool is_power_of_two() const noexcept {
		return (bits & 0x807FFFFFu) == 0u && field_exp(bits) != 0u;
	}
	[[nodiscard]] constexpr bool is_neg_power_of_two() const noexcept {
		return (bits & 0x807FFFFFu) == 0x80000000u && field_exp(bits) != 0u;
	}

	// ------------------------------------------------------------------
	// abs — constexpr, branch-free (ASR+EOR+SUB on ARM)
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SF_INLINE SoftFloat abs() const noexcept {
		SoftFloat r;
		r.bits = bits & 0x7FFFFFFFu;
		return r;
	}

	// ------------------------------------------------------------------
	// Clamp — constexpr
	// ------------------------------------------------------------------
	[[nodiscard]] constexpr SoftFloat clamp(SoftFloat lo, SoftFloat hi) const noexcept {
		if (*this < lo) return lo;
		if (*this > hi) return hi;
		return *this;
	}
	[[nodiscard]] constexpr SF_INLINE SoftFloat clamp(float lo, SoftFloat hi) const noexcept { return clamp(SoftFloat(lo), hi); }
	[[nodiscard]] constexpr SF_INLINE SoftFloat clamp(SoftFloat lo, float hi) const noexcept { return clamp(lo, SoftFloat(hi)); }
	[[nodiscard]] constexpr SF_INLINE SoftFloat clamp(float lo, float hi) const noexcept { return clamp(SoftFloat(lo), SoftFloat(hi)); }

	// ------------------------------------------------------------------
	// Math functions — constexpr via integer arithmetic only
	// ------------------------------------------------------------------
	// NOTE: sin/cos/tan now take Angle (autoconverts from SoftFloat radians)
	//       asin/acos/atan/atan2 now return Angle
	[[nodiscard]] constexpr SF_HOT SF_PURE FixedQ30 sin() const noexcept { return Angle(*this).sin(); }
	[[nodiscard]] constexpr SF_HOT SF_PURE FixedQ30 cos() const noexcept { return Angle(*this).cos(); }
	[[nodiscard]] constexpr SF_HOT SF_PURE SinCosPair sincos() const noexcept { return Angle(*this).sincos(); }
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat tan() const noexcept { return Angle(*this).tan(); }

	[[nodiscard]] constexpr SF_HOT SF_PURE Angle asin() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE Angle acos() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE Angle atan() const noexcept;

	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat sinh() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat cosh() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat tanh() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat inv_sqrt() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat sqrt()     const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat exp()     const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat log() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat log2() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat log10() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat pow(SoftFloat y) const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat trunc() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat floor() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat ceil() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat round() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE SoftFloat fract() const noexcept;
	[[nodiscard]] constexpr SF_HOT SF_PURE IntFractPair modf() const noexcept;
	[[nodiscard]] constexpr SoftFloat copysign(SoftFloat sign) const noexcept;
	[[nodiscard]] constexpr SoftFloat fmod(SoftFloat y) const noexcept;
	[[nodiscard]] constexpr SoftFloat fma(SoftFloat b, SoftFloat c) const noexcept;
	friend constexpr SF_HOT Angle atan2(SoftFloat y, SoftFloat x) noexcept;
	friend constexpr SF_HOT SoftFloat hypot(SoftFloat x, SoftFloat y) noexcept;
	friend constexpr SF_HOT SoftFloat lerp(SoftFloat a, SoftFloat b, SoftFloat t) noexcept;

	// ------------------------------------------------------------------
	// Compound assignment — constexpr
	// ------------------------------------------------------------------
	constexpr SoftFloat& operator+=(SoftFloat r) noexcept { *this = *this + r; return *this; }
	constexpr SoftFloat& operator-=(SoftFloat r) noexcept { *this = *this - r; return *this; }
	constexpr SoftFloat& operator*=(SoftFloat r) noexcept;
	constexpr SoftFloat& operator/=(SoftFloat r) noexcept { *this = *this / r; return *this; }

	// --- FMA autodetection: a += b * c  →  fused_mul_add(a, b, c) ---
	constexpr SF_HOT SF_INLINE SoftFloat& operator+=(const MulExpr& m) noexcept;
	constexpr SF_HOT SF_INLINE SoftFloat& operator-=(const MulExpr& m) noexcept;
	
	// ------------------------------------------------------------------
	// Constants
	// ------------------------------------------------------------------
	[[nodiscard]] static constexpr SoftFloat zero()     noexcept { return from_bits_unchecked(0x00000000u); }
	[[nodiscard]] static constexpr SoftFloat one()      noexcept { return from_bits_unchecked(0x3F800000u); }
	[[nodiscard]] static constexpr SoftFloat neg_one()  noexcept { return from_bits_unchecked(0xBF800000u); }
	[[nodiscard]] static constexpr SoftFloat half()     noexcept { return from_bits_unchecked(0x3F000000u); }
	[[nodiscard]] static constexpr SoftFloat two()      noexcept { return from_bits_unchecked(0x40000000u); }
	[[nodiscard]] static constexpr SoftFloat three()    noexcept { return from_bits_unchecked(0x40400000u); }
	[[nodiscard]] static constexpr SoftFloat four()     noexcept { return from_bits_unchecked(0x40800000u); }

	// Trigonometric constants - hardcoded to break recursion cycle with
	// Angle(SoftFloat) ctor and Angle::operator SoftFloat() which both use pi().
	[[nodiscard]] static constexpr SoftFloat pi()        noexcept { return from_bits_unchecked(0x40490FDBu); }
	[[nodiscard]] static constexpr SoftFloat half_pi()   noexcept { return from_bits_unchecked(0x3FC90FDBu); }
	[[nodiscard]] static constexpr SoftFloat two_pi()    noexcept { return from_bits_unchecked(0x40C90FDBu); }

	// ------------------------------------------------------------------
	// Fused operations (friend declarations)
	// ------------------------------------------------------------------
	friend constexpr SF_HOT SoftFloat fused_mul_add(SoftFloat a, SoftFloat b, SoftFloat c) noexcept;
	friend constexpr SF_HOT SoftFloat fused_mul_sub(SoftFloat a, SoftFloat b, SoftFloat c) noexcept;
	friend constexpr SF_HOT SoftFloat fused_mul_mul_add(SoftFloat a, SoftFloat b, SoftFloat c, SoftFloat d) noexcept;
	friend constexpr SF_HOT SoftFloat fused_mul_mul_sub(SoftFloat a, SoftFloat b, SoftFloat c, SoftFloat d) noexcept;
};

// =========================================================================
// Angle operator definitions (require SoftFloat definition; moved out-of-line
// to avoid incomplete-type SoftFloat during Angle class definition)
// =========================================================================
constexpr Angle Angle::operator+(SoftFloat x) const noexcept { return *this + Angle(x); }
constexpr Angle& Angle::operator+=(SoftFloat x) noexcept { return *this += Angle(x); }
constexpr Angle& Angle::operator-=(SoftFloat x) noexcept { return *this -= Angle(x); }

// =========================================================================
// Return types
// =========================================================================

struct IntFractPair { SoftFloat intpart; SoftFloat fracpart; };

// =========================================================================
// Expression-template proxy for a deferred single multiplication.
// Allows the compiler to fuse  a + b*c  into a single FMA call.
// =========================================================================
struct SoftFloat::MulExpr {
	SoftFloat lhs;
	SoftFloat rhs;

	[[nodiscard]] constexpr SF_INLINE SoftFloat eval() const noexcept {
#if 0
		// Optimizations when values are known at compile time - Disabled, as they increase code size and prevent some optimizations in the caller (e.g. constant folding of the whole expression).
		if (IS_CONST(lhs)) {
			if (lhs.is_power_of_two()) {
				return rhs << (lhs.exponent + SoftFloat::MANT_BITS);
			}
			if (lhs.is_neg_power_of_two()) {
				return -(rhs << (lhs.exponent + SoftFloat::MANT_BITS));
			}
		}
		if (IS_CONST(rhs)) {
			if (rhs.is_power_of_two()) {
				return lhs << (rhs.exponent + SoftFloat::MANT_BITS);
			}
			if (rhs.is_neg_power_of_two()) {
				return -(lhs << (rhs.exponent + SoftFloat::MANT_BITS));
			}
		}
		// fallback to normal multiplication
#endif
		return SoftFloat::mul_plain(lhs, rhs);
	}

	[[nodiscard]] constexpr explicit operator float()     const noexcept { return eval().to_float(); }
	[[nodiscard]] constexpr float     to_float()          const noexcept { return eval().to_float(); }
	[[nodiscard]] constexpr int32_t   to_int32()          const noexcept { return eval().to_int32(); }
	[[nodiscard]] constexpr bool      is_zero()           const noexcept { return eval().is_zero(); }
	[[nodiscard]] constexpr bool      is_negative()       const noexcept { return eval().is_negative(); }
	[[nodiscard]] constexpr SoftFloat abs()               const noexcept { return eval().abs(); }
	[[nodiscard]] constexpr SoftFloat sqrt()              const noexcept { return eval().sqrt(); }
	[[nodiscard]] constexpr SoftFloat exp()               const noexcept { return eval().exp(); }
	[[nodiscard]] constexpr SoftFloat log()               const noexcept { return eval().log(); }
	[[nodiscard]] constexpr SoftFloat log2()              const noexcept { return eval().log2(); }
	[[nodiscard]] constexpr SoftFloat log10()             const noexcept { return eval().log10(); }
	[[nodiscard]] constexpr SoftFloat pow(SoftFloat y)    const noexcept { return eval().pow(y); }
	[[nodiscard]] constexpr SoftFloat trunc()             const noexcept { return eval().trunc(); }
	[[nodiscard]] constexpr SoftFloat floor()             const noexcept { return eval().floor(); }
	[[nodiscard]] constexpr SoftFloat ceil()              const noexcept { return eval().ceil(); }
	[[nodiscard]] constexpr SoftFloat round()             const noexcept { return eval().round(); }
	[[nodiscard]] constexpr SoftFloat fract()             const noexcept { return eval().fract(); }
	[[nodiscard]] constexpr IntFractPair modf()           const noexcept { return eval().modf(); }
	[[nodiscard]] constexpr SoftFloat copysign(SoftFloat sign) const noexcept { return eval().copysign(sign); }
	[[nodiscard]] constexpr SoftFloat fmod(SoftFloat y)        const noexcept { return eval().fmod(y); }
	[[nodiscard]] constexpr SoftFloat fma(SoftFloat b, SoftFloat c) const noexcept { return eval().fma(b, c); }
	[[nodiscard]] constexpr SoftFloat inv_sqrt()          const noexcept { return eval().inv_sqrt(); }
	[[nodiscard]] constexpr SoftFloat reciprocal()        const noexcept { return eval().reciprocal(); }
	[[nodiscard]] constexpr SoftFloat clamp(SoftFloat lo, SoftFloat hi) const noexcept {
		return eval().clamp(lo, hi);
	}
	[[nodiscard]] constexpr FixedQ30 sin()               const noexcept { return eval().sin(); }
	[[nodiscard]] constexpr FixedQ30 cos()               const noexcept { return eval().cos(); }
	[[nodiscard]] constexpr SinCosPair sincos()           const noexcept { return eval().sincos(); }
	[[nodiscard]] constexpr SoftFloat tan()               const noexcept { return eval().tan(); }
	[[nodiscard]] constexpr Angle asin()              const noexcept { return eval().asin(); }
	[[nodiscard]] constexpr Angle acos()              const noexcept { return eval().acos(); }
	[[nodiscard]] constexpr Angle atan()              const noexcept { return eval().atan(); }
	[[nodiscard]] constexpr SoftFloat sinh()              const noexcept { return eval().sinh(); }
	[[nodiscard]] constexpr SoftFloat cosh()              const noexcept { return eval().cosh(); }
	[[nodiscard]] constexpr SoftFloat tanh()              const noexcept { return eval().tanh(); }
	[[nodiscard]] constexpr SoftFloat operator/(SoftFloat r)  const noexcept { return eval() / r; }
	[[nodiscard]] constexpr SoftFloat operator/(float r)      const noexcept { return eval() / SoftFloat(r); }
	[[nodiscard]] constexpr SoftFloat operator/(int32_t r)    const noexcept { return eval() / SoftFloat(r); }
#ifndef SF_INT_EQUALS_INT32
	[[nodiscard]] constexpr SoftFloat operator/(int r)    const noexcept { return eval() / SoftFloat(r); }
#endif
	[[nodiscard]] constexpr SoftFloat operator>>(int s)       const noexcept { return eval() >> s; }
	[[nodiscard]] constexpr SoftFloat operator<<(int s)       const noexcept { return eval() << s; }

	// Negate the expression (flips lhs sign, lazy — no evaluation)
	[[nodiscard]] constexpr MulExpr operator-() const noexcept {
		MulExpr r = *this;
		r.lhs = -r.lhs;
		return r;
	}
};

[[nodiscard]] constexpr int32_t FixedQ30::float_to_raw(SoftFloat v) noexcept {
	// Q30 stores value = raw / 2^30.  It can represent approximately
	// [-2.0, +2.0), which comfortably covers sin/cos and normalized factors.
	constexpr SoftFloat scale = SoftFloat(1073741824.0f); // 2^30

	if (v >= SoftFloat(2.0f)) return INT32_MAX;
	if (v <= SoftFloat(-2.0f)) return INT32_MIN;

	const SoftFloat scaled = v * scale;
	const SoftFloat rounded = (scaled >= SoftFloat::zero())
		? (scaled + SoftFloat::half())
		: (scaled - SoftFloat::half());
	return static_cast<int32_t>(rounded);
}

constexpr FixedQ30::FixedQ30(SoftFloat v) noexcept
	: raw_(float_to_raw(v)) {}

// =========================================================================
// SoftFloat proxy constructor (defined here so MulExpr is complete)
// =========================================================================
constexpr SF_HOT SF_INLINE SoftFloat::SoftFloat(const MulExpr& m) noexcept {
	SoftFloat v = m.eval();
	bits = v.bits;
}

// =========================================================================
// operator* — returns deferred proxy (constexpr, no computation yet)
// =========================================================================
[[nodiscard]] constexpr SF_HOT SF_INLINE SoftFloat::MulExpr operator*(SoftFloat a, SoftFloat b) noexcept {
	return { a, b };
}
[[nodiscard]] constexpr SF_HOT SF_INLINE SoftFloat::MulExpr operator*(SoftFloat a, float b) noexcept {
	return a * SoftFloat(b);
}
#ifndef SF_INT_EQUALS_INT32
[[nodiscard]] constexpr SF_HOT SF_INLINE SoftFloat::MulExpr operator*(SoftFloat a, int b) noexcept {
	return a * SoftFloat(b);
}
#endif
[[nodiscard]] constexpr SF_HOT SF_INLINE SoftFloat::MulExpr operator*(SoftFloat a, int32_t b) noexcept {
	return a * SoftFloat(b);
}
[[nodiscard]] constexpr SF_HOT SF_INLINE SoftFloat::MulExpr operator*(float a, SoftFloat b) noexcept {
	return SoftFloat(a) * b;
}
#ifndef SF_INT_EQUALS_INT32
[[nodiscard]] constexpr SF_HOT SF_INLINE SoftFloat::MulExpr operator*(int a, SoftFloat b) noexcept {
	return SoftFloat(a) * b;
}
#endif
[[nodiscard]] constexpr SF_HOT SF_INLINE SoftFloat::MulExpr operator*(int32_t a, SoftFloat b) noexcept {
	return SoftFloat(a) * b;
}

// --- FMA autodetection: a += b * c  →  fused_mul_add(a, b, c) ---
constexpr SF_HOT SF_INLINE SoftFloat& SoftFloat::operator+=(const MulExpr& m) noexcept {
	*this = fused_mul_add(*this, m.lhs, m.rhs);
	return *this;
}
constexpr SF_HOT SF_INLINE SoftFloat& SoftFloat::operator-=(const MulExpr& m) noexcept {
	*this = fused_mul_sub(*this, m.lhs, m.rhs);
	return *this;
}
constexpr SoftFloat& SoftFloat::operator*=(SoftFloat r) noexcept {
	*this = mul_plain(*this, r);
	return *this;
}

// =========================================================================
// normalise_fast definition
// =========================================================================
constexpr SF_INLINE void SoftFloat::normalise_fast(int32_t& m, int32_t& e) noexcept
{
	{
		// Unified path — no ssat needed since pack_normalized_bits already
		// clamps the biased exponent to [1, 255].
		uint32_t sign = static_cast<uint32_t>(m >> 31);
		uint32_t a = (static_cast<uint32_t>(m) ^ sign) - sign;
		if (LIKELY((a & MANT_TOP_TWO) == MANT_MIN)) {
			return;
		}
		if (UNLIKELY(a & MANT_OVERFLOW)) {
			a >>= 1;
			e += 1;
		}
		else {
#if defined(__arm__)

			if (!SF_IS_CONSTEVAL()) {
				uint32_t lz;
				__asm__("clz %0, %1" : "=r"(lz) : "r"(a));
				int32_t shift = static_cast<int32_t>(lz) - 2;
				a <<= shift;
				e -= shift;
				m = static_cast<int32_t>((a ^ sign) - sign);
				return;
			}
#endif
			int sh = clz(a) - 2;
			e -= sh;
			a <<= sh;
		}
		m = static_cast<int32_t>((a ^ sign) - sign);
	}
}

// =========================================================================
// Fused arithmetic — constexpr (implicitly inline since constexpr)
// =========================================================================

[[nodiscard]] constexpr SF_HOT SoftFloat fused_mul_add(SoftFloat a, SoftFloat b, SoftFloat c) noexcept {
	const uint32_t ab = a.bits;
	const uint32_t bb = b.bits;
	const uint32_t cb = c.bits;
	const uint32_t abe = SoftFloat::field_exp(ab);
	const uint32_t bbe = SoftFloat::field_exp(bb);
	const uint32_t cbe = SoftFloat::field_exp(cb);
	if (UNLIKELY((bbe == 0u) | (cbe == 0u))) return a;
	if (UNLIKELY(abe == 0u)) return SoftFloat::mul_plain(b, c);

	// Use unsigned mantissa multiply + separate sign tracking.
	// Avoids 2 conditional negates (sign-encode of b,c) present in the old code.
	// Product sign = b.negative XOR c.negative.
	uint16_t bc_neg = SoftFloat::sign_from_bits(bb ^ cb);
	uint32_t pm_u = SoftFloat::mul24_to_q29(SoftFloat::mant24_from_bits(bb), SoftFloat::mant24_from_bits(cb));

	// Branchless overflow normalisation for product
	uint32_t ov = pm_u >> 30;
	pm_u >>= ov;

        // Compute exponent difference directly, deferring pe computation.
        // d = (abe - EXP_BIAS) - (bbe + cbe - 2*EXP_BIAS + MANT_BITS + ov)
        //   = abe - (bbe + cbe) + (EXP_BIAS - MANT_BITS) - ov
        int d = static_cast<int>(abe) - static_cast<int>(bbe + cbe)
              + (SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS) - static_cast<int>(ov);
        // Combined range check: (unsigned)(d + 30) >= 60 catches both
        // d >= 30 and d < -30 in a single comparison (saves 1 instruction
        // on the hot path vs two separate checks).
        if (UNLIKELY(static_cast<uint32_t>(d + 30) >= 60u)) {
                if (d >= 30) return a;
                // d <= -30
                int32_t pe = static_cast<int32_t>(bbe + cbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS) + static_cast<int32_t>(ov);
                return SoftFloat::finish_addsub_u(pm_u, pe, bc_neg);
        }

	uint32_t am_u = SoftFloat::mant_from_bits(ab);
	const uint16_t an = SoftFloat::sign_from_bits(ab);

	int32_t re;
	if (d >= 0) { pm_u >>= d; re = SoftFloat::exp_from_biased(abe); }
        else {
                am_u >>= -d;
                re = static_cast<int32_t>(bbe + cbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS) + static_cast<int32_t>(ov);
        }

	// Unsigned add/sub with sign tracking — mirrors operator+ logic
	if (an == bc_neg) {
		return SoftFloat::finish_add_samesign(am_u + pm_u, re, an);
	}
	if (am_u >= pm_u)
                return SoftFloat::finish_sub_mag(am_u - pm_u, re, an);
	else
                return SoftFloat::finish_sub_mag(pm_u - am_u, re, bc_neg);
}

[[nodiscard]] constexpr SF_HOT SoftFloat fused_mul_sub(SoftFloat a, SoftFloat b, SoftFloat c) noexcept {
	const uint32_t ab = a.bits;
	const uint32_t bb = b.bits;
	const uint32_t cb = c.bits;
	const uint32_t abe = SoftFloat::field_exp(ab);
	const uint32_t bbe = SoftFloat::field_exp(bb);
	const uint32_t cbe = SoftFloat::field_exp(cb);
	if (UNLIKELY((bbe == 0u) | (cbe == 0u))) return a;
	if (UNLIKELY(abe == 0u)) return -SoftFloat::mul_plain(b, c);

	// a - b*c: product sign = b.negative XOR c.negative XOR 1 (negated product)
	uint16_t bc_neg = static_cast<uint16_t>(SoftFloat::sign_from_bits(bb ^ cb) ^ 1u);
	uint32_t pm_u = SoftFloat::mul24_to_q29(SoftFloat::mant24_from_bits(bb), SoftFloat::mant24_from_bits(cb));

	uint32_t ov = pm_u >> 30;
	pm_u >>= ov;

        // Compute d directly, deferring pe (same optimization as fused_mul_add)
        int d = static_cast<int>(abe) - static_cast<int>(bbe + cbe)
              + (SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS) - static_cast<int>(ov);
        // Combined range check (same as fused_mul_add)
        if (UNLIKELY(static_cast<uint32_t>(d + 30) >= 60u)) {
                if (d >= 30) return a;
                // d <= -30
                int32_t pe = static_cast<int32_t>(bbe + cbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS) + static_cast<int32_t>(ov);
                return SoftFloat::finish_addsub_u(pm_u, pe, bc_neg);
        }

	uint32_t am_u = SoftFloat::mant_from_bits(ab);
	const uint16_t an = SoftFloat::sign_from_bits(ab);

	int32_t re;
	if (d >= 0) { pm_u >>= d; re = SoftFloat::exp_from_biased(abe); }
        else {
                am_u >>= -d;
                re = static_cast<int32_t>(bbe + cbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS) + static_cast<int32_t>(ov);
        }

	if (an == bc_neg) {
		return SoftFloat::finish_add_samesign(am_u + pm_u, re, an);
	}
	if (am_u >= pm_u)
                return SoftFloat::finish_sub_mag(am_u - pm_u, re, an);
	else
                return SoftFloat::finish_sub_mag(pm_u - am_u, re, bc_neg);
}

[[nodiscard]] constexpr SF_HOT SoftFloat fused_mul_mul_add(SoftFloat a, SoftFloat b, SoftFloat c, SoftFloat d) noexcept {
	const uint32_t ab = a.bits;
	const uint32_t bb = b.bits;
	const uint32_t cb = c.bits;
	const uint32_t db = d.bits;
	const uint32_t abe = SoftFloat::field_exp(ab);
	const uint32_t bbe = SoftFloat::field_exp(bb);
	const uint32_t cbe = SoftFloat::field_exp(cb);
	const uint32_t dbe = SoftFloat::field_exp(db);
	bool abz = (abe == 0u) | (bbe == 0u);
	bool cdz = (cbe == 0u) | (dbe == 0u);
	if (UNLIKELY(abz || cdz)) {
		if (abz && cdz) return SoftFloat::zero();
		if (abz)        return SoftFloat::mul_plain(c, d);
		return SoftFloat::mul_plain(a, b);
	}

	// Unsigned products with separate sign tracking — eliminates 4 conditional
	// negates (a.negative ? -m : m) that the old signed version had.
	// Same pattern as fused_mul_add and mul_plain.

	// ---- Product 1: a * b ----
	uint16_t neg1 = SoftFloat::sign_from_bits(ab ^ bb);
	uint32_t pm1 = SoftFloat::mul24_to_q29(SoftFloat::mant24_from_bits(ab), SoftFloat::mant24_from_bits(bb));
	int32_t  pe1 = static_cast<int32_t>(abe + bbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS);

	// ---- Product 2: c * d ----
	uint16_t neg2 = SoftFloat::sign_from_bits(cb ^ db);
	uint32_t pm2 = SoftFloat::mul24_to_q29(SoftFloat::mant24_from_bits(cb), SoftFloat::mant24_from_bits(db));
	int32_t  pe2 = static_cast<int32_t>(cbe + dbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS);

	// Branchless overflow normalisation (constexpr-friendly: use local copies)
	{
		uint32_t ov1 = pm1 >> 30;
		uint32_t ov2 = pm2 >> 30;
		pm1 = (pm1 >> ov1);   pe1 += static_cast<int32_t>(ov1);
		pm2 = (pm2 >> ov2);   pe2 += static_cast<int32_t>(ov2);
	}

	// ---- Align exponents and add/sub with sign tracking ----
	int d_exp = pe1 - pe2;
	if (UNLIKELY(d_exp >= 30))
		return SoftFloat::finish_addsub_u(pm1, pe1, neg1);
	if (UNLIKELY(d_exp <= -30))
		return SoftFloat::finish_addsub_u(pm2, pe2, neg2);

	int32_t re;
	if (d_exp >= 0) { pm2 >>= d_exp; re = pe1; }
	else { pm1 >>= -d_exp; re = pe2; }

	if (neg1 == neg2) {
		return SoftFloat::finish_add_samesign(pm1 + pm2, re, neg1);
	}
        // Different signs: use finish_sub_mag (no overflow check needed)
	if (pm1 >= pm2)
                return SoftFloat::finish_sub_mag(pm1 - pm2, re, neg1);
	else
                return SoftFloat::finish_sub_mag(pm2 - pm1, re, neg2);
}

[[nodiscard]] constexpr SF_HOT SoftFloat fused_mul_mul_sub(SoftFloat a, SoftFloat b,
	SoftFloat c, SoftFloat d) noexcept {
	const uint32_t ab = a.bits;
	const uint32_t bb = b.bits;
	const uint32_t cb = c.bits;
	const uint32_t db = d.bits;
	const uint32_t abe = SoftFloat::field_exp(ab);
	const uint32_t bbe = SoftFloat::field_exp(bb);
	const uint32_t cbe = SoftFloat::field_exp(cb);
	const uint32_t dbe = SoftFloat::field_exp(db);
	bool abz = (abe == 0u) | (bbe == 0u);
	bool cdz = (cbe == 0u) | (dbe == 0u);
	if (UNLIKELY(abz || cdz)) {
		if (abz && cdz) return SoftFloat::zero();
		if (abz)        return -SoftFloat::mul_plain(c, d);
		return SoftFloat::mul_plain(a, b);
	}

	uint16_t neg1 = SoftFloat::sign_from_bits(ab ^ bb);
	uint32_t pm1 = SoftFloat::mul24_to_q29(SoftFloat::mant24_from_bits(ab), SoftFloat::mant24_from_bits(bb));
	int32_t  pe1 = static_cast<int32_t>(abe + bbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS);

	uint16_t neg2 = static_cast<uint16_t>(SoftFloat::sign_from_bits(cb ^ db) ^ 1u);
	uint32_t pm2 = SoftFloat::mul24_to_q29(SoftFloat::mant24_from_bits(cb), SoftFloat::mant24_from_bits(db));
	int32_t  pe2 = static_cast<int32_t>(cbe + dbe) - (2 * SoftFloat::EXP_BIAS - SoftFloat::MANT_BITS);

	uint32_t ov1 = pm1 >> 30;
	uint32_t ov2 = pm2 >> 30;
	pm1 >>= ov1;   pe1 += static_cast<int32_t>(ov1);
	pm2 >>= ov2;   pe2 += static_cast<int32_t>(ov2);

	int d_exp = pe1 - pe2;
	if (UNLIKELY(d_exp >= 30))
		return SoftFloat::finish_addsub_u(pm1, pe1, neg1);
	if (UNLIKELY(d_exp <= -30))
		return SoftFloat::finish_addsub_u(pm2, pe2, neg2);

	int32_t re;
	if (d_exp >= 0) { pm2 >>= d_exp; re = pe1; }
	else { pm1 >>= -d_exp; re = pe2; }

	if (neg1 == neg2)
		return SoftFloat::finish_add_samesign(pm1 + pm2, re, neg1);
        // Different signs: use finish_sub_mag (no overflow check needed)
	if (pm1 >= pm2)
                return SoftFloat::finish_sub_mag(pm1 - pm2, re, neg1);
        return SoftFloat::finish_sub_mag(pm2 - pm1, re, neg2);
}

// =========================================================================
// Mixed expression-template operators
// =========================================================================

[[nodiscard]] constexpr SF_INLINE
SoftFloat operator+(const SoftFloat::MulExpr& x, const SoftFloat::MulExpr& y) noexcept {
	return fused_mul_mul_add(x.lhs, x.rhs, y.lhs, y.rhs);
}
[[nodiscard]] constexpr SF_INLINE
SoftFloat operator-(const SoftFloat::MulExpr& x, const SoftFloat::MulExpr& y) noexcept {
	return fused_mul_mul_sub(x.lhs, x.rhs, y.lhs, y.rhs);
}

[[nodiscard]] constexpr SF_INLINE
SoftFloat operator+(SoftFloat a, const SoftFloat::MulExpr& m) noexcept {
	return fused_mul_add(a, m.lhs, m.rhs);
}
[[nodiscard]] constexpr SF_INLINE
SoftFloat operator-(SoftFloat a, const SoftFloat::MulExpr& m) noexcept {
	return fused_mul_sub(a, m.lhs, m.rhs);
}

[[nodiscard]] constexpr SF_INLINE
SoftFloat operator+(const SoftFloat::MulExpr& m, SoftFloat a) noexcept {
	return fused_mul_add(a, m.lhs, m.rhs);
}

[[nodiscard]] constexpr SF_INLINE
SoftFloat operator-(const SoftFloat::MulExpr& m, SoftFloat a) noexcept {
	return -fused_mul_sub(a, m.lhs, m.rhs);
}

// =========================================================================
// User-defined literals — constexpr
// =========================================================================
[[nodiscard]] constexpr SoftFloat operator""_sf(long double v) noexcept {
	return SoftFloat(static_cast<float>(v));
}
[[nodiscard]] constexpr SoftFloat operator""_sf(unsigned long long v) noexcept {
	return SoftFloat(static_cast<int32_t>(v));
}

// =========================================================================
// Convenience free functions — constexpr, no sf_ prefix
// =========================================================================
[[nodiscard]] constexpr SoftFloat abs(SoftFloat x)                                noexcept { return x.abs(); }
[[nodiscard]] constexpr DeltaAngle abs(DeltaAngle d)                              noexcept { return d.abs(); }
[[nodiscard]] constexpr SoftFloat sqrt(SoftFloat x)                               noexcept { return x.sqrt(); }
[[nodiscard]] constexpr SoftFloat exp(SoftFloat x)                                noexcept { return x.exp(); }
[[nodiscard]] constexpr SoftFloat log(SoftFloat x)                                noexcept { return x.log(); }
[[nodiscard]] constexpr SoftFloat log2(SoftFloat x)                               noexcept { return x.log2(); }
[[nodiscard]] constexpr SoftFloat log10(SoftFloat x)                              noexcept { return x.log10(); }
[[nodiscard]] constexpr SoftFloat pow(SoftFloat x, SoftFloat y)                   noexcept { return x.pow(y); }
[[nodiscard]] constexpr SoftFloat trunc(SoftFloat x)                              noexcept { return x.trunc(); }
[[nodiscard]] constexpr SoftFloat floor(SoftFloat x)                              noexcept { return x.floor(); }
[[nodiscard]] constexpr SoftFloat ceil(SoftFloat x)                               noexcept { return x.ceil(); }
[[nodiscard]] constexpr SoftFloat round(SoftFloat x)                              noexcept { return x.round(); }
[[nodiscard]] constexpr SoftFloat fract(SoftFloat x)                              noexcept { return x.fract(); }
[[nodiscard]] constexpr IntFractPair modf(SoftFloat x)                            noexcept { return x.modf(); }
[[nodiscard]] constexpr SoftFloat copysign(SoftFloat x, SoftFloat sign)           noexcept { return x.copysign(sign); }
[[nodiscard]] constexpr SoftFloat fmod(SoftFloat x, SoftFloat y)                  noexcept { return x.fmod(y); }
[[nodiscard]] constexpr SoftFloat fma(SoftFloat x, SoftFloat b, SoftFloat c)      noexcept { return x.fma(b, c); }
[[nodiscard]] constexpr SoftFloat inv_sqrt(SoftFloat x)                           noexcept { return x.inv_sqrt(); }
[[nodiscard]] constexpr SoftFloat reciprocal(SoftFloat x)                         noexcept { return x.reciprocal(); }
[[nodiscard]] constexpr SoftFloat min(SoftFloat a, SoftFloat b)                   noexcept { return (a < b) ? a : b; }
[[nodiscard]] constexpr SoftFloat max(SoftFloat a, SoftFloat b)                   noexcept { return (a > b) ? a : b; }
[[nodiscard]] constexpr SoftFloat clamp(SoftFloat v, SoftFloat lo, SoftFloat hi)  noexcept { return v.clamp(lo, hi); }
[[nodiscard]] constexpr SinCosPair sincos(SoftFloat x)                            noexcept { return x.sincos(); }
[[nodiscard]] constexpr FixedQ30 sin(Angle x)                                noexcept { return x.sin(); }
[[nodiscard]] constexpr FixedQ30 cos(Angle x)                                noexcept { return x.cos(); }
[[nodiscard]] constexpr SoftFloat tan(Angle x)                                noexcept { return x.tan(); }
[[nodiscard]] constexpr Angle asin(SoftFloat x)                                   noexcept { return x.asin(); }
[[nodiscard]] constexpr Angle acos(SoftFloat x)                                   noexcept { return x.acos(); }
[[nodiscard]] constexpr Angle atan(SoftFloat x)                                   noexcept { return x.atan(); }
[[nodiscard]] constexpr SoftFloat sinh(SoftFloat x)                               noexcept { return x.sinh(); }
[[nodiscard]] constexpr SoftFloat cosh(SoftFloat x)                               noexcept { return x.cosh(); }
[[nodiscard]] constexpr SoftFloat tanh(SoftFloat x)                               noexcept { return x.tanh(); }

// =========================================================================
// DeltaAngle full definitions (conversions + operators; placed here after
// SoftFloat + MulExpr are fully defined to avoid incomplete-type MulExpr errors
// on * and related. This enables DeltaAngle * SoftFloat for angle integration
// e.g. current += omega * dt  where omega, dt are SoftFloat).
// =========================================================================
constexpr DeltaAngle::DeltaAngle(SoftFloat d) noexcept {
    raw_ = static_cast<int32_t>(Angle(d).get_raw());
}

constexpr DeltaAngle::operator SoftFloat() const noexcept {
    // raw * pi / 2^31, preserving the signed delta.  Use unsigned abs to handle
    // INT32_MIN (-π) without UB.
    uint32_t uv = static_cast<uint32_t>(raw_);
    uint16_t neg = static_cast<uint16_t>(uv >> 31);
    uint32_t mask = 0u - static_cast<uint32_t>(neg);
    uint32_t mag = (uv ^ mask) - mask;
    return SoftFloat::from_u64_scaled(static_cast<uint64_t>(mag) * 843314857ull, -59, neg);
}

constexpr DeltaAngle DeltaAngle::operator*(SoftFloat s) const noexcept {
    if (UNLIKELY(s.is_zero() || raw_ == 0)) return DeltaAngle(0);
    SoftFloat d = *this;  // to radians
    SoftFloat prod = d * s;
    return DeltaAngle(prod);  // back to delta, with reduction
}

constexpr DeltaAngle DeltaAngle::operator*(float s) const noexcept {
    return (*this) * SoftFloat(s);
}

constexpr DeltaAngle operator*(SoftFloat s, DeltaAngle d) noexcept {
	return d * s;
}

constexpr DeltaAngle DeltaAngle::abs() const noexcept {
    // Safe abs to avoid UB on INT32_MIN (which is used to represent -π)
    if (raw_ == INT32_MIN) return DeltaAngle(INT32_MAX);
    return DeltaAngle(raw_ < 0 ? -raw_ : raw_);
}

constexpr DeltaAngle DeltaAngle::operator-() const noexcept {
    // Safe negation to avoid UB on INT32_MIN (maps -π to +π representation)
    if (raw_ == INT32_MIN) return DeltaAngle(INT32_MAX);
    return DeltaAngle(-raw_);
}

// Mixed comparisons DeltaAngle vs float (member side; use SoftFloat bridge for consistency/precision).
// 
// Note on alternative ("cast float to DeltaAngle then raw-compare"):
//   DeltaAngle(SoftFloat(rhs)) would use the heavy Delta ctor (fmod(two_pi) + scaling by 2^31/pi + saturation branches + multiple SoftFloat mul/div).
//   That is more expensive than the current path:
//     - Delta -> SoftFloat (lightweight: simple scale by pi/2^31)
//     - SoftFloat(float) (cheap normalization, often constant-folded)
//     - SoftFloat == / < etc (direct mant/exp/neg compare, very cheap)
//   Current path is faster in practice + has better semantics (rhs float is not auto-reduced/wrapped like a delta would be).
//   Raw Delta-Delta compares are already used for Delta vs Delta (exact + cheap).
constexpr bool DeltaAngle::operator==(float rhs) const noexcept { return static_cast<SoftFloat>(*this) == SoftFloat(rhs); }
constexpr bool DeltaAngle::operator!=(float rhs) const noexcept { return static_cast<SoftFloat>(*this) != SoftFloat(rhs); }
constexpr bool DeltaAngle::operator< (float rhs) const noexcept { return static_cast<SoftFloat>(*this) <  SoftFloat(rhs); }
constexpr bool DeltaAngle::operator> (float rhs) const noexcept { return static_cast<SoftFloat>(*this) >  SoftFloat(rhs); }
constexpr bool DeltaAngle::operator<=(float rhs) const noexcept { return static_cast<SoftFloat>(*this) <= SoftFloat(rhs); }
constexpr bool DeltaAngle::operator>=(float rhs) const noexcept { return static_cast<SoftFloat>(*this) >= SoftFloat(rhs); }

// Reverse mixed: float vs DeltaAngle (free funcs for symmetry, like SoftFloat)
[[nodiscard]] constexpr bool operator==(float lhs, DeltaAngle rhs) noexcept { return rhs == lhs; }
[[nodiscard]] constexpr bool operator!=(float lhs, DeltaAngle rhs) noexcept { return rhs != lhs; }
[[nodiscard]] constexpr bool operator< (float lhs, DeltaAngle rhs) noexcept { return rhs >  lhs; }  // note swapped
[[nodiscard]] constexpr bool operator> (float lhs, DeltaAngle rhs) noexcept { return rhs <  lhs; }
[[nodiscard]] constexpr bool operator<=(float lhs, DeltaAngle rhs) noexcept { return rhs >= lhs; }
[[nodiscard]] constexpr bool operator>=(float lhs, DeltaAngle rhs) noexcept { return rhs <= lhs; }

// DeltaAngle + DeltaAngle and - (result reduced back to [-π, π) range).
// Uses pure 32-bit modular arithmetic: cast raw to uint32_t, add/sub (which
// wraps mod 2^32 in a well-defined way with no signed overflow), then
// reinterpret the bits as signed int32_t via static_cast. This produces the
// correct reduced DeltaAngle value directly in the representation units.
// Yes — modular arithmetic on uint32_t handles adding/subtracting DeltaAngles
// without needing 64-bit expansion at all (and is faster on targets like Cortex-M3).
constexpr DeltaAngle operator+(DeltaAngle a, DeltaAngle b) noexcept {
    uint32_t ua = static_cast<uint32_t>(a.raw_);
    uint32_t ub = static_cast<uint32_t>(b.raw_);
    uint32_t usum = ua + ub;  // modular wrap mod 2^32 (no UB)
    return DeltaAngle(static_cast<int32_t>(usum));
}

constexpr DeltaAngle operator-(DeltaAngle a, DeltaAngle b) noexcept {
    uint32_t ua = static_cast<uint32_t>(a.raw_);
    uint32_t ub = static_cast<uint32_t>(b.raw_);
    uint32_t udiff = ua - ub;  // modular wrap mod 2^32 (no UB)
    return DeltaAngle(static_cast<int32_t>(udiff));
}

// DeltaAngle / DeltaAngle -> dimensionless SoftFloat ratio.
// The two operands use the same fixed angular unit (2^31 units == π), so the
// scale cancels and the result is simply raw_a / raw_b.  This avoids converting
// both operands to radians (faster, no π roundtrip error) while preserving the
// usual SoftFloat division-by-zero behaviour.
[[nodiscard]] constexpr SoftFloat operator/(DeltaAngle a, DeltaAngle b) noexcept {
	auto raw_to_sf = [](int32_t r) constexpr noexcept -> SoftFloat {
		// SoftFloat(int32_t) would try to negate INT32_MIN, so handle the
		// -π boundary raw value explicitly as -2^31.
		if (UNLIKELY(r == INT32_MIN)) {
			return SoftFloat::from_raw_normalized(SoftFloat::MANT_MIN, 2, 1);
		}
		return SoftFloat(r);
	};

	return raw_to_sf(a.raw_) / raw_to_sf(b.raw_);
}

constexpr SF_HOT SoftFloat Angle::tan() const noexcept {
	if (UNLIKELY(raw_ == 0)) return SoftFloat::zero();

	// tan has period π.  Doubling the 2π phase maps one tan period to the full
	// signed int32 range, with ±π/2 at INT32_MIN.  Then mirror to [0, π/4] and
	// either use tan(x) directly or cot(x)=1/tan(x).  This avoids sincos() plus
	// two normalisations plus a generic mantissa divide on the common path.
	int32_t p = static_cast<int32_t>(raw_ << 1);
	uint32_t uv = static_cast<uint32_t>(p);
	uint16_t neg = static_cast<uint16_t>(uv >> 31);
	uint32_t mask = 0u - static_cast<uint32_t>(neg);
	uint32_t mag = (uv ^ mask) - mask; // safe abs, INT32_MIN -> 0x80000000

	if (UNLIKELY(mag == 0x80000000u))
		return SoftFloat::from_raw_normalized(SoftFloat::MANT_MIN, SoftFloat::EXP_MAX, neg);

	bool recip = false;
	if (mag > 0x40000000u) {
		mag = 0x80000000u - mag;
		recip = true;
		if (UNLIKELY(mag == 0u))
			return SoftFloat::from_raw_normalized(SoftFloat::MANT_MIN, SoftFloat::EXP_MAX, neg);
	}

	const uint32_t idx = mag >> 20;          // 0..1024 over [0, π/4]
	const uint32_t frac = mag & 0x000FFFFFu;
	uint32_t q;
	if (LIKELY(idx < 1024u)) {
		const uint32_t y0 = SoftFloat::TAN_PI4_Q30[idx];
		const uint32_t y1 = SoftFloat::TAN_PI4_Q30[idx + 1u];
#if defined(__arm__)
		if (!SF_IS_CONSTEVAL()) {
			uint32_t diff = y1 - y0;
			uint32_t lo, hi;
			__asm__(
				"umull %0, %1, %2, %3"
				: "=&r"(lo), "=&r"(hi)
				: "r"(diff), "r"(frac));
			q = y0 + ((lo >> 20) | (hi << 12));
		} else
#endif
		{
			q = y0 + static_cast<uint32_t>((static_cast<uint64_t>(y1 - y0) * frac) >> 20);
		}
	} else {
		q = 0x40000000u; // tan(π/4) == 1.0 in Q30
	}

	if (UNLIKELY(q == 0u))
		return recip ? SoftFloat::from_raw_normalized(SoftFloat::MANT_MIN, SoftFloat::EXP_MAX, neg)
		: SoftFloat::zero();

	auto norm_q30 = [](uint32_t a, uint32_t& m, int32_t& e) constexpr noexcept {
		if (a >= SoftFloat::MANT_OVERFLOW) { m = a >> 1; e = -29; }
		else if (a >= SoftFloat::MANT_MIN) { m = a; e = -30; }
		else { int sh = SoftFloat::clz(a) - 2; m = a << sh; e = -30 - sh; }
		};

	uint32_t m;
	int32_t e;
	norm_q30(q, m, e);

	if (!recip)
		return SoftFloat::from_raw_normalized(m, e, neg);

	// reciprocal of q/2^30, inlined from SoftFloat::reciprocal() with the
	// normalized q30 mantissa/exponent already available.
	uint32_t qm;
	int32_t qe = -e - (2 * SoftFloat::MANT_BITS + 1); // == 127-(e+EXP_BIAS)-30
	{
		constexpr uint32_t ua = SoftFloat::MANT_MIN;
		uint32_t x = ua << 2;
		uint32_t yh = (m + (1u << 13)) >> 14;
		uint32_t q1 = x / yh;
		uint32_t r1 = (x << 14) - q1 * m;
		int32_t  dq = static_cast<int32_t>(r1) / static_cast<int32_t>(yh);
		qm = (q1 << 14) + static_cast<uint32_t>(dq);
	}
	uint32_t rshift = qm >> (SoftFloat::MANT_BITS + 1);
	qm >>= rshift;
	qe += static_cast<int32_t>(rshift);
	return SoftFloat::from_raw_normalized(qm, qe, neg);
}

[[nodiscard]] constexpr SF_HOT SF_FLATTEN
Angle SoftFloat::asin() const noexcept
{
	SoftFloat x = *this;
	bool neg = x.is_negative();
	x = x.abs();
	if (UNLIKELY(x > SoftFloat::one())) return Angle::zero();

	Angle result = atan2(x, (SoftFloat::one() - x * x).sqrt());
	// Negate the angle using DeltaAngle: zero + (-result)
	return neg ? (Angle::zero() + (-result)) : result;
}

[[nodiscard]] constexpr SF_HOT SF_FLATTEN
Angle SoftFloat::acos() const noexcept
{
	SoftFloat ax = this->abs();
	if (UNLIKELY(ax > SoftFloat::one())) return Angle::zero();

	SoftFloat y = (SoftFloat::one() - ax * ax).sqrt();
	Angle r = atan2(y, ax);

	if (this->is_negative()) {
		// Compute pi - r directly in Angle units (RAW_PI - r.raw()) to avoid
		// temporary conversion to SoftFloat + back to Angle. This is faster
		// and stays within the modular Angle representation.
		r = Angle(Angle::RAW_PI - r.raw_);
	}
	return r;
}

[[nodiscard]] constexpr SF_HOT SF_FLATTEN
Angle SoftFloat::atan() const noexcept
{
	// atan(x) = atan2(x, 1)
	return atan2(*this, SoftFloat::one());
}

constexpr SoftFloat SoftFloat::sinh() const noexcept {
	SoftFloat e = exp();
	return (e - e.reciprocal()) >> 1;
}
constexpr SoftFloat SoftFloat::cosh() const noexcept {
	SoftFloat e = exp();
	return (e + e.reciprocal()) >> 1;
}
constexpr SoftFloat SoftFloat::tanh() const noexcept {
	SoftFloat e2 = (*this << 1).exp();
	SoftFloat num = e2 - SoftFloat::one();
	SoftFloat den = e2 + SoftFloat::one();
	// A single division is cheaper than reciprocal()+mul_plain: both need one
	// recip32, but operator/ folds the final multiply+rounding into one UMULL
	// path, whereas reciprocal() pays an extra normalisation + a full mul_plain.
	return num / den;
}

constexpr SF_HOT SoftFloat SoftFloat::inv_sqrt() const noexcept
{
	const uint32_t sb = bits;
	const uint32_t sbe = field_exp(sb);
	if (UNLIKELY(sbe == 0u || (sb >> 31))) return zero();

	const int32_t  E_raw = exp_from_biased(sbe) + 29;
	const uint32_t a = mant_from_bits(sb);

	const uint32_t offset = a - MANT_MIN;
	const uint32_t idx = offset >> 21;
	const uint32_t frac8 = (offset >> 13) & 0xFFu;

	const int32_t v0 = INV_SQRT_Q29[idx];
	const int32_t v1 = INV_SQRT_Q29[idx + 1];
	int32_t y_q29 = v0 + (((v1 - v0) * static_cast<int32_t>(frac8)) >> 8);

	int32_t yy, ay, r_q29;

#if defined(__arm__)
	if (!SF_IS_CONSTEVAL()) {
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo),
			"=&r"(hi)
			: "r"(y_q29),
			"r"(y_q29));
		yy = (hi << 3) | (static_cast<uint32_t>(lo) >> 29);
	}
	else
#endif
		yy = static_cast<int32_t>(
			(static_cast<int64_t>(y_q29) * y_q29) >> 29);

#if defined(__arm__)
	if (!SF_IS_CONSTEVAL()) {
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo),
			"=&r"(hi)
			: "r"(static_cast<int32_t>(a)),
			"r"(yy));
		ay = (hi << 3) | (static_cast<uint32_t>(lo) >> 29);
	}
	else
#endif
		ay = static_cast<int32_t>(
			(static_cast<int64_t>(a) * yy) >> 29);

	r_q29 = 0x30000000 - (ay >> 1);

#if defined(__arm__)
	if (!SF_IS_CONSTEVAL()) {
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo),
			"=&r"(hi)
			: "r"(y_q29),
			"r"(r_q29));
		y_q29 = (hi << 3) | (static_cast<uint32_t>(lo) >> 29);
	}
	else
#endif
		y_q29 = static_cast<int32_t>(
			(static_cast<int64_t>(y_q29) * r_q29) >> 29);

	if (E_raw & 1) {
#if defined(__arm__)
		if (!SF_IS_CONSTEVAL()) {
			int32_t lo, hi;
			__asm__(
				"smull %0, %1, %2, %3"
				: "=&r"(lo), "=&r"(hi)
				: "r"(y_q29), "r"(0x16A09E66));
			y_q29 = (hi << 3) | (static_cast<uint32_t>(lo) >> 29);
		}
		else
#endif
			y_q29 = static_cast<int32_t>((static_cast<int64_t>(y_q29) * 0x16A09E66LL) >> 29);
	}

	const int32_t carry = static_cast<uint32_t>(y_q29) >> 29;
	y_q29 <<= 1;
	int32_t result_e = -29 - (E_raw >> 1) - 1 + carry;


	uint32_t out_m = static_cast<uint32_t>(y_q29);
	if (UNLIKELY(out_m >= MANT_OVERFLOW)) out_m >>= 1;
	return from_raw_normalized(out_m, result_e);
}

constexpr SF_HOT SoftFloat SoftFloat::sqrt() const noexcept
{
	const uint32_t sb = bits;
	const uint32_t sbe = field_exp(sb);
	if (UNLIKELY(sbe == 0u || (sb >> 31))) return zero();

	const uint32_t s_mant = mant_from_bits(sb);
	const int32_t  s_exp = exp_from_biased(sbe);

	if (SF_IS_CONSTEVAL()) {
		int32_t m = static_cast<int32_t>(s_mant);
		int32_t e = s_exp;
		if (e & 1) { m <<= 1; e -= 1; }
		uint64_t scaled = static_cast<uint64_t>(m) << 30;
		uint64_t root = isqrt64(scaled);
		int32_t  rm = static_cast<int32_t>(root);
		int32_t  re = (e >> 1) - 15;
		return SoftFloat(rm, re);
	}

#if defined(__arm__)
	// Direct sqrt kernel for Cortex-M3.  This uses a small clean-room generated
	// Q7 sqrt table plus two signed divide corrections and a final remainder
	// correction.  It works directly in the packed 24-bit mantissa domain and
	// packs the final IEEE/Fused-normal result without going through the Q29
	// reciprocal-sqrt/Newton path below.
	{
		const uint32_t m24 = mant24_from_bits(sb);
		const bool odd = (sbe & 1u) != 0u;
		const uint32_t idx = (m24 >> 16) & 0x7Fu;
		uint32_t r = odd ? SQRT_ODD_Q7[idx] : SQRT_EVEN_Q7[idx];
		uint32_t p = r * r;
		int32_t c;

		if (odd) {
			c = static_cast<int32_t>((m24 - (p << 9)) << 13) / static_cast<int32_t>(r);
			r = (r << 7) + static_cast<uint32_t>(c >> 16);
			p = r * r;
			c = static_cast<int32_t>(((m24 << 5) - p) << 9) / static_cast<int32_t>(r);
			r = (r << 10) + static_cast<uint32_t>(c);
			p = r * r;
			uint32_t rem = (m24 << 25) - p;
			if (static_cast<int32_t>(rem) < 0) {
				--r;
				p = r * r;
				rem = (m24 << 25) - p;
				if (static_cast<int32_t>(rem) < 0) --r;
			}
		} else {
			c = static_cast<int32_t>((m24 - (p << 8)) << 14) / static_cast<int32_t>(r);
			r = (r << 7) + static_cast<uint32_t>(c >> 16);
			p = r * r;
			c = static_cast<int32_t>(((m24 << 6) - p) << 9) / static_cast<int32_t>(r);
			r = (r << 10) + static_cast<uint32_t>(c);
			p = r * r;
			uint32_t rem = (m24 << 26) - p;
			if (static_cast<int32_t>(rem) < 0) {
				--r;
				p = r * r;
				rem = (m24 << 26) - p;
				if (static_cast<int32_t>(rem) < 0) --r;
			}
		}

		const uint32_t be_m1 = (sbe >> 1) + (odd ? 0x3Fu : 0x3Eu);
		uint32_t out = r;
		__asm__(
			"lsrs %[out], %[out], #1\n\t"
			"adc  %[out], %[out], %[be_m1], lsl #23"
			: [out] "+r"(out)
			: [be_m1] "r"(be_m1)
			: "cc");
		return from_bits_unchecked(out);
	}
#endif

	const int32_t  E_raw = s_exp + 29;
	const uint32_t a = s_mant;

	const uint32_t offset = a - MANT_MIN;
	const uint32_t idx = offset >> 21;
	const uint32_t frac8 = (offset >> 13) & 0xFFu;

	const int32_t v0 = INV_SQRT_Q29[idx];
	const int32_t v1 = INV_SQRT_Q29[idx + 1];
	int32_t y_q29 = v0 + (((v1 - v0) * static_cast<int32_t>(frac8)) >> 8);

	int32_t g_q29;
#if defined(__arm__)
	{
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo), "=&r"(hi)
			: "r"(static_cast<int32_t>(a)), "r"(y_q29));
		g_q29 = (hi << 3) | (static_cast<uint32_t>(lo) >> 29);
	}
#else
	g_q29 = static_cast<int32_t>(
		(static_cast<uint64_t>(a) * static_cast<uint32_t>(y_q29)) >> 29);
#endif

	int32_t r_q30;
#if defined(__arm__)
	{
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo), "=&r"(hi)
			: "r"(y_q29), "r"(g_q29));
		r_q30 = static_cast<int32_t>(0x20000000u)
			- ((hi << 3) | (static_cast<uint32_t>(lo) >> 29));
	}
#else
	{
		const int64_t yg = static_cast<int64_t>(y_q29) * g_q29;
		r_q30 = static_cast<int32_t>(0x20000000u) - static_cast<int32_t>(yg >> 29);
	}
#endif

#if defined(__arm__)
	{
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo), "=&r"(hi)
			: "r"(g_q29), "r"(r_q30));
		g_q29 += (hi << 2) | (static_cast<uint32_t>(lo) >> 30);
	}
#else
	{
		const int64_t gr = static_cast<int64_t>(g_q29) * r_q30;
		g_q29 += static_cast<int32_t>(gr >> 30);
	}
#endif

	if (E_raw & 1) {
#if defined(__arm__)
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo), "=&r"(hi)
			: "r"(g_q29), "r"(0x2D413CCD));
		g_q29 = (hi << 3) | (static_cast<uint32_t>(lo) >> 29);
#else
		g_q29 = static_cast<int32_t>(
			(static_cast<int64_t>(g_q29) * 0x2D413CCDLL) >> 29);
#endif
	}

	const int32_t result_e = (E_raw >> 1) - 29;
	return from_raw_normalized(g_q29, result_e);
}

constexpr SF_HOT Angle atan2(SoftFloat y, SoftFloat x) noexcept
{


	// Decode each operand's packed fields exactly once.  Going through the
	// mantissa/exponent/negative macros re-extracts field_exp(bits) (plus a
	// be?…:0 branch) on every access; on Cortex-M3 that is several wasted
	// instructions because atan2 reads each field 2-3 times.
	const uint32_t xb = x.bits;
	const uint32_t yb = y.bits;
	const uint32_t xbe = SoftFloat::field_exp(xb);
	const uint32_t ybe = SoftFloat::field_exp(yb);

	if (UNLIKELY((xbe == 0u) & (ybe == 0u)))
		return SoftFloat::zero();

	const bool x_neg = (xb >> 31) != 0u;
	const bool y_neg = (yb >> 31) != 0u;

	if (UNLIKELY(ybe == 0u))
		return x_neg ? Angle::pi() : Angle::zero();
	if (UNLIKELY(xbe == 0u)) {
		Angle hpi = Angle::half_pi();
		return y_neg ? (Angle::zero() + DeltaAngle::from_raw(-static_cast<int32_t>(hpi.get_raw()))) : hpi;
	}

	uint32_t ax = SoftFloat::mant24_from_bits(xb);
	uint32_t ay = SoftFloat::mant24_from_bits(yb);
	int32_t ex = SoftFloat::exp_from_biased(xbe);
	int32_t ey = SoftFloat::exp_from_biased(ybe);

	bool swap = false;
	uint32_t num = ay, den = ax;
	int32_t num_e = ey, den_e = ex;

	if (ey > ex || (ey == ex && ay > ax)) {
		swap = true;
		num = ax; den = ay;
		num_e = ex; den_e = ey;
	}

	int32_t shift = 24 + num_e - den_e;
	uint32_t t_Q24;

	// Runtime path: hardware-udiv two-stage quotient (qfplib-m3 style).
	// Consteval path: plain integer division.
	if (SF_IS_CONSTEVAL()) {
		// Consteval fallback: integer division
		if (LIKELY(shift >= 0)) {
			uint64_t n = static_cast<uint64_t>(num) << shift;
			t_Q24 = static_cast<uint32_t>(n / den);
		}
		else {
			uint32_t sn = num >> (-shift);
			t_Q24 = sn / den;
		}
	}
	else {
		// Cortex-M3 hardware-udiv ratio: qm30 = floor(num * 2^30 / den), num,den
		// are 24-bit implicit-one mantissas (num <= den after the swap).
		// This is the same two-stage quotient as SoftFloat division, but kept in
		// the packed 24-bit domain to save two Q29 unpack shifts/masks.
		uint32_t qm30;
		{
			uint32_t x = num << 8;                      // num * 2^8
			uint32_t yh = (den + (1u << 7)) >> 8;        // 16-bit divisor approx
			uint32_t q1 = x / yh;
			uint32_t r1 = (x << 14) - q1 * (den << 6);
			int32_t  dq = static_cast<int32_t>(r1) / static_cast<int32_t>(yh);
			qm30 = (q1 << 14) + static_cast<uint32_t>(dq);
		}
		if (LIKELY(shift >= 0)) {
			// Since num<=den after the swap, shift is never above 24 here.
			t_Q24 = qm30 >> (30 - shift);
		}
		else {
			// shift == -1 can still contribute the top bit; shift <= -2 is below
			// the 24-bit table coordinate resolution.
			t_Q24 = (shift == -1) ? (qm30 >> 31) : 0u;
		}
	}

	uint32_t idx = t_Q24 >> 16;
	uint32_t frac = t_Q24 & 0xFFFFu;

	// Interpolate directly in Angle raw units instead of first producing Q29
	// radians and then multiplying by 2^31/pi.  This removes one 32x32->64
	// multiply from the hot path and also avoids a signed intermediate.
        uint32_t a0 = SoftFloat::ATAN_RAW_TAB[idx];
        uint32_t a1 = SoftFloat::ATAN_RAW_TAB[idx + 1];
	uint32_t angle_raw = a0 + static_cast<uint32_t>((static_cast<uint64_t>(a1 - a0) * frac) >> 16);

	if (swap)  angle_raw = 0x40000000u - angle_raw;
	if (x_neg) angle_raw = 0x80000000u - angle_raw;
	if (y_neg) angle_raw = 0u - angle_raw;

	return Angle::from_raw(angle_raw);
}

constexpr SF_HOT SoftFloat SoftFloat::exp() const noexcept
{

	// Decode the packed fields once instead of through the
	// mantissa/exponent/negative macros (each re-extracts field_exp + branch).
	const uint32_t sb  = bits;
	const uint32_t sbe = field_exp(sb);
	if (UNLIKELY(sbe == 0u)) return one();

#if defined(__arm__)
	if (!SF_IS_CONSTEVAL()) {
		constexpr int32_t LN2_Q29 = 0x162E42FF;
		const uint32_t m24 = mant24_from_bits(sb);
		const int32_t eplus1 = static_cast<int32_t>(sbe) - 126;

		auto exp_core = [](int32_t r_q29, int32_t k) constexpr noexcept -> SoftFloat {
			uint32_t idx = static_cast<uint32_t>((r_q29 + (1 << 21)) >> 22); // 0..89 after range reduction

			const int32_t y = EXP_E_Q8[idx];
			const int32_t r = r_q29 - EXP_LN_E_Q29[idx];
			const int32_t rq23 = r >> 6;
			const int32_t r2 = rq23 * rq23;
			const int32_t corr = r + static_cast<int32_t>(static_cast<uint32_t>(r2) >> 18);

			const int32_t prod = corr * y;
			int32_t t = prod >> 14;
			t += static_cast<int32_t>((static_cast<uint32_t>(prod) >> 13) & 1u);

			uint32_t mant = static_cast<uint32_t>(t + (y << 15)); // Q23, usually [1,2)
			if (UNLIKELY(mant < 0x00800000u)) { mant <<= 1; --k; }

			uint32_t out;
			int32_t be;
			if (UNLIKELY(mant & 0x01000000u)) {
				uint32_t outmant = (mant >> 1) + (mant & 1u);
				be = k + 128;
				if (UNLIKELY(be <= 0)) return zero();
				if (UNLIKELY(be > 255)) return from_bits_unchecked(0x7FFFFFFFu);
				out = outmant + (static_cast<uint32_t>(be - 1) << 23);
				if (UNLIKELY(out & 0x80000000u)) out = 0x7FFFFFFFu;
			} else {
				be = k + 127;
				if (UNLIKELY(be <= 0)) return zero();
				if (UNLIKELY(be > 255)) return from_bits_unchecked(0x7FFFFFFFu);
				out = mant + (static_cast<uint32_t>(be - 1) << 23);
			}
			return from_bits_unchecked(out);
		};

		int32_t k;
		int32_t r_q29;
		if ((sb >> 31) == 0u) {
			if (UNLIKELY(eplus1 >= 7 && sb > 0x42B17217u))
				return from_bits_unchecked(0x7FFFFFFFu);
			if (eplus1 < 0) {
				const int sh = eplus1 + 5;
				r_q29 = (sh >= 0) ? static_cast<int32_t>(m24 << sh)
					                 : static_cast<int32_t>(m24 >> -sh);
				k = 0;
			} else {
				const int32_t x_q24 = static_cast<int32_t>(m24 << eplus1);
				const int32_t q9 = x_q24 >> 15;
				k = (q9 * 0xB8AA) >> 24; // floor(x/ln2), qfplib-compatible underestimate
				r_q29 = static_cast<int32_t>((static_cast<uint32_t>(x_q24) << 5)
					- static_cast<uint32_t>(k * LN2_Q29));
			}
		} else {
			if (UNLIKELY(eplus1 >= 7 && sb > 0xC2AEAC50u)) return zero();
			if (eplus1 < 0) {
				const int sh = eplus1 + 5;
				r_q29 = (sh >= 0) ? -static_cast<int32_t>(m24 << sh)
					                 : -static_cast<int32_t>(m24 >> -sh);
				r_q29 += LN2_Q29;
				k = -1;
			} else {
				const int32_t x_q24 = -static_cast<int32_t>(m24 << eplus1);
				const int32_t q9 = x_q24 >> 15;
				k = (q9 * 0x5C55) >> 23;
				r_q29 = static_cast<int32_t>((static_cast<uint32_t>(x_q24) << 5)
					- static_cast<uint32_t>(k * LN2_Q29));
			}
		}
		return exp_core(r_q29, k);
	}
#endif

	const uint32_t s_mant = mant_from_bits(sb);
	const int32_t  s_exp  = exp_from_biased(sbe);
	const uint16_t s_neg  = sign_from_bits(sb);

	constexpr int32_t INV_LN2_M = 0x2E2A8ECB;

	int64_t kprod;
#if defined(__arm__)
	if (!SF_IS_CONSTEVAL()) {
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo), "=&r"(hi)
			: "r"(s_neg ? -static_cast<int32_t>(s_mant) : static_cast<int32_t>(s_mant)), "r"(INV_LN2_M));
		kprod = (static_cast<int64_t>(hi) << 32) | static_cast<uint32_t>(lo);
	}
	else
#endif
	{
		int32_t signed_m = s_neg ? -static_cast<int32_t>(s_mant) : static_cast<int32_t>(s_mant);
		kprod = static_cast<int64_t>(signed_m) * static_cast<int64_t>(INV_LN2_M);
	}

	const int32_t k_rshift = 29 - s_exp;
	if (UNLIKELY(k_rshift <= 0))
		return s_neg ? zero() : from_raw_normalized(MANT_MIN, EXP_MAX);

	int32_t  k;
	uint32_t u_8_21;

	if (LIKELY(k_rshift <= 63)) {
		k = static_cast<int32_t>(kprod >> k_rshift);

		const uint64_t mask = (uint64_t(1) << k_rshift) - 1u;
		const uint64_t frac_bits = static_cast<uint64_t>(kprod) & mask;

		// k_rshift = 29 - exponent. For normalised inputs exponent is in
		// k_rshift >= 29 for all normal inputs (exponent <= 0).
		// The left-shift branch is unreachable in practice; the unconditional
		// right-shift saves a compare + branch on Cortex-M3.
		u_8_21 = static_cast<uint32_t>(frac_bits >> (k_rshift - 29));
	}
	else {
		k = (kprod < 0) ? -1 : 0;
		const uint32_t rsh = static_cast<uint32_t>(k_rshift - 29);
		u_8_21 = (rsh < 60)
			? static_cast<uint32_t>(static_cast<uint64_t>(kprod) >> rsh)
			: 0u;
	}

	const uint32_t idx = u_8_21 >> 21;
	const int32_t  frac = static_cast<int32_t>(u_8_21 & 0x1FFFFFu);

        const int32_t m0 = SoftFloat::EXP_MANT[idx];
        const int32_t m1 = LIKELY(idx < 255) ? SoftFloat::EXP_MANT[idx + 1] : int32_t(0x40000000);
	const int32_t delta = m1 - m0;

	int32_t result_q29;
#if defined(__arm__)
	if (!SF_IS_CONSTEVAL()) {
		int32_t lo, hi;
		__asm__(
			"smull %0, %1, %2, %3"
			: "=&r"(lo), "=&r"(hi)
			: "r"(delta), "r"(frac));
		result_q29 = m0 + ((hi << 11) | (static_cast<uint32_t>(lo) >> 21));
	}
	else
#endif
	{
		result_q29 = m0 + static_cast<int32_t>((static_cast<int64_t>(delta) * frac) >> 21);
	}

	const int32_t final_exp = k - 29;
	if (UNLIKELY(final_exp > EXP_MAX))  return from_raw_normalized(MANT_MIN, EXP_MAX);
	if (UNLIKELY(final_exp < EXP_MIN)) return zero();
	return from_raw_normalized(result_q29, final_exp);
}

constexpr SF_HOT SoftFloat SoftFloat::log2() const noexcept
{

	if (UNLIKELY(mantissa == 0 || negative)) return zero();

	int32_t  E = static_cast<int32_t>(exponent) + 29;
	uint32_t m_abs = mantissa;

	uint32_t low = m_abs - MANT_MIN;
	uint32_t t_int = low >> 21;
	uint32_t frac = (low >> 13) & 0xFFu;

        int32_t v0 = SoftFloat::LOG2_Q30[t_int];
        int32_t v1 = SoftFloat::LOG2_Q30[t_int + 1];

	int32_t delta = v1 - v0;
	// Use int64_t for the interpolation multiply to avoid any potential overflow
	// (delta can be up to ~6M, frac up to 255; product up to ~1.5e9 which is
	// tight for int32_t). The int64_t path is free on x86; on ARM the compiler
	// uses SMULL and takes the high word which is a single instruction.
	int32_t log2_frac_q30 = v0 + static_cast<int32_t>(
		(static_cast<int64_t>(delta) * static_cast<int64_t>(frac)) >> 8);

	SoftFloat fractional_part(log2_frac_q30, -30);
	SoftFloat integer_part(E);
	return fractional_part + integer_part;
}

constexpr SoftFloat SoftFloat::log() const noexcept {
#if defined(__arm__)
	uint32_t sb = bits;
	uint32_t sbe = field_exp(sb);
	if (!SF_IS_CONSTEVAL()) {
		// Preserve existing finite-only semantics: log(0) and log(negative) return 0.
		if (UNLIKELY((sbe == 0u) | ((sb >> 31) != 0u))) return zero();
		if (UNLIKELY(sb == 0x3F800000u)) return zero();

		auto pack_mag_q = [](uint32_t mag, int frac_bits, uint16_t neg) constexpr noexcept -> SoftFloat {
			if (UNLIKELY(mag == 0u)) return zero();
			int top = 31 - clz(mag);
			uint32_t m;
			int32_t e;
			if (top <= MANT_BITS) {
				int sh = MANT_BITS - top;
				m = mag << sh;
				e = -frac_bits - sh;
			} else {
				int sh = top - MANT_BITS;
				m = (mag + (1u << (sh - 1))) >> sh;
				e = -frac_bits + sh;
				if (UNLIKELY(m >= MANT_OVERFLOW)) { m >>= 1; ++e; }
			}
			return from_raw_normalized(m, e, neg);
		};

		auto pack_q25 = [&](uint32_t mag, uint16_t neg) constexpr noexcept -> SoftFloat {
			if (UNLIKELY(mag == 0u)) return zero();
			uint32_t out, sh;
			if (!SF_IS_CONSTEVAL()) {
				__asm__(
					"clz  %[sh], %[mag]\n\t"
					"rsb  %[sh], %[sh], #8\n\t"
					"lsrs %[out], %[mag], %[sh]\n\t"
					"add  %[sh], %[sh], %[bias]\n\t"
					"adc  %[out], %[out], %[sh], lsl #23"
					: [out] "=&r"(out), [sh] "=&r"(sh)
					: [mag] "r"(mag), [bias] "r"(neg ? 0x17Cu : 0x7Cu)
					: "cc");
				return from_bits_unchecked(out);
			}
			return pack_mag_q(mag, 25, neg);
		};

		auto ln_mant_q31 = [](uint32_t x_q24) constexpr noexcept -> int32_t {
			// x_q24 is the mantissa interpreted as Q24 in [.5,1).  Approximate
			// ln(x) by choosing y ~= 1/x in Q7, then ln(x)=ln(x*y)-ln(y).
			uint32_t y = 0x807F0000u / x_q24; // 128..256 on Cortex-M3 UDIV
			int32_t a = static_cast<int32_t>(x_q24 * y - 0x80000000u); // xy-1, Q31
			int32_t r = static_cast<int32_t>(LN_NEG_Q31[y - 128u]) + a;
			int32_t aq23 = a >> 8;
			int32_t a2 = aq23 * aq23;
			uint32_t a2q30 = static_cast<uint32_t>(a2) >> 16;
			r -= static_cast<int32_t>(a2q30);
			int32_t a3 = aq23 * static_cast<int32_t>(a2q30);
			a3 += a3 >> 2; // ~4/3 a^3
			r += a3 >> 24;
			return r;
		};

		const uint32_t m24 = mant24_from_bits(sb);
		const int32_t eplus1 = static_cast<int32_t>(sbe) - 126;
		const int32_t lm = ln_mant_q31(m24);

		constexpr uint32_t LN2_Q25_POS = 0x0162E430u;
		constexpr uint32_t LN2_Q25_NEG = 0xFE9D1BD0u; // -ln2 in Q25, modulo 2^32
		constexpr int32_t  LN2_Q31     = 0x58B90BFC;

		if (eplus1 >= 2) {
			uint32_t mag = static_cast<uint32_t>(eplus1) * LN2_Q25_POS
			             + static_cast<uint32_t>(lm >> 6);
			return pack_q25(mag, 0);
		}
		if (eplus1 < 0) {
			uint32_t mag = static_cast<uint32_t>(eplus1) * LN2_Q25_NEG
			             - static_cast<uint32_t>(lm >> 6);
			return pack_q25(mag, 1);
		}

		int32_t total = lm + eplus1 * LN2_Q31;
		uint16_t neg = static_cast<uint16_t>(total < 0);
		uint32_t mag = neg ? static_cast<uint32_t>(-total) : static_cast<uint32_t>(total);
		return pack_mag_q(mag, 31, neg);
	}
#endif

	constexpr SoftFloat LN2 = from_raw_normalized(0x2C5C85FE, -30);
	return log2() * LN2;
}

constexpr SoftFloat SoftFloat::log10() const noexcept {
	constexpr SoftFloat LOG10_2 = SoftFloat::from_raw_normalized(0x268826A1, -31);
	return log2() * LOG10_2;
}

constexpr SoftFloat SoftFloat::pow(SoftFloat y) const noexcept {
	if (mantissa == 0) return y.mantissa == 0 ? one() : zero();
	if (y.mantissa == 0) return one();

	if (y == one())       return *this;
	if (y == two())       { SoftFloat t = *this; return SoftFloat::mul_plain(t, t); }
	if (y == three())     { SoftFloat t = *this; return SoftFloat::mul_plain(t, SoftFloat::mul_plain(t, t)); }
	if (y == four())      { SoftFloat t = *this; t = SoftFloat::mul_plain(t, t); return SoftFloat::mul_plain(t, t); }
	if (y == neg_one())   return reciprocal();
	if (y == half())      return sqrt();
	if (y == -half())     return inv_sqrt();

	if (y.mantissa == 0x30000000 && y.exponent == -29) {
		SoftFloat t = sqrt();
		return SoftFloat::mul_plain(*this, t);
	}
	if (y.mantissa == 0x20000000 && y.exponent == -31)
		return sqrt().sqrt();

	int32_t n = 0;
	bool is_int = false;
	if (y.exponent < 0) {
		int32_t shift = -y.exponent;
		if (shift <= 30) {
			uint32_t a = abs32(y.mantissa);
			uint32_t mask = (1u << shift) - 1u;
			if ((a & mask) == 0u) {
				is_int = true;
				n = static_cast<int32_t>(a >> shift);
				if (y.negative) n = -n;
			}
		}
	}

	if (is_int) {
		if (n == 0) return one();
		if (n == 1) return *this;
		if (n == -1) return reciprocal();

		bool neg = n < 0;
		uint32_t un = neg
		    ? static_cast<uint32_t>(-(static_cast<int64_t>(n)))
		    : static_cast<uint32_t>(n);

		if (exponent > 0 && un > 127u / static_cast<uint32_t>(exponent))
			return from_raw_normalized(MANT_MIN, EXP_MAX, negative);
		if (exponent < 0 && un > 128u / static_cast<uint32_t>(-static_cast<int32_t>(exponent)))
			return zero();

		SoftFloat result = one();
		SoftFloat base   = *this;
		for (; un; un >>= 1) {
			if (un & 1u) result = SoftFloat::mul_plain(result, base);
			if (un == 1u) break;
			base = SoftFloat::mul_plain(base, base);
		}
		return neg ? result.reciprocal() : result;
	}

	if (is_negative()) return zero();
	return (y * log()).exp();
}

constexpr SF_HOT SoftFloat hypot(SoftFloat x, SoftFloat y) noexcept {
	x = x.abs();
	y = y.abs();

	if (x.mantissa == 0) return y;
	if (y.mantissa == 0) return x;
	if (x < y) { SoftFloat t = x; x = y; y = t; }

	int32_t ex = static_cast<int32_t>(x.exponent);
	int32_t d  = ex - static_cast<int32_t>(y.exponent);

	if (d >= 15) return x;

	uint32_t mx = x.mantissa;
	uint32_t my = y.mantissa;

	uint64_t mx2 = static_cast<uint64_t>(mx) * mx;
	uint64_t my2 = static_cast<uint64_t>(my) * my;
	uint64_t S   = mx2 + (my2 >> (2 * d));

	uint32_t s_hi = static_cast<uint32_t>(S >> 29);
	int32_t  s_e  = 29;

	if (s_hi >= 0x80000000u) {
		s_hi >>= 2;
		s_e  += 2;
	}
	else if (s_hi >= SoftFloat::MANT_OVERFLOW) {
		s_hi >>= 1;
		s_e  += 1;
	}

	SoftFloat s_sf = SoftFloat::from_raw_normalized(static_cast<int32_t>(s_hi), s_e);

	SoftFloat r = s_sf.sqrt();
	return SoftFloat::from_raw_normalized(r.mantissa, r.exponent + ex);
}

constexpr SF_HOT SoftFloat SoftFloat::trunc() const noexcept {
	uint32_t be = field_exp(bits);
	if (UNLIKELY(be == 0u)) return *this;
	int32_t e = exp_from_biased(be);
	if (e >= 0) return *this;

	int32_t rs = -e;
	if (rs >= 30) return zero();

	uint32_t im = mant_from_bits(bits) >> rs;
	if (bits >> 31) im = 0u - im;
	return SoftFloat(static_cast<int32_t>(im));
}

constexpr SF_HOT SoftFloat SoftFloat::floor() const noexcept {
	uint32_t be = field_exp(bits);
	if (UNLIKELY(be == 0u)) return *this;
	int32_t e = exp_from_biased(be);
	if (e >= 0) return *this;

	uint32_t neg = bits >> 31;
	int32_t rs = -e;
	if (rs >= 30) {
		return neg ? SoftFloat::neg_one() : SoftFloat::zero();
	}

	uint32_t a = mant_from_bits(bits);

	uint32_t frac_mask = (1u << rs) - 1u;
	uint32_t int_part = a >> rs;
	if (neg) {
		int_part += ((a & frac_mask) != 0u);
		int_part = 0u - int_part;
		}
	return SoftFloat(static_cast<int32_t>(int_part));
}

constexpr SoftFloat SoftFloat::ceil() const noexcept {
	uint32_t be = field_exp(bits);
	if (UNLIKELY(be == 0u)) return *this;
	int32_t e = exp_from_biased(be);
	if (e >= 0) return *this;

	uint32_t neg = bits >> 31;
	int32_t rs = -e;
	if (rs >= 30) {
		return neg ? zero() : one();
	}

	uint32_t a = mant_from_bits(bits);
	uint32_t frac_mask = (1u << rs) - 1u;
	uint32_t int_part = a & ~frac_mask;

	if (neg) {
		return from_raw_normalized(int_part, e, 1);
	}
	else {
		uint32_t new_m = int_part + (((a & frac_mask) != 0u) ? (1u << rs) : 0u);
		if (UNLIKELY(new_m >= MANT_OVERFLOW))
			return from_raw_normalized(MANT_MIN, e + 1, 0);
		return from_raw_normalized(new_m, e, 0);
		}
}

constexpr SF_HOT SoftFloat SoftFloat::round() const noexcept {
	uint32_t be = field_exp(bits);
	if (UNLIKELY(be == 0u)) return *this;
	int32_t e = exp_from_biased(be);
	if (e >= 0) return *this;

	int32_t rs = -e;

	if (rs >= 31) return SoftFloat::zero();

	uint32_t sum = mant_from_bits(bits) + (1u << (rs - 1));
	uint32_t im = sum >> rs;
	if (bits >> 31) im = 0u - im;
	return SoftFloat(static_cast<int32_t>(im));
}

constexpr SoftFloat SoftFloat::fract() const noexcept {
	return *this - trunc();
}

constexpr IntFractPair SoftFloat::modf() const noexcept {
	SoftFloat intpart = trunc();
	return { intpart, *this - intpart };
}

constexpr SoftFloat SoftFloat::copysign(SoftFloat sign) const noexcept {
	SoftFloat r;
	uint32_t mag = bits & 0x7FFFFFFFu;
	r.bits = (mag | (sign.bits & 0x80000000u)) & (0u - static_cast<uint32_t>(mag != 0u));
	return r;
}

constexpr SoftFloat SoftFloat::fmod(SoftFloat y) const noexcept {
	if (UNLIKELY(y.mantissa == 0)) return *this;
	if (UNLIKELY(mantissa == 0))   return *this;

	uint16_t sx_neg = negative;
	uint32_t ax = mantissa;
	uint32_t ay = y.mantissa;
	int32_t  d = static_cast<int32_t>(exponent) - static_cast<int32_t>(y.exponent);

	if (d < 0) return *this;
	if (d == 0 && ax < ay) return *this;

	// Fast path for d == 0
	if (d == 0) {
		uint32_t r = ax - ay;
		if (r == 0) return zero();
		int32_t rm = static_cast<int32_t>(r);
		int32_t re = y.exponent;
		normalise_fast(rm, re);
		return from_raw_normalized(static_cast<uint32_t>(rm), static_cast<int32_t>(re), sx_neg);
	}

	// General case: fmod(x,y) = x - trunc(x/y)*y
	// This is O(1) in softfloat ops (div + trunc + mul + sub) vs the previous
	// O(log d) integer-division loop.  Correct to full 29-bit mantissa precision.
	{
		SoftFloat self = from_raw_normalized(ax, exponent, sx_neg);
		SoftFloat yabs = from_raw_normalized(ay, y.exponent); // y magnitude, positive
		SoftFloat q = (self / yabs).trunc();
		SoftFloat r = self - q * yabs;
		// Enforce sign of dividend (C standard fmod semantics)
		if (r.mantissa != 0) r = from_raw_normalized(r.mantissa, r.exponent, sx_neg);
		return r;
	}
}

constexpr SoftFloat SoftFloat::fma(SoftFloat b, SoftFloat c) const noexcept {
	return fused_mul_add(*this, b, c);
}

constexpr SF_HOT SoftFloat lerp(SoftFloat a, SoftFloat b, SoftFloat t) noexcept {
	return a + t * (b - a);
}

// =========================================================================
// FixedQ30 and Angle implementation (constexpr friendly)
// =========================================================================

constexpr SoftFloat FixedQ30::to_softfloat() const noexcept {
    if (raw_ == 0) return SoftFloat::zero();

    uint32_t uv = static_cast<uint32_t>(raw_);
    uint16_t neg = static_cast<uint16_t>(uv >> 31);
    uint32_t mask = 0u - static_cast<uint32_t>(neg);
    uint32_t a = (uv ^ mask) - mask;

    if (a >= SoftFloat::MANT_OVERFLOW) {
        a >>= 1;
        return SoftFloat::from_raw_normalized(a, -29, neg);
    }
    if (a >= SoftFloat::MANT_MIN)
        return SoftFloat::from_raw_normalized(a, -30, neg);

    int shift = SoftFloat::clz(a) - 2;
    a <<= shift;
    return SoftFloat::from_raw_normalized(a, -30 - shift, neg);
}

// Out-of-line definitions for FixedQ30 (to avoid incomplete SoftFloat in early class def)
constexpr float FixedQ30::to_float() const noexcept { return to_softfloat().to_float(); }
//constexpr FixedQ30::operator float() const noexcept { return to_float(); }
constexpr FixedQ30::operator SoftFloat() const noexcept { return to_softfloat(); }

// Specialized multiplication: SoftFloat × FixedQ30
constexpr SoftFloat operator*(SoftFloat a, FixedQ30 b) noexcept {
    const uint32_t ab = a.bits;
    const uint32_t abe = SoftFloat::field_exp(ab);
    if (UNLIKELY(abe == 0u || b.get_raw() == 0)) return SoftFloat::zero();

    uint32_t bv = static_cast<uint32_t>(b.get_raw());
    uint16_t bneg = static_cast<uint16_t>(bv >> 31);
    uint32_t bmask = 0u - static_cast<uint32_t>(bneg);
    uint32_t bm = (bv ^ bmask) - bmask;
    uint16_t neg = SoftFloat::sign_from_bits(ab) ^ bneg;

    uint64_t prod = static_cast<uint64_t>(SoftFloat::mant_from_bits(ab)) * bm;
    uint32_t rm = static_cast<uint32_t>(prod >> 30);

	// Q30 means b = raw / 2^30.  prod >> 30 already applied the Q30 scale.
	// Do NOT subtract one more exponent bit.
    int32_t re = SoftFloat::exp_from_biased(abe);

    if (rm >= SoftFloat::MANT_OVERFLOW) {
        rm >>= 1;
        re += 1;
    }
    if (rm < SoftFloat::MANT_MIN && rm != 0) {
        int lz = SoftFloat::clz(rm);
        int sh = lz - 2;
        rm <<= sh;
        re -= sh;
    }

    return SoftFloat::from_raw_normalized(rm, re, neg);
}

constexpr SoftFloat operator*(FixedQ30 a, SoftFloat b) noexcept {
    return b * a;
}

// =========================================================================
// Angle implementation
// =========================================================================

constexpr inline Angle::Angle(SoftFloat r) noexcept {
    // Convert radians directly to the modulo-2^32 angle representation:
    //   raw = radians * 2^31 / pi  (mod 2^32)
    // This avoids fmod(two_pi), a SoftFloat division, a multiply, and the
    // split int32 conversion in the old bridge.  The integer scale is rounded;
    // the remaining error is far below the 256-entry sin/cos table error.
    constexpr uint32_t RAD_TO_RAW = 0x28BE60DCu; // round(2^31 / pi)

    const uint32_t rb = r.bits;
    const uint32_t be = SoftFloat::field_exp(rb);
    if (UNLIKELY(be == 0u)) { raw_ = 0u; return; }

    const uint32_t m = SoftFloat::mant_from_bits(rb);
    const int32_t e = SoftFloat::exp_from_biased(be);
    const uint64_t prod = static_cast<uint64_t>(m) * static_cast<uint64_t>(RAD_TO_RAW);

    uint32_t u;
    if (e >= 0) {
        u = (e >= 32) ? 0u : (static_cast<uint32_t>(prod) << e);
    } else {
        const uint32_t sh = static_cast<uint32_t>(-e);
        u = (sh >= 64u) ? 0u : static_cast<uint32_t>(prod >> sh);
    }

    raw_ = SoftFloat::sign_from_bits(rb) ? (0u - u) : u;
}

constexpr inline Angle::operator SoftFloat() const noexcept {
    // raw * pi / 2^31.  pi is stored as 843314857 * 2^-28, so the product is
    // raw * 843314857 * 2^-59.  Build the SoftFloat directly from that 64-bit
    // scaled integer instead of doing split-uint32 reconstruction + mul/div.
    return SoftFloat::from_u64_scaled(static_cast<uint64_t>(raw_) * 843314857ull, -59, 0);
    }

// Out-of-line (to avoid incomplete SoftFloat at Angle class definition point)
constexpr float Angle::to_float() const noexcept { 
	return static_cast<float>(static_cast<SoftFloat>(*this)); 
}

constexpr Angle::operator float() const noexcept { 
	return to_float(); 
}

constexpr float DeltaAngle::to_float() const noexcept {
	return static_cast<float>(static_cast<SoftFloat>(*this));
}

inline constexpr int32_t SF_SIN_Q30_FAST[257] = {
        0,          26350943,   52686014,   78989349,
        105245103,  131437462,  157550647,  183568930,
        209476638,  235258165,  260897982,  286380643,
        311690799,  336813204,  361732726,  386434353,
        410903207,  435124548,  459083786,  482766489,
        506158392,  529245404,  552013618,  574449320,
        596538995,  618269338,  639627258,  660599890,
        681174602,  701339000,  721080937,  740388522,
        759250125,  777654384,  795590213,  813046808,
        830013654,  846480531,  862437520,  877875009,
        892783698,  907154608,  920979082,  934248793,
        946955747,  959092290,  970651112,  981625251,
        992008094, 1001793390, 1010975242, 1019548121,
        1027506862, 1034846671, 1041563127, 1047652185,
        1053110176, 1057933813, 1062120190, 1065666786,
        1068571464, 1070832474, 1072448455, 1073418433,
        1073741824, 1073418433, 1072448455, 1070832474,
        1068571464, 1065666786, 1062120190, 1057933813,
        1053110176, 1047652185, 1041563127, 1034846671,
        1027506862, 1019548121, 1010975242, 1001793390,
        992008094,  981625251,  970651112,  959092290,
        946955747,  934248793,  920979082,  907154608,
        892783698,  877875009,  862437520,  846480531,
        830013654,  813046808,  795590213,  777654384,
        759250125,  740388522,  721080937,  701339000,
        681174602,  660599890,  639627258,  618269338,
        596538995,  574449320,  552013618,  529245404,
        506158392,  482766489,  459083786,  435124548,
        410903207,  386434353,  361732726,  336813204,
        311690799,  286380643,  260897982,  235258165,
        209476638,  183568930,  157550647,  131437462,
        105245103,   78989349,   52686014,   26350943,
        0,          -26350943,  -52686014,  -78989349,
        -105245103, -131437462, -157550647, -183568930,
        -209476638, -235258165, -260897982, -286380643,
        -311690799, -336813204, -361732726, -386434353,
        -410903207, -435124548, -459083786, -482766489,
        -506158392, -529245404, -552013618, -574449320,
        -596538995, -618269338, -639627258, -660599890,
        -681174602, -701339000, -721080937, -740388522,
        -759250125, -777654384, -795590213, -813046808,
        -830013654, -846480531, -862437520, -877875009,
        -892783698, -907154608, -920979082, -934248793,
        -946955747, -959092290, -970651112, -981625251,
        -992008094,-1001793390,-1010975242,-1019548121,
        -1027506862,-1034846671,-1041563127,-1047652185,
        -1053110176,-1057933813,-1062120190,-1065666786,
        -1068571464,-1070832474,-1072448455,-1073418433,
        -1073741824,-1073418433,-1072448455,-1070832474,
        -1068571464,-1065666786,-1062120190,-1057933813,
        -1053110176,-1047652185,-1041563127,-1034846671,
        -1027506862,-1019548121,-1010975242,-1001793390,
        -992008094, -981625251, -970651112, -959092290,
        -946955747, -934248793, -920979082, -907154608,
        -892783698, -877875009, -862437520, -846480531,
        -830013654, -813046808, -795590213, -777654384,
        -759250125, -740388522, -721080937, -701339000,
        -681174602, -660599890, -639627258, -618269338,
        -596538995, -574449320, -552013618, -529245404,
        -506158392, -482766489, -459083786, -435124548,
        -410903207, -386434353, -361732726, -336813204,
        -311690799, -286380643, -260897982, -235258165,
        -209476638, -183568930, -157550647, -131437462,
        -105245103,  -78989349,  -52686014,  -26350943,
        0
    };

[[nodiscard]] constexpr SF_INLINE int32_t sf_sin_q30_from_phase(uint32_t phase) noexcept {
    uint32_t idx   = (phase >> 24) & 0xFFu;
    uint32_t frac  = phase & 0x00FFFFFFu;
    uint32_t idx1  = idx + 1u;

    int32_t s0 = SF_SIN_Q30_FAST[idx];
    int32_t s1 = SF_SIN_Q30_FAST[idx1];
    int32_t ds = s1 - s0;

#if defined(__arm__)
    if (!SF_IS_CONSTEVAL()) {
        int32_t lo, hi;
        __asm__(
            "smull %0, %1, %2, %3"
            : "=&r"(lo), "=&r"(hi)
            : "r"(ds), "r"(frac));
        return s0 + ((static_cast<uint32_t>(lo) >> 24) | (hi << 8));
    } else
#endif
    {
    return s0 + static_cast<int32_t>((static_cast<int64_t>(ds) * frac) >> 24);
}
}

constexpr inline SinCosPair Angle::sincos() const noexcept {
    uint32_t phase = raw_;
    uint32_t idx   = (phase >> 24) & 0xFFu;
    uint32_t frac  = phase & 0x00FFFFFFu;

    uint32_t idx1   = idx + 1u;
    uint32_t c_idx  = (idx + 64u) & 0xFFu;
    uint32_t c_idx1 = c_idx + 1u;

    int32_t s0 = SF_SIN_Q30_FAST[idx];
    int32_t s1 = SF_SIN_Q30_FAST[idx1];
    int32_t c0 = SF_SIN_Q30_FAST[c_idx];
    int32_t c1 = SF_SIN_Q30_FAST[c_idx1];

    int32_t ds = s1 - s0;
    int32_t dc = c1 - c0;

#if defined(__arm__)
    if (!SF_IS_CONSTEVAL()) {
        int32_t lo, hi;
        __asm__(
            "smull %0, %1, %2, %3"
            : "=&r"(lo), "=&r"(hi)
            : "r"(ds), "r"(frac));
        int32_t sin_q30 = s0 + ((static_cast<uint32_t>(lo) >> 24) | (hi << 8));

        __asm__(
            "smull %0, %1, %2, %3"
            : "=&r"(lo), "=&r"(hi)
            : "r"(dc), "r"(frac));
        int32_t cos_q30 = c0 + ((static_cast<uint32_t>(lo) >> 24) | (hi << 8));

        return { FixedQ30(sin_q30), FixedQ30(cos_q30) };
    } else
#endif
    {
    int32_t sin_q30 = s0 + static_cast<int32_t>((static_cast<int64_t>(ds) * frac) >> 24);
    int32_t cos_q30 = c0 + static_cast<int32_t>((static_cast<int64_t>(dc) * frac) >> 24);

    return { FixedQ30(sin_q30), FixedQ30(cos_q30) };
}
}

constexpr inline FixedQ30 Angle::sin() const noexcept {
    return FixedQ30(sf_sin_q30_from_phase(raw_));
}

constexpr inline FixedQ30 Angle::cos() const noexcept {
    return FixedQ30(sf_sin_q30_from_phase(raw_ + RAW_HALF_PI));
}


#undef mantissa
#undef exponent
#undef negative

// =========================================================================
// Compile-time evaluation tests
// =========================================================================
[[nodiscard]] consteval bool ct_approx(float a, float b, int max_ulp = 16) {
	if (a == b) return true;
	uint32_t ua = SoftFloat::bitcast<uint32_t>(a);
	uint32_t ub = SoftFloat::bitcast<uint32_t>(b);
	if ((ua ^ ub) & 0x80000000u) return false;
	int32_t diff = static_cast<int32_t>(ua - ub);
	return (diff < 0 ? -diff : diff) <= max_ulp;
}

[[nodiscard]] consteval bool ct_is_normalized(int32_t mantissa, int32_t exponent) {
	(void)exponent;
	if (mantissa == 0) return true;
	uint32_t uv = static_cast<uint32_t>(mantissa);
	uint32_t sign = uv >> 31;
	uint32_t mask = 0u - sign;
	uint32_t a = (uv ^ mask) - mask;
	int lz = __builtin_clz(a);
	return lz == 2 && (a & 0xC0000000u) == 0;
}

[[nodiscard]] consteval bool ct_float_eq(SoftFloat sf, float expected) {
	return SoftFloat::bitcast<uint32_t>(sf.to_float()) == SoftFloat::bitcast<uint32_t>(expected);
}

static_assert(SoftFloat::zero().is_zero(), "zero.is_zero");
static_assert(!SoftFloat::one().is_zero(), "one.not_zero");
static_assert(SoftFloat::one().to_float() == 1.0f, "one==1");
static_assert(SoftFloat::neg_one().to_float() == -1.0f, "neg_one==-1");
static_assert(SoftFloat::half().to_float() == 0.5f, "half==0.5");
static_assert(SoftFloat::two().to_float() == 2.0f, "two==2");
static_assert((SoftFloat::one() + SoftFloat::one()).to_float() == 2.0f, "1+1==2");
static_assert((SoftFloat::two() - SoftFloat::one()).to_float() == 1.0f, "2-1==1");
static_assert((SoftFloat::two()* SoftFloat::two()).to_float() == 4.0f, "2*2==4");
static_assert((SoftFloat::two() / SoftFloat::two()).to_float() == 1.0f, "2/2==1");
static_assert(SoftFloat::zero().sin().is_zero(), "sin(0)==0");
static_assert(SoftFloat::one() < SoftFloat::two(), "1<2");
static_assert(SoftFloat::neg_one() < SoftFloat::zero(), "-1<0");
static_assert(SoftFloat::one() == SoftFloat::one(), "1==1");

// Public-interface constant checks (internal representation is intentionally private)
static_assert(SoftFloat::zero().is_zero());
static_assert(ct_float_eq(SoftFloat::zero(), 0.0f));
static_assert(ct_float_eq(SoftFloat::one(), 1.0f));
static_assert(!SoftFloat::one().is_negative());
static_assert(ct_float_eq(SoftFloat::neg_one(), -1.0f));
static_assert(SoftFloat::neg_one().is_negative());
static_assert(ct_float_eq(SoftFloat::half(), 0.5f));
static_assert(ct_float_eq(SoftFloat::two(), 2.0f));
static_assert(ct_approx(SoftFloat::pi().to_float(), 3.14159265f, 2));
static_assert(SoftFloat::pi().is_positive());
static_assert(ct_approx(SoftFloat::two_pi().to_float(), 6.2831853f, 2));
static_assert(ct_approx(SoftFloat::half_pi().to_float(), 1.5707963f, 2));

// Relationships between constants
static_assert((SoftFloat::pi()* SoftFloat::two()).to_float() == SoftFloat::two_pi().to_float());
static_assert((SoftFloat::two_pi() / SoftFloat::two()).to_float() == SoftFloat::pi().to_float());
static_assert((SoftFloat::pi() / SoftFloat::two()).to_float() == SoftFloat::half_pi().to_float());
static_assert((SoftFloat::one() + SoftFloat::one()).to_float() == SoftFloat::two().to_float());
static_assert((SoftFloat::half() + SoftFloat::half()).to_float() == SoftFloat::one().to_float());
static_assert((-SoftFloat::one()).to_float() == SoftFloat::neg_one().to_float());

// Constants used in math functions
static_assert(ct_is_normalized(0x2C5C85FE, -30));
static_assert(ct_is_normalized(0x2E2B8A3E, -29));
static_assert(ct_is_normalized(0x2E2B8A3E, -21));
static_assert(ct_is_normalized(843314857, -36));
static_assert(ct_is_normalized(683565276, -23));
static_assert(ct_is_normalized(683565276, -32));

// Basic arithmetic
static_assert((SoftFloat::one() + SoftFloat::one()).to_float() == 2.0f);
static_assert((SoftFloat::two() - SoftFloat::one()).to_float() == 1.0f);
static_assert((SoftFloat::two()* SoftFloat::two()).to_float() == 4.0f);
static_assert((SoftFloat::two() / SoftFloat::two()).to_float() == 1.0f);
static_assert((-SoftFloat::one()).to_float() == -1.0f);
static_assert(SoftFloat::neg_one().abs().to_float() == 1.0f);

// Comparisons
static_assert(SoftFloat::one() < SoftFloat::two());
static_assert(SoftFloat::neg_one() < SoftFloat::zero());
static_assert(SoftFloat::one() == SoftFloat::one());
static_assert(SoftFloat::one() != SoftFloat::two());

// Shifts
static_assert((SoftFloat::one() << 2).to_float() == 4.0f);
static_assert((SoftFloat(8.0f) >> 2).to_float() == 2.0f);

// Fused operations
static_assert(fused_mul_add(SoftFloat::one(), SoftFloat::two(), SoftFloat::three()).to_float() == 7.0f);
static_assert(fused_mul_sub(SoftFloat::one(), SoftFloat::two(), SoftFloat::three()).to_float() == -5.0f);
static_assert(fused_mul_mul_add(SoftFloat::one(), SoftFloat::two(),
	SoftFloat::three(), SoftFloat::four()).to_float() == 14.0f);
static_assert(fused_mul_mul_sub(SoftFloat::one(), SoftFloat::two(),
	SoftFloat::three(), SoftFloat::four()).to_float() == -10.0f);

// Trigonometry
static_assert(SoftFloat::zero().sin().to_float() == 0.0f);
static_assert(ct_approx(SoftFloat::half_pi().sin().to_float(), 1.0f, 256));
static_assert(SoftFloat::zero().cos().to_float() == 1.0f);
static_assert(ct_approx(SoftFloat::pi().cos().to_float(), -1.0f, 1024));
static_assert(SoftFloat::zero().tan().to_float() == 0.0f);

static_assert(ct_approx(SoftFloat::zero().asin().to_float(), 0.0f, 2));
static_assert(ct_approx(SoftFloat::one().asin().to_float(), 1.57079633f, 4));
static_assert(ct_approx(SoftFloat(-0.5f).asin().to_float(), 5.75958653f /* 2π - 0.52359878 */, 256));  // wrapped [0,2π) representation in Angle
static_assert(ct_approx(SoftFloat::half().asin().to_float(), 0.52359878f, 16));
static_assert(ct_approx(SoftFloat::half().acos().to_float(), 1.04719755f, 16));
static_assert(ct_approx(SoftFloat(-0.5f).acos().to_float(), 2.09439510f, 16));
static_assert(ct_approx(SoftFloat::zero().acos().to_float(), 1.57079633f, 4));
static_assert(ct_approx(SoftFloat::half().acos().to_float(), 1.04719755f, 16));
static_assert(ct_approx(SoftFloat::one().acos().to_float(), 0.0f, 2));
static_assert(ct_approx(SoftFloat(-0.5f).acos().to_float(), 2.09439510f, 4));
static_assert(ct_approx(SoftFloat(-1.0f).acos().to_float(), 3.14159265f, 4));

static_assert(atan(SoftFloat::zero()).to_float() == 0.0f);
static_assert(ct_approx(atan(SoftFloat::one()).to_float(), SoftFloat::half_pi().to_float() / 2.0f, 256));

// atan2
static_assert(atan2(SoftFloat::one(), SoftFloat::zero()).to_float() == SoftFloat::half_pi().to_float());
static_assert(ct_approx(atan2(SoftFloat::one(), SoftFloat::one()).to_float(), SoftFloat::half_pi().to_float() / 2.0f, 256));

// ---------- Exponential & Logarithm ----------
static_assert(SoftFloat::zero().exp().to_float() == 1.0f);
static_assert(ct_approx(SoftFloat::one().exp().to_float(), 2.7182818f, 512));
static_assert(SoftFloat::one().log().to_float() == 0.0f);
static_assert(ct_approx(SoftFloat::two().log().to_float(), 0.693147f, 512));
static_assert(SoftFloat::one().log2().to_float() == 0.0f);
static_assert(ct_approx(SoftFloat::two().log2().to_float(), 1.0f, 512));
static_assert(SoftFloat::one().log10().to_float() == 0.0f);
static_assert(ct_approx(SoftFloat(10.0f).log10().to_float(), 1.0f, 512));

// ---------- Power ----------
static_assert(SoftFloat::two().pow(SoftFloat::three()).to_float() == 8.0f);
static_assert(SoftFloat(4.0f).pow(SoftFloat::half()).to_float() == 2.0f);
static_assert(SoftFloat::zero().pow(SoftFloat::one()).to_float() == 0.0f);
static_assert(SoftFloat::one().pow(SoftFloat::zero()).to_float() == 1.0f);

// ---------- Square roots ----------
static_assert(SoftFloat(16.0f).sqrt().to_float() == 4.0f);
//static_assert(ct_approx(SoftFloat(2.0f).inv_sqrt().to_float(), 0.70710678f, 256));

// ---------- Rounding ----------
static_assert(SoftFloat(1.3f).trunc().to_float() == 1.0f);
static_assert(SoftFloat(-1.3f).trunc().to_float() == -1.0f);
static_assert(SoftFloat(1.3f).floor().to_float() == 1.0f);
static_assert(SoftFloat(-1.3f).floor().to_float() == -2.0f);
static_assert(SoftFloat(1.3f).ceil().to_float() == 2.0f);
static_assert(SoftFloat(-1.3f).ceil().to_float() == -1.0f);
static_assert(SoftFloat(1.5f).round().to_float() == 2.0f);
static_assert(SoftFloat(-1.5f).round().to_float() == -2.0f);
static_assert(SoftFloat(1.3f).fract().to_float() > 0.29f && SoftFloat(1.3f).fract().to_float() < 0.31f);

// ---------- modf ----------
static_assert([]() consteval {
	auto [i, f] = SoftFloat(1.3f).modf();
	return i.to_float() == 1.0f && f.to_float() > 0.29f && f.to_float() < 0.31f;
	}());

// ---------- Hyperbolic ----------
static_assert(SoftFloat::zero().sinh().to_float() == 0.0f);
static_assert(SoftFloat::zero().cosh().to_float() == 1.0f);
static_assert(SoftFloat::zero().tanh().to_float() == 0.0f);
static_assert(ct_approx(SoftFloat::one().sinh().to_float(), 1.175201f, 512));
static_assert(ct_approx(SoftFloat::one().cosh().to_float(), 1.543080f, 512));
static_assert(ct_approx(SoftFloat::one().tanh().to_float(), 0.761594f, 512));

// ---------- Sign manipulation ----------
static_assert(SoftFloat::one().copysign(SoftFloat::neg_one()).to_float() == -1.0f);
static_assert(SoftFloat::neg_one().copysign(SoftFloat::one()).to_float() == 1.0f);
static_assert(SoftFloat::neg_one().is_negative());

// ---------- Remainder ----------
static_assert(SoftFloat(5.3f).fmod(SoftFloat::two()).to_float() > 1.29f &&
	SoftFloat(5.3f).fmod(SoftFloat::two()).to_float() < 1.31f);
static_assert(SoftFloat(-5.3f).fmod(SoftFloat::two()).to_float() < -1.29f &&
	SoftFloat(-5.3f).fmod(SoftFloat::two()).to_float() > -1.31f);

// ---------- fma (member) ----------
static_assert(SoftFloat::two().fma(SoftFloat::three(), SoftFloat::four()).to_float() == 14.0f);

// ---------- Utility ----------
static_assert(min(SoftFloat::one(), SoftFloat::two()).to_float() == 1.0f);
static_assert(max(SoftFloat::one(), SoftFloat::two()).to_float() == 2.0f);
static_assert(clamp(SoftFloat::three(), SoftFloat::zero(), SoftFloat::two()).to_float() == 2.0f);
static_assert(lerp(SoftFloat::zero(), SoftFloat::two(), SoftFloat::half()).to_float() == 1.0f);
static_assert(hypot(SoftFloat::three(), SoftFloat::four()).to_float() == 5.0f);

// ---------- Expression template interactions ----------
static_assert((SoftFloat::one() + SoftFloat::two() * SoftFloat::three()).to_float() == 7.0f);
static_assert((SoftFloat::two() * SoftFloat::three() - SoftFloat::one()).to_float() == 5.0f);
static_assert((-(SoftFloat::two() * SoftFloat::three())).to_float() == -6.0f);

// FMA autodetection via += / -=
static_assert([]() consteval {
	SoftFloat a = SoftFloat::one();
	a += SoftFloat::two() * SoftFloat::three(); // 1 + 2*3 = 7
	return a.to_float() == 7.0f;
}());

static_assert([]() consteval {
	SoftFloat a = SoftFloat(10.0f);
	a -= SoftFloat::two() * SoftFloat::three(); // 10 - 2*3 = 4
	return a.to_float() == 4.0f;
}());

// Verify it's actually fused (same precision as explicit fused_mul_add)
static_assert([]() consteval {
	SoftFloat a = SoftFloat::one();
	SoftFloat b = SoftFloat::two();
	SoftFloat c = SoftFloat::three();
	SoftFloat r1 = a; r1 += b * c;
	SoftFloat r2 = fused_mul_add(a, b, c);
	return r1 == r2;
}());

// Missing constants
static_assert(SoftFloat::three().to_float() == 3.0f, "three == 3");
static_assert(SoftFloat::four().to_float() == 4.0f, "four == 4");

// Unary plus is identity
static_assert((+SoftFloat::one()).to_float() == 1.0f, "+one == 1");
static_assert((+SoftFloat::neg_one()).to_float() == -1.0f, "+neg_one == -1");

// Reciprocal
static_assert(ct_approx(SoftFloat::two().reciprocal().to_float(), 0.5f), "recip(2) ≈ 0.5");
static_assert((SoftFloat::half().reciprocal()).to_float() == 2.0f, "recip(0.5) == 2");
static_assert(reciprocal(SoftFloat::two()).to_float() == 0.5f, "free recip(2) == 0.5");

// atan (member)
static_assert(SoftFloat::zero().atan().to_float() == 0.0f, "atan(0) == 0");
static_assert(ct_approx(SoftFloat::one().atan().to_float(), SoftFloat::pi().to_float() / 4.0f, 256),
	"atan(1) ≈ pi/4");

// Mixed-type addition / subtraction / multiplication / division (with int/float)
static_assert((SoftFloat::one() + 2.0f).to_float() == 3.0f, "1 + 2.0f == 3");
static_assert((3.0f + SoftFloat::two()).to_float() == 5.0f, "3.0f + 2 == 5");
static_assert((SoftFloat::two() - 1).to_float() == 1.0f, "2 - 1 == 1");
static_assert((5 - SoftFloat::three()).to_float() == 2.0f, "5 - 3 == 2");
static_assert((SoftFloat::three() * 2.0f).to_float() == 6.0f, "3 * 2.0f == 6");
static_assert((4.0f * SoftFloat::one()).to_float() == 4.0f, "4.0f * 1 == 4");
static_assert((SoftFloat::one() * 10).to_float() == 10.0f, "1 * 10 == 10");
static_assert((10 * SoftFloat::one()).to_float() == 10.0f, "10 * 1 == 10");
static_assert(ct_approx((SoftFloat(12.0f) / 3).to_float(), 4.0f, 4), "12 / 3 ≈ 4");
static_assert(ct_approx((15.0f / SoftFloat::three()).to_float(), 5.0f, 4), "15.0f / 3 ≈ 5");

// Mixed comparisons
static_assert(SoftFloat::one() == 1.0f, "1 == 1.0f");
static_assert(1 == SoftFloat::one(), "1 == 1 (int)");
static_assert(SoftFloat::two() > 1.9f, "2 > 1.9");
static_assert(2 < SoftFloat::three(), "2 < 3");
static_assert(SoftFloat::neg_one() <= 0, "-1 <= 0");
static_assert(-1 <= SoftFloat::zero(), "-1 <= 0");
static_assert(SoftFloat::one() >= 0.5f, "1 >= 0.5");

// Assignment operators
static_assert([]() consteval {
	SoftFloat a;
	a = 42;          return a.to_float() == 42.0f;
	}(), "assign int");
static_assert([]() consteval {
	SoftFloat a;
	a = 3.125f;      return a.to_float() == 3.125f;
	}(), "assign float");
static_assert([]() consteval {
	SoftFloat a;
	a = int16_t(7);  return a.to_float() == 7.0f;
	}(), "assign int16_t");

// Compound assignment with MulExpr
static_assert([]() consteval {
	SoftFloat a(5.0f);
	a += SoftFloat::two() * SoftFloat::three();  // 5 + 6 = 11
	return a.to_float() == 11.0f;
	}(), "+= mul_expr");
static_assert([]() consteval {
	SoftFloat a(20.0f);
	a -= SoftFloat::two() * SoftFloat::three();  // 20 - 6 = 14
	return a.to_float() == 14.0f;
	}(), "-= mul_expr");

// MulExpr chaining: (a*b).sqrt(), etc.
static_assert(ct_approx(
	(SoftFloat::two()* SoftFloat::two()).sqrt().to_float(),
	2.0f, 4), "(2*2).sqrt() == 2");
static_assert(ct_approx(
	(SoftFloat::two()* SoftFloat::three()).exp().to_float(),
	403.42879349273512260838718054339f /*=expf(6.0f)*/, 1024), "(2*3).exp() ≈ exp(6)");
static_assert(ct_approx(
	(SoftFloat::two()* SoftFloat::four()).log2().to_float(),
	3.0f, 512), "(2*4).log2() == 3");

// User-defined literal
static_assert((1.5_sf).to_float() == 1.5f, "1.5_sf literal");
static_assert((3_sf).to_float() == 3.0f, "3_sf literal");
