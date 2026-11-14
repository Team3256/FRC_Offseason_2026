#pragma once
// mayhem::sim deterministic random numbers for Monte Carlo runs. Header-only, C++17.
//
// std::*_distribution output differs between standard libraries, so seeded runs would not match
// across machines. Rng (xoshiro256**, seeded through SplitMix64) and its Uniform/Normal use only
// integer arithmetic plus std::log/std::sqrt/std::cos, so a given seed gives the same sequence on
// every platform with IEEE doubles and a correctly rounded libm (and always on the same binary).
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace mayhem::sim {

class Rng {
 public:
  explicit Rng(std::uint64_t seed = 0) { Seed(seed); }
  void Seed(std::uint64_t seed) {
    for (auto& s : s_) s = SplitMix(seed);
    haveSpare_ = false;
  }
  /** Seed from the MAYHEM_SIM_SEED environment variable (fallback if unset). */
  static std::uint64_t SeedFromEnv(std::uint64_t fallback = 0) {
    const char* v = std::getenv("MAYHEM_SIM_SEED");
    return v && *v ? std::strtoull(v, nullptr, 10) : fallback;
  }
  std::uint64_t Next() {
    const std::uint64_t result = Rotl(s_[1] * 5, 7) * 9, t = s_[1] << 17;
    s_[2] ^= s_[0]; s_[3] ^= s_[1]; s_[1] ^= s_[2]; s_[0] ^= s_[3]; s_[2] ^= t; s_[3] = Rotl(s_[3], 45);
    return result;
  }
  /** Uniform in [0, 1). */
  double Uniform() { return static_cast<double>(Next() >> 11) * 0x1.0p-53; }
  double Uniform(double lo, double hi) { return lo + (hi - lo) * Uniform(); }
  /** Normal(mean, stddev) by Box-Muller. */
  double Normal(double mean = 0, double stddev = 1) {
    if (haveSpare_) { haveSpare_ = false; return mean + stddev * spare_; }
    double u1 = Uniform();
    while (u1 <= 0) u1 = Uniform();
    const double u2 = Uniform();
    const double r = std::sqrt(-2 * std::log(u1)), a = 6.283185307179586 * u2;
    spare_ = r * std::sin(a);
    haveSpare_ = true;
    return mean + stddev * r * std::cos(a);
  }
  /** Integer in [0, n). */
  std::uint64_t Below(std::uint64_t n) { return n ? Next() % n : 0; }

 private:
  std::uint64_t s_[4]{};
  double spare_ = 0;
  bool haveSpare_ = false;
  static std::uint64_t Rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  static std::uint64_t SplitMix(std::uint64_t& x) {
    std::uint64_t z = (x += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
  }
};

}  // namespace mayhem::sim
