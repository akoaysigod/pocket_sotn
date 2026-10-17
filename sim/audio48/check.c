// compare pocket_audio48's 48 kHz output (out.txt) with an ideal 128-tap windowed-sinc resample of its 44.1 kHz
// input (in.txt), at the best delay; also sample and hold (what sound_i2s did before) for reference
// usage: check in.txt out.txt
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static double I0(double x) {
   double s = 1, t = 1;
   for (int k = 1; k < 30; k++) { t *= (x / 2 / k) * (x / 2 / k); s += t; }
   return s;
}
static double ker(double u) {
   if (fabs(u) >= 64) return 0;
   double s = fabs(u) < 1e-12 ? 1 : sin(M_PI * u) / (M_PI * u), r = u / 64;
   return s * I0(9 * sqrt(1 - r * r)) / I0(9);
}
static float xi[1 << 20], yo[1 << 20];
static long nin, nout;
static double ideal(double t) {
   long k = (long)floor(t); double a = 0;
   for (long q = k - 63; q <= k + 64; q++) if (q >= 0 && q < nin) a += xi[q] * ker(t - q);
   return a;
}
// output n taken at input position n * 44100 / 48000 - d; hold: the newest input sample instead of the output
static double err(double d, int hold, double* maxe) {
   double e = 0, s = 0; *maxe = 0;
   for (long n = 400; n < nout - 400; n++) {
      double t = n * 44100.0 / 48000.0 - d, y = hold ? xi[(long)floor(t)] : yo[n], r = ideal(t);
      e += (y - r) * (y - r); s += r * r;
      if (fabs(y - r) > *maxe) *maxe = fabs(y - r);
   }
   return 10 * log10(e / s);
}
int main(int argc, char** argv) {
   FILE* a = fopen(argv[1], "r"); FILE* b = fopen(argv[2], "r"); int l, r;
   while (fscanf(a, "%d %d", &l, &r) == 2) xi[nin++] = l;
   while (fscanf(b, "%d %d", &l, &r) == 2) yo[nout++] = l;
   if (nout * 44100.0 / 48000.0 > nin) nout = (long)(nin * 48000.0 / 44100.0);
   double best = 1e9, bd = 0, m;
   for (double d = -2; d <= 10; d += 0.05) { double e = err(d, 0, &m); if (e < best) { best = e; bd = d; } }
   for (double d = bd - 0.05; d <= bd + 0.05; d += 0.005) { double e = err(d, 0, &m); if (e < best) { best = e; bd = d; } }
   err(bd, 0, &m);
   double mh, eh = err(0.5, 1, &mh);
   printf("input %ld samples, output %ld; delay %.3f input samples; error vs ideal: %.1f dB (max %.0f); "
          "sample and hold: %.1f dB (max %.0f)\n", nin, nout, bd, best, m, eh, mh);
   int ok = best < -45 && m < 32;   // a misread history tap shows as a max error of ~100+
   printf("%s\n", ok ? "PASS" : "FAIL");
   return ok ? 0 : 1;
}
