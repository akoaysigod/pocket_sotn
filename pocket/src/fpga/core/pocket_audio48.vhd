-- Analogue Pocket: the core's 44.1 kHz audio to the Pocket's 48 kHz I2S frames, interpolated.
-- Before, sound_i2s took the newest 44.1 kHz sample at each 48 kHz frame (sample and hold): some samples twice,
-- at irregular times. On SotN's XA music that leaves distortion only 21-34 dB below the music (bright, sustained
-- tracks worst), heard as crackle / grit. This 8-tap windowed sinc (Kaiser 3, 64 phases, Q14) brings it to
-- 45-57 dB below (measured offline on the game's music against a 128-tap reference; linear interpolation: 30-55).
--  * clk1x domain. Each sample the SPU finishes (sound_valid) goes into a 4-entry queue; each SPU sample tick (every
--    768 clk1x) moves the oldest queued sample into a short history and restarts the phase count: phase = clocks
--    since the tick / 12. The queue starts once it holds 2 samples, so an SPU sample that comes up to ~1.5 sample
--    periods late (the SPU falls behind when its sound RAM is slow; on the Pocket it shares the SDRAM with the CPU)
--    still reaches the history in order. (Taken straight at the tick, a late sample was replaced by the one before
--    and then skipped: a click.) If the queue runs empty anyway, the newest sample is repeated and it refills to 2.
--  * At each falling edge of the I2S word clock (lrck, 48 kHz, synchronized here) one output sample is computed for
--    the position 4 samples back + phase: 16 multiply-adds, one per clock (~20 clocks). It is stable long before
--    sound_i2s loads it at the next rising edge of lrck (half a frame later).
--  * A tick during the computation shifts the history by one: the remaining taps read one entry further back.
--  * INTERP = false: no interpolation (no multiplier, no coefficient ROM): out_l / out_r are the queued samples in
--    order, one per tick, and sound_i2s takes the newest at each 48 kHz frame as it did before (smaller builds).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pocket_audio48 is
   generic
   (
      INTERP   : boolean := true
   );
   port
   (
      clk      : in  std_logic;                       -- clk1x
      tick     : in  std_logic;                       -- SPU sample tick (44.1 kHz)
      valid    : in  std_logic;                       -- SPU: in_l / in_r hold a newly finished sample
      in_l     : in  std_logic_vector(15 downto 0);   -- core output, signed
      in_r     : in  std_logic_vector(15 downto 0);
      lrck     : in  std_logic;                       -- I2S word clock (sound_i2s, clk_74a domain)
      out_l    : out std_logic_vector(15 downto 0) := (others => '0');
      out_r    : out std_logic_vector(15 downto 0) := (others => '0')
   );
end entity;

architecture arch of pocket_audio48 is

   -- coefficient (phase p, tap j) for input sample (newest - 7 + j), i.e. offset j - 3 from the position's sample
   type tcoef is array (0 to 511) of integer range -32768 to 32767;
   constant COEF : tcoef :=
   (
           0,      0,      0,  16384,      0,      0,      0,      0,   -- phase 0
         -39,     92,   -233,  16389,    242,    -94,     40,    -13,   -- phase 1
         -77,    181,   -457,  16382,    492,   -191,     81,    -27,   -- phase 2
        -113,    268,   -673,  16360,    751,   -290,    123,    -42,   -- phase 3
        -148,    352,   -881,  16325,   1018,   -391,    166,    -57,   -- phase 4
        -182,    433,  -1079,  16275,   1293,   -494,    210,    -72,   -- phase 5
        -215,    511,  -1268,  16211,   1576,   -598,    255,    -88,   -- phase 6
        -245,    586,  -1448,  16133,   1866,   -704,    301,   -105,   -- phase 7
        -275,    657,  -1619,  16042,   2164,   -810,    346,   -121,   -- phase 8
        -303,    726,  -1780,  15936,   2468,   -918,    393,   -138,   -- phase 9
        -329,    790,  -1932,  15819,   2779,  -1026,    439,   -156,   -- phase 10
        -354,    851,  -2074,  15687,   3096,  -1135,    486,   -173,   -- phase 11
        -377,    908,  -2207,  15543,   3419,  -1244,    533,   -191,   -- phase 12
        -398,    962,  -2330,  15384,   3747,  -1352,    580,   -209,   -- phase 13
        -418,   1012,  -2443,  15213,   4080,  -1460,    627,   -227,   -- phase 14
        -436,   1058,  -2547,  15030,   4418,  -1567,    673,   -245,   -- phase 15
        -452,   1100,  -2642,  14835,   4760,  -1673,    719,   -263,   -- phase 16
        -467,   1138,  -2727,  14630,   5105,  -1778,    764,   -281,   -- phase 17
        -479,   1172,  -2803,  14411,   5454,  -1880,    808,   -299,   -- phase 18
        -491,   1203,  -2869,  14181,   5805,  -1981,    852,   -316,   -- phase 19
        -500,   1230,  -2926,  13941,   6158,  -2080,    894,   -333,   -- phase 20
        -508,   1252,  -2975,  13692,   6513,  -2175,    935,   -350,   -- phase 21
        -514,   1271,  -3014,  13431,   6870,  -2268,    975,   -367,   -- phase 22
        -519,   1287,  -3044,  13162,   7226,  -2358,   1013,   -383,   -- phase 23
        -522,   1298,  -3066,  12882,   7584,  -2443,   1050,   -399,   -- phase 24
        -524,   1306,  -3080,  12596,   7940,  -2525,   1085,   -414,   -- phase 25
        -524,   1310,  -3086,  12301,   8296,  -2603,   1118,   -428,   -- phase 26
        -522,   1311,  -3083,  11996,   8650,  -2675,   1148,   -441,   -- phase 27
        -520,   1308,  -3073,  11686,   9003,  -2743,   1177,   -454,   -- phase 28
        -516,   1302,  -3055,  11369,   9353,  -2806,   1203,   -466,   -- phase 29
        -510,   1293,  -3030,  11045,   9699,  -2863,   1227,   -477,   -- phase 30
        -504,   1281,  -2998,  10716,  10043,  -2914,   1247,   -487,   -- phase 31
        -496,   1266,  -2959,  10380,  10382,  -2959,   1266,   -496,   -- phase 32
        -487,   1247,  -2914,  10043,  10716,  -2998,   1281,   -504,   -- phase 33
        -477,   1227,  -2863,   9699,  11045,  -3030,   1293,   -510,   -- phase 34
        -466,   1203,  -2806,   9353,  11369,  -3055,   1302,   -516,   -- phase 35
        -454,   1177,  -2743,   9003,  11686,  -3073,   1308,   -520,   -- phase 36
        -441,   1148,  -2675,   8649,  11997,  -3083,   1311,   -522,   -- phase 37
        -428,   1118,  -2603,   8297,  12300,  -3086,   1310,   -524,   -- phase 38
        -414,   1085,  -2525,   7940,  12596,  -3080,   1306,   -524,   -- phase 39
        -399,   1050,  -2443,   7583,  12883,  -3066,   1298,   -522,   -- phase 40
        -383,   1013,  -2358,   7226,  13162,  -3044,   1287,   -519,   -- phase 41
        -367,    975,  -2268,   6869,  13432,  -3014,   1271,   -514,   -- phase 42
        -350,    935,  -2175,   6513,  13692,  -2975,   1252,   -508,   -- phase 43
        -333,    894,  -2080,   6157,  13942,  -2926,   1230,   -500,   -- phase 44
        -316,    852,  -1981,   5804,  14182,  -2869,   1203,   -491,   -- phase 45
        -299,    808,  -1880,   5454,  14411,  -2803,   1172,   -479,   -- phase 46
        -281,    764,  -1778,   5106,  14629,  -2727,   1138,   -467,   -- phase 47
        -263,    719,  -1673,   4759,  14836,  -2642,   1100,   -452,   -- phase 48
        -245,    673,  -1567,   4417,  15031,  -2547,   1058,   -436,   -- phase 49
        -227,    627,  -1460,   4080,  15213,  -2443,   1012,   -418,   -- phase 50
        -209,    580,  -1352,   3747,  15384,  -2330,    962,   -398,   -- phase 51
        -191,    533,  -1244,   3420,  15542,  -2207,    908,   -377,   -- phase 52
        -173,    486,  -1135,   3097,  15686,  -2074,    851,   -354,   -- phase 53
        -156,    439,  -1026,   2780,  15818,  -1932,    790,   -329,   -- phase 54
        -138,    393,   -918,   2467,  15937,  -1780,    726,   -303,   -- phase 55
        -121,    346,   -810,   2164,  16042,  -1619,    657,   -275,   -- phase 56
        -105,    301,   -704,   1866,  16133,  -1448,    586,   -245,   -- phase 57
         -88,    255,   -598,   1576,  16211,  -1268,    511,   -215,   -- phase 58
         -72,    210,   -494,   1293,  16275,  -1079,    433,   -182,   -- phase 59
         -57,    166,   -391,   1019,  16324,   -881,    352,   -148,   -- phase 60
         -42,    123,   -290,    751,  16360,   -673,    268,   -113,   -- phase 61
         -27,     81,   -191,    492,  16382,   -457,    181,    -77,   -- phase 62
         -13,     40,    -94,    241,  16390,   -233,     92,    -39    -- phase 63
   );

   type tq is array (0 to 3) of std_logic_vector(31 downto 0);
   signal q          : tq := (others => (others => '0'));
   signal q_wp, q_rp : unsigned(1 downto 0) := (others => '0');
   signal q_n        : integer range 0 to 4 := 0;
   signal primed     : std_logic := '0';

   type thist is array (0 to 8) of signed(15 downto 0);
   signal hist_l, hist_r : thist := (others => (others => '0'));   -- 0 = newest

   signal sub        : integer range 0 to 11 := 0;
   signal phase      : unsigned(5 downto 0) := (others => '0');
   signal lrck_s     : std_logic_vector(2 downto 0) := (others => '0');

   signal running    : std_logic := '0';
   signal step       : integer range 0 to 15 := 0;      -- tap being addressed: 0-7 left, 8-15 right
   signal shifted    : std_logic := '0';                 -- a tick came during this computation
   signal ph_run     : unsigned(5 downto 0) := (others => '0');
   signal coef_q     : signed(15 downto 0) := (others => '0');
   signal samp_q     : signed(15 downto 0) := (others => '0');
   signal prod       : signed(31 downto 0) := (others => '0');
   signal acc        : signed(34 downto 0) := (others => '0');
   signal v1, v2, v3 : std_logic := '0';                 -- pipeline valid: operands, product, accumulate
   signal last1, last2, last3 : std_logic := '0';        -- ... last tap of a channel
   signal right1, right2, right3 : std_logic := '0';     -- ... right channel

   function sat(a : signed(34 downto 0)) return std_logic_vector is
      variable r : signed(34 downto 0);
   begin
      r := shift_right(a + 8192, 14);
      if (r > 32767) then return x"7FFF"; elsif (r < -32768) then return x"8000"; end if;
      return std_logic_vector(r(15 downto 0));
   end function;

begin

   process (clk)
      variable idx : integer range 0 to 8;
      variable n   : integer range 0 to 4;
      variable rp  : unsigned(1 downto 0);
   begin
      if rising_edge(clk) then

         -- queue of finished SPU samples; at a tick the oldest goes into the history
         n  := q_n;
         rp := q_rp;
         if (tick = '1') then
            if (primed = '1' and n > 0) then
               hist_l(0) <= signed(q(to_integer(rp))(31 downto 16));
               hist_r(0) <= signed(q(to_integer(rp))(15 downto 0));
               rp := rp + 1;
               n  := n - 1;
            else
               hist_l(0) <= hist_l(0);   -- not started or ran empty: the newest again
               hist_r(0) <= hist_r(0);
               primed    <= '0';
            end if;
            if (n >= 2) then primed <= '1'; end if;
         end if;
         if (valid = '1') then
            q(to_integer(q_wp)) <= in_l & in_r;
            q_wp <= q_wp + 1;
            if (n = 4) then rp := rp + 1; else n := n + 1; end if;   -- full: the oldest is dropped
         end if;
         q_n  <= n;
         q_rp <= rp;

         -- history and phase (44.1 kHz grid)
         if (tick = '1') then
            for i in 1 to 8 loop
               hist_l(i) <= hist_l(i - 1);
               hist_r(i) <= hist_r(i - 1);
            end loop;
            sub    <= 0;
            phase  <= (others => '0');
         elsif (sub = 11) then
            sub <= 0;
            if (phase /= 63) then phase <= phase + 1; end if;
         else
            sub <= sub + 1;
         end if;

         -- without interpolation: the newest sample of the 44.1 kHz grid
         if (not INTERP) then
            out_l <= std_logic_vector(hist_l(0));
            out_r <= std_logic_vector(hist_r(0));
         end if;

         -- one output per lrck falling edge
         lrck_s <= lrck_s(1 downto 0) & lrck;
         if (running = '0') then
            if (INTERP and lrck_s(2) = '1' and lrck_s(1) = '0') then
               running <= '1';
               step    <= 0;
               shifted <= tick;   -- a tick now: the history shifts before the first read
               ph_run  <= phase;
            end if;
         else
            if (tick = '1') then shifted <= '1'; end if;
            if (step = 15) then running <= '0'; else step <= step + 1; end if;
         end if;

         -- stage 1: coefficient and sample (tap j = step mod 8: sample newest - 7 + j, one further if shifted)
         v1 <= running;
         last1  <= '0';
         right1 <= '0';
         if (running = '1') then
            coef_q <= to_signed(COEF(to_integer(ph_run) * 8 + step mod 8), 16);
            idx := 7 - step mod 8;
            if (shifted = '1') then idx := idx + 1; end if;   -- (a tick now shifts after this read)
            if (step < 8) then samp_q <= hist_l(idx); else samp_q <= hist_r(idx); end if;
            if (step mod 8 = 7) then last1 <= '1'; end if;
            if (step >= 8) then right1 <= '1'; end if;
         end if;

         -- stage 2: product
         v2 <= v1; last2 <= last1; right2 <= right1;
         prod <= coef_q * samp_q;

         -- stage 3: accumulate, output after the last tap of each channel
         v3 <= v2; last3 <= last2; right3 <= right2;
         if (v2 = '1') then
            if (last3 = '1' or v3 = '0') then
               acc <= resize(prod, 35);
            else
               acc <= acc + resize(prod, 35);
            end if;
         end if;
         if (v3 = '1' and last3 = '1') then
            if (right3 = '1') then out_r <= sat(acc); else out_l <= sat(acc); end if;
         end if;

      end if;
   end process;

end architecture;
