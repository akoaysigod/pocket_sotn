-- pocket_audio48 test: 44.1 kHz input samples (text file "L R" per line) arrive like the SPU's output, each with its
-- valid strobe a variable time after the tick of its period (150 .. 740 clk1x); with LATE > 0, every LATE-th one
-- only after the next tick (up to 1.4 periods late, as when the SPU falls behind) and the next one right after it.
-- The 48 kHz word clock comes from a 74.25 MHz clock exactly as sound_i2s makes it; every output sample sound_i2s
-- would load (rising edge of lrck) is written to OUTFILE.
-- sim/audio48/check.c compares that against an ideal resample of the input.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

entity tb_audio48 is
   generic
   (
      INFILE  : string := "in.txt";
      OUTFILE : string := "out.txt";
      NSAMP   : integer := 13230;
      LATE    : integer := 0;
      INTERP  : boolean := true
   );
end entity;

architecture sim of tb_audio48 is
   signal clk1x, clk74 : std_logic := '0';
   signal done         : boolean := false;
   signal tick         : std_logic := '0';
   signal valid        : std_logic := '0';
   signal in_l, in_r   : std_logic_vector(15 downto 0) := (others => '0');
   signal out_l, out_r : std_logic_vector(15 downto 0);
   signal mclk, lrck   : std_logic := '0';
begin

   process begin   -- 33.8688 MHz
      while not done loop clk1x <= '1'; wait for 14.7630 ns; clk1x <= '0'; wait for 14.7630 ns; end loop; wait;
   end process;
   process begin   -- 74.25 MHz
      while not done loop clk74 <= '1'; wait for 6.7340 ns; clk74 <= '0'; wait for 6.7340 ns; end loop; wait;
   end process;

   -- sound_i2s: MCLK 12.288 MHz from a fractional accumulator, SCLK = MCLK / 4, 32 SCLK per channel
   process (clk74)
      variable acc    : integer := 0;
      variable div    : unsigned(1 downto 0) := "00";
      variable pm, ps : std_logic := '0';
      variable cnt    : integer range 0 to 31 := 0;
   begin
      if rising_edge(clk74) then
         acc := acc + 245760;
         if (acc >= 742500) then mclk <= not mclk; acc := acc - 742500; end if;
         if (mclk = '1' and pm = '0') then div := div + 1; end if;
         pm := mclk;
         if (ps = '1' and div(1) = '0') then
            if (cnt = 31) then lrck <= not lrck; cnt := 0; else cnt := cnt + 1; end if;
         end if;
         ps := div(1);
      end if;
   end process;

   idut : entity work.pocket_audio48
   generic map (INTERP => INTERP)
   port map (clk => clk1x, tick => tick, valid => valid, in_l => in_l, in_r => in_r, lrck => lrck, out_l => out_l,
             out_r => out_r);

   -- ticks every 768 clocks
   process
   begin
      while not done loop
         wait until rising_edge(clk1x);
         tick <= '1';
         wait until rising_edge(clk1x);
         tick <= '0';
         for i in 1 to 766 loop wait until rising_edge(clk1x); end loop;
      end loop;
      wait;
   end process;

   -- SPU-like source: sample k is finished (valid) at 150 .. 740 clocks after tick k; a late one (LATE) at 900 ..
   -- 1100 clocks, the next one right after it
   process
      file f          : text open read_mode is INFILE;
      variable ln     : line;
      variable l, r   : integer;
      variable s1, s2 : positive := 7;
      variable x      : real;
      variable t, d   : integer := 0;   -- clocks since tick 0; finish time of the sample
      variable prev   : integer := 0;
   begin
      for i in 1 to 2 loop wait until rising_edge(clk1x); end loop;   -- tick 0 has been
      for k in 0 to NSAMP - 1 loop
         uniform(s1, s2, x);
         d := k * 768 + 150 + integer(trunc(x * 590.0));
         if (LATE > 0 and k mod LATE = LATE - 1) then d := k * 768 + 900 + integer(trunc(x * 200.0)); end if;
         if (d <= prev) then d := prev + 30; end if;                   -- after the previous one (one SPU)
         while (t < d) loop wait until rising_edge(clk1x); t := t + 1; end loop;
         prev := d;
         if not endfile(f) then
            readline(f, ln); read(ln, l); read(ln, r);
            in_l <= std_logic_vector(to_signed(l, 16));
            in_r <= std_logic_vector(to_signed(r, 16));
         end if;
         valid <= '1';
         wait until rising_edge(clk1x); t := t + 1;
         valid <= '0';
      end loop;
      for i in 1 to 4000 loop wait until rising_edge(clk1x); end loop;
      done <= true;
      wait;
   end process;

   process (lrck)
      file f      : text open write_mode is OUTFILE;
      variable ln : line;
   begin
      if rising_edge(lrck) then
         write(ln, to_integer(signed(out_l))); write(ln, string'(" ")); write(ln, to_integer(signed(out_r)));
         writeline(f, ln);
      end if;
   end process;

end architecture;
