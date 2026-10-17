-- SPU timing: how long the SPU takes per output sample (budget: 768 clk1x between sample ticks) for a given sound
-- RAM latency. All 24 voices playing (pitch PITCH), reverb on, CD audio input on; every RAM request is answered
-- LAT clk1x later (LATJIT > 0: plus 0..LATJIT random clocks, like SDRAM contention).
-- Reported per run: the end of the 24-voice loop after the tick that started the sample (that is where the SPU
-- reads the CD / XA sample: later than tick + 768 + 7 picks up the next CD sample, the following one repeats it), the
-- whole sample's processing, and the samples the SPU started late (its stashedSamples count went up). With CDQ = 1
-- (an SPU with the CD input queue): the CD sample it mixes must be each period's in order (cd_left = 37 k mod 30000
-- in period k): skips / repeats are counted, and its sound_valid strobes (one per sample).
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

entity tb_spu_timing is
   generic
   (
      SAMPLES : integer := 1500;
      LAT     : integer := 5;
      LATJIT  : integer := 0;
      PITCH   : integer := 16#1000#;
      REVERB  : integer := 1;
      CDQ     : integer := 1
   );
end entity;

architecture sim of tb_spu_timing is

   signal clk1x, clk2x    : std_logic := '1';
   signal clk1xToggle     : std_logic := '0';
   signal clk1xToggle2x   : std_logic := '0';
   signal clk2xIndex      : std_logic := '0';
   signal reset           : std_logic := '1';
   signal SS_reset        : std_logic := '0';
   signal SS_wren         : std_logic := '0';
   signal SS_Adr          : unsigned(8 downto 0) := (others => '0');
   signal cd_left, cd_right : signed(15 downto 0) := (others => '0');
   signal bus_addr        : unsigned(9 downto 0) := (others => '0');
   signal bus_dataWrite   : std_logic_vector(15 downto 0) := (others => '0');
   signal bus_write       : std_logic := '0';
   signal sdram_dataRead  : std_logic_vector(31 downto 0) := (others => '0');
   signal sdram_done      : std_logic := '0';
   signal sdram_ena       : std_logic;
   signal spu_tick        : std_logic;
   signal valid           : std_logic := '0';
   signal cdstep_s        : integer := 4;
   signal stash_s         : integer := 0;
   signal used_l          : signed(15 downto 0) := (others => '0');
   signal running         : boolean := false;
   signal done            : boolean := false;

begin

   clk2x <= not clk2x after 5 ns when not done;
   clk1x <= not clk1x after 10 ns when not done;

   process (clk1x) begin
      if rising_edge(clk1x) then clk1xToggle <= not clk1xToggle; end if;
   end process;
   process (clk2x) begin
      if rising_edge(clk2x) then
         clk1xToggle2x <= clk1xToggle;
         clk2xIndex <= '0';
         if (clk1xToggle2x = clk1xToggle) then clk2xIndex <= '1'; end if;
      end if;
   end process;

   -- CDQ = 1: the SPU with the CD input queue (rtl/spu.vhd now); 0: one without it (SPUFILE in run.sh)
   gq : if CDQ /= 0 generate
      idut : entity work.spu_new
   port map (clk1x => clk1x, clk2x => clk2x, clk2xIndex => clk2xIndex, ce => '1', reset => reset,
         SPUon => '1', SPUIRQTrigger => '0', useSDRAM => '1', REPRODUCIBLESPUIRQ => '0', REPRODUCIBLESPUDMA => '0',
         REVERBOFF => '0', cpuPaused => '0', spu_tick => spu_tick, cd_left => cd_left, cd_right => cd_right,
         irqOut => open, sound_timeout => open, sound_out_left => open, sound_out_right => open,
         sound_valid => valid, late => open,
         bus_addr => bus_addr, bus_dataWrite => bus_dataWrite, bus_read => '0', bus_write => bus_write,
         bus_dataRead => open, spu_dmaRequest => open, dma_read => '0', dma_readdata => open,
         dma_write => '0', dma_writedata => (others => '0'),
         sdram_dataWrite => open, sdram_Adr => open, sdram_be => open, sdram_rnw => open, sdram_ena => sdram_ena,
         sdram_dataRead => sdram_dataRead, sdram_done => sdram_done,
         mem_request => open, mem_BURSTCNT => open, mem_ADDR => open, mem_DIN => open, mem_BE => open,
         mem_WE => open, mem_RD => open, mem_ack => '0', mem_DOUT => (others => '0'), mem_DOUT_READY => '0',
         SS_reset => SS_reset, loading_savestate => '0', SS_DataWrite => (others => '0'), SS_Adr => SS_Adr,
         SS_wren => SS_wren, SS_rden => '0', SS_DataRead => open, SS_idle => open,
         SS_RAM_dataWrite => (others => '0'), SS_RAM_Adr => (others => '0'), SS_RAM_request => '0',
         SS_RAM_rnw => '0', SS_RAM_dataRead => open, SS_RAM_done => open);
      cdstep_s <= << signal .tb_spu_timing.gq.idut.cdProcStep : integer >>;
      stash_s  <= << signal .tb_spu_timing.gq.idut.stashedSamples : integer >>;
      used_l   <= << signal .tb_spu_timing.gq.idut.cd_use_left : signed(15 downto 0) >>;
   end generate;
   gold : if CDQ = 0 generate
      idut : entity work.spu_new
   port map (clk1x => clk1x, clk2x => clk2x, clk2xIndex => clk2xIndex, ce => '1', reset => reset,
         SPUon => '1', SPUIRQTrigger => '0', useSDRAM => '1', REPRODUCIBLESPUIRQ => '0', REPRODUCIBLESPUDMA => '0',
         REVERBOFF => '0', cpuPaused => '0', spu_tick => spu_tick, cd_left => cd_left, cd_right => cd_right,
         irqOut => open, sound_timeout => open, sound_out_left => open, sound_out_right => open,
         bus_addr => bus_addr, bus_dataWrite => bus_dataWrite, bus_read => '0', bus_write => bus_write,
         bus_dataRead => open, spu_dmaRequest => open, dma_read => '0', dma_readdata => open,
         dma_write => '0', dma_writedata => (others => '0'),
         sdram_dataWrite => open, sdram_Adr => open, sdram_be => open, sdram_rnw => open, sdram_ena => sdram_ena,
         sdram_dataRead => sdram_dataRead, sdram_done => sdram_done,
         mem_request => open, mem_BURSTCNT => open, mem_ADDR => open, mem_DIN => open, mem_BE => open,
         mem_WE => open, mem_RD => open, mem_ack => '0', mem_DOUT => (others => '0'), mem_DOUT_READY => '0',
         SS_reset => SS_reset, loading_savestate => '0', SS_DataWrite => (others => '0'), SS_Adr => SS_Adr,
         SS_wren => SS_wren, SS_rden => '0', SS_DataRead => open, SS_idle => open,
         SS_RAM_dataWrite => (others => '0'), SS_RAM_Adr => (others => '0'), SS_RAM_request => '0',
         SS_RAM_rnw => '0', SS_RAM_dataRead => open, SS_RAM_done => open);
      cdstep_s <= << signal .tb_spu_timing.gold.idut.cdProcStep : integer >>;
      stash_s  <= << signal .tb_spu_timing.gold.idut.stashedSamples : integer >>;
      used_l   <= cd_left;   -- the old SPU reads cd_left itself at cdProcStep 0
   end generate;

   -- sound RAM: answer after LAT (+ 0..LATJIT) clocks; data: ADPCM blocks that keep playing (header 0x0000 =
   -- shift 0, filter 0, no loop end), random sample nibbles
   process (clk1x)
      variable s1 : positive := 11;
      variable s2 : positive := 7777;
      variable r  : real;
      variable pending : integer := -1;
   begin
      if rising_edge(clk1x) then
         sdram_done <= '0';
         if (sdram_ena = '1') then
            pending := LAT - 1;
            if (LATJIT > 0) then uniform(s1, s2, r); pending := pending + integer(trunc(r * real(LATJIT + 1))); end if;
         end if;
         if (pending = 0) then
            sdram_done <= '1';
            uniform(s1, s2, r);
            sdram_dataRead <= std_logic_vector(to_unsigned(integer(trunc(r * 65536.0)), 16)) & x"0000";
         end if;
         if (pending >= 0) then pending := pending - 1; end if;
      end if;
   end process;

   stim : process
      procedure wr(a : integer; d : integer) is
      begin
         wait until rising_edge(clk1x);
         bus_write <= '1'; bus_addr <= to_unsigned(a, 10); bus_dataWrite <= std_logic_vector(to_unsigned(d, 16));
         wait until rising_edge(clk1x);
         bus_write <= '0';
      end procedure;
   begin
      reset <= '1';
      for i in 0 to 5 loop wait until rising_edge(clk1x); end loop;
      SS_reset <= '1';
      for i in 0 to 5 loop wait until rising_edge(clk1x); end loop;
      SS_reset <= '0';
      for i in 0 to 10 loop wait until rising_edge(clk1x); end loop;
      reset <= '0';
      for i in 0 to 2000 loop wait until rising_edge(clk1x); end loop;
      if (REVERB /= 0) then wr(16#1AA#, 16#C0C1#); else wr(16#1AA#, 16#C041#); end if;   -- on, unmuted, (reverb), CD in
      wr(16#180#, 16#3FFF#); wr(16#182#, 16#3FFF#); wr(16#1B0#, 16#7FFF#); wr(16#1B2#, 16#7FFF#);
      wr(16#198#, 16#FFFF#); wr(16#19A#, 16#00FF#);                                         -- reverb on for all voices
      for v in 0 to 23 loop
         wr(v * 16 + 0, 16#3FFF#); wr(v * 16 + 2, 16#3FFF#); wr(v * 16 + 4, PITCH);
         wr(v * 16 + 6, 16#200# + v * 16#100#); wr(v * 16 + 8, 16#00FF#); wr(v * 16 + 10, 16#0000#);
      end loop;
      wr(16#188#, 16#FFFF#); wr(16#18A#, 16#00FF#);                                         -- key on all 24
      running <= true;
      wait;
   end process;

   -- CD input changes every tick (as cd_top's mix output)
   process (clk1x)
      variable k : integer := 0;
   begin
      if rising_edge(clk1x) and spu_tick = '1' then
         k := k + 1;
         cd_left <= to_signed((k * 37) mod 30000, 16); cd_right <= to_signed((k * 53) mod 30000, 16);
      end if;
   end process;

   meas : process (clk1x)
      alias cdstep is cdstep_s;
      alias stash  is stash_s;
      variable since, n, voiceMax, voiceSum, cdLate, stashUps, last_stash, ticks : integer := 0;
      variable stepPrev : integer := 4;
      variable prevUsed : integer := -1;
      variable cdBad, valids : integer := 0;
   begin
      if rising_edge(clk1x) and running then
         since := since + 1;
         if (spu_tick = '1') then
            ticks := ticks + 1;
            if (stash = 0) then since := 0; end if;   -- not behind: this sample's processing starts now
         end if;
         if (stash > last_stash) then stashUps := stashUps + 1; end if;
         last_stash := stash;
         if (cdstep = 0 and stepPrev /= 0 and ticks > 20) then   -- end of the voice loop: the CD sample is read now
            n := n + 1; voiceSum := voiceSum + since;
            if (since > voiceMax) then voiceMax := since; end if;
         end if;
         if (cdstep = 0 and stepPrev /= 0 and stash > 0) then cdLate := cdLate + 1; end if;
         -- the CD sample used: read at cdProcStep 0 (one clock after the voice loop end), must follow the previous
         if (cdstep = 0 and ticks > 20) then
            if (prevUsed >= 0 and to_integer(used_l) /= (prevUsed + 37) mod 30000) then cdBad := cdBad + 1; end if;
            prevUsed := to_integer(used_l);
         end if;
         if (valid = '1') then valids := valids + 1; end if;
         stepPrev := cdstep;
         if (ticks = SAMPLES + 20) then
            report "LAT " & integer'image(LAT) & " +0.." & integer'image(LATJIT) & ", pitch " & integer'image(PITCH) &
                   ", reverb " & integer'image(REVERB) & ": voice loop ends after the tick: mean " &
                   integer'image(voiceSum / n) & ", max " & integer'image(voiceMax) & " clk1x (budget 768); samples started late " &
                   integer'image(stashUps) & ", CD sample read late " & integer'image(cdLate) & " of " & integer'image(n) &
                   "; CD samples out of order (skip / repeat) " & integer'image(cdBad) & ", sound_valid " &
                   integer'image(valids) & " of " & integer'image(ticks) & " ticks (CDQ " & integer'image(CDQ) & ")";
            done <= true;
         end if;
      end if;
   end process;

end architecture;
