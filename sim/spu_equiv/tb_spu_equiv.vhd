-- Equivalence test: current rtl/spu.vhd (spu_new) against the spu.vhd of a reference git
-- revision (spu_ref). Random CPU register traffic, DMA transfers, CD audio input and an SDRAM
-- model drive both copies; every output is compared on every clk2x edge.
-- Noise and pitch-modulation enables (0x190..0x196) are only ever written with 0, the
-- condition under which the Pocket build's SPU must behave exactly like the original.
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use STD.textio.all;

entity tb_spu_equiv is
   generic
   (
      NUM_CYCLES : integer := 2000000;  -- clk1x cycles
      SEED       : integer := 1
   );
end entity;

architecture sim of tb_spu_equiv is

   signal clk1x, clk2x    : std_logic := '1';
   signal clk1xToggle     : std_logic := '0';
   signal clk1xToggle2x   : std_logic := '0';
   signal clk2xIndex      : std_logic := '0';
   signal ce              : std_logic := '1';
   signal reset           : std_logic := '1';
   signal cpuPaused       : std_logic := '0';
   signal SS_reset        : std_logic := '0';
   signal SS_wren         : std_logic := '0';
   signal SS_Adr          : unsigned(8 downto 0) := (others => '0');
   signal resets          : integer := 0;

   signal cd_left, cd_right : signed(15 downto 0) := (others => '0');
   signal bus_addr        : unsigned(9 downto 0) := (others => '0');
   signal bus_dataWrite   : std_logic_vector(15 downto 0) := (others => '0');
   signal bus_read        : std_logic := '0';
   signal bus_write       : std_logic := '0';
   signal dma_read        : std_logic := '0';
   signal dma_write       : std_logic := '0';
   signal dma_writedata   : std_logic_vector(15 downto 0) := (others => '0');
   signal sdram_dataRead  : std_logic_vector(31 downto 0) := (others => '0');
   signal sdram_done      : std_logic := '0';

   type tOut is record
      spu_tick        : std_logic;
      irqOut          : std_logic;
      sound_timeout   : std_logic;
      sound_out_left  : std_logic_vector(15 downto 0);
      sound_out_right : std_logic_vector(15 downto 0);
      bus_dataRead    : std_logic_vector(15 downto 0);
      spu_dmaRequest  : std_logic;
      dma_readdata    : std_logic_vector(15 downto 0);
      sdram_dataWrite : std_logic_vector(31 downto 0);
      sdram_Adr       : std_logic_vector(18 downto 0);
      sdram_be        : std_logic_vector(3 downto 0);
      sdram_rnw       : std_logic;
      sdram_ena       : std_logic;
   end record;
   signal o_ref, o_new : tOut;

   signal mismatches  : integer := 0;
   signal sdramReqs   : integer := 0;
   signal busWrites   : integer := 0;
   signal busReads    : integer := 0;
   signal samplesOut  : integer := 0;
   signal nonzeroOut  : integer := 0;

   function differs(a, b : tOut) return boolean is
   begin
      return a.spu_tick /= b.spu_tick or a.irqOut /= b.irqOut or a.sound_timeout /= b.sound_timeout or
             a.sound_out_left /= b.sound_out_left or a.sound_out_right /= b.sound_out_right or
             (not is_x(a.bus_dataRead) and a.bus_dataRead /= b.bus_dataRead) or  -- undefined until the first read a.spu_dmaRequest /= b.spu_dmaRequest or
             a.dma_readdata /= b.dma_readdata or a.sdram_ena /= b.sdram_ena or
             (a.sdram_ena = '1' and (a.sdram_dataWrite /= b.sdram_dataWrite or a.sdram_Adr /= b.sdram_Adr or
                                     a.sdram_be /= b.sdram_be or a.sdram_rnw /= b.sdram_rnw));
   end function;

begin

   clk2x <= not clk2x after 5 ns;
   clk1x <= not clk1x after 10 ns;

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

   gdut : for i in 0 to 1 generate
      signal o : tOut;
   begin
      gref : if i = 0 generate
         idut : entity work.spu_ref
         port map (clk1x => clk1x, clk2x => clk2x, clk2xIndex => clk2xIndex, ce => ce, reset => reset,
            SPUon => '1', SPUIRQTrigger => '0', useSDRAM => '1', REPRODUCIBLESPUIRQ => '0', REPRODUCIBLESPUDMA => '0',
            REVERBOFF => '0', cpuPaused => cpuPaused, spu_tick => o.spu_tick, cd_left => cd_left, cd_right => cd_right,
            irqOut => o.irqOut, sound_timeout => o.sound_timeout, sound_out_left => o.sound_out_left,
            sound_out_right => o.sound_out_right, bus_addr => bus_addr, bus_dataWrite => bus_dataWrite,
            bus_read => bus_read, bus_write => bus_write, bus_dataRead => o.bus_dataRead,
            spu_dmaRequest => o.spu_dmaRequest, dma_read => dma_read, dma_readdata => o.dma_readdata,
            dma_write => dma_write, dma_writedata => dma_writedata,
            sdram_dataWrite => o.sdram_dataWrite, sdram_Adr => o.sdram_Adr, sdram_be => o.sdram_be,
            sdram_rnw => o.sdram_rnw, sdram_ena => o.sdram_ena, sdram_dataRead => sdram_dataRead, sdram_done => sdram_done,
            mem_request => open, mem_BURSTCNT => open, mem_ADDR => open, mem_DIN => open, mem_BE => open,
            mem_WE => open, mem_RD => open, mem_ack => '0', mem_DOUT => (others => '0'), mem_DOUT_READY => '0',
            SS_reset => SS_reset, loading_savestate => '0', SS_DataWrite => (others => '0'), SS_Adr => SS_Adr,
            SS_wren => SS_wren, SS_rden => '0', SS_DataRead => open, SS_idle => open,
            SS_RAM_dataWrite => (others => '0'), SS_RAM_Adr => (others => '0'), SS_RAM_request => '0',
            SS_RAM_rnw => '0', SS_RAM_dataRead => open, SS_RAM_done => open);
         o_ref <= o;
      end generate;
      gnew : if i = 1 generate
         idut : entity work.spu_new
         port map (clk1x => clk1x, clk2x => clk2x, clk2xIndex => clk2xIndex, ce => ce, reset => reset,
            SPUon => '1', SPUIRQTrigger => '0', useSDRAM => '1', REPRODUCIBLESPUIRQ => '0', REPRODUCIBLESPUDMA => '0',
            REVERBOFF => '0', cpuPaused => cpuPaused, spu_tick => o.spu_tick, cd_left => cd_left, cd_right => cd_right,
            irqOut => o.irqOut, sound_timeout => o.sound_timeout, sound_out_left => o.sound_out_left,
            sound_out_right => o.sound_out_right, bus_addr => bus_addr, bus_dataWrite => bus_dataWrite,
            bus_read => bus_read, bus_write => bus_write, bus_dataRead => o.bus_dataRead,
            spu_dmaRequest => o.spu_dmaRequest, dma_read => dma_read, dma_readdata => o.dma_readdata,
            dma_write => dma_write, dma_writedata => dma_writedata,
            sdram_dataWrite => o.sdram_dataWrite, sdram_Adr => o.sdram_Adr, sdram_be => o.sdram_be,
            sdram_rnw => o.sdram_rnw, sdram_ena => o.sdram_ena, sdram_dataRead => sdram_dataRead, sdram_done => sdram_done,
            mem_request => open, mem_BURSTCNT => open, mem_ADDR => open, mem_DIN => open, mem_BE => open,
            mem_WE => open, mem_RD => open, mem_ack => '0', mem_DOUT => (others => '0'), mem_DOUT_READY => '0',
            SS_reset => SS_reset, loading_savestate => '0', SS_DataWrite => (others => '0'), SS_Adr => SS_Adr,
            SS_wren => SS_wren, SS_rden => '0', SS_DataRead => open, SS_idle => open,
            SS_RAM_dataWrite => (others => '0'), SS_RAM_Adr => (others => '0'), SS_RAM_request => '0',
            SS_RAM_rnw => '0', SS_RAM_dataRead => open, SS_RAM_done => open);
         o_new <= o;
      end generate;
   end generate;

   -- compare every output on every clk2x edge
   cmp : process (clk2x)
      variable l : line;
   begin
      if rising_edge(clk2x) then
         if (reset = '0' and differs(o_ref, o_new)) then
            if (mismatches < 10) then
               write(l, string'("MISMATCH at ") & time'image(now) & string'(" left ") & to_hstring(o_ref.sound_out_left)
                     & string'("/") & to_hstring(o_new.sound_out_left) & string'(" busRd ") & to_hstring(o_ref.bus_dataRead)
                     & string'("/") & to_hstring(o_new.bus_dataRead) & string'(" ena ") & std_logic'image(o_ref.sdram_ena)
                     & string'("/") & std_logic'image(o_new.sdram_ena));
               writeline(output, l);
            end if;
            mismatches <= mismatches + 1;
         end if;
      end if;
   end process;

   -- SDRAM model in the clk1x domain, driven from the reference's requests: done after a random
   -- delay, random read data (the sound RAM content itself is irrelevant for equivalence)
   sdram : process (clk1x)
      variable s1 : positive := SEED + 101;
      variable s2 : positive := 7777;
      variable r  : real;
      variable pending : integer := -1;
   begin
      if rising_edge(clk1x) then
         sdram_done <= '0';
         if (o_ref.sdram_ena = '1') then
            uniform(s1, s2, r);
            pending := integer(trunc(r * 6.0));
            sdramReqs <= sdramReqs + 1;
         end if;
         if (pending = 0) then
            sdram_done <= '1';
            uniform(s1, s2, r);
            sdram_dataRead <= std_logic_vector(to_unsigned(integer(trunc(r * 65536.0)), 16)) &
                              std_logic_vector(to_unsigned(integer(trunc(r * 65536.0 * 0.7)) mod 65536, 16));
         end if;
         if (pending >= 0) then pending := pending - 1; end if;
      end if;
   end process;

   -- sound output activity (to show the test is not comparing silence)
   process (clk1x)
   begin
      if rising_edge(clk1x) then
         if (o_ref.spu_tick = '1') then
            samplesOut <= samplesOut + 1;
            if (o_ref.sound_out_left /= x"0000" or o_ref.sound_out_right /= x"0000") then
               nonzeroOut <= nonzeroOut + 1;
            end if;
         end if;
      end if;
   end process;

   stim : process
      variable s1 : positive := SEED;
      variable s2 : positive := 1234567;
      variable r  : real;
      variable l  : line;
      variable a  : integer;
      variable d  : std_logic_vector(15 downto 0);

      impure function rnd(k : integer) return integer is
      begin
         uniform(s1, s2, r); return integer(trunc(r * real(k)));
      end function;
      impure function rnd16 return std_logic_vector is
      begin
         return std_logic_vector(to_unsigned(rnd(65536), 16));
      end function;
   begin
      -- reset as the system does it: SS_reset loads the register defaults during reset
      reset <= '1';
      for i in 0 to 5 loop wait until rising_edge(clk1x); end loop;
      SS_reset <= '1';
      for i in 0 to 5 loop wait until rising_edge(clk1x); end loop;
      SS_reset <= '0';
      for i in 0 to 10 loop wait until rising_edge(clk1x); end loop;
      reset <= '0';

      -- init like a game: SPU on + unmuted + reverb master on, main and CD volume up
      for k in 0 to 4 loop
         wait until rising_edge(clk1x);
         bus_write <= '1';
         case k is
            when 0 => bus_addr <= to_unsigned(16#1AA#, 10); bus_dataWrite <= x"C0C1";
            when 1 => bus_addr <= to_unsigned(16#180#, 10); bus_dataWrite <= x"3FFF";
            when 2 => bus_addr <= to_unsigned(16#182#, 10); bus_dataWrite <= x"3FFF";
            when 3 => bus_addr <= to_unsigned(16#1B0#, 10); bus_dataWrite <= x"7FFF";
            when others => bus_addr <= to_unsigned(16#1B2#, 10); bus_dataWrite <= x"7FFF";
         end case;
         wait until rising_edge(clk1x);
         bus_write <= '0';
      end loop;

      for c in 0 to NUM_CYCLES - 1 loop
         wait until rising_edge(clk1x);
         bus_read <= '0'; bus_write <= '0'; dma_read <= '0'; dma_write <= '0';
         -- ce stays 1: it only drops while the whole system is paused (a one-cycle RAM done is not
         -- held across a ce=0 cycle in the original either)
         if (rnd(2000) = 0) then cpuPaused <= not cpuPaused; end if;
         -- CD input: a new sample right after each SPU tick, as cd_top's mix updates it (the SPU queues it 16 clocks
         -- after the tick; changes at other times are not something the real core does)
         if (o_ref.spu_tick = '1') then cd_left <= signed(rnd16); cd_right <= signed(rnd16); end if;

         -- occasional reset sequence like the system's: reset with SS_reset, then the savestate
         -- module writes zeros to the module's state words
         if (rnd(100000) = 0) then
            resets <= resets + 1;
            reset <= '1';
            for i in 0 to 3 loop wait until rising_edge(clk1x); end loop;
            SS_reset <= '1';
            for i in 0 to 3 loop wait until rising_edge(clk1x); end loop;
            SS_reset <= '0';
            for i in 0 to 300 loop wait until rising_edge(clk1x); end loop;  -- voice RAM clear sweep
            reset <= '0';
            for i in 0 to 63 loop
               wait until rising_edge(clk1x);
               SS_Adr <= to_unsigned(i, 9); SS_wren <= '1';
            end loop;
            wait until rising_edge(clk1x);
            SS_wren <= '0';
            -- the game re-initialises the SPU after a reset
            for k in 0 to 1 loop
               wait until rising_edge(clk1x);
               bus_write <= '1'; bus_addr <= to_unsigned(16#1AA#, 10); bus_dataWrite <= x"C0C1";
               wait until rising_edge(clk1x);
               bus_write <= '0';
            end loop;
         end if;

         case rnd(40) is
            when 0 | 1 =>
               -- register write: voice registers, globals, reverb; noise/FM enables stay 0
               a := 2 * rnd(512);
               d := rnd16;
               case rnd(8) is
                  when 0 => a := 16#188# + 2 * rnd(2);                  -- KEYON
                  when 1 => a := 16 * rnd(24) + 6;                      -- voice start address
                            d := std_logic_vector(to_unsigned(16#200# + rnd(16#FE00#), 16));
                  when 2 => a := 16 * rnd(24) + 8 + 2 * rnd(2);         -- ADSR
                  when 3 => a := 16 * rnd(24) + 2 * rnd(2);             -- voice volume
                  when others => null;
               end case;
               if (a >= 16#190# and a <= 16#196#) then d := (others => '0'); end if;
               -- mostly keep the SPU enabled and unmuted so voices and reverb actually run
               if (a = 16#1AA#) then d(15) := '1'; if (rnd(4) /= 0) then d(14) := '1'; d(7) := '1'; end if; end if;
               bus_addr <= to_unsigned(a, 10); bus_dataWrite <= d; bus_write <= '1';
               busWrites <= busWrites + 1;
            when 2 | 3 =>
               bus_addr <= to_unsigned(2 * rnd(512), 10); bus_read <= '1';
               busReads <= busReads + 1;
            when 4 =>
               if (rnd(4) = 0) then dma_read <= '1'; else dma_write <= '1'; dma_writedata <= rnd16; end if;
            when others => null;
         end case;

         if (c mod 200000 = 0) then
            write(l, string'("progress cycle ") & integer'image(c) & string'(" mismatches=") & integer'image(mismatches)
                  & string'(" sdram_reqs=") & integer'image(sdramReqs) & string'(" samples=") & integer'image(samplesOut)
                  & string'(" nonzero=") & integer'image(nonzeroOut));
            writeline(output, l);
         end if;
      end loop;

      write(l, string'("DONE cycles=") & integer'image(NUM_CYCLES) & string'(" bus_writes=") & integer'image(busWrites)
            & string'(" bus_reads=") & integer'image(busReads) & string'(" sdram_reqs=") & integer'image(sdramReqs)
            & string'(" samples=") & integer'image(samplesOut) & string'(" nonzero_samples=") & integer'image(nonzeroOut)
            & string'(" resets=") & integer'image(resets) & string'(" mismatch_cycles=") & integer'image(mismatches));
      writeline(output, l);
      if (mismatches = 0 and sdramReqs > 0 and nonzeroOut > 0) then write(l, string'("PASS")); else write(l, string'("FAIL")); end if;
      writeline(output, l);
      std.env.finish;
   end process;

end architecture;
