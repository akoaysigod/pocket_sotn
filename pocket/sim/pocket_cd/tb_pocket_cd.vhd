-- Testbench for pocket/src/fpga/core/pocket_cd.vhd
--  * APF side model: data slot table (datatable, 2 clocks latency, entries in any order) and the target command
--    "data slot read" (as core_bridge_cmd + APF behave: done cleared when the command starts, data written to the
--    bridge address as 32-bit words with gaps, done set at the end), any multiple of 2352 bytes into the sector ring;
--    LAT = 1: an SD card on a bad day: 2..8 ms before the data of each command, one in ten 10..30 ms;
--    LAT = 2: worse: 5..12 ms (more than a sector period on average), one in ten 15..40 ms
--  * NSTREAM > 0: then NSTREAM sectors read in order at double speed (one per 6.67 ms, as the drive asks; a late one
--    delays the rest, as the drive waits): sectors late and the total wait are reported
--  * core side model (clk1x): reset until cd_ready, trackinfo capture, sector requests like cd_top.vhd
--    (req + lba, req dropped when ack is seen, 16-bit words counted on cd_hps_write until ack falls)
-- Checks: trackinfo words against MiSTer psx.cpp's layout, every 16-bit word of every requested sector.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_pocket_cd is
   generic
   (
      NUM_REQ  : integer := 60;
      SEED     : integer := 1;
      LAYOUT   : integer := 0;     -- 0: SotN (2 tracks), 1: 5 tracks, 2: no disc, 3: audio CD (track 1 in slot 15)
      MUTATION : integer := 0;     -- testbench self check: 1 = expect the wrong file offset
      LAT      : integer := 0;     -- APF read time: 0 fast (us), 1 slow SD (ms, with spikes), 2 slower
      NSTREAM  : integer := 0;     -- sectors of the double speed stream after the random requests
      STREAM0  : integer := 20000; -- its first LBA (< 0: that many sectors before track 2, the stream crosses into it)
      SLOT_BITS : integer := 3;    -- pocket_cd's ring (generics)
      KMAX     : integer := 4;
      BUF_AW   : integer := 13
   );
end entity;

architecture sim of tb_pocket_cd is

   constant TRACK_ID_BASE : integer := 16;

   signal clk_74a   : std_logic := '0';
   signal clk1x     : std_logic := '0';
   signal reset_n   : std_logic := '0';

   signal cd_ready, has_cd : std_logic;
   signal dt_addr   : std_logic_vector(9 downto 0);
   signal dt_q      : std_logic_vector(31 downto 0) := (others => '0');
   signal dsr_read  : std_logic;
   signal dsr_id    : std_logic_vector(15 downto 0);
   signal dsr_offset, dsr_bridgeaddr, dsr_length : std_logic_vector(31 downto 0);
   signal dsr_done  : std_logic := '0';
   signal bridge_addr    : std_logic_vector(31 downto 0) := (others => '0');
   signal bridge_wr      : std_logic := '0';
   signal bridge_wr_data : std_logic_vector(31 downto 0) := (others => '0');

   signal psx_reset : std_logic := '1';
   signal req       : std_logic := '0';
   signal lba       : std_logic_vector(31 downto 0) := (others => '1');
   signal ack, wr   : std_logic;
   signal data      : std_logic_vector(15 downto 0);
   signal ti_data   : std_logic_vector(31 downto 0);
   signal ti_addr   : std_logic_vector(8 downto 0);
   signal ti_write  : std_logic;

   type tInt is array(natural range <>) of integer;
   type tTable is array(0 to 255) of std_logic_vector(31 downto 0);
   signal datatable : tTable := (others => (others => '0'));

   type tTi is array(0 to 511) of std_logic_vector(31 downto 0);
   signal ti_mem    : tTi := (others => (others => '0'));
   signal ti_count  : integer := 0;

   signal done_sim  : boolean := false;
   signal cmds      : integer := 0;   -- data slot read commands

   function sectors(layout : integer) return tInt is
   begin
      case layout is
         when 0 => return (229020, 18995);                   -- SotN (USA) track sizes / 2352
         when 1 => return (1000, 300, 2, 77, 4501);
         when 3 => return (18995, 1200);
         when others => return (0 => 0);
      end case;
   end function;
   constant SECS : tInt := sectors(LAYOUT);
   constant NT   : integer := SECS'length;

   function first_lba(t : integer) return integer is
      variable a : integer := 150;
   begin
      for i in 0 to t - 1 loop a := a + SECS(i); end loop;
      return a;
   end function;

   function ntracks return integer is
   begin
      if (LAYOUT = 2) then return 0; else return NT; end if;
   end function;

   -- data slot of track t (0 based): audio CD track 1 in slot TRACK_ID_BASE - 1
   function slot(t : integer) return integer is
   begin
      if (LAYOUT = 3 and t = 0) then return TRACK_ID_BASE - 1; else return TRACK_ID_BASE + t; end if;
   end function;

   -- trackinfo audio flag of track t
   function audio(t : integer) return integer is
   begin
      if (t > 0 or LAYOUT = 3) then return 65536; else return 0; end if;
   end function;

   function bcd(v : integer) return integer is begin return (v / 10) * 16 + (v mod 10); end function;
   function msf(l : integer) return integer is
   begin
      return bcd((l / 75) / 60) * 256 + bcd((l / 75) mod 60);
   end function;

   -- file contents: word at byte offset o of slot id
   function fileword(id : integer; o : integer) return std_logic_vector is
      variable x : unsigned(31 downto 0);
   begin
      x := resize(to_unsigned(o / 4, 32) * to_unsigned(40503, 16), 32) + to_unsigned(id * 7919, 32);
      x := x xor shift_right(x, 13);
      return std_logic_vector(x(31 downto 0));
   end function;

   shared variable rnd_s1 : positive := 1;
   shared variable rnd_s2 : positive := 1;

begin

   clk_74a <= not clk_74a after 6734 ps when not done_sim;
   clk1x   <= not clk1x   after 14763 ps when not done_sim;

   idut : entity work.pocket_cd
   generic map (TRACK_ID_BASE => TRACK_ID_BASE, MAX_TRACKS => 24, SLOT_BITS => SLOT_BITS, KMAX => KMAX, BUF_AW => BUF_AW)
   port map
   (
      clk_74a => clk_74a, reset_n_74a => reset_n, cd_ready => cd_ready, has_cd => has_cd,
      datatable_addr => dt_addr, datatable_q => dt_q,
      dsr_read => dsr_read, dsr_id => dsr_id, dsr_offset => dsr_offset, dsr_bridgeaddr => dsr_bridgeaddr,
      dsr_length => dsr_length, dsr_done => dsr_done,
      bridge_addr => bridge_addr, bridge_wr => bridge_wr, bridge_wr_data => bridge_wr_data,
      clk1x => clk1x, psx_reset => psx_reset,
      cd_hps_req => req, cd_hps_lba => lba, cd_hps_ack => ack, cd_hps_write => wr, cd_hps_data => data,
      trackinfo_data => ti_data, trackinfo_addr => ti_addr, trackinfo_write => ti_write
   );

   ---------------------------------------------------------------- APF: data slot table
   process
      variable e : integer := 0;
   begin
      -- other slots around the tracks, track entries in reverse order
      datatable(0) <= x"00000000"; datatable(1) <= x"00000200";              -- slot 0 (instance JSON)
      datatable(2) <= x"00000001"; datatable(3) <= x"00080000";              -- slot 1 (BIOS)
      e := 2;
      if (LAYOUT /= 2) then
         for t in NT - 1 downto 0 loop
            datatable(2 * e)     <= std_logic_vector(to_unsigned(slot(t), 32));
            datatable(2 * e + 1) <= std_logic_vector(to_unsigned(SECS(t) * 2352, 32));
            e := e + 1;
         end loop;
      end if;
      datatable(2 * e) <= x"00000002"; datatable(2 * e + 1) <= x"00020000";  -- memory card
      wait;
   end process;

   -- like mf_datatable port a: address register + output register
   process (clk_74a)
      variable a : std_logic_vector(9 downto 0) := (others => '0');
   begin
      if rising_edge(clk_74a) then
         dt_q <= datatable(to_integer(unsigned(a(7 downto 0))));
         a    := dt_addr;
      end if;
   end process;

   ---------------------------------------------------------------- APF: data slot read command
   process
      variable u    : real;
      variable id, offs, len, t : integer;
      variable last : std_logic := '0';
   begin
      wait until rising_edge(clk_74a);
      if (dsr_read = '1' and last = '0') then
         id   := to_integer(unsigned(dsr_id));
         offs := to_integer(unsigned(dsr_offset));
         len  := to_integer(unsigned(dsr_length));
         assert unsigned(dsr_bridgeaddr) >= x"40000000" and (to_integer(unsigned(dsr_bridgeaddr(27 downto 0))) mod 2352) = 0
                report "bridge address " & to_hstring(dsr_bridgeaddr) severity failure;
         assert len > 0 and len mod 2352 = 0 and to_integer(unsigned(dsr_bridgeaddr(27 downto 0))) + len <= 4 * 2 ** BUF_AW
                report "length " & integer'image(len) & " at " & to_hstring(dsr_bridgeaddr) severity failure;
         cmds <= cmds + 1;
         t := -1;
         for i in 0 to NT - 1 loop
            if (slot(i) = id) then t := i; end if;
         end loop;
         assert t >= 0 report "slot id " & integer'image(id) severity failure;
         assert offs + len <= SECS(t) * 2352 report "read past end of slot" severity failure;
         wait until rising_edge(clk_74a);
         wait until rising_edge(clk_74a);
         dsr_done <= '0';
         uniform(rnd_s1, rnd_s2, u);
         if (LAT = 0) then
            for i in 0 to 50 + integer(u * 400.0) loop wait until rising_edge(clk_74a); end loop;
         else
            if (LAT = 1) then
               if (u < 0.1) then wait for 10 ms + u * 200 ms; else wait for 2 ms + (u - 0.1) * 6.67 ms; end if;
            else
               if (u < 0.1) then wait for 15 ms + u * 250 ms; else wait for 5 ms + (u - 0.1) * 7.78 ms; end if;
            end if;
            wait until rising_edge(clk_74a);
         end if;
         for w in 0 to len / 4 - 1 loop
            bridge_addr    <= std_logic_vector(to_unsigned(to_integer(unsigned(dsr_bridgeaddr)) + 4 * w, 32));
            bridge_wr_data <= fileword(id, offs + 4 * w);
            bridge_wr      <= '1';
            wait until rising_edge(clk_74a);
            bridge_wr      <= '0';
            uniform(rnd_s1, rnd_s2, u);
            for i in 0 to 2 + integer(u * 6.0) loop wait until rising_edge(clk_74a); end loop;
         end loop;
         -- unrelated bridge write elsewhere
         bridge_addr <= x"F8001000"; bridge_wr_data <= x"DEADBEEF"; bridge_wr <= '1';
         wait until rising_edge(clk_74a);
         bridge_wr <= '0';
         for i in 0 to 20 loop wait until rising_edge(clk_74a); end loop;
         dsr_done <= '1';
      end if;
      last := dsr_read;
   end process;

   ---------------------------------------------------------------- core side
   process (clk1x)
      variable rs : std_logic_vector(1 downto 0) := "11";
   begin
      if rising_edge(clk1x) then
         rs        := rs(0) & (not cd_ready);
         psx_reset <= rs(1) or (not reset_n);
         if (ti_write = '1') then
            assert psx_reset = '0' report "trackinfo written during reset" severity failure;
            ti_mem(to_integer(unsigned(ti_addr))) <= ti_data;
            ti_count <= ti_count + 1;
         end if;
      end if;
   end process;

   process
      variable u        : real;
      variable l, t, nwords, errors, zeroes, reads : integer := 0;
      variable expw     : std_logic_vector(31 downto 0);
      variable exph     : std_logic_vector(15 downto 0);
      variable o        : integer;
      variable ack_seen : boolean;
      variable total    : integer;
      variable exp_ti   : integer;
      variable ti_err   : integer := 0;
      variable c0, late : integer := 0;
      variable tnext, treq, stall, maxwait : time := 0 ns;

      procedure check_ti(a : integer; v : integer) is
      begin
         if (to_integer(unsigned(ti_mem(a))) /= v) then
            report "trackinfo word " & integer'image(a) & " = " & integer'image(to_integer(unsigned(ti_mem(a)))) &
                   " expected " & integer'image(v) severity error;
            ti_err := ti_err + 1;
         end if;
      end procedure;
   begin
      rnd_s1 := SEED; rnd_s2 := SEED * 7 + 3;
      reset_n <= '0';
      for i in 0 to 20 loop wait until rising_edge(clk_74a); end loop;
      reset_n <= '1';
      wait until psx_reset = '0';
      for i in 0 to 2000 loop wait until rising_edge(clk1x); end loop;

      ---------------- trackinfo
      total := first_lba(NT);
      if (LAYOUT = 2) then
         assert has_cd = '0' report "has_cd without tracks" severity failure;
         assert ti_count = 0 report "trackinfo without tracks" severity failure;
      else
         assert has_cd = '1' report "has_cd" severity failure;
         assert ti_count = 4 + 4 * NT report "trackinfo word count " & integer'image(ti_count) severity failure;
         check_ti(0, bcd(NT) * 256 + NT);
         check_ti(1, total);
         check_ti(2, msf(total));
         check_ti(3, 0);
         for i in 0 to NT - 1 loop
            if (i = 0) then check_ti(4, 0); else check_ti(4 + 4 * i, first_lba(i)); end if;
            check_ti(5 + 4 * i, first_lba(i + 1) - 1);
            if (i = 0) then check_ti(6, msf(150) + audio(0)); else check_ti(6 + 4 * i, msf(first_lba(i) + 150) + audio(i)); end if;
            check_ti(7 + 4 * i, 0);
         end loop;
         report "trackinfo: " & integer'image(ti_count) & " words, total LBA " & integer'image(total) &
                ", word2 " & to_hstring(ti_mem(2)) & ", track 2 word " & to_hstring(ti_mem(10));
      end if;

      ---------------- sector requests
      for n in 0 to NUM_REQ - 1 loop
         -- edge LBAs first, then random ones (also beyond the end)
         if (LAYOUT = 2) then
            l := n * 13;
         elsif (n < 2 * NT + 4) then
            case n is
               when 0 => l := 0;
               when 1 => l := 149;
               when 2 => l := total;
               when 3 => l := total + 3;
               when others =>
                  if (n mod 2 = 0) then l := first_lba((n - 4) / 2); else l := first_lba((n - 4) / 2 + 1) - 1; end if;
            end case;
         else
            uniform(rnd_s1, rnd_s2, u);
            l := integer(u * real(total + 20));
         end if;

         wait until rising_edge(clk1x);
         lba <= std_logic_vector(to_unsigned(l, 32));
         req <= '1';
         ack_seen := false;
         nwords   := 0;
         loop
            wait until rising_edge(clk1x);
            if (ack = '1' and not ack_seen) then
               ack_seen := true;
               req      <= '0';
               assert wr = '0' report "write together with ack rising" severity failure;
            elsif (ack_seen) then
               if (wr = '1') then
                  -- expected 16-bit word nwords
                  t := -1;
                  for i in 0 to NT - 1 loop
                     if (LAYOUT /= 2 and l >= first_lba(i) and l < first_lba(i + 1)) then t := i; end if;
                  end loop;
                  if (t < 0) then
                     exph := (others => '0');
                  else
                     o := (l - first_lba(t)) * 2352 + (nwords / 2) * 4;
                     if (MUTATION = 1) then o := o + 4; end if;
                     expw := fileword(slot(t), o);
                     if (nwords mod 2 = 0) then exph := expw(15 downto 0); else exph := expw(31 downto 16); end if;
                  end if;
                  if (data /= exph) then
                     if (errors < 10) then
                        report "LBA " & integer'image(l) & " word " & integer'image(nwords) & ": " & to_hstring(data) &
                               " expected " & to_hstring(exph) severity error;
                     end if;
                     errors := errors + 1;
                  end if;
                  nwords := nwords + 1;
               end if;
               exit when ack = '0';
            end if;
         end loop;
         assert nwords = 1176 report "LBA " & integer'image(l) & ": " & integer'image(nwords) & " words" severity failure;
         if (t < 0) then zeroes := zeroes + 1; else reads := reads + 1; end if;
         -- like cd_top: a few clocks before the next request
         uniform(rnd_s1, rnd_s2, u);
         for i in 0 to 15 + integer(u * 100.0) loop wait until rising_edge(clk1x); end loop;
      end loop;

      report "DONE requests=" & integer'image(NUM_REQ) & " slot_reads=" & integer'image(reads) & " zero_sectors=" &
             integer'image(zeroes) & " word_errors=" & integer'image(errors) & " trackinfo_errors=" & integer'image(ti_err) &
             " commands=" & integer'image(cmds);

      ---------------- double speed stream: sector i asked for at max(T0 + i * 6.667 ms, when sector i - 1 was in)
      if (NSTREAM > 0 and LAYOUT /= 2) then
         c0 := cmds;
         tnext := now;
         for i in 0 to NSTREAM - 1 loop
            if (STREAM0 < 0) then l := first_lba(1) + STREAM0 + i; else l := STREAM0 + i; end if;
            if (now < tnext) then wait for tnext - now; end if;
            wait until rising_edge(clk1x);
            treq := now;
            lba <= std_logic_vector(to_unsigned(l, 32));
            req <= '1';
            ack_seen := false;
            nwords   := 0;
            loop
               wait until rising_edge(clk1x);
               if (ack = '1' and not ack_seen) then
                  ack_seen := true;
                  req      <= '0';
               elsif (ack_seen) then
                  if (wr = '1') then
                     t := 0;
                     for k in 0 to NT - 1 loop
                        if (l >= first_lba(k)) then t := k; end if;
                     end loop;
                     o := (l - first_lba(t)) * 2352 + (nwords / 2) * 4;
                     expw := fileword(slot(t), o);
                     if (nwords mod 2 = 0) then exph := expw(15 downto 0); else exph := expw(31 downto 16); end if;
                     if (data /= exph) then errors := errors + 1; end if;
                     nwords := nwords + 1;
                  end if;
                  exit when ack = '0';
               end if;
            end loop;
            -- the drive needs the sector before its next one is due (period minus the ~4 us its countdown keeps)
            if (now - treq > 6662 us) then
               late := late + 1;
               stall := stall + (now - treq - 6662 us);
            end if;
            if (now - treq > maxwait) then maxwait := now - treq; end if;
            tnext := treq + 6667 us;
            if (now > tnext) then tnext := now; end if;
         end loop;
         report "STREAM sectors=" & integer'image(NSTREAM) & " late=" & integer'image(late) & " total_wait=" &
                time'image(stall) & " max_request_to_data=" & time'image(maxwait) & " commands=" &
                integer'image(cmds - c0) & " word_errors=" & integer'image(errors);
      end if;
      if (errors = 0 and ti_err = 0) then report "PASS"; else report "FAIL"; end if;
      done_sim <= true;
      wait;
   end process;

end architecture;
