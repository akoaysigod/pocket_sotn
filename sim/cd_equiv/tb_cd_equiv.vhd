-- CD controller equivalence test: rtl/cd_top.vhd ("new") against the frozen reference ref/pocket/rtl/cd_top.vhd
-- ("ref"), both driven by the same CPU model and each fed by its own sector supplier (same disc, same delays).
-- Every output of the two instances is compared on every clock (lockstep).
--  * disc (synthetic, structure as pocket_cd serves it): track 1 = MODE2 data, LBA 0..149 pregap (zeros), then
--    T1N sectors with BCD MSF headers and subheaders: form 1 data, and in the XA zone form 2 XA-ADPCM audio sectors
--    (file 1, channels 0 / 1) interleaved with data; track 2 = audio (T2N sectors incl. its 150 sector pregap)
--  * trackinfo written like pocket/src/fpga/core/pocket_cd.vhd sends it
--  * CPU model scripts (generic SCRIPT): 0 BIOS-like disc boot, 1 SotN-like loading + XA streaming, 2 CD audio
--    (Play, reports, GetlocP: the sim/cd_cdda sequence), 3 random commands (SEED, NCMD), 4 SotN music (XA only
--    zone as SotN's music file, Setfilter + SetMode 0xC8 + ReadS, XAMS ms, GetlocL polls); interrupts serviced by
--    polling, sector data requested and read through dma_read; with RANDCE, random ce = 0 bursts (core pauses;
--    in the first 2 s they make the new-disc pulse flicker and both controllers repeat the lid-open INT5)
--  * XA / CD-DA monitor (both instances): sector starts with the output fifo level (the buffer left when a sector
--    arrives: 1 sample in the original), sectors skipped as near full, output fifo underruns
--  * FREEZE: while the new instance's CD engine works (signal eng_freeze), the clock of everything else is held, so
--    engine latency is invisible and the comparison stays cycle exact
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package cd_equiv_pkg is
   -- one byte of the synthetic disc (absolute LBA, byte 0..2351 of the raw sector)
   -- xaall /= 0: every sector of the XA zone is XA audio, channel = LBA mod 8 (as SotN's music files)
   function disc_byte(lba : integer; i : integer; t1n : integer; xaall : integer := 0) return integer;
   constant XA0 : integer := 150 + 1000;   -- XA zone in track 1
   constant XA1 : integer := 150 + 1600;
end package;

package body cd_equiv_pkg is
   function bcd(n : integer) return integer is
   begin
      return (n / 10) * 16 + n mod 10;
   end function;

   function hash(a, b : integer) return integer is
      variable x : unsigned(31 downto 0);
   begin
      x := to_unsigned(a mod 65536, 16) * to_unsigned(40503, 16);
      x := x xor to_unsigned(((b mod 4096) * 2654 + b / 4096) mod 1048576, 32);
      x := x(15 downto 0) * to_unsigned(9973, 16);
      return to_integer(x(23 downto 16));
   end function;

   function disc_byte(lba : integer; i : integer; t1n : integer; xaall : integer := 0) return integer is
      variable s2      : integer := 150 + t1n;
      variable m, s, f : integer;
      variable kind    : integer;   -- 0 form 1 data, 1 XA audio ch 0, 2 XA audio ch 1
      variable submode : integer;
      variable coding  : integer;
      variable chan    : integer;
      variable g, gi   : integer;
   begin
      if (lba < 150) then
         return 0;
      end if;
      if (lba >= s2) then
         -- audio: left = (LBA mod 64) << 10 | sample, right = left xor 0x5A5A (16 bit little endian)
         s := (i / 4);
         if (i mod 4 = 0) then return (s mod 256); end if;
         if (i mod 4 = 1) then return ((lba mod 64) * 4 + s / 256); end if;
         if (i mod 4 = 2) then return to_integer(to_unsigned(s mod 256, 8) xor x"5A"); end if;
         return to_integer(to_unsigned((lba mod 64) * 4 + s / 256, 8) xor x"5A");
      end if;
      if (i = 0 or i = 11) then return 0; end if;
      if (i < 11) then return 255; end if;
      m := lba / 4500; s := (lba / 75) mod 60; f := lba mod 75;
      if (i = 12) then return bcd(m); end if;
      if (i = 13) then return bcd(s); end if;
      if (i = 14) then return bcd(f); end if;
      if (i = 15) then return 2; end if;
      kind := 0;
      if (lba >= XA0 and lba < XA1) then
         if (lba mod 8 = 0) then kind := 1; elsif (lba mod 8 = 4) then kind := 2; end if;
         if (xaall /= 0) then kind := 1 + lba mod 8; end if;
      end if;
      if (kind = 0) then
         chan := 0; coding := 0;
         submode := 16#08#;                                  -- data
         if (lba mod 97 = 13) then submode := 16#89#; end if;   -- data + EOF + EOR
      else
         chan := kind - 1;
         submode := 16#64#;                                  -- realtime, form 2, audio
         if (lba mod 128 = 120 and xaall = 0) then submode := 16#E4#; end if; -- + EOF
         if (kind = 1 or xaall /= 0) then coding := 16#01#; else coding := 16#00#; end if;   -- stereo / mono, 37.8 kHz, 4 bit
      end if;
      if (i = 16 or i = 20) then return 1; end if;          -- file
      if (i = 17 or i = 21) then return chan; end if;
      if (i = 18 or i = 22) then return submode; end if;
      if (i = 19 or i = 23) then return coding; end if;
      if (kind /= 0 and i >= 24 and i < 24 + 18 * 128) then
         g  := (i - 24) / 128;
         gi := (i - 24) mod 128;
         if (gi < 16) then   -- sound parameters: shift 0..11, filter 0..3
            return (hash(lba * 18 + g, gi) mod 4) * 16 + (hash(lba, g * 16 + gi) mod 12);
         end if;
      end if;
      return hash(lba, i);
   end function;
end package body;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.cd_equiv_pkg.all;

-- sector supplier like pocket_cd: optionally a window of real sectors (DISCWIN: one line of 4704 hex digits per raw
-- sector, absolute LBAs WINLBA .. WINLBA + WINN - 1, e.g. from the user's own disc image, never in git), and every
-- SLOWEVERY-th read slow (SLOWUS us instead of 60 .. 650 us: an SD card hiccup)
entity cd_disc is
   generic
   (
      T1N       : integer;
      XAALL     : integer := 0;
      DISCWIN   : string  := "";
      WINLBA    : integer := 0;
      WINN      : integer := 1;
      SLOWEVERY : integer := 0;
      SLOWUS    : integer := 8000
   );
   port
   (
      clk   : in  std_logic;
      req   : in  std_logic;
      lba   : in  std_logic_vector(31 downto 0);
      ack   : out std_logic := '0';
      wr    : out std_logic := '0';
      data  : out std_logic_vector(15 downto 0) := (others => '0')
   );
end entity;

architecture sim of cd_disc is
begin
   process
      type tmem is array (natural range <>) of character;
      variable mem   : tmem(0 to WINN * 2352 - 1);
      file fwin      : text;
      variable ln    : line;
      variable c     : character;
      variable v, nb : integer;
      variable l     : integer;
      variable delay : integer;
      variable reads : integer := 0;
      variable inwin : boolean;
      impure function byte(i : integer) return integer is
      begin
         if (inwin) then return character'pos(mem((l - WINLBA) * 2352 + i)); end if;
         return disc_byte(l, i, T1N, XAALL);
      end function;
      function hexval(ch : character) return integer is
      begin
         if (ch >= '0' and ch <= '9') then return character'pos(ch) - character'pos('0'); end if;
         if (ch >= 'a' and ch <= 'f') then return character'pos(ch) - character'pos('a') + 10; end if;
         return character'pos(ch) - character'pos('A') + 10;
      end function;
   begin
      if (DISCWIN'length > 0) then
         file_open(fwin, DISCWIN, read_mode);
         for sct in 0 to WINN - 1 loop
            readline(fwin, ln);
            for i in 0 to 2351 loop
               read(ln, c); v := hexval(c) * 16; read(ln, c); v := v + hexval(c);
               mem(sct * 2352 + i) := character'val(v);
            end loop;
         end loop;
         file_close(fwin);
      end if;
      loop
         wait until rising_edge(clk);
         if (req = '1') then
            l     := to_integer(unsigned(lba(19 downto 0)));
            inwin := DISCWIN'length > 0 and l >= WINLBA and l < WINLBA + WINN;
            delay := 2000 + ((l mod 20000) * 7919) mod 20000;   -- 60 .. 650 us, as APF reads take (any lba)
            reads := reads + 1;
            if (SLOWEVERY > 0 and reads mod SLOWEVERY = 0) then delay := SLOWUS * 34; end if;
            for i in 1 to delay loop wait until rising_edge(clk); end loop;
            ack <= '1';
            for i in 1 to 3 loop wait until rising_edge(clk); end loop;
            for w in 0 to 1175 loop
               wr   <= '1';
               data <= std_logic_vector(to_unsigned(byte(2 * w + 1), 8)) & std_logic_vector(to_unsigned(byte(2 * w), 8));
               wait until rising_edge(clk);
               wr   <= '0';
               wait until rising_edge(clk);
            end loop;
            for i in 1 to 3 loop wait until rising_edge(clk); end loop;
            ack <= '0';
            wait until rising_edge(clk);
         end if;
      end loop;
   end process;
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.cd_equiv_pkg.all;
use std.textio.all;

use ieee.math_real.all;

entity tb_cd_equiv is
   generic
   (
      SCRIPT : integer := 3;      -- 0 boot, 1 sotn, 2 cdda, 3 random, 4 SotN music (XA only zone, long play),
                                  -- 5 a song from real sectors (DISCWIN, PLAYLBA, XAFILE, XACHAN)
      SEED   : integer := 1;
      NCMD   : integer := 200;    -- random script: number of commands
      HASCD  : integer := 1;
      RANDCE : integer := 1;      -- random ce = 0 bursts
      FREEZE : boolean := false;  -- hold the world while the new instance's engine works
      CPUSRC : integer := 0;      -- instance the CPU model reads (0 ref, 1 new): with 1 and no FREEZE the new one
                                  -- runs as in hardware (engine time visible); compare the summary with a ref run
      DBGMS  : integer := -1;     -- >= 0: from this ms on, trace the new instance's parameter fifo and sector requests
      T1N    : integer := 3000;   -- sectors of the data track (after its 150 sector pregap)
      T2N    : integer := 2000;   -- sectors of the audio track (incl. 150 pregap)
      XAMS   : integer := 2000;   -- script 4 / 5: ms of XA play
      -- script 5: real sectors (see cd_disc), XA file / channel to play from absolute LBA PLAYLBA; OUTPRE: write each
      -- instance's output (left right per spu_tick, from its first non-zero sample) to OUTPRE0.txt / OUTPRE1.txt
      DISCWIN   : string  := "";
      WINLBA    : integer := 0;
      WINN      : integer := 1;
      SLOWEVERY : integer := 0;
      SLOWUS    : integer := 8000;
      PLAYLBA   : integer := 0;
      XAFILE    : integer := 1;
      XACHAN    : integer := 0;
      OUTPRE    : string  := ""
   );
end entity;

architecture sim of tb_cd_equiv is

   constant T2START : integer := 150 + T1N;
   constant TOTAL   : integer := T2START + T2N;

   signal clk1x     : std_logic := '0';   -- the world (CPU model, suppliers, ref, new without its engine)
   signal clkeng    : std_logic := '0';   -- the new instance's engine (never held)
   signal frz       : std_logic := '0';   -- new instance: engine busy (only with FREEZE)
   signal busy_new  : std_logic := '0';
   signal reset     : std_logic := '1';
   signal ss_reset  : std_logic := '0';
   signal ce        : std_logic := '1';
   signal hascd_s   : std_logic := '1';
   signal spu_tick  : std_logic := '0';
   signal tickcnt   : integer range 0 to 767 := 0;
   signal finish    : boolean := false;
   signal done      : boolean := false;

   signal bus_addr      : unsigned(3 downto 0) := (others => '0');
   signal bus_dataWrite : std_logic_vector(7 downto 0) := (others => '0');
   signal bus_read      : std_logic := '0';
   signal bus_write     : std_logic := '0';
   signal dma_read      : std_logic := '0';

   signal ti_data  : std_logic_vector(31 downto 0) := (others => '0');
   signal ti_addr  : std_logic_vector(8 downto 0) := (others => '0');
   signal ti_write : std_logic := '0';

   type tslv32 is array (0 to 1) of std_logic_vector(31 downto 0);
   type tslv16 is array (0 to 1) of std_logic_vector(15 downto 0);
   type tslv8  is array (0 to 1) of std_logic_vector(7 downto 0);
   type tsig16 is array (0 to 1) of signed(15 downto 0);
   type tuns20 is array (0 to 1) of unsigned(19 downto 0);
   type tslv2  is array (0 to 1) of std_logic_vector(1 downto 0);
   type tsl    is array (0 to 1) of std_logic;
   signal o_region : tslv2;
   signal o_pauseCD, o_pauseIdle, o_fullyIdle, o_cdSlow, o_error, o_irq, o_hpsreq, o_resetFromCD, o_ssidle : tsl;
   signal o_lbadisp : tuns20;
   signal o_left, o_right : tsig16;
   signal o_busread, o_dmaread : tslv8;
   signal o_hpslba : tslv32;
   signal hps_ack, hps_wr : tsl;
   signal hps_data : tslv16;

   signal mismatches : integer := 0;
   signal audio_n    : integer := 0;
   signal audio_sum  : integer := 0;
   signal cycles     : integer := 0;
   signal frozen     : integer := 0;   -- engine cycles with the world held
   signal req_1      : std_logic := '0';
   signal req_n      : integer := 0;
   signal req_first  : integer := 0;
   signal req_last   : integer := 0;

begin

   hascd_s <= '1' when HASCD /= 0 else '0';
   frz     <= busy_new when FREEZE else '0';

   -- clocks: 33.8688 MHz; the world's rising edge is skipped while frz = '1' (both edges in the same delta)
   process
   begin
      while not done loop
         wait for 14763 ps;
         clkeng <= '1';
         if (frz = '0') then clk1x <= '1'; end if;
         wait for 14763 ps;
         clkeng <= '0';
         clk1x  <= '0';
      end loop;
      wait;
   end process;

   process (clkeng)
   begin
      if rising_edge(clkeng) and frz = '1' then frozen <= frozen + 1; end if;
   end process;

   -- engine statistics: which events start it (new instance, FREEZE only)
   gstat : if FREEZE generate
      type tcnt is array (0 to 31) of integer;
      signal starts : tcnt := (others => 0);
      signal runs   : integer := 0;
   begin
      process (clkeng)
         alias ev    is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl_sim.ictrl.ev_now : std_logic_vector(31 downto 0) >>;
         alias start is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl_sim.ictrl.eng_start : std_logic >>;
         variable c : tcnt;
      begin
         if rising_edge(clkeng) then
            if (start = '1' and busy_new = '0') then
               runs <= runs + 1;
               c := starts;
               for i in 0 to 15 loop
                  if (ev(i) = '1') then c(i) := c(i) + 1; end if;
               end loop;
               starts <= c;
            end if;
            if (finish) then
               for i in 0 to 15 loop
                  if (starts(i) > 0) then report "engine starts with event bit " & integer'image(i) & ": " & integer'image(starts(i)); end if;
               end loop;
            end if;
         end if;
      end process;
      process
      begin
         wait for 50 ms;
         loop
            report "progress: world cycles " & integer'image(cycles) & ", engine runs " & integer'image(runs) & ", held " & integer'image(frozen);
            wait for 50 ms;
         end loop;
      end process;
   end generate;

   -- 44.1 kHz SPU tick
   process (clk1x)
   begin
      if rising_edge(clk1x) then
         cycles   <= cycles + 1;
         spu_tick <= '0';
         if (tickcnt = 767) then
            tickcnt  <= 0;
            spu_tick <= '1';
         else
            tickcnt <= tickcnt + 1;
         end if;
      end if;
   end process;

   ginst : for i in 0 to 1 generate
   begin
      gref : if i = 0 generate
         icd : entity work.cd_top_ref
         port map
         (
            clk1x => clk1x, ce => ce, reset => reset,
            INSTANTSEEK => '0', FORCECDSPEED => "000", LIMITREADSPEED => '0', hasCD => hascd_s, LIDopen => '0',
            fastCD => '0', testSeek => '0', pauseOnCDSlow => '0', region => "00", region_out => o_region(i),
            pauseCD => o_pauseCD(i), Pause_idle_cd => o_pauseIdle(i), fullyIdle => o_fullyIdle(i), cdSlow => o_cdSlow(i),
            error => o_error(i), LBAdisplay => o_lbadisp(i), irqOut => o_irq(i),
            spu_tick => spu_tick, cd_left => o_left(i), cd_right => o_right(i), mdec_idle => '1',
            bus_addr => bus_addr, bus_dataWrite => bus_dataWrite, bus_read => bus_read, bus_write => bus_write,
            bus_dataRead => o_busread(i), dma_read => dma_read, dma_readdata => o_dmaread(i),
            cd_hps_req => o_hpsreq(i), cd_hps_lba => o_hpslba(i), cd_hps_lba_sim => open,
            cd_hps_ack => hps_ack(i), cd_hps_write => hps_wr(i), cd_hps_data => hps_data(i),
            trackinfo_data => ti_data, trackinfo_addr => ti_addr, trackinfo_write => ti_write,
            resetFromCD => o_resetFromCD(i),
            SS_reset => ss_reset, SS_DataWrite => (others => '0'), SS_Adr => (others => '0'), SS_wren => '0',
            SS_rden => '0', SS_DataRead => open, SS_idle => o_ssidle(i)
         );
      end generate;
      gnew : if i = 1 generate
         icd : entity work.cd_top_new
         generic map (SIM_FREEZE => FREEZE)
         port map
         (
            clk1x => clk1x, clk_engine => clkeng, eng_busy => busy_new, ce => ce, reset => reset,
            INSTANTSEEK => '0', FORCECDSPEED => "000", LIMITREADSPEED => '0', hasCD => hascd_s, LIDopen => '0',
            fastCD => '0', testSeek => '0', pauseOnCDSlow => '0', region => "00", region_out => o_region(i),
            pauseCD => o_pauseCD(i), Pause_idle_cd => o_pauseIdle(i), fullyIdle => o_fullyIdle(i), cdSlow => o_cdSlow(i),
            error => o_error(i), LBAdisplay => o_lbadisp(i), irqOut => o_irq(i),
            spu_tick => spu_tick, cd_left => o_left(i), cd_right => o_right(i), mdec_idle => '1',
            bus_addr => bus_addr, bus_dataWrite => bus_dataWrite, bus_read => bus_read, bus_write => bus_write,
            bus_dataRead => o_busread(i), dma_read => dma_read, dma_readdata => o_dmaread(i),
            cd_hps_req => o_hpsreq(i), cd_hps_lba => o_hpslba(i), cd_hps_lba_sim => open,
            cd_hps_ack => hps_ack(i), cd_hps_write => hps_wr(i), cd_hps_data => hps_data(i),
            trackinfo_data => ti_data, trackinfo_addr => ti_addr, trackinfo_write => ti_write,
            resetFromCD => o_resetFromCD(i),
            SS_reset => ss_reset, SS_DataWrite => (others => '0'), SS_Adr => (others => '0'), SS_wren => '0',
            SS_rden => '0', SS_DataRead => open, SS_idle => o_ssidle(i)
         );
      end generate;
      isup : entity work.cd_disc
      generic map (T1N => T1N, XAALL => boolean'pos(SCRIPT = 4), DISCWIN => DISCWIN, WINLBA => WINLBA, WINN => WINN,
                   SLOWEVERY => SLOWEVERY, SLOWUS => SLOWUS)
      port map (clk => clk1x, req => o_hpsreq(i), lba => o_hpslba(i), ack => hps_ack(i), wr => hps_wr(i), data => hps_data(i));
   end generate;

   -- lockstep: every output on every clock
   process (clk1x)
      procedure diff(name : string; a, b : std_logic_vector) is
      begin
         if (a /= b and reset = '0') then
            if (mismatches < 20) then
               report "lockstep: " & name & " ref " & to_hstring(a) & " new " & to_hstring(b) &
                      " at cycle " & integer'image(cycles) severity warning;
            end if;
            mismatches <= mismatches + 1;
         end if;
      end procedure;
   begin
      if rising_edge(clk1x) then
         diff("cd_left", std_logic_vector(o_left(0)), std_logic_vector(o_left(1)));
         diff("cd_right", std_logic_vector(o_right(0)), std_logic_vector(o_right(1)));
         diff("irqOut", (0 => o_irq(0)), (0 => o_irq(1)));
         diff("bus_dataRead", o_busread(0), o_busread(1));
         diff("dma_readdata", o_dmaread(0), o_dmaread(1));
         diff("cd_hps_req", (0 => o_hpsreq(0)), (0 => o_hpsreq(1)));
         diff("cd_hps_lba", o_hpslba(0), o_hpslba(1));
         diff("LBAdisplay", std_logic_vector(o_lbadisp(0)), std_logic_vector(o_lbadisp(1)));
         diff("pauseCD/idle/fullyIdle/cdSlow/error/resetFromCD/SS_idle",
              o_pauseCD(0) & o_pauseIdle(0) & o_fullyIdle(0) & o_cdSlow(0) & o_error(0) & o_resetFromCD(0) & o_ssidle(0),
              o_pauseCD(1) & o_pauseIdle(1) & o_fullyIdle(1) & o_cdSlow(1) & o_error(1) & o_resetFromCD(1) & o_ssidle(1));
         diff("region_out", o_region(0), o_region(1));
      end if;
   end process;

   -- CPU model
   process
      variable rd       : std_logic_vector(7 downto 0);
      variable flags    : integer;
      variable cmd_int  : integer;   -- last INT number seen by service
      variable ints     : integer := 0;
      variable int1s    : integer := 0;
      variable sectors  : integer := 0;
      variable dbytes   : integer := 0;
      variable dsum     : integer := 0;
      variable answers  : integer := 0;
      variable noanswer : integer := 0;
      variable mode     : integer := 16#20#;   -- the mode the CPU model last set (data size for reads)
      variable readdata : boolean := true;      -- request and read sector data on INT1
      variable s1, s2   : integer := 1;         -- random state
      variable ce_next  : integer := 4000;      -- clocks to the next ce = 0 burst

      impure function rnd(n : integer) return integer is   -- 0 .. n-1
         variable r : real;
      begin
         uniform(s1, s2, r);
         return integer(trunc(r * real(n))) mod n;
      end function;

      -- clocks; with RANDCE an occasional ce = 0 burst while the CPU model has nothing on the bus
      procedure clk(n : integer := 1) is
      begin
         for i in 1 to n loop
            wait until rising_edge(clk1x);
            if (RANDCE /= 0) then
               if (ce_next > 0) then
                  ce_next := ce_next - 1;
               elsif (bus_write = '0' and bus_read = '0' and dma_read = '0') then
                  ce <= '0';
                  for k in 1 to 1 + rnd(300) loop wait until rising_edge(clk1x); end loop;
                  ce <= '1';
                  ce_next := rnd(8000);
               end if;
            end if;
         end loop;
      end procedure;

      procedure wr(a : integer; d : integer) is
      begin
         bus_addr      <= to_unsigned(a, 4);
         bus_dataWrite <= std_logic_vector(to_unsigned(d, 8));
         bus_write     <= '1';
         wait until rising_edge(clk1x);
         bus_write     <= '0';
         clk(3);
      end procedure;

      procedure rdb(a : integer; d : out std_logic_vector(7 downto 0)) is
      begin
         bus_addr <= to_unsigned(a, 4);
         bus_read <= '1';
         wait until rising_edge(clk1x);
         bus_read <= '0';
         wait until rising_edge(clk1x);
         wait until rising_edge(clk1x);
         d := o_busread(CPUSRC);
         clk(1);
      end procedure;

      -- request the sector data and read it through DMA (n bytes, n = 0: the size of the current mode)
      procedure readsector(n : integer := 0) is
         variable cnt : integer;
      begin
         wr(0, 0);
         wr(3, 16#80#);
         cnt := n;
         if (cnt = 0) then
            if ((mode / 32) mod 2 = 1) then cnt := 2340; else cnt := 2048; end if;
         end if;
         clk(8);
         for i in 1 to cnt loop
            dma_read <= '1';
            wait until rising_edge(clk1x);
            dma_read <= '0';
            dsum   := (dsum * 31 + to_integer(unsigned(o_dmaread(CPUSRC)))) mod 1000003;
            dbytes := dbytes + 1;
            if (rnd(64) = 0) then clk(1 + rnd(4)); end if;
         end loop;
         sectors := sectors + 1;
      end procedure;

      -- one pending interrupt: read the response, acknowledge; on INT1 read the sector; cmd_int = INT number
      procedure service(quiet : boolean := true) is
         variable s    : std_logic_vector(7 downto 0);
         variable line : string(1 to 64);
         variable n    : integer;
      begin
         wr(0, 1);
         rdb(3, rd);
         flags := to_integer(unsigned(rd(2 downto 0)));
         cmd_int := flags;
         if (flags /= 0) then
            n := 0;
            for i in 1 to 16 loop
               rdb(0, s);
               exit when s(5) = '0';
               rdb(1, s);
               if (n < 16) then
                  line(n * 3 + 1 to n * 3 + 3) := to_hstring(s) & " ";
                  n := n + 1;
               end if;
            end loop;
            if (not quiet or flags /= 1) then
               report "  INT" & integer'image(flags) & ": " & line(1 to n * 3);
            end if;
            if (SCRIPT = 3 and rnd(8) = 0) then clk(rnd(20000)); end if;   -- late acknowledge
            wr(0, 1);
            wr(3, 16#1F#);   -- index 1: acknowledge all
            ints := ints + 1;
            if (flags = 1) then
               int1s := int1s + 1;
               if (readdata and (SCRIPT /= 3 or rnd(6) /= 0)) then
                  if (SCRIPT = 3 and rnd(10) = 0) then readsector(1 + rnd(2340)); else readsector; end if;
               end if;
            end if;
         end if;
         wr(0, 0);
      end procedure;

      -- run n clocks, servicing interrupts (polled every 1000 clocks)
      procedure run(n : integer; quiet : boolean := true) is
         variable t : integer := 0;
      begin
         while (t < n) loop
            clk(1000);
            t := t + 1000;
            service(quiet);
         end loop;
      end procedure;

      procedure send(c : integer; p0, p1, p2 : integer := -1; p3, p4, p5 : integer := -1) is
      begin
         wr(0, 0);
         if (p0 >= 0) then wr(2, p0); end if;
         if (p1 >= 0) then wr(2, p1); end if;
         if (p2 >= 0) then wr(2, p2); end if;
         if (p3 >= 0) then wr(2, p3); end if;
         if (p4 >= 0) then wr(2, p4); end if;
         if (p5 >= 0) then wr(2, p5); end if;
         wr(1, c);
      end procedure;

      -- send a command and wait for its INT3 / INT5 answer (INT1 / INT2 / INT4 on the way are serviced)
      procedure command(name : string; c : integer; p0, p1, p2 : integer := -1; quiet : boolean := false) is
         variable t : integer := 0;
      begin
         if (not quiet) then report "command " & name; end if;
         send(c, p0, p1, p2);
         loop
            clk(200);
            service(quiet);
            exit when cmd_int = 3 or cmd_int = 5;
            t := t + 1;
            if (t >= 20000) then
               noanswer := noanswer + 1;
               if (SCRIPT /= 3) then report "no answer to " & name severity failure; end if;
               exit;
            end if;
         end loop;
         if (cmd_int = 3 or cmd_int = 5) then answers := answers + 1; end if;
      end procedure;

      function ms(m : real) return integer is
      begin
         return integer(m * 33868.8);
      end function;

      function bcd(n : integer) return integer is
      begin
         return (n / 10) * 16 + n mod 10;
      end function;

      procedure ti(a : integer; d : std_logic_vector(31 downto 0)) is
      begin
         ti_addr  <= std_logic_vector(to_unsigned(a, 9));
         ti_data  <= d;
         ti_write <= '1';
         wait until rising_edge(clk1x);
         ti_write <= '0';
         clk(3);
      end procedure;

      function msf_word(lba : integer; audio : std_logic) return std_logic_vector is
         variable s : integer := lba / 75;
      begin
         return x"000" & "000" & audio & std_logic_vector(to_unsigned(bcd(s / 60), 8)) & std_logic_vector(to_unsigned(bcd(s mod 60), 8));
      end function;

      procedure setloc(lba : integer; quiet : boolean := false) is
      begin
         command("Setloc " & integer'image(lba), 16#02#, bcd(lba / 4500), bcd((lba / 75) mod 60), bcd(lba mod 75), quiet);
      end procedure;

      procedure setmode(m : integer; quiet : boolean := false) is
      begin
         mode := m;
         command("SetMode", 16#0E#, m, quiet => quiet);
      end procedure;

      variable lba, c, k, p : integer;
   begin
      s1 := SEED * 7919 + 17; s2 := SEED * 104729 + 3;
      ss_reset <= '1';
      clk(10);
      ss_reset <= '0';
      clk(10);
      reset <= '0';
      clk(100);

      -- trackinfo as pocket_cd sends it
      ti(0, x"0000" & std_logic_vector(to_unsigned(bcd(2), 8)) & x"02");
      ti(1, std_logic_vector(to_unsigned(TOTAL, 32)));
      ti(2, msf_word(TOTAL, '0'));
      ti(3, x"00000000");
      ti(4, x"00000000");                                            -- track 1: start 0
      ti(5, std_logic_vector(to_unsigned(T2START - 1, 32)));         --          end
      ti(6, msf_word(150, '0'));
      ti(7, x"00000000");
      ti(8, std_logic_vector(to_unsigned(T2START, 32)));             -- track 2: start
      ti(9, std_logic_vector(to_unsigned(TOTAL - 1, 32)));           --          end
      ti(10, msf_word(T2START + 150, '1'));
      ti(11, x"00000000");

      wr(0, 1);
      wr(2, 16#1F#);   -- index 1: enable all interrupts
      wr(0, 0);

      run(ms(5.0), false);
      command("GetStat", 16#01#);
      run(ms(400.0), false);   -- motor spin up
      command("GetStat", 16#01#);

      if (SCRIPT = 0) then        -- BIOS-like disc boot
         command("Init", 16#0A#);
         run(ms(200.0), false);
         command("Demute", 16#0C#);
         command("Test 20h", 16#19#, 16#20#);
         command("GetID", 16#1A#);
         run(ms(10.0), false);
         command("GetTN", 16#13#);
         command("GetTD 0", 16#14#, 0);
         command("GetTD 1", 16#14#, 1);
         command("GetTD 2", 16#14#, 2);
         setmode(16#80#);         -- speed change: 0.8 s
         command("ReadTOC", 16#1E#);
         run(ms(900.0), false);
         setloc(150 + 16);
         command("SeekL", 16#15#);
         run(ms(150.0), false);
         command("ReadN", 16#06#);
         run(ms(80.0));
         command("Pause", 16#09#);
         run(ms(30.0), false);
         command("GetlocL", 16#10#);
         command("GetlocP", 16#11#);
         command("GetParam", 16#0F#);
         setloc(150 + 24);
         command("ReadN", 16#06#);
         run(ms(60.0));
         command("Pause", 16#09#);
         run(ms(30.0), false);
         command("Stop", 16#08#);
         run(ms(300.0), false);

      elsif (SCRIPT = 1) then     -- SotN-like: loads with ReadN, music as XA (ReadS, filter)
         command("Init", 16#0A#);
         run(ms(200.0), false);
         command("Demute", 16#0C#);
         setmode(16#80#);
         setloc(400);
         command("ReadN", 16#06#);
         run(ms(80.0));
         command("Pause", 16#09#);
         run(ms(20.0), false);
         command("Setfilter", 16#0D#, 1, 0);
         setmode(16#C8#);
         setloc(XA0);
         command("ReadS", 16#1B#);
         run(ms(300.0));
         command("GetlocL", 16#10#);
         command("Setfilter", 16#0D#, 1, 1);
         run(ms(200.0));
         command("Pause", 16#09#);
         run(ms(20.0), false);
         setmode(16#80#);
         setloc(900);
         command("SeekL", 16#15#);
         run(ms(100.0), false);
         command("ReadN", 16#06#);
         run(ms(60.0));
         command("Pause", 16#09#);
         run(ms(20.0), false);
         setmode(16#C8#);
         setloc(XA1 - 40);
         command("ReadS", 16#1B#);
         run(ms(400.0));   -- runs out of the XA zone
         command("Pause", 16#09#);
         run(ms(50.0), false);

      elsif (SCRIPT = 4) then     -- SotN music: ReadS through an XA only zone, filter on one channel; from half
                                  -- time on GetlocL every 50 ms (more engine work while XA plays)
         command("Init", 16#0A#);
         run(ms(200.0), false);
         command("Demute", 16#0C#);
         command("Setfilter", 16#0D#, 1, 0);
         setmode(16#C8#);
         setloc(XA0);
         command("ReadS", 16#1B#);
         run(ms(real(XAMS) / 2.0));
         for i in 1 to XAMS / 100 loop
            command("GetlocL", 16#10#, quiet => true);
            run(ms(50.0));
         end loop;
         command("Pause", 16#09#);
         run(ms(50.0), false);

      elsif (SCRIPT = 5) then     -- SotN starting a song: Setfilter, SetMode 0xC8, Setloc, ReadS; play XAMS ms
         command("Init", 16#0A#);
         run(ms(200.0), false);
         command("Demute", 16#0C#);
         command("Setfilter", 16#0D#, XAFILE, XACHAN);
         setmode(16#C8#);
         setloc(PLAYLBA);
         command("ReadS", 16#1B#);
         run(ms(real(XAMS)));
         command("Pause", 16#09#);
         run(ms(20.0), false);

      elsif (SCRIPT = 2) then     -- CD audio (sim/cd_cdda)
         command("Init", 16#0A#);
         run(ms(200.0), false);
         setmode(16#05#);
         command("Demute", 16#0C#);
         command("Play track 2", 16#03#, 16#02#);
         run(ms(700.0));
         lba := T2START + 150 + 20 * 75;
         setloc(lba);
         command("Play", 16#03#);
         run(ms(500.0));
         command("Forward", 16#04#);
         run(ms(100.0));
         command("Backward", 16#05#);
         run(ms(100.0));
         command("GetlocP", 16#11#);
         setloc(TOTAL - 30);
         command("Play", 16#03#);
         run(ms(600.0));   -- into the lead-out
         command("Stop", 16#08#);
         run(ms(300.0), false);

      else                        -- random commands
         for it in 1 to NCMD loop
            readdata := rnd(5) /= 0;
            c := rnd(100);
            if    (c < 12) then
               k := rnd(10);
               if    (k < 5) then lba := 150 + rnd(T1N);
               elsif (k < 7) then lba := XA0 + rnd(XA1 - XA0);
               elsif (k < 9) then lba := T2START + rnd(T2N);
               else               lba := TOTAL - 5 + rnd(20);
               end if;
               setloc(lba, true);
            elsif (c < 18) then command("SeekL", 16#15#, quiet => true);
            elsif (c < 21) then command("SeekP", 16#16#, quiet => true);
            elsif (c < 27) then command("ReadN", 16#06#, quiet => true);
            elsif (c < 31) then command("ReadS", 16#1B#, quiet => true);
            elsif (c < 36) then command("Pause", 16#09#, quiet => true);
            elsif (c < 39) then command("Stop", 16#08#, quiet => true);
            elsif (c < 43) then
               k := rnd(5);
               if (k = 0) then command("Play", 16#03#, quiet => true);
               else command("Play n", 16#03#, k - 1, quiet => true); end if;
            elsif (c < 45) then command("Forward", 16#04#, quiet => true);
            elsif (c < 47) then command("Backward", 16#05#, quiet => true);
            elsif (c < 50) then command("GetlocL", 16#10#, quiet => true);
            elsif (c < 53) then command("GetlocP", 16#11#, quiet => true);
            elsif (c < 55) then command("GetTN", 16#13#, quiet => true);
            elsif (c < 57) then command("GetTD", 16#14#, rnd(4), quiet => true);
            elsif (c < 59) then command("GetStat", 16#01#, quiet => true);
            elsif (c < 60) then command("GetParam", 16#0F#, quiet => true);
            elsif (c < 65) then
               k := rnd(4);
               if    (k = 0) then setmode(16#80#, true);
               elsif (k = 1) then setmode(16#C8#, true);
               elsif (k = 2) then setmode(rnd(256), true);
               else               setmode(16#05# + 128 * rnd(2) + 2 * rnd(2), true);
               end if;
            elsif (c < 67) then command("Setfilter", 16#0D#, rnd(3), rnd(3), quiet => true);
            elsif (c < 69) then command("Mute", 16#0B#, quiet => true);
            elsif (c < 71) then command("Demute", 16#0C#, quiet => true);
            elsif (c < 73) then command("Init", 16#0A#, quiet => true);
            elsif (c < 74) then command("MotorOn", 16#07#, quiet => true);
            elsif (c < 76) then command("GetID", 16#1A#, quiet => true);
            elsif (c < 77) then command("ReadTOC", 16#1E#, quiet => true);
            elsif (c < 79) then
               k := rnd(6);
               if (k = 0) then p := 16#04#; elsif (k = 1) then p := 16#05#; elsif (k = 2) then p := 16#20#;
               elsif (k = 3) then p := 16#22#; else p := rnd(256); end if;
               command("Test", 16#19#, p, quiet => true);
            elsif (c < 80) then command("SetSession", 16#12#, 1 + rnd(2), quiet => true);
            elsif (c < 82) then
               k := rnd(6);
               if (k = 0) then p := 16#00#; elsif (k = 1) then p := 16#17#; elsif (k = 2) then p := 16#18#;
               elsif (k = 3) then p := 16#1D#; elsif (k = 4) then p := 16#1F#; else p := 16#20# + rnd(224); end if;
               command("invalid", p, quiet => true);
            elsif (c < 84) then   -- wrong number of parameters
               k := rnd(3);
               if (k = 0) then command("Setloc 1 param", 16#02#, 0, quiet => true);
               elsif (k = 1) then command("Test no param", 16#19#, quiet => true);
               else command("GetStat + params", 16#01#, 1, 2, 3, quiet => true); end if;
            elsif (c < 87) then   -- a second command while the first is still pending
               send(16#01#);
               clk(rnd(30000));
               k := rnd(4);
               if (k = 0) then command("busy Setloc", 16#02#, bcd(0), bcd(2), bcd(rnd(75)), quiet => true);
               elsif (k = 1) then command("busy SetMode", 16#0E#, 16#80#, quiet => true);
               elsif (k = 2) then command("busy ReadN", 16#06#, quiet => true);
               else command("busy Pause", 16#09#, quiet => true); end if;
            elsif (c < 88) then
               reset <= '1';
               clk(1 + rnd(10));
               reset <= '0';
               clk(100);
               wr(0, 1);
               wr(2, 16#1F#);
               wr(0, 0);
            else
               null;
            end if;
            run(1000 * (1 + rnd(1000)));
         end loop;
      end if;

      report "interrupts " & integer'image(ints) & " (INT1 " & integer'image(int1s) & "), answers " &
             integer'image(answers) & ", unanswered " & integer'image(noanswer) & ", sectors read " &
             integer'image(sectors) & ", bytes " & integer'image(dbytes) & ", data checksum " & integer'image(dsum);
      clk(10);
      finish <= true;
      wait;
   end process;

   -- XA / CD-DA output monitor, both instances: sector starts (period, fifo levels), skipped sectors, output fifo
   -- underruns (spu_tick with nothing to play while streaming: the last sample is held, an audible click)
   gxamon : for k in 0 to 1 generate
      signal x_start, x_reset, x_cdda, x_oempty, x_anf : std_logic;
      signal x_ocnt : unsigned(9 downto 0);
      signal x_acnt : unsigned(11 downto 0);
   begin
      g0 : if k = 0 generate
         x_start  <= << signal .tb_cd_equiv.ginst(0).gref.icd.XA_start : std_logic >>;
         x_reset  <= << signal .tb_cd_equiv.ginst(0).gref.icd.XA_reset : std_logic >>;
         x_cdda   <= << signal .tb_cd_equiv.ginst(0).gref.icd.CDDA_write : std_logic >>;
         x_oempty <= << signal .tb_cd_equiv.ginst(0).gref.icd.icd_xa.FifoOut_Empty : std_logic >>;
         x_anf    <= << signal .tb_cd_equiv.ginst(0).gref.icd.icd_xa.FifoADPCM_NearFull : std_logic >>;
         x_ocnt   <= << signal .tb_cd_equiv.ginst(0).gref.icd.icd_xa.ififoOut.fifocnt : unsigned(9 downto 0) >>;
         x_acnt   <= << signal .tb_cd_equiv.ginst(0).gref.icd.icd_xa.ififoADPCM_L.fifocnt : unsigned(11 downto 0) >>;
      end generate;
      g1 : if k = 1 generate
         x_start  <= << signal .tb_cd_equiv.ginst(1).gnew.icd.XA_start : std_logic >>;
         x_reset  <= << signal .tb_cd_equiv.ginst(1).gnew.icd.XA_reset : std_logic >>;
         x_cdda   <= << signal .tb_cd_equiv.ginst(1).gnew.icd.CDDA_write : std_logic >>;
         x_oempty <= << signal .tb_cd_equiv.ginst(1).gnew.icd.icd_xa.FifoOut_Empty : std_logic >>;
         x_anf    <= << signal .tb_cd_equiv.ginst(1).gnew.icd.icd_xa.FifoADPCM_NearFull : std_logic >>;
         x_ocnt   <= << signal .tb_cd_equiv.ginst(1).gnew.icd.icd_xa.ififoOut.fifocnt : unsigned(9 downto 0) >>;
         x_acnt   <= << signal .tb_cd_equiv.ginst(1).gnew.icd.icd_xa.ififoADPCM_L.fifocnt : unsigned(11 downto 0) >>;
      end generate;
      process (clk1x)
         constant nm : string := "xamon " & integer'image(k) & ": ";
         variable streaming, gap, said : boolean := false;
         variable starts, skips, gaps, empties, glen, nlog, last : integer := 0;
      begin
         if rising_edge(clk1x) then
            -- the monitor sees the signals one delta late (concurrent copies): same clock, fine for counting
            if (x_reset = '1') then streaming := false; gap := false; end if;
            if (x_start = '1') then
               starts := starts + 1;
               if (x_anf = '1') then skips := skips + 1; end if;
               if (nlog < 300) then
                  report nm & "XA start at cycle " & integer'image(cycles) & " (+" & integer'image(cycles - last) &
                         "), out fifo " & integer'image(to_integer(x_ocnt)) & ", adpcm fifo " &
                         integer'image(to_integer(x_acnt)) & ", near full " & std_logic'image(x_anf);
                  nlog := nlog + 1;
               end if;
               last := cycles;
               streaming := true;
            end if;
            if (x_cdda = '1') then streaming := true; end if;
            if (spu_tick = '1' and streaming) then
               if (x_oempty = '1') then
                  empties := empties + 1;
                  glen := glen + 1;
                  if (not gap) then
                     gaps := gaps + 1;
                     gap  := true;
                     glen := 1;
                  end if;
               elsif (gap) then
                  gap := false;
                  if (nlog < 300) then
                     report nm & "underrun of " & integer'image(glen) & " samples, ended at cycle " & integer'image(cycles);
                     nlog := nlog + 1;
                  end if;
               end if;
            end if;
            if (finish and not said) then
               said := true;
               report nm & "XA sector starts " & integer'image(starts) & ", started while near full (skipped) " &
                      integer'image(skips) & ", underruns " & integer'image(gaps) & " (" & integer'image(empties) &
                      " samples, last one still open: " & boolean'image(gap) & ")";
            end if;
         end if;
      end process;
   end generate;

   -- output capture (OUTPRE): both instances, every spu_tick from the first non-zero sample on
   gcap : if OUTPRE'length > 0 generate
      gk : for k in 0 to 1 generate
         process (clk1x)
            file fo        : text open write_mode is OUTPRE & integer'image(k) & ".txt";
            variable ln    : line;
            variable start : boolean := false;
         begin
            if rising_edge(clk1x) and spu_tick = '1' then
               if (o_left(k) /= 0 or o_right(k) /= 0) then start := true; end if;
               if (start) then
                  write(ln, to_integer(o_left(k))); write(ln, string'(" ")); write(ln, to_integer(o_right(k)));
                  writeline(fo, ln);
               end if;
            end if;
         end process;
      end generate;
   end generate;

   -- audio summary of the instance the CPU model uses: samples sent to the SPU that are not silent, and a sum
   process (clk1x)
   begin
      if rising_edge(clk1x) then
         if (spu_tick = '1' and (o_left(CPUSRC) /= 0 or o_right(CPUSRC) /= 0)) then
            audio_n   <= audio_n + 1;
            audio_sum <= (audio_sum + to_integer(o_left(CPUSRC)) + 3 * to_integer(o_right(CPUSRC))) mod 1000003;
         end if;
      end if;
   end process;

   -- debug trace (DBGMS >= 0): the new instance's parameter fifo and sector requests
   gdbg : if DBGMS >= 0 generate
      process (clk1x)
         alias p_wr  is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoParam_Wr : std_logic >>;
         alias p_din is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoParam_Din : std_logic_vector(7 downto 0) >>;
         alias p_rd  is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoParam_Rd : std_logic >>;
         alias p_rst is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoParam_reset : std_logic >>;
         alias p_out is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoParam_Dout : std_logic_vector(7 downto 0) >>;
         alias p_emp is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoParam_Empty : std_logic >>;
         alias busy  is << signal .tb_cd_equiv.ginst(1).gnew.icd.eng_busy : std_logic >>;
      begin
         if rising_edge(clk1x) and now >= DBGMS * 1 ms then
            if (p_wr = '1') then report "dbg param write " & to_hstring(p_din) & " busy " & std_logic'image(busy); end if;
            if (p_rd = '1') then report "dbg param read, out " & to_hstring(p_out) & " empty " & std_logic'image(p_emp); end if;
            if (p_rst = '1') then report "dbg param reset"; end if;
            if (o_hpsreq(1) = '1' and req_1 = '0') then report "dbg sector request " & to_hstring(o_hpslba(1)); end if;
         end if;
      end process;
      -- hardware mode only: interrupt flags, what the CPU sees, quiet, response fifo strobes, CD-DA report pulses
      process (clk1x)
         alias i_flag is << signal .tb_cd_equiv.ginst(1).gnew.icd.CDROM_IRQFLAG : std_logic_vector(4 downto 0) >>;
         alias i_cpu  is << signal .tb_cd_equiv.ginst(1).gnew.icd.irqFlagCPU : std_logic_vector(4 downto 0) >>;
         alias i_q    is << signal .tb_cd_equiv.ginst(1).gnew.icd.ctrl_quiet : std_logic >>;
         alias r_wr   is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoResponse_Wr : std_logic >>;
         alias r_din  is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoResponse_Din : std_logic_vector(7 downto 0) >>;
         alias r_rst  is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoResponse_reset : std_logic >>;
         alias c8     is << signal .tb_cd_equiv.ginst(1).gnew.icd.ctrl_cdda_out8 : std_logic >>;
         variable f1, c1 : std_logic_vector(4 downto 0) := (others => '0');
         variable q1 : std_logic := '1';
      begin
         if rising_edge(clk1x) and now >= DBGMS * 1 ms then
            if (i_flag /= f1) then report "irq flags " & to_hstring(i_flag); end if;
            if (i_cpu /= c1) then report "irq flags seen by the cpu " & to_hstring(i_cpu); end if;
            if (i_q /= q1) then report "quiet " & std_logic'image(i_q); end if;
            if (r_wr = '1') then report "response push " & to_hstring(r_din); end if;
            if (r_rst = '1') then report "response reset"; end if;
            if (c8 = '1') then report "cdda report pulse"; end if;
            f1 := i_flag; c1 := i_cpu; q1 := i_q;
         end if;
      end process;
      -- hardware mode only (CPUSRC = 1 without FREEZE): the engine's starts and I/O writes
      process (clk1x)
         alias e_st  is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl.ictrl.eng_start : std_logic >>;
         alias e_bsy is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl.ictrl.busy : std_logic >>;
         alias e_ev  is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl.ictrl.ev_now : std_logic_vector(31 downto 0) >>;
         alias e_we  is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl.ictrl.io_we : std_logic >>;
         alias e_wa  is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl.ictrl.io_waddr : unsigned(4 downto 0) >>;
         alias e_wd  is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl.ictrl.io_wdata : std_logic_vector(31 downto 0) >>;
         alias e_cin is << signal .tb_cd_equiv.ginst(1).gnew.icd.gctrl.ictrl.s_cmdin : std_logic_vector(31 downto 0) >>;
         alias p_out is << signal .tb_cd_equiv.ginst(1).gnew.icd.FifoParam_Dout : std_logic_vector(7 downto 0) >>;
      begin
         if rising_edge(clk1x) and now >= DBGMS * 1 ms and now < (DBGMS + 2) * 1 ms then
            if (e_st = '1' and e_bsy = '0') then
               report "dbg start ev " & to_hstring(e_ev) & " param " & to_hstring(p_out) & " (latched before: " & to_hstring(e_cin) & ")";
            end if;
            if (e_we = '1') then
               report "dbg   io write " & integer'image(to_integer(e_wa)) & " = " & to_hstring(e_wd);
            end if;
         end if;
      end process;
   end generate;

   -- sector requests of the instance the CPU model uses: number and clock of the first and the last (the drive's
   -- pace: a hardware-mode run must keep the reference's, see cd_ctrl's drive catch-up)
   process (clk1x)
   begin
      if rising_edge(clk1x) then
         req_1 <= o_hpsreq(CPUSRC);
         if (o_hpsreq(CPUSRC) = '1' and req_1 = '0') then
            if (req_n = 0) then req_first <= cycles; end if;
            req_last <= cycles;
            req_n    <= req_n + 1;
         end if;
      end if;
   end process;

   process
   begin
      wait until rising_edge(clk1x) and finish;
      report "sector requests " & integer'image(req_n) & ", first at cycle " & integer'image(req_first) &
             ", last at cycle " & integer'image(req_last);
      report "audio samples (not silent) " & integer'image(audio_n) & ", audio checksum " & integer'image(audio_sum);
      report "cycles " & integer'image(cycles) & ", engine cycles with the world held " & integer'image(frozen);
      report "lockstep mismatches " & integer'image(mismatches);
      if (mismatches = 0 or (not FREEZE and CPUSRC = 1)) then
         report "PASS";
      else
         report "FAIL" severity failure;
      end if;
      done <= true;
      wait for 1 us;
      std.env.finish;   -- the progress report process would run on
      wait;
   end process;

end architecture;
