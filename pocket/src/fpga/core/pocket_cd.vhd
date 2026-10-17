-- Analogue Pocket: CD image streaming for the PSX core. Does the job of MiSTer's HPS (Main_MiSTer
-- support/psx/psx.cpp) for the core's trackinfo_* and cd_hps_* ports.
--
-- Disc: one .bin file per track (Redump style cue: track 1 MODE2/2352 data, further tracks AUDIO with their
-- 2 s pregap inside the file and INDEX 01 at 00:02:00). Track k (0 based) is the deferred-load APF data slot
-- TRACK_ID_BASE + k. Audio CD: track 1 in slot TRACK_ID_BASE - 1 instead of TRACK_ID_BASE (not both), which
-- marks track 1 as audio. Layout as psx.cpp builds it for such a cue:
--   track 0: LBA 150 .. 150 + N0 - 1 (LBA 0..149 = 2 s pregap, read as zeros), file offset (LBA - 150) * 2352
--   track k: starts right after track k-1, file offset (LBA - start) * 2352
--   trackinfo: header (count, total, MM:SS of total, libcrypt/region = 0), per track
--   {start (0 for track 0), end, MM:SS of start + 150 | audio << 16, 0} (audio: all tracks but a data track 1)
-- Sizes come from the APF data slot table (core_bridge_cmd datatable: entry e = {id, size} at words 2e, 2e+1),
-- read when APF releases reset_n; the core is held in reset (cd_ready = '0') until the table is built.
-- Sector reads: target command 0x0180 (data slot read) into a ring of SLOTS sector buffers at bridge address
-- 0x40000000 (slot i at byte 2352 i), then the requested sector goes to the core like MiSTer's sd interface does it
-- (ack high during the block, one cd_hps_write per 16-bit word, low half of each little-endian 32-bit word first).
-- Read-ahead: one data slot read per sector took longer than a sector period (6.6 ms at double speed) often enough
-- that the drive kept waiting for data ("CD" marker on most of the time), and every wait is a gap in XA / CD audio
-- (clicks). Now the ring holds the sectors after the last request: while a slot is free, the following sectors of
-- the same track are read ahead, up to KMAX per command (fewer commands, each one's overhead counts once). A request
-- found in the ring is sent at once; one in a read still running waits for it; any other (a seek, another track)
-- starts the ring anew with a single-sector read, so it is sent as soon as possible. The commands run one at a time
-- (cmd_st) beside the request handling; a sent sector's slot is free again.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library mem;

entity pocket_cd is
   generic
   (
      TRACK_ID_BASE   : integer := 16;
      MAX_TRACKS      : integer := 24;   -- <= 31
      SLOT_BITS       : integer := 3;    -- the read-ahead ring has 2^SLOT_BITS sectors (x 588 words <= 2^BUF_AW)
      KMAX            : integer := 4;    -- sectors per read-ahead command (at most, <= 2^SLOT_BITS)
      BUF_AW          : integer := 13    -- word address bits of the ring
   );
   port
   (
      clk_74a         : in  std_logic;
      reset_n_74a     : in  std_logic;   -- APF reset_n: low while APF loads data slots
      cd_ready        : out std_logic := '0';   -- track table built, the core may leave reset
      has_cd          : out std_logic := '0';   -- valid while cd_ready = '1'
      -- APF data slot table (core_bridge_cmd datatable port a, 2 clocks read latency)
      datatable_addr  : out std_logic_vector(9 downto 0) := (others => '0');
      datatable_q     : in  std_logic_vector(31 downto 0);
      -- target command "data slot read" (core_bridge_cmd target_dataslot_*)
      dsr_read        : out std_logic := '0';   -- rising edge starts the command
      dsr_id          : out std_logic_vector(15 downto 0);
      dsr_offset      : out std_logic_vector(31 downto 0) := (others => '0');
      dsr_bridgeaddr  : out std_logic_vector(31 downto 0) := x"40000000";
      dsr_length      : out std_logic_vector(31 downto 0) := (others => '0');
      dsr_done        : in  std_logic;          -- cleared when a command starts, set when it has finished
      -- APF bridge writes (sector data)
      bridge_addr     : in  std_logic_vector(31 downto 0);
      bridge_wr       : in  std_logic;
      bridge_wr_data  : in  std_logic_vector(31 downto 0);
      -- core side (clk1x)
      clk1x           : in  std_logic;
      psx_reset       : in  std_logic;
      cd_hps_req      : in  std_logic;
      cd_hps_lba      : in  std_logic_vector(31 downto 0);
      cd_hps_ack      : out std_logic := '0';
      cd_hps_write    : out std_logic := '0';
      cd_hps_data     : out std_logic_vector(15 downto 0) := (others => '0');
      trackinfo_data  : out std_logic_vector(31 downto 0);
      trackinfo_addr  : out std_logic_vector(8 downto 0);
      trackinfo_write : out std_logic := '0'
   );
end entity;

architecture arch of pocket_cd is

   constant SECTOR_BYTES : integer := 2352;
   constant SLOTS        : integer := 2 ** SLOT_BITS;
   constant BUF_BASE     : std_logic_vector(3 downto 0) := x"4";   -- sector ring at bridge 0x40000000

   type tState is
   (
      S_WAITRESET, S_CLEAR,
      S_DT_ID, S_DT_SIZE, S_DT_CHECK, S_DT_STORE, S_DT_NEXT,
      S_SUM_WAIT, S_SUM, S_SUM_DONE,
      S_TI_START, S_TI_RD, S_TI_NEXT, S_TI_MSF, S_TI_MSF_S, S_TI_MSF_M, S_TI_MSF_B, S_TI_MSF_E, S_TI_SEND, S_TI_WAIT,
      S_IDLE, S_FIND, S_LOOK, S_SECT_DONE, S_SECT_WAIT,
      S_DIV
   );
   signal st         : tState := S_WAITRESET;
   signal div_ret    : tState := S_WAITRESET;

   -- datatable access: address set, value read 4 clocks later
   signal dt_entry   : unsigned(4 downto 0) := (others => '0');
   signal dt_wait    : integer range 0 to 3 := 0;
   signal dt_id      : unsigned(15 downto 0) := (others => '0');

   -- track table: entry k = first LBA of track k (entry ntracks = total LBAs); during the scan: sectors of track k
   signal tab_we     : std_logic := '0';
   signal tab_wa     : std_logic_vector(4 downto 0) := (others => '0');
   signal tab_d      : std_logic_vector(19 downto 0) := (others => '0');
   signal tab_ra     : std_logic_vector(4 downto 0) := (others => '0');
   signal tab_q      : std_logic_vector(19 downto 0);
   signal k          : unsigned(4 downto 0) := (others => '0');
   signal ntracks    : unsigned(4 downto 0) := (others => '0');
   signal audio_cd   : std_logic := '0';   -- track 1 is audio (slot TRACK_ID_BASE - 1)
   signal total      : unsigned(19 downto 0) := (others => '0');

   -- sequential divider (restoring, 32 steps): div_n / div_d -> quotient in div_n, remainder in div_r
   signal div_n      : unsigned(31 downto 0) := (others => '0');
   signal div_sel    : integer range 0 to 3 := 0;   -- divisor: 2352, 75, 60, 10
   signal div_d      : unsigned(11 downto 0);
   signal div_r      : unsigned(11 downto 0) := (others => '0');
   signal div_cnt    : integer range 0 to 32 := 0;

   -- trackinfo words (word w: 0-3 header, 4 + 4 * track + 0..3 track entries)
   signal ti_word    : unsigned(8 downto 0) := (others => '0');
   signal msf_audio  : std_logic := '0';
   signal msf_sec    : unsigned(7 downto 0) := (others => '0');
   signal msf_mbcd   : std_logic_vector(7 downto 0) := (others => '0');
   signal ti_req     : std_logic := '0';
   signal ti_ack_s   : std_logic_vector(2 downto 0) := (others => '0');
   signal ti_d       : std_logic_vector(31 downto 0) := (others => '0');
   signal ti_a       : std_logic_vector(8 downto 0) := (others => '0');
   signal psxrst_s   : std_logic_vector(1 downto 0) := (others => '1');

   -- sector requests
   signal req_s      : std_logic_vector(1 downto 0) := (others => '0');
   signal lba        : unsigned(19 downto 0) := (others => '0');
   signal start      : unsigned(19 downto 0) := (others => '0');
   signal sec_zero   : std_logic := '0';
   signal sec_tog    : std_logic := '0';
   signal sec_ack_s  : std_logic_vector(2 downto 0) := (others => '0');

   -- sector ring (SLOTS x 588 32-bit words): written by APF on clk_74a, read on clk1x
   signal buf_we     : std_logic;
   signal buf_ra     : unsigned(BUF_AW - 1 downto 0) := (others => '0');
   signal buf_end    : unsigned(BUF_AW - 1 downto 0) := (others => '0');
   signal buf_q      : std_logic_vector(31 downto 0) := (others => '0');
   type tRing is array (0 to SLOTS * (SECTOR_BYTES / 4) - 1) of std_logic_vector(31 downto 0);
   signal ring       : tRing;
   -- M10K as 256 x 40 (19 blocks for 8 slots); left to itself Quartus took 32 for the 4704 words
   attribute ramstyle  : string;
   attribute ramstyle  of ring : signal is "M10K";
   attribute max_depth : natural;
   attribute max_depth of ring : signal is 256;
   signal serve_base : unsigned(BUF_AW - 1 downto 0) := (others => '0');   -- first word of the sector to send (held)

   -- ring: sectors head_lba .. head_lba + r_cnt - 1 of track r_track in slots head_slot, head_slot + 1, ... (mod
   -- SLOTS), then r_pend sectors of the command running; r_start / r_end: the track's first LBA / the next one's
   type tCmd is (C_IDLE, C_PREP, C_START, C_WAIT0, C_WAIT1);
   signal cmd_st     : tCmd := C_IDLE;
   signal r_valid    : std_logic := '0';
   signal r_track    : unsigned(4 downto 0) := (others => '0');
   signal r_start    : unsigned(19 downto 0) := (others => '0');
   signal r_end      : unsigned(19 downto 0) := (others => '0');
   signal head_lba   : unsigned(19 downto 0) := (others => '0');
   signal head_slot  : unsigned(SLOT_BITS - 1 downto 0) := (others => '0');   -- (wraps: the ring)
   signal r_cnt      : unsigned(SLOT_BITS downto 0) := (others => '0');
   signal r_pend     : unsigned(SLOT_BITS downto 0) := (others => '0');
   -- next read-ahead, worked out one clock before from the registered ring (used only if the ring did not change)
   signal ring_chg   : std_logic := '1';
   signal pf_ok      : std_logic := '0';
   signal pf_lba     : unsigned(19 downto 0) := (others => '0');
   signal pf_n       : unsigned(SLOT_BITS downto 0) := (others => '0');
   signal pf_ws      : unsigned(SLOT_BITS - 1 downto 0) := (others => '0');
   -- the command to start: first LBA, sectors, slot
   signal c_lba      : unsigned(19 downto 0) := (others => '0');
   signal c_n        : unsigned(SLOT_BITS downto 0) := (others => '0');
   signal c_ws       : unsigned(SLOT_BITS - 1 downto 0) := (others => '0');

   -- clk1x side
   signal ti_req_s   : std_logic_vector(2 downto 0) := (others => '0');
   signal ti_ack     : std_logic := '0';
   signal sec_tog_s  : std_logic_vector(2 downto 0) := (others => '0');
   signal sec_ack    : std_logic := '0';
   signal zero_1x    : std_logic := '0';
   signal sending    : std_logic := '0';
   signal phase      : integer range 0 to 3 := 0;
   signal gap        : integer range 0 to 3 := 0;

   -- BCD of 0..31
   function bcd_small(n : unsigned(4 downto 0)) return std_logic_vector is
      variable t : integer range 0 to 3;
   begin
      if (n >= 30) then t := 3; elsif (n >= 20) then t := 2; elsif (n >= 10) then t := 1; else t := 0; end if;
      return std_logic_vector(to_unsigned(t, 4)) & std_logic_vector(resize(n - to_unsigned(t * 10, 5), 4));
   end function;

   procedure divide(signal n : out unsigned(31 downto 0); signal d : out integer range 0 to 3;
                    signal r : out unsigned(11 downto 0); signal cnt : out integer range 0 to 32;
                    nv : unsigned; dv : integer) is
   begin
      n   <= resize(nv, 32);
      case dv is
         when 2352   => d <= 0;
         when 75     => d <= 1;
         when 60     => d <= 2;
         when others => d <= 3;
      end case;
      r   <= (others => '0');
      cnt <= 32;
   end procedure;

begin

   -- the ring's track (1 based) is stable while its commands run; audio CD: track 1 from slot TRACK_ID_BASE - 1
   dsr_id         <= std_logic_vector(to_unsigned(TRACK_ID_BASE - 1, 16)) when (r_track = 1 and audio_cd = '1') else
                     std_logic_vector(to_unsigned(TRACK_ID_BASE - 1, 16) + r_track);

   with div_sel select div_d <=
      to_unsigned(SECTOR_BYTES, 12) when 0,
      to_unsigned(75, 12)           when 1,
      to_unsigned(60, 12)           when 2,
      to_unsigned(10, 12)           when others;

   itab : entity mem.RamMLAB
   generic map (width => 20, widthad => 5)
   port map (inclock => clk_74a, wren => tab_we, data => tab_d, wraddress => tab_wa, rdaddress => tab_ra, q => tab_q);

   buf_we <= '1' when (bridge_wr = '1' and bridge_addr(31 downto 28) = BUF_BASE) else '0';

   -- the ring: block RAM of exactly SLOTS x 588 words (written by APF on clk_74a, read on clk1x one clock after the
   -- address, like mem.dpram, whose power-of-two depth would take 32 M10K)
   process (clk_74a)
      variable a : integer;
   begin
      if rising_edge(clk_74a) then
         a := to_integer(unsigned(bridge_addr(BUF_AW + 1 downto 2)));
         if (buf_we = '1' and a < SLOTS * (SECTOR_BYTES / 4)) then
            ring(a) <= bridge_wr_data;
         end if;
      end if;
   end process;
   process (clk1x)
   begin
      if rising_edge(clk1x) then
         if (to_integer(buf_ra) < SLOTS * (SECTOR_BYTES / 4)) then
            buf_q <= ring(to_integer(buf_ra));
         end if;
      end if;
   end process;

   ---------------------------------------------------------------- clk_74a: table, trackinfo, sector requests
   process (clk_74a)
      variable rr   : unsigned(12 downto 0);
      variable diff : unsigned(19 downto 0);
      variable kk   : unsigned(4 downto 0);
      -- ring bookkeeping of this clock (the request side and the command side both change it)
      variable hl   : unsigned(19 downto 0);
      variable hs   : unsigned(SLOT_BITS - 1 downto 0);
      variable cnt  : unsigned(SLOT_BITS downto 0);
      variable pend : unsigned(SLOT_BITS downto 0);
      variable cmd  : tCmd;
      variable chg  : std_logic;
      variable d    : unsigned(19 downto 0);
   begin
      if rising_edge(clk_74a) then
         hl   := head_lba;
         hs   := head_slot;
         cnt  := r_cnt;
         pend := r_pend;
         cmd  := cmd_st;
         chg  := '0';
         tab_we    <= '0';
         req_s     <= req_s(0) & cd_hps_req;
         ti_ack_s  <= ti_ack_s(1 downto 0) & ti_ack;
         sec_ack_s <= sec_ack_s(1 downto 0) & sec_ack;
         psxrst_s  <= psxrst_s(0) & psx_reset;
         kk        := ti_word(6 downto 2) - 1;   -- track of trackinfo word ti_word (>= 4)

         if (reset_n_74a = '0') then
            st       <= S_WAITRESET;
            cd_ready <= '0';
            has_cd   <= '0';
            dsr_read <= '0';
            cmd      := C_IDLE;
            pend     := (others => '0');
            cnt      := (others => '0');
            chg      := '1';
            r_valid  <= '0';
         else
            case (st) is

               when S_WAITRESET =>              -- no handshake with the clk1x side still open
                  k        <= (others => '0');
                  audio_cd <= '0';
                  if (ti_ack_s(2) = ti_req and sec_ack_s(2) = sec_tog) then
                     st <= S_CLEAR;
                  end if;

               when S_CLEAR =>                  -- all track entries to 0 sectors
                  tab_we <= '1';
                  tab_wa <= std_logic_vector(k);
                  tab_d  <= (others => '0');
                  k      <= k + 1;
                  if (k = 31) then
                     dt_entry       <= (others => '0');
                     datatable_addr <= (others => '0');
                     dt_wait        <= 3;
                     st             <= S_DT_ID;
                  end if;

               -- data slot table: for each entry with a track id, sectors = size / 2352
               when S_DT_ID =>
                  if (dt_wait > 0) then
                     dt_wait <= dt_wait - 1;
                  else
                     dt_id          <= unsigned(datatable_q(15 downto 0));
                     datatable_addr <= "0000" & std_logic_vector(dt_entry) & '1';
                     dt_wait        <= 3;
                     st             <= S_DT_SIZE;
                  end if;

               when S_DT_SIZE =>
                  if (dt_wait > 0) then
                     dt_wait <= dt_wait - 1;
                  else
                     divide(div_n, div_sel, div_r, div_cnt, unsigned(datatable_q), SECTOR_BYTES);
                     st <= S_DT_CHECK;
                  end if;

               when S_DT_CHECK =>
                  if (dt_id >= TRACK_ID_BASE - 1 and dt_id < TRACK_ID_BASE + MAX_TRACKS and div_n /= 0) then
                     div_ret <= S_DT_STORE;
                     st      <= S_DIV;
                  else
                     st      <= S_DT_NEXT;
                  end if;

               when S_DT_STORE =>
                  tab_we <= '1';
                  tab_wa <= std_logic_vector(resize(dt_id - TRACK_ID_BASE, 5));
                  if (dt_id = TRACK_ID_BASE - 1) then   -- audio CD: track 1
                     tab_wa   <= (others => '0');
                     audio_cd <= '1';
                  end if;
                  tab_d  <= std_logic_vector(div_n(19 downto 0));
                  st     <= S_DT_NEXT;

               when S_DT_NEXT =>
                  dt_entry       <= dt_entry + 1;
                  datatable_addr <= "0000" & std_logic_vector(dt_entry + 1) & '0';
                  dt_wait        <= 3;
                  if (dt_entry = 31) then
                     k      <= (others => '0');
                     tab_ra <= (others => '0');
                     total  <= to_unsigned(150, 20);
                     st     <= S_SUM_WAIT;
                  else
                     st     <= S_DT_ID;
                  end if;

               when S_SUM_WAIT =>               -- last table write has landed
                  if (dt_wait > 0) then
                     dt_wait <= dt_wait - 1;
                  else
                     st <= S_SUM;
                  end if;

               -- sectors -> first LBA of each track; the tracks end at the first missing one
               when S_SUM =>
                  tab_we <= '1';
                  tab_wa <= std_logic_vector(k);
                  tab_d  <= std_logic_vector(total);
                  if (unsigned(tab_q) = 0 or k = MAX_TRACKS) then
                     ntracks <= k;
                     dt_wait <= 3;
                     st      <= S_SUM_DONE;
                  else
                     total  <= total + unsigned(tab_q);
                     k      <= k + 1;
                     tab_ra <= std_logic_vector(k + 1);
                  end if;

               when S_SUM_DONE =>
                  if (dt_wait > 0) then
                     dt_wait <= dt_wait - 1;
                  else
                     cd_ready <= '1';
                     ti_word  <= (others => '0');
                     if (ntracks /= 0) then
                        has_cd <= '1';
                        st     <= S_TI_START;
                     else
                        st     <= S_IDLE;
                     end if;
                  end if;

               -- trackinfo, sent once the core is out of reset (it ignores trackinfo during reset)
               when S_TI_START =>
                  if (psxrst_s(1) = '0') then
                     st <= S_TI_RD;
                  end if;

               when S_TI_RD =>                  -- table entry needed by word ti_word
                  if (ti_word(1 downto 0) = "01") then tab_ra <= std_logic_vector(kk + 1); else tab_ra <= std_logic_vector(kk); end if;
                  st <= S_TI_NEXT;

               when S_TI_NEXT =>
                  ti_d    <= (others => '0');
                  msf_audio <= '0';
                  st        <= S_TI_SEND;
                  if (ti_word < 4) then
                     case (ti_word(1 downto 0)) is
                        when "00" => ti_d <= x"0000" & bcd_small(ntracks) & "000" & std_logic_vector(ntracks);
                        when "01" => ti_d <= x"000" & std_logic_vector(total);
                        when "10" =>
                           divide(div_n, div_sel, div_r, div_cnt, total, 75);
                           st <= S_TI_MSF;
                        when others => null;
                     end case;
                  elsif (kk = ntracks) then
                     st <= S_IDLE;
                  else
                     case (ti_word(1 downto 0)) is
                        when "00" =>            -- start (0 for track 1)
                           if (kk /= 0) then ti_d <= x"000" & tab_q; end if;
                        when "01" =>            -- end
                           ti_d <= x"000" & std_logic_vector(unsigned(tab_q) - 1);
                        when "10" =>            -- MM:SS of start + 150 (track 1: of 150), audio flag
                           if (kk = 0) then
                              divide(div_n, div_sel, div_r, div_cnt, to_unsigned(150, 20), 75);
                              msf_audio <= audio_cd;
                           else
                              divide(div_n, div_sel, div_r, div_cnt, unsigned(tab_q) + 150, 75);
                              msf_audio <= '1';
                           end if;
                           st <= S_TI_MSF;
                        when others => null;    -- reserved word; the core stores the entry on this one
                     end case;
                  end if;

               -- ti_d <= audio << 16 | BCD(minutes) << 8 | BCD(seconds) of the LBA in div_n
               when S_TI_MSF =>                 -- LBA / 75 = seconds
                  div_ret <= S_TI_MSF_S;
                  st      <= S_DIV;

               when S_TI_MSF_S =>               -- seconds / 60 = minutes, seconds
                  divide(div_n, div_sel, div_r, div_cnt, div_n, 60);
                  div_ret <= S_TI_MSF_M;
                  st      <= S_DIV;

               when S_TI_MSF_M =>               -- minutes / 10
                  msf_sec <= resize(div_r, 8);
                  divide(div_n, div_sel, div_r, div_cnt, div_n(7 downto 0), 10);
                  div_ret <= S_TI_MSF_B;
                  st      <= S_DIV;

               when S_TI_MSF_B =>               -- seconds / 10
                  msf_mbcd <= std_logic_vector(div_n(3 downto 0)) & std_logic_vector(div_r(3 downto 0));
                  divide(div_n, div_sel, div_r, div_cnt, msf_sec, 10);
                  div_ret <= S_TI_MSF_E;
                  st      <= S_DIV;

               when S_TI_MSF_E =>
                  ti_d <= x"000" & "000" & msf_audio & msf_mbcd & std_logic_vector(div_n(3 downto 0)) & std_logic_vector(div_r(3 downto 0));
                  st     <= S_TI_SEND;

               when S_TI_SEND =>
                  ti_a   <= std_logic_vector(ti_word);
                  ti_req <= not ti_req;
                  st     <= S_TI_WAIT;

               when S_TI_WAIT =>
                  if (ti_ack_s(2) = ti_req) then
                     ti_word <= ti_word + 1;
                     st      <= S_TI_RD;
                  end if;

               -- sector requests
               when S_IDLE =>
                  if (req_s(1) = '1') then
                     if (unsigned(cd_hps_lba(31 downto 20)) /= 0 or unsigned(cd_hps_lba(19 downto 0)) < 150 or ntracks = 0) then
                        sec_zero <= '1';
                        st       <= S_SECT_DONE;
                     else
                        lba    <= unsigned(cd_hps_lba(19 downto 0));
                        start  <= to_unsigned(150, 20);
                        k      <= to_unsigned(1, 5);
                        tab_ra <= std_logic_vector(to_unsigned(1, 5));
                        st     <= S_FIND;
                     end if;
                  end if;

               when S_FIND =>                   -- tab_q = first LBA of track k (k = ntracks: end of disc)
                  if (lba < unsigned(tab_q)) then
                     st <= S_LOOK;              -- track k: start .. tab_q - 1
                  elsif (k = ntracks) then      -- past the last track
                     sec_zero <= '1';
                     st       <= S_SECT_DONE;
                  else
                     start  <= unsigned(tab_q);
                     k      <= k + 1;
                     tab_ra <= std_logic_vector(k + 1);
                  end if;

               when S_LOOK =>
                  if (r_valid = '1' and r_track = k and lba >= hl and lba < hl + cnt) then
                     -- in the ring: drop the sectors before it (fewer than cnt), send it
                     d   := lba - hl;
                     hs  := hs + d(SLOT_BITS - 1 downto 0);
                     hl  := lba;
                     cnt := cnt - d(SLOT_BITS downto 0);
                     chg := '1';
                     serve_base <= to_unsigned(to_integer(hs) * (SECTOR_BYTES / 4), BUF_AW);
                     sec_zero   <= '0';
                     st         <= S_SECT_DONE;
                  elsif (cmd = C_IDLE) then
                     -- not there and no read running (it may have brought it, see above): a new ring at lba
                     r_valid <= '1';
                     r_track <= k;
                     r_start <= start;
                     r_end   <= unsigned(tab_q);
                     hl   := lba;
                     hs   := (others => '0');
                     cnt  := (others => '0');
                     chg  := '1';
                     c_lba <= lba;
                     c_n   <= to_unsigned(1, SLOT_BITS + 1);
                     c_ws  <= (others => '0');
                     pend := to_unsigned(1, SLOT_BITS + 1);
                     cmd  := C_PREP;
                  end if;

               when S_SECT_DONE =>
                  sec_tog <= not sec_tog;
                  st      <= S_SECT_WAIT;

               when S_SECT_WAIT =>              -- the core has taken the whole sector: its slot is free
                  if (sec_ack_s(2) = sec_tog) then
                     if (sec_zero = '0') then
                        hs  := hs + 1;
                        hl  := hl + 1;
                        cnt := cnt - 1;
                        chg := '1';
                     end if;
                     st <= S_IDLE;
                  end if;

               when S_DIV =>
                  rr := div_r & div_n(31);
                  if (rr >= div_d) then
                     div_r <= resize(rr - div_d, 12);
                     div_n <= div_n(30 downto 0) & '1';
                  else
                     div_r <= rr(11 downto 0);
                     div_n <= div_n(30 downto 0) & '0';
                  end if;
                  div_cnt <= div_cnt - 1;
                  if (div_cnt = 1) then
                     st <= div_ret;
                  end if;

            end case;

            -- read-ahead: the next sectors of the ring's track, as worked out from last clock's ring (pf_*, see
            -- below; not if the ring has changed since), not while a request is being looked up (that may start the
            -- ring anew)
            if (cmd = C_IDLE and pf_ok = '1' and ring_chg = '0' and (st = S_IDLE or st = S_SECT_DONE or st = S_SECT_WAIT)) then
               c_lba <= pf_lba;
               c_n   <= pf_n;
               c_ws  <= pf_ws;
               pend  := pf_n;
               cmd   := C_PREP;
            end if;

            -- the command (target data slot read): its sectors join the ring when it is done
            case (cmd_st) is                  -- (as registered: a command set up above starts with C_PREP next clock)
               when C_IDLE  => null;
               when C_PREP  =>                  -- file offset, ring address, length of c_n sectors from c_lba
                  dsr_offset     <= std_logic_vector(resize((c_lba - r_start) * to_unsigned(SECTOR_BYTES, 12), 32));
                  dsr_bridgeaddr <= std_logic_vector(to_unsigned(16#40000000# + to_integer(c_ws) * SECTOR_BYTES, 32));
                  dsr_length     <= std_logic_vector(to_unsigned(to_integer(c_n) * SECTOR_BYTES, 32));
                  cmd            := C_START;
               when C_START =>
                  dsr_read <= '1';
                  cmd      := C_WAIT0;
               when C_WAIT0 =>                  -- taken (done cleared)
                  if (dsr_done = '0') then
                     dsr_read <= '0';
                     cmd      := C_WAIT1;
                  end if;
               when C_WAIT1 =>
                  if (dsr_done = '1') then
                     cnt  := cnt + pend;
                     pend := (others => '0');
                     cmd  := C_IDLE;
                     chg  := '1';
                  end if;
            end case;
         end if;

         head_lba  <= hl;
         head_slot <= hs;
         r_cnt     <= cnt;
         r_pend    <= pend;
         cmd_st    <= cmd;
         ring_chg  <= chg;
      end if;
   end process;

   -- the next read-ahead from the registered ring: n = min(free slots, slots to the ring's end, KMAX, sectors left in
   -- the track), taken when it is KMAX (fewer, larger commands: each one's overhead counts once) or limited by the
   -- ring's / the track's end, or when the ring is nearly empty
   process (clk_74a)
      variable ws                    : unsigned(SLOT_BITS - 1 downto 0);
      variable nxt, rest             : unsigned(19 downto 0);
      variable restc, n, free, toend : unsigned(SLOT_BITS downto 0);
   begin
      if rising_edge(clk_74a) then
         ws    := head_slot + r_cnt(SLOT_BITS - 1 downto 0);
         nxt   := head_lba + r_cnt;
         rest  := r_end - nxt;
         if (rest > SLOTS) then restc := to_unsigned(SLOTS, SLOT_BITS + 1); else restc := rest(SLOT_BITS downto 0); end if;
         free  := to_unsigned(SLOTS, SLOT_BITS + 1) - r_cnt;
         toend := to_unsigned(SLOTS, SLOT_BITS + 1) - ('0' & ws);
         n := free;
         if (toend < n) then n := toend; end if;
         if (KMAX < n) then n := to_unsigned(KMAX, SLOT_BITS + 1); end if;
         if (restc < n) then n := restc; end if;
         pf_lba <= nxt;
         pf_ws  <= ws;
         pf_n   <= n;
         pf_ok  <= '0';
         if (r_valid = '1' and n /= 0 and (n = KMAX or n = restc or n = toend or r_cnt <= 1)) then
            pf_ok <= '1';
         end if;
      end if;
   end process;

   ---------------------------------------------------------------- clk1x: trackinfo writes, sector transfer
   -- trackinfo address / data: set on clk_74a before the request toggle and held until it is acknowledged
   trackinfo_data <= ti_d;
   trackinfo_addr <= ti_a;

   process (clk1x)
   begin
      if rising_edge(clk1x) then
         ti_req_s        <= ti_req_s(1 downto 0) & ti_req;
         sec_tog_s       <= sec_tog_s(1 downto 0) & sec_tog;
         trackinfo_write <= '0';
         cd_hps_write    <= '0';

         if (ti_req_s(2) /= ti_ack) then
            trackinfo_write <= not psx_reset;
            ti_ack          <= ti_req_s(2);
         end if;

         if (sending = '0') then
            if (sec_tog_s(2) /= sec_ack) then
               sending    <= '1';
               zero_1x    <= sec_zero;
               cd_hps_ack <= '1';
               buf_ra     <= serve_base;
               buf_end    <= serve_base + SECTOR_BYTES / 4;
               phase      <= 0;
               gap        <= 3;
            end if;
         elsif (gap > 0) then
            gap <= gap - 1;
         else
            -- 4 clocks per 32-bit word: low half, pause, high half, pause
            phase <= (phase + 1) mod 4;
            case (phase) is
               when 0 =>
                  cd_hps_write <= '1';
                  cd_hps_data  <= buf_q(15 downto 0);
                  if (zero_1x = '1') then cd_hps_data <= (others => '0'); end if;
               when 2 =>
                  cd_hps_write <= '1';
                  cd_hps_data  <= buf_q(31 downto 16);
                  if (zero_1x = '1') then cd_hps_data <= (others => '0'); end if;
                  buf_ra       <= buf_ra + 1;
               when 3 =>
                  if (buf_ra = buf_end) then
                     sending    <= '0';
                     cd_hps_ack <= '0';
                     sec_ack    <= sec_tog_s(2);
                  end if;
               when others => null;
            end case;
         end if;
      end if;
   end process;

end architecture;
