-- Analogue Pocket glue around the PSX core (VHDL part of the Pocket top level, instantiated by core_top.v):
--  * the PSX core with all MiSTer-only options tied off (psx_pocket_core, generated, see pocket/tools)
--  * VRAM / memory card buffer: the core's DDR3-style port served from the two PSRAMs (vram_psram)
--  * BIOS download: APF bridge writes (data slot 0, clk_74a) -> FIFO -> SDRAM channel 3 at the BIOS address
--  * SPU RAM: the core's spuram port on SDRAM channel 3 once the BIOS is loaded
--  * CD: track files streamed sector by sector from APF data slots (pocket_cd)
--  * memory card 1: the APF save slot (0x20000000) loaded into / read back from the core's card in PSRAM
--    (pocket_memcard)
--  * controller 1 (APF cont1_key) -> PSX digital pad, L2 / R2 also from button combos (pocket_buttons) as chosen in
--    Core Settings (interact.json: bridge writes to 0x80000000 / 0x80000004)
--  * video: core output (clkvid, 53.69 MHz, one pixel per video_ce) -> Pocket scaler signals on clkpix (half of
--    clkvid, same PLL; APF takes pixel clocks up to ~50 MHz), video_skip on clocks without a new pixel, scaler slot
--    0-4 = width 256/320/368/512/640 sent when DE falls
-- The SDRAM controller (rtl/sdram.sv), the PLL, the APF bridge and I2S audio are in core_top.v.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library altera_mf;
use altera_mf.altera_mf_components.all;

entity pocket_psx is
   port
   (
      clk_74a            : in  std_logic;
      clk1x              : in  std_logic;
      clk2x              : in  std_logic;
      clk3x              : in  std_logic;
      clkvid             : in  std_logic;
      clkpix             : in  std_logic;   -- clkvid / 2, phase aligned (Pocket pixel clock)
      -- 74a domain control
      reset_n_74a        : in  std_logic;   -- APF reset_n (released after the data slots are loaded)
      pll_locked         : in  std_logic;
      loading_74a        : in  std_logic;   -- APF is writing data slots
      -- APF bridge (clk_74a)
      bridge_addr        : in  std_logic_vector(31 downto 0);
      bridge_wr          : in  std_logic;
      bridge_wr_data     : in  std_logic_vector(31 downto 0);
      bridge_rd          : in  std_logic;
      memcard_rd_data    : out std_logic_vector(31 downto 0);   -- bridge read data for 0x2xxxxxxx
      -- APF data slot table and target command "data slot read" (core_bridge_cmd, clk_74a), for the CD
      datatable_addr     : out std_logic_vector(9 downto 0);
      datatable_q        : in  std_logic_vector(31 downto 0);
      datatable_wren     : out std_logic;
      datatable_data     : out std_logic_vector(31 downto 0);
      dsr_read           : out std_logic;
      dsr_id             : out std_logic_vector(15 downto 0);
      dsr_offset         : out std_logic_vector(31 downto 0);
      dsr_bridgeaddr     : out std_logic_vector(31 downto 0);
      dsr_length         : out std_logic_vector(31 downto 0);
      dsr_done           : in  std_logic;
      cont1_key          : in  std_logic_vector(31 downto 0);
      loader_overflow    : out std_logic;   -- sticky, for debugging (BIOS words lost)
      -- SDRAM controller (rtl/sdram.sv in core_top.v), channels as on MiSTer: ch1 = core reads, ch2 = core
      -- writes, ch3 = BIOS download then SPU RAM
      sdram_init         : out std_logic;
      ram_refresh        : out std_logic;
      ram_addr           : out std_logic_vector(26 downto 0);
      ram_din            : out std_logic_vector(31 downto 0);
      ram_be             : out std_logic_vector(3 downto 0);
      ram_rd_req         : out std_logic;
      ram_wr_req         : out std_logic;
      ram_dma            : out std_logic;
      ram_cntDMA         : out std_logic_vector(1 downto 0);
      ram_cache          : out std_logic;
      ram_dout32         : in  std_logic_vector(31 downto 0);
      ram_rd_ready       : in  std_logic;
      ram_wr_ready       : in  std_logic;
      cache_wr           : in  std_logic_vector(3 downto 0);
      cache_data         : in  std_logic_vector(31 downto 0);
      cache_addr         : in  std_logic_vector(7 downto 0);
      dma_wr             : in  std_logic;
      dma_reqprocessed   : in  std_logic;
      dma_data           : in  std_logic_vector(31 downto 0);
      dmafifo_adr        : out std_logic_vector(26 downto 0);
      dmafifo_data       : out std_logic_vector(31 downto 0);
      dmafifo_empty      : out std_logic;
      dmafifo_read       : in  std_logic;
      ch3_addr           : out std_logic_vector(26 downto 0);
      ch3_din            : out std_logic_vector(31 downto 0);
      ch3_be             : out std_logic_vector(3 downto 0);
      ch3_req            : out std_logic;
      ch3_rnw            : out std_logic;
      ch3_dout           : in  std_logic_vector(31 downto 0);
      ch3_ready          : in  std_logic;
      -- PSRAM pins (dq split into out / output enable / in; joined in core_top.v)
      cram0_a            : out std_logic_vector(21 downto 16);
      cram0_dq_out       : out std_logic_vector(15 downto 0);
      cram0_dq_oe        : out std_logic;
      cram0_dq_in        : in  std_logic_vector(15 downto 0);
      cram0_clk          : out std_logic;
      cram0_adv_n        : out std_logic;
      cram0_cre          : out std_logic;
      cram0_ce0_n        : out std_logic;
      cram0_ce1_n        : out std_logic;
      cram0_oe_n         : out std_logic;
      cram0_we_n         : out std_logic;
      cram0_ub_n         : out std_logic;
      cram0_lb_n         : out std_logic;
      cram1_a            : out std_logic_vector(21 downto 16);
      cram1_dq_out       : out std_logic_vector(15 downto 0);
      cram1_dq_oe        : out std_logic;
      cram1_dq_in        : in  std_logic_vector(15 downto 0);
      cram1_clk          : out std_logic;
      cram1_adv_n        : out std_logic;
      cram1_cre          : out std_logic;
      cram1_ce0_n        : out std_logic;
      cram1_ce1_n        : out std_logic;
      cram1_oe_n         : out std_logic;
      cram1_we_n         : out std_logic;
      cram1_ub_n         : out std_logic;
      cram1_lb_n         : out std_logic;
      -- video to the Pocket scaler (clkvid domain)
      video_rgb          : out std_logic_vector(23 downto 0) := (others => '0');
      video_de           : out std_logic := '0';
      video_skip         : out std_logic := '0';
      video_vs           : out std_logic := '0';
      video_hs           : out std_logic := '0';
      -- audio: 48 kHz samples for sound_i2s (clk1x domain, signed), one per frame of its word clock audio_lrck
      audio_lrck         : in  std_logic;
      sound_left         : out std_logic_vector(15 downto 0);
      sound_right        : out std_logic_vector(15 downto 0)
   );
end entity;

architecture arch of pocket_psx is

   -- BIOS goes to SDRAM byte address 0x800000 (as on MiSTer), SPU RAM to 0x1000000
   constant BIOS_BASE    : unsigned(26 downto 0) := to_unsigned(16#0800000#, 27);
   constant SPURAM_BASE  : unsigned(26 downto 0) := to_unsigned(16#1000000#, 27);

   -- resets (two-flop synchronisers)
   signal rst74_s        : std_logic_vector(1 downto 0) := (others => '1');
   signal resetsync      : std_logic_vector(2 downto 0) := (others => '1');
   signal psx_reset      : std_logic := '1';
   signal loading_1x     : std_logic_vector(2 downto 0) := (others => '0');

   -- BIOS loader FIFO: {word address(16:0), data(31:0)}
   signal fifo_wrreq     : std_logic := '0';
   signal fifo_data      : std_logic_vector(48 downto 0) := (others => '0');
   signal fifo_wrfull    : std_logic;
   signal fifo_rdreq     : std_logic := '0';
   signal fifo_q         : std_logic_vector(48 downto 0);
   signal fifo_rdempty   : std_logic;
   signal overflow       : std_logic := '0';
   type tLoad is (L_IDLE, L_READ, L_WRITE, L_WAIT);
   signal lstate         : tLoad := L_IDLE;
   signal l_addr         : unsigned(26 downto 0) := (others => '0');
   signal l_data         : std_logic_vector(31 downto 0) := (others => '0');
   signal l_req          : std_logic := '0';
   signal loader_busy    : std_logic;

   -- bring-up diagnostics (pocket_diag hex page, Select held): snapshots of the core's debug outputs
   -- (psx_top debug_out), refreshed every 2^24 clk1x (~0.5 s); 4 PC samples spread over that window
   constant DIAG_PAGE    : boolean := false;   -- hex diagnostics page while Select is held (bring-up builds)
   constant AUDIO_INTERP : boolean := false;   -- 8-tap 44.1 -> 48 kHz interpolation (175 ALMs, 1 M10K, 1 DSP)
   constant DIAG_ROWS    : integer := 8;
   signal core_debug     : std_logic_vector(255 downto 0);
   signal dbg_tick       : unsigned(23 downto 0) := (others => '0');
   signal diag_we        : std_logic := '0';
   signal diag_wa        : std_logic_vector(2 downto 0) := (others => '0');
   signal diag_wd        : std_logic_vector(31 downto 0) := (others => '0');
   signal snd_l, snd_r   : std_logic_vector(15 downto 0);
   signal snd_tick       : std_logic;
   signal snd_valid      : std_logic;
   signal core_rgb       : std_logic_vector(23 downto 0);
   signal core_de, core_skip, core_vs, core_hs : std_logic;

   -- CD
   signal cd_ready       : std_logic;
   signal cd_dt_addr     : std_logic_vector(9 downto 0);
   signal has_cd         : std_logic;
   signal cdready_1x     : std_logic_vector(2 downto 0) := (others => '0');
   signal hascd_1x       : std_logic_vector(1 downto 0) := (others => '0');
   signal cd_hps_req     : std_logic;
   signal cd_hps_lba     : std_logic_vector(31 downto 0);
   signal cd_hps_ack     : std_logic;
   signal cd_hps_write   : std_logic;
   signal cd_hps_data    : std_logic_vector(15 downto 0);
   signal trackinfo_data : std_logic_vector(31 downto 0);
   signal trackinfo_addr : std_logic_vector(8 downto 0);
   signal trackinfo_write: std_logic;

   -- memory card / PSRAM
   signal vramrst_2x     : std_logic_vector(1 downto 0) := (others => '1');
   signal mc_busy        : std_logic;
   signal mcbusy_1x      : std_logic_vector(2 downto 0) := (others => '1');
   signal mc_overflow    : std_logic;
   signal aux_req, aux_we, aux_done, card_write : std_logic;
   signal aux_addr       : std_logic_vector(20 downto 0);
   signal aux_din, aux_dout : std_logic_vector(31 downto 0);

   -- core memory interfaces
   signal core_ram_Adr   : std_logic_vector(24 downto 0);
   signal core_ram_ena   : std_logic;
   signal core_ram_rnw   : std_logic;
   signal core_dmafifo_adr : std_logic_vector(22 downto 0);
   signal ddr_burstcnt   : std_logic_vector(7 downto 0);
   signal ddr_addr       : std_logic_vector(28 downto 0);
   signal ddr_dout       : std_logic_vector(63 downto 0);
   signal ddr_dout_ready : std_logic;
   signal ddr_rd         : std_logic;
   signal ddr_din        : std_logic_vector(63 downto 0);
   signal ddr_be         : std_logic_vector(7 downto 0);
   signal ddr_we         : std_logic;
   signal ddr_busy       : std_logic;
   signal spu_dataWrite  : std_logic_vector(31 downto 0);
   signal spu_Adr        : std_logic_vector(18 downto 0);
   signal spu_be         : std_logic_vector(3 downto 0);
   signal spu_rnw        : std_logic;
   signal spu_ena        : std_logic;

   -- controller
   signal keys_s1, keys_s2 : std_logic_vector(15 downto 0) := (others => '0');
   signal keys_pad       : std_logic_vector(15 downto 0);
   signal cfg_r2_74, cfg_l2_74 : std_logic_vector(1 downto 0) := "00";   -- Core Settings "R2 button" / "L2 button"
   signal cfg_s1, cfg_s2 : std_logic_vector(3 downto 0) := (others => '0');

   -- video
   signal hblank, vblank, hsync, vsync, video_ce : std_logic;
   signal video_r, video_g, video_b : std_logic_vector(7 downto 0);
   signal video_hResMode : std_logic_vector(2 downto 0);

begin

   ---------------------------------------------------------------- resets
   process (clk1x)
   begin
      if rising_edge(clk1x) then
         loading_1x <= loading_1x(1 downto 0) & loading_74a;
         resetsync  <= resetsync(1 downto 0) & ((not reset_n_74a) or (not pll_locked));
         cdready_1x <= cdready_1x(1 downto 0) & cd_ready;
         hascd_1x   <= hascd_1x(0) & has_cd;
         mcbusy_1x  <= mcbusy_1x(1 downto 0) & mc_busy;
         -- the core is held in reset while APF loads data slots, until the loader FIFOs are drained and until
         -- the CD track table is built
         psx_reset  <= resetsync(2) or loading_1x(2) or (not fifo_rdempty) or loader_busy or (not cdready_1x(2)) or
                       mcbusy_1x(2);
      end if;
   end process;
   sdram_init <= not pll_locked;

   ---------------------------------------------------------------- BIOS loader
   -- APF writes 32-bit words of data slot 0 at bridge addresses 0x00000000.. (bridge_endian_little = 1)
   process (clk_74a)
   begin
      if rising_edge(clk_74a) then
         fifo_wrreq <= '0';
         if (bridge_wr = '1' and bridge_addr(31 downto 28) = x"0") then
            if (fifo_wrfull = '1') then
               overflow <= '1';
            else
               fifo_wrreq <= '1';
               fifo_data  <= bridge_addr(18 downto 2) & bridge_wr_data;
            end if;
         end if;
      end if;
   end process;
   loader_overflow <= overflow or mc_overflow;

   iloadfifo : dcfifo
   generic map
   (
      intended_device_family => "Cyclone V", lpm_numwords => 256, lpm_showahead => "OFF", lpm_type => "dcfifo",
      lpm_width => 49, lpm_widthu => 8, overflow_checking => "ON", rdsync_delaypipe => 4,
      underflow_checking => "ON", use_eab => "ON", wrsync_delaypipe => 4
   )
   port map
   (
      data => fifo_data, rdclk => clk1x, rdreq => fifo_rdreq, wrclk => clk_74a, wrreq => fifo_wrreq,
      q => fifo_q, rdempty => fifo_rdempty, wrfull => fifo_wrfull
   );

   process (clk1x)
   begin
      if rising_edge(clk1x) then
         fifo_rdreq   <= '0';
         case (lstate) is
            when L_IDLE =>
               if (fifo_rdempty = '0' and fifo_rdreq = '0') then
                  fifo_rdreq <= '1';
                  lstate     <= L_READ;
               end if;
            when L_READ =>                 -- q is valid one clock after rdreq
               lstate <= L_WRITE;
            when L_WRITE =>
               l_addr   <= BIOS_BASE + resize(unsigned(fifo_q(48 downto 32)) & "00", 27);
               l_data   <= fifo_q(31 downto 0);
               l_req    <= '1';
               lstate   <= L_WAIT;
            when L_WAIT =>
               l_req <= '0';             -- sdram.sv takes one request per one-cycle pulse
               if (ch3_ready = '1') then
                  lstate <= L_IDLE;
               end if;
         end case;
      end if;
   end process;

   loader_busy <= '0' when (lstate = L_IDLE) else '1';

   -- channel 3: loader while loading (core in reset), SPU RAM afterwards. sdram.sv takes one request per
   -- one-cycle ch3_req pulse and answers with one ch3_ready pulse, like the core's spuram_ena/spuram_done.
   ch3_addr <= std_logic_vector(l_addr) when (psx_reset = '1') else std_logic_vector(SPURAM_BASE + resize(unsigned(spu_Adr), 27));
   ch3_din  <= l_data                   when (psx_reset = '1') else spu_dataWrite;
   ch3_be   <= "1111"                   when (psx_reset = '1') else spu_be;
   ch3_req  <= l_req                    when (psx_reset = '1') else spu_ena;
   ch3_rnw  <= '0'                      when (psx_reset = '1') else spu_rnw;

   ---------------------------------------------------------------- controller 1
   -- Core Settings pickers (interact.json, written by APF when changed and at start for persisted values; 0 until
   -- then = the default): value in the low bits, also taken from the top byte in case it arrives byte-swapped
   process (clk_74a)
   begin
      if rising_edge(clk_74a) then
         if (bridge_wr = '1' and bridge_addr = x"80000000") then
            cfg_r2_74 <= bridge_wr_data(1 downto 0) or bridge_wr_data(25 downto 24);
         end if;
         if (bridge_wr = '1' and bridge_addr = x"80000004") then
            cfg_l2_74 <= bridge_wr_data(1 downto 0) or bridge_wr_data(25 downto 24);
         end if;
      end if;
   end process;

   process (clk1x)
   begin
      if rising_edge(clk1x) then
         keys_s1 <= cont1_key(15 downto 0);
         keys_s2 <= keys_s1;
         cfg_s1  <= cfg_r2_74 & cfg_l2_74;   -- static settings: a plain two-register sync is enough
         cfg_s2  <= cfg_s1;
      end if;
   end process;

   ibuttons : entity work.pocket_buttons
   port map (clk => clk1x, cfg_r2 => cfg_s2(3 downto 2), cfg_l2 => cfg_s2(1 downto 0), keys_in => keys_s2, keys_out => keys_pad);

   ---------------------------------------------------------------- the core
   ipsx : entity work.psx_pocket_core
   port map
   (
      clk1x => clk1x, clk2x => clk2x, clk3x => clk3x, clkvid => clkvid,
      reset => psx_reset,
      hasCD => hascd_1x(1),
      isPaused => open,
      ram_refresh => ram_refresh, ram_dataWrite => ram_din, ram_dataRead32 => ram_dout32, ram_Adr => core_ram_Adr,
      ram_cntDMA => ram_cntDMA, ram_be => ram_be, ram_rnw => core_ram_rnw, ram_ena => core_ram_ena, ram_dma => ram_dma,
      ram_cache => ram_cache, ram_done => (ram_rd_ready or ram_wr_ready),
      ram_dmafifo_adr => core_dmafifo_adr, ram_dmafifo_data => dmafifo_data, ram_dmafifo_empty => dmafifo_empty,
      ram_dmafifo_read => dmafifo_read,
      cache_wr => cache_wr, cache_data => cache_data, cache_addr => cache_addr,
      dma_wr => dma_wr, dma_reqprocessed => dma_reqprocessed, dma_data => dma_data,
      DDRAM_BUSY => ddr_busy, DDRAM_BURSTCNT => ddr_burstcnt, DDRAM_ADDR => ddr_addr, DDRAM_DOUT => ddr_dout,
      DDRAM_DOUT_READY => ddr_dout_ready, DDRAM_RD => ddr_rd, DDRAM_DIN => ddr_din, DDRAM_BE => ddr_be, DDRAM_WE => ddr_we,
      trackinfo_data => trackinfo_data, trackinfo_addr => trackinfo_addr, trackinfo_write => trackinfo_write,
      resetFromCD => open,
      cd_hps_req => cd_hps_req, cd_hps_lba => cd_hps_lba, cd_hps_ack => cd_hps_ack, cd_hps_write => cd_hps_write,
      cd_hps_data => cd_hps_data,
      spuram_dataWrite => spu_dataWrite, spuram_Adr => spu_Adr, spuram_be => spu_be, spuram_rnw => spu_rnw,
      spuram_ena => spu_ena, spuram_dataRead => ch3_dout, spuram_done => (ch3_ready and not psx_reset),
      memcard_changed => open, saving_memcard => open,
      -- the card lives in PSRAM region 1 (loaded / read back by APF through pocket_memcard), so MiSTer's HPS
      -- load / save transfers are not used
      memcard1_load => '0', memcard_save => '0', memcard1_mounted => '0', memcard1_available => '1',
      memcard1_rd => open, memcard1_wr => open, memcard1_lba => open, memcard1_ack => '0', memcard1_write => '0',
      memcard1_addr => (others => '0'), memcard1_dataIn => (others => '0'), memcard1_dataOut => open,
      hsync => hsync, vsync => vsync, hblank => hblank, vblank => vblank, video_ce => video_ce,
      video_interlace => open, video_r => video_r, video_g => video_g, video_b => video_b,
      video_hResMode => video_hResMode,
      -- APF cont1_key: 0 up, 1 down, 2 left, 3 right, 4 A, 5 B, 6 X, 7 Y, 8 L1, 9 R1, 10 L2, 11 R2,
      -- 12 L3, 13 R3, 14 select, 15 start. PSX: cross = B, circle = A, square = Y, triangle = X.
      KeyUp => keys_pad(0), KeyDown => keys_pad(1), KeyLeft => keys_pad(2), KeyRight => keys_pad(3),
      KeyCircle => keys_pad(4), KeyCross => keys_pad(5), KeyTriangle => keys_pad(6), KeySquare => keys_pad(7),
      KeyL1 => keys_pad(8), KeyR1 => keys_pad(9), KeyL2 => keys_pad(10), KeyR2 => keys_pad(11),
      KeyL3 => keys_pad(12), KeyR3 => keys_pad(13), KeySelect => keys_pad(14), KeyStart => keys_pad(15),
      sound_out_left => snd_l, sound_out_right => snd_r, sound_tick => snd_tick, sound_valid => snd_valid,
      debug_out => core_debug
   );

   ram_addr    <= "00" & core_ram_Adr;
   ram_rd_req  <= core_ram_ena and core_ram_rnw;
   ram_wr_req  <= core_ram_ena and (not core_ram_rnw);
   dmafifo_adr <= "0000" & core_dmafifo_adr;

   -- APF data slot table (core_bridge_cmd, port a): pocket_cd reads the track sizes from it after each reset;
   -- afterwards the memory card's size is kept in it, entry 2 (data slot index 2 in data.json, not id 2) =
   -- words 4 (id) and 5 (size): APF saves a nonvolatile slot with the size in this table when the core exits,
   -- and without a save file loaded at start that size would be 0 (no save written)
   datatable_addr <= cd_dt_addr when (cd_ready = '0') else "0000000101";
   datatable_wren <= cd_ready;
   datatable_data <= x"00020000";   -- 128 KB memory card

   ---------------------------------------------------------------- CD image from APF data slots
   icd : entity work.pocket_cd
   generic map (TRACK_ID_BASE => 16, MAX_TRACKS => 24)
   port map
   (
      clk_74a => clk_74a, reset_n_74a => reset_n_74a, cd_ready => cd_ready, has_cd => has_cd,
      datatable_addr => cd_dt_addr, datatable_q => datatable_q,
      dsr_read => dsr_read, dsr_id => dsr_id, dsr_offset => dsr_offset, dsr_bridgeaddr => dsr_bridgeaddr,
      dsr_length => dsr_length, dsr_done => dsr_done,
      bridge_addr => bridge_addr, bridge_wr => bridge_wr, bridge_wr_data => bridge_wr_data,
      clk1x => clk1x, psx_reset => psx_reset,
      cd_hps_req => cd_hps_req, cd_hps_lba => cd_hps_lba, cd_hps_ack => cd_hps_ack, cd_hps_write => cd_hps_write,
      cd_hps_data => cd_hps_data,
      trackinfo_data => trackinfo_data, trackinfo_addr => trackinfo_addr, trackinfo_write => trackinfo_write
   );

   ---------------------------------------------------------------- memory card 1 file <-> PSRAM
   process (clk2x)
   begin
      if rising_edge(clk2x) then
         vramrst_2x <= vramrst_2x(0) & (not pll_locked);
      end if;
   end process;

   imemcard : entity work.pocket_memcard
   port map
   (
      clk_74a => clk_74a, bridge_addr => bridge_addr, bridge_wr => bridge_wr, bridge_wr_data => bridge_wr_data,
      bridge_rd => bridge_rd, bridge_rd_data => memcard_rd_data, overflow => mc_overflow,
      clk2x => clk2x, reset_2x => vramrst_2x(1), busy => mc_busy,
      aux_req => aux_req, aux_we => aux_we, aux_addr => aux_addr, aux_din => aux_din, aux_dout => aux_dout,
      aux_done => aux_done, card_write => card_write
   );

   ---------------------------------------------------------------- VRAM / memory card buffer in PSRAM
   -- not reset with the core: APF loads the memory card into it while the core is held in reset
   ivram : entity work.vram_psram
   generic map (CLOCK_SPEED => 67.7376)
   port map
   (
      clk => clk2x, reset => vramrst_2x(1),
      ddr_busy => ddr_busy, ddr_burstcnt => ddr_burstcnt, ddr_addr => ddr_addr(24 downto 0), ddr_dout => ddr_dout,
      ddr_dout_ready => ddr_dout_ready, ddr_rd => ddr_rd, ddr_din => ddr_din, ddr_be => ddr_be, ddr_we => ddr_we,
      aux_req => aux_req, aux_we => aux_we, aux_addr => aux_addr, aux_din => aux_din, aux_dout => aux_dout,
      aux_done => aux_done, card_write => card_write,
      cram0_a => cram0_a, cram0_dq_out => cram0_dq_out, cram0_dq_oe => cram0_dq_oe, cram0_dq_in => cram0_dq_in,
      cram0_clk => cram0_clk, cram0_adv_n => cram0_adv_n, cram0_cre => cram0_cre, cram0_ce0_n => cram0_ce0_n,
      cram0_ce1_n => cram0_ce1_n, cram0_oe_n => cram0_oe_n, cram0_we_n => cram0_we_n, cram0_ub_n => cram0_ub_n,
      cram0_lb_n => cram0_lb_n,
      cram1_a => cram1_a, cram1_dq_out => cram1_dq_out, cram1_dq_oe => cram1_dq_oe, cram1_dq_in => cram1_dq_in,
      cram1_clk => cram1_clk, cram1_adv_n => cram1_adv_n, cram1_cre => cram1_cre, cram1_ce0_n => cram1_ce0_n,
      cram1_ce1_n => cram1_ce1_n, cram1_oe_n => cram1_oe_n, cram1_we_n => cram1_we_n, cram1_ub_n => cram1_ub_n,
      cram1_lb_n => cram1_lb_n
   );

   ---------------------------------------------------------------- video to the Pocket scaler
   ivideo : entity work.pocket_video
   port map
   (
      clkvid => clkvid, hblank => hblank, vblank => vblank, hsync => hsync, vsync => vsync, video_ce => video_ce,
      video_r => video_r, video_g => video_g, video_b => video_b, video_hResMode => video_hResMode,
      clkpix => clkpix, video_rgb => core_rgb, video_de => core_de, video_skip => core_skip,
      video_vs => core_vs, video_hs => core_hs
   );

   ---------------------------------------------------------------- audio, bring-up diagnostics screen
   -- the SPU's samples in order on the 44.1 kHz grid (queue: the SPU may finish one late); with INTERP 44.1 kHz ->
   -- 48 kHz interpolated as well (otherwise sound_i2s takes the newest sample at each frame, as before)
   iaudio48 : entity work.pocket_audio48
   generic map (INTERP => AUDIO_INTERP)
   port map
   (
      clk => clk1x, tick => snd_tick, valid => snd_valid, in_l => snd_l, in_r => snd_r, lrck => audio_lrck,
      out_l => sound_left, out_r => sound_right
   );

   gdiag : if DIAG_PAGE generate
      -- snapshots for the diagnostics page (clk1x), written into pocket_diag's value RAM: row 1 = psx_top debug_out
      -- word 0 at the start of a 2^24 clk1x (~0.5 s) window, rows 2-8 = words 1-7 at its end, so the rows stay still
      -- (the layout of debug_out is described in psx_top: profiling rows with POCKET_PROFILE, row 8 = CPU PC)
      process (clk1x)
      begin
         if rising_edge(clk1x) then
            dbg_tick <= dbg_tick + 1;
            diag_we  <= '0';
            if (dbg_tick = x"000000") then
               diag_we <= '1';
               diag_wa <= "000";
               diag_wd <= core_debug(31 downto 0);
            elsif (dbg_tick(23 downto 3) = "111111111111111111111" and dbg_tick(2 downto 0) /= "111") then
               diag_we <= '1';                                   -- FFFFF8..FFFFFE -> words 1..7
               diag_wa <= std_logic_vector(dbg_tick(2 downto 0) + 1);
               case dbg_tick(2 downto 0) is
                  when "000"  => diag_wd <= core_debug( 63 downto  32);
                  when "001"  => diag_wd <= core_debug( 95 downto  64);
                  when "010"  => diag_wd <= core_debug(127 downto  96);
                  when "011"  => diag_wd <= core_debug(159 downto 128);
                  when "100"  => diag_wd <= core_debug(191 downto 160);
                  when "101"  => diag_wd <= core_debug(223 downto 192);
                  when others => diag_wd <= core_debug(255 downto 224);
               end case;
            end if;
         end if;
      end process;

      idiag : entity work.pocket_diag
      generic map (ROWS => DIAG_ROWS)
      port map
      (
         clkpix => clkpix, val_clk => clk1x, val_we => diag_we, val_addr => diag_wa, val_data => diag_wd,
         select_key => cont1_key(14),
         core_rgb => core_rgb, core_de => core_de, core_skip => core_skip, core_vs => core_vs, core_hs => core_hs,
         video_rgb => video_rgb, video_de => video_de, video_skip => video_skip, video_vs => video_vs, video_hs => video_hs
      );
   end generate;

   -- release build: the core's picture goes straight to the video output (registered like pocket_diag does)
   gnodiag : if not DIAG_PAGE generate
      process (clkpix)
      begin
         if rising_edge(clkpix) then
            video_rgb  <= core_rgb;
            video_de   <= core_de;
            video_skip <= core_skip;
            video_vs   <= core_vs;
            video_hs   <= core_hs;
         end if;
      end process;
   end generate;

end architecture;
