-- CD controller: command processing and drive, as a small engine plus the per-clock "fast path" (design:
-- docs/cd_engine.md). Replaces cd_top's command and drive processes with the same outputs, clock by clock:
--  * fast path (here): counters and their count / hold rules, pulses cleared again, delayed copies
--  * event vector ev_now: an edge at which the original processes do more than the fast path; at such an edge the
--    event vector and the state before the edge (snapshots) are latched and the engine starts; it runs the
--    original's actions for that edge (firmware rtl/cd_fw_pkg.vhd) and writes the results into the registers here
--  * while the engine runs (busy), this block's registers hold (no fast path, no new snapshots): its part of the
--    controller pauses ("virtual time"). Pulses and strobes to the rest of cd_top are shown only while the engine
--    is idle, so they last one clock. In hardware the rest of the core keeps running: its pulses that come
--    meanwhile are kept until the next edge this block handles, and the drive countdown goes on (see below).
--    In sim/cd_equiv (SIM_FREEZE) the testbench holds everything else too, so the comparison with the original is
--    cycle exact.
-- Engine I/O: reads = io read mux below (snapshots), writes = the io_we case below; rtl/cd_fw/cd_fw.py uses the
-- same numbers.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cd_ctrl is
   generic
   (
      SIM_FREEZE : boolean := false   -- simulation: the testbench holds everything else while the engine runs
   );
   port
   (
      clk                     : in  std_logic;   -- clk1x (simulation: the engine's own clock, see cd_top)
      ce                      : in  std_logic;
      reset                   : in  std_logic;
      hasCD                   : in  std_logic;
      region                  : in  std_logic_vector(1 downto 0);
      SS_reset                : in  std_logic;
      SS_wren                 : in  std_logic;
      dma_read                : in  std_logic;

      -- from the CPU interface
      beginCommand            : in  std_logic;
      newCmd                  : in  std_logic_vector(7 downto 0);
      CDROM_IRQFLAG           : in  std_logic_vector(4 downto 0);
      ackRead_valid           : in  std_logic;
      ackPendingIRQ           : in  std_logic;
      ackPendingSector        : in  std_logic;
      pendingDriveResponse    : in  std_logic_vector(7 downto 0);
      FifoParam_Dout          : in  std_logic_vector(7 downto 0);
      FifoParam_Empty         : in  std_logic;
      FifoParam_Wr            : in  std_logic;
      FifoResponse_Empty      : in  std_logic;

      -- from the sector datapath and the track table
      newCD                   : in  std_logic;
      isAudioCD               : in  std_logic;
      trackcountBCD           : in  std_logic_vector(7 downto 0);
      totalMinutesBCD         : in  std_logic_vector(7 downto 0);
      totalSecondsBCD         : in  std_logic_vector(7 downto 0);
      trackInfoA_lba          : in  std_logic_vector(18 downto 0);   -- trackInfo_DataOutA(18 downto 0)
      minutesBCD              : in  std_logic_vector(7 downto 0);
      secondsBCD              : in  std_logic_vector(7 downto 0);
      isAudio                 : in  std_logic;
      lastReadSector          : in  integer range 0 to 524287;
      header                  : in  std_logic_vector(31 downto 0);
      subheader               : in  std_logic_vector(31 downto 0);
      headerIsData            : in  std_logic;
      headerDataSector        : in  std_logic;
      nextSubdata             : in  std_logic_vector(95 downto 0);   -- byte i at 8i + 7 .. 8i
      peakvolumeL             : in  signed(15 downto 0);
      peakvolumeR             : in  signed(15 downto 0);
      UpdateSubchannelDone    : in  std_logic;
      fetchIdle               : in  std_logic;   -- sector fetch / process / copy idle and no read buffered
      sectorFetchIdle         : in  std_logic;
      bufferAhead             : in  std_logic;   -- sectorBufferSizes(writeSectorPointer + 4) /= 0
      XA_reset                : in  std_logic;
      XA_start                : in  std_logic;

      -- to the CPU interface
      FifoResponse_reset      : out std_logic;
      FifoResponse_Wr         : out std_logic;
      FifoResponse_Din        : out std_logic_vector(7 downto 0);
      FifoParam_Rd            : out std_logic;
      FifoParam_reset         : out std_logic;
      cmdPending              : out std_logic;
      cmdAck                  : out std_logic;
      cmdIRQ                  : out std_logic;
      driveAck                : out std_logic;
      getIDAck                : out std_logic;
      softReset               : out std_logic;
      errorResponseNext_new   : out std_logic;
      ackPendingIRQNext       : out std_logic;
      cdda_out8               : out std_logic;   -- the original's CDDA_outcnt = 8
      ackDrive                : out std_logic;
      ackDriveEnd             : out std_logic;
      ackRead                 : out std_logic;
      internalStatus          : out std_logic_vector(7 downto 0);
      internalStatus_1        : out std_logic_vector(7 downto 0);

      -- to the sector datapath, audio and the rest
      cmdResetXa              : out std_logic;
      ClearXACurrentSet       : out std_logic;
      XaFilterFile            : out std_logic_vector(7 downto 0);
      XaFilterChannel         : out std_logic_vector(7 downto 0);
      muted                   : out std_logic;
      cmd_busy                : out std_logic;
      working                 : out std_logic;
      driveBusy               : out std_logic;
      driveDelay              : out integer range 0 to 134217727;
      driveState              : out std_logic_vector(3 downto 0);   -- tdrivestate'pos
      modeReg                 : out std_logic_vector(7 downto 0);
      currentLBA              : out integer range 0 to 524287;
      physicalLBA             : out integer range 0 to 524287;
      readLBA                 : out integer range 0 to 524287;
      writeSectorPointer      : out unsigned(2 downto 0);
      readSectorPointer       : out unsigned(2 downto 0);
      readOnDisk              : out std_logic;
      seekOnDiskDrive         : out std_logic;
      startReading            : out std_logic;
      startPlaying            : out std_logic;
      processDataSector       : out std_logic;
      processCDDASector       : out std_logic;
      processSeekHeader       : out std_logic;
      clearSectorBuffers      : out std_logic;
      triggerUpdateSubchannel : out std_logic;
      handleDrive             : out std_logic;
      newCD_1                 : out std_logic;

      eng_busy                : out std_logic;  -- simulation: the testbench holds the world while '1'
      quiet                   : out std_logic   -- hardware: '0' while the engine has results of an edge to come
   );
end entity;

architecture arch of cd_ctrl is

   signal eng_start  : std_logic;
   signal busy       : std_logic;
   signal world      : std_logic;   -- this clock edge is also an edge of the rest of the core
   signal show       : std_logic;   -- pulses visible to the rest of the core
   signal io_raddr   : unsigned(4 downto 0);
   signal io_rdata   : std_logic_vector(31 downto 0);
   signal io_waddr   : unsigned(4 downto 0);
   signal io_wdata   : std_logic_vector(31 downto 0);
   signal io_we      : std_logic;

   -- level registers
   signal r_cmdPending   : std_logic := '0';
   signal r_cmd_busy     : std_logic := '0';
   signal r_working      : std_logic := '0';
   signal r_driveBusy    : std_logic := '0';
   signal r_muted        : std_logic := '0';
   signal r_driveDelay   : integer range 0 to 134217727 := 0;
   signal r_driveState   : std_logic_vector(3 downto 0) := (others => '0');
   signal r_internal     : std_logic_vector(7 downto 0) := x"10";
   signal r_internal_1   : std_logic_vector(7 downto 0) := x"00";
   signal r_modeReg      : std_logic_vector(7 downto 0) := x"20";
   signal r_currentLBA   : integer range 0 to 524287 := 0;
   signal r_physicalLBA  : integer range 0 to 524287 := 0;
   signal r_readLBA      : integer range 0 to 524287 := 0;
   signal r_wsp          : unsigned(2 downto 0) := (others => '0');
   signal r_rsp          : unsigned(2 downto 0) := (others => '0');
   signal r_ackPendNext  : std_logic := '0';
   signal k_ackPendNext  : std_logic := '0';   -- hardware: r_ackPendNext came while the engine ran
   signal r_newCD_1      : std_logic := '0';
   signal r_XaFile       : std_logic_vector(7 downto 0) := (others => '0');
   signal r_XaChannel    : std_logic_vector(7 downto 0) := (others => '0');
   signal r_RespDin      : std_logic_vector(7 downto 0) := (others => '0');

   -- pulse / strobe registers (p_x: set by the engine, cleared by the fast path)
   type tpulse is array (0 to 31) of std_logic;
   signal pA : std_logic_vector(4 downto 0)  := (others => '0');   -- strobes, cleared at every edge
   signal pB : std_logic_vector(7 downto 0)  := (others => '0');   -- command pulses, cleared at ce edges
   signal pC : std_logic_vector(12 downto 0) := (others => '0');   -- drive pulses, cleared at ce edges without soft reset

   -- fast path state
   signal cmd_delay       : integer range 0 to 120000 := 0;
   signal workDelay       : integer range 0 to 33554431 := 0;
   signal handleCommand   : std_logic := '0';
   signal fifoParamCount  : unsigned(4 downto 0) := (others => '0');   -- 5 bit, wraps (as the original)
   signal Xa_playing      : std_logic := '0';
   signal sectorBufferFilled : std_logic := '0';
   signal startMotorReset : std_logic := '0';
   signal workIsGetID     : std_logic := '0';   -- engine: the work command is GetID
   signal longcmd         : std_logic := '0';   -- engine: nextCmd sends a long response at cmd_delay 11 .. 1
   signal eng_cont        : std_logic := '0';   -- engine: the next ce edge needs it
   signal physWaitSub     : std_logic := '0';   -- engine: physical position update waits for the subchannel
   signal inited          : std_logic := '0';   -- engine: power-on values of its variables set
   signal cdda_rep        : std_logic := '0';   -- engine: CD-DA report bytes to push after their interrupt, or
                                                 -- Setloc / Setfilter parameter steps left (hold, see quiet)
   signal o_flags         : std_logic_vector(4 downto 0) := (others => '0');   -- hardware: flags shown (see cmdPending)
   signal run_ce          : integer range 0 to 4095 := 0;   -- hardware: ce clocks the engine ran since a chain began
   signal drive_dec       : std_logic;
   signal drive_wr        : unsigned(27 downto 0);   -- engine write to driveDelay less the ce clocks since its edge
   signal irq_evt         : std_logic;               -- hardware: an interrupt is raised (pulse to cd_top) this clock
   signal irq_wait        : std_logic := '0';        --   ... raised, the run with its response bytes not started yet
   signal irq_run         : std_logic := '0';        --   ... that run started, not finished
   signal ce_u            : unsigned(0 downto 0);

   -- pulses from the rest of the core that come while the engine runs (hardware only)
   signal k_begin, k_ackRead, k_ackPend, k_ackSector, k_subDone, k_xaReset, k_xaStart : std_logic := '0';
   signal k_newCmd   : std_logic_vector(7 downto 0) := (others => '0');
   signal e_begin, e_ackRead, e_ackPend, e_ackSector, e_subDone, e_xaReset, e_xaStart : std_logic;
   signal e_newCmd   : std_logic_vector(7 downto 0);

   -- event vector and snapshots (state before the last edge this block handled)
   signal ev_now      : std_logic_vector(31 downto 0);
   signal s_ev        : std_logic_vector(31 downto 0) := (others => '0');
   signal s_flags     : std_logic_vector(31 downto 0) := (others => '0');
   signal s_cmdin     : std_logic_vector(31 downto 0) := (others => '0');
   signal s_cmd_delay : unsigned(3 downto 0) := (others => '0');   -- low bits: read in the long response window 11 .. 1
   signal s_workDelay : integer range 0 to 33554431 := 0;
   signal s_driveDelay: integer range 0 to 134217727 := 0;
   signal s_ptr       : std_logic_vector(31 downto 0) := (others => '0');

begin

   eng_busy <= busy;
   -- hardware: the original pushes an interrupt's response byte one clock after the pulse that raises it (or in the
   -- same clock, ackPendingIRQNext); here that is the next engine run. From such a pulse to the end of the run after
   -- it, quiet is low and cd_top shows the CPU no new interrupt (see irqOut there), so no interrupt comes before its
   -- bytes. Only that one run (the engine may run every clock for a while, e.g. the drive waiting for an ack),
   -- except a CD-DA report: its 8 bytes follow its interrupt on 8 edges, and Setloc / Setfilter: they read their
   -- parameters on the edges after their interrupt and reset the parameter fifo at the last one, which must not
   -- wipe the next command's parameters (cdda_rep, set by the engine for both).
   irq_evt  <= '1' when (show = '1' and (pB(0) = '1' or pB(1) = '1' or pB(2) = '1' or pB(3) = '1' or pB(5) = '1' or
                                         pB(7) = '1' or pC(1) = '1' or pC(2) = '1')) or r_ackPendNext = '1' else '0';
   quiet    <= '1' when SIM_FREEZE else not (irq_wait or irq_run or cdda_rep);
   world    <= (not busy) when SIM_FREEZE else '1';
   show     <= not busy;

   -- hardware: the engine may clear and set a flag again within one run (the original's last assignment wins);
   -- the rest of the core sees the flags of finished runs only
   cmdPending         <= r_cmdPending when SIM_FREEZE else o_flags(0);
   cmd_busy           <= r_cmd_busy   when SIM_FREEZE else o_flags(1);
   working            <= r_working    when SIM_FREEZE else o_flags(2);
   driveBusy          <= r_driveBusy  when SIM_FREEZE else o_flags(3);
   muted              <= r_muted      when SIM_FREEZE else o_flags(4);
   driveDelay         <= r_driveDelay;
   driveState         <= r_driveState;
   internalStatus     <= r_internal;
   internalStatus_1   <= r_internal_1;
   modeReg            <= r_modeReg;
   currentLBA         <= r_currentLBA;
   physicalLBA        <= r_physicalLBA;
   readLBA            <= r_readLBA;
   writeSectorPointer <= r_wsp;
   readSectorPointer  <= r_rsp;
   ackPendingIRQNext  <= r_ackPendNext;
   newCD_1            <= r_newCD_1;
   XaFilterFile       <= r_XaFile;
   XaFilterChannel    <= r_XaChannel;
   FifoResponse_Din   <= r_RespDin;

   FifoResponse_reset      <= pA(0) and show;
   FifoResponse_Wr         <= pA(1) and show;
   FifoParam_Rd            <= pA(2) and show;
   FifoParam_reset         <= pA(3) and show;
   cmdResetXa              <= pA(4) and show;
   cmdAck                  <= pB(0) and show;
   cmdIRQ                  <= pB(1) and show;
   driveAck                <= pB(2) and show;
   getIDAck                <= pB(3) and show;
   softReset               <= pB(4) and show;
   errorResponseNext_new   <= pB(5) and show;
   ClearXACurrentSet       <= pB(6) and show;
   cdda_out8               <= pB(7) and show;
   readOnDisk              <= pC(0) and show;
   ackDrive                <= pC(1) and show;
   ackDriveEnd             <= pC(2) and show;
   seekOnDiskDrive         <= pC(3) and show;
   startReading            <= pC(4) and show;
   startPlaying            <= pC(5) and show;
   ackRead                 <= pC(6) and show;
   processDataSector       <= pC(7) and show;
   processCDDASector       <= pC(8) and show;
   processSeekHeader       <= pC(9) and show;
   clearSectorBuffers      <= pC(10) and show;
   triggerUpdateSubchannel <= pC(11) and show;
   handleDrive             <= pC(12) and show;

   -- pulses of the rest of the core, including those kept while the engine ran
   e_begin     <= beginCommand or k_begin;
   e_newCmd    <= k_newCmd when (k_begin = '1' and beginCommand = '0') else newCmd;
   e_ackRead   <= ackRead_valid or k_ackRead;
   e_ackPend   <= ackPendingIRQ or k_ackPend;
   e_ackSector <= ackPendingSector or k_ackSector;
   e_subDone   <= UpdateSubchannelDone or k_subDone;
   e_xaReset   <= XA_reset or k_xaReset;
   e_xaStart   <= XA_start or k_xaStart;

   -- the drive's count down (fast path; in hardware also while the engine runs)
   drive_dec <= '1' when (r_driveBusy = '1' and r_driveDelay > 0 and
                          (sectorBufferFilled = '0' or CDROM_IRQFLAG = "00000" or r_driveDelay > 8191) and
                          (sectorFetchIdle = '1' or r_driveDelay > 127)) else '0';

   ce_u(0)  <= ce;
   drive_wr <= ('0' & unsigned(io_wdata(26 downto 0))) - to_unsigned(run_ce, 28) - ce_u;   -- < 0: bit 27

   -- event vector (from the state before the edge)
   ev_now(0)  <= reset;
   ev_now(1)  <= pB(4);   -- softReset
   ev_now(2)  <= ce;
   ev_now(3)  <= e_begin;
   ev_now(4)  <= handleCommand;
   ev_now(5)  <= '1' when (r_cmd_busy = '1' and longcmd = '1' and cmd_delay >= 1 and cmd_delay <= 11) else '0';
   ev_now(6)  <= '1' when (r_working = '1' and workDelay = 0) else '0';
   ev_now(7)  <= '1' when (r_working = '1' and workIsGetID = '1' and workDelay >= 2 and workDelay <= 10) else '0';
   ev_now(8)  <= e_ackRead;
   ev_now(9)  <= e_ackPend;
   ev_now(10) <= r_ackPendNext or k_ackPendNext;
   ev_now(11) <= newCD and not r_newCD_1;
   ev_now(12) <= '1' when (r_driveBusy = '1' and r_driveDelay = 0 and fetchIdle = '1' and pC(0) = '0') else '0';
   ev_now(13) <= eng_cont;
   ev_now(14) <= e_subDone and physWaitSub;
   ev_now(15) <= e_ackSector;
   ev_now(16) <= '1' when (workDelay = 11 and CDROM_IRQFLAG /= "00000") else '0';
   ev_now(17) <= startMotorReset;
   ev_now(18) <= hasCD;
   ev_now(19) <= isAudioCD;
   ev_now(20) <= FifoParam_Empty;
   ev_now(21) <= FifoResponse_Empty;
   ev_now(22) <= headerIsData;
   ev_now(23) <= headerDataSector;
   ev_now(24) <= isAudio;
   ev_now(25) <= dma_read;
   ev_now(26) <= SS_wren;
   ev_now(31 downto 27) <= (others => '0');

   eng_start <= '1' when (reset = '1' or pB(4) = '1' or (ce = '1' and (e_begin = '1' or handleCommand = '1' or
                          ev_now(5) = '1' or ev_now(6) = '1' or ev_now(7) = '1' or e_ackRead = '1' or e_ackPend = '1' or
                          ev_now(10) = '1' or ev_now(11) = '1' or ev_now(12) = '1' or eng_cont = '1' or ev_now(14) = '1'))) else '0';

   -- engine I/O reads: snapshots of the state before the edge (this block's registers, the event vector, the
   -- command and parameter bytes); the rest is read live. Those inputs change only when the sector datapath works
   -- (lastReadSector, header, subheader, nextSubdata), after the pulses of a finished run, or with the CPU's own
   -- commands (track info), and the firmware reads each of them once per run where they may be changing: reading
   -- a little later is the same as the edge coming a little later.
   process (all)
   begin
      io_rdata <= (others => '0');
      case (to_integer(io_raddr)) is
         when 0  => io_rdata <= s_ev;
         when 1  => io_rdata <= s_flags;
         when 2  => io_rdata <= s_cmdin;
         when 3  => io_rdata <= x"0000000" & std_logic_vector(s_cmd_delay);
         when 4  => io_rdata <= std_logic_vector(to_unsigned(s_workDelay, 32));
         when 5  => io_rdata <= std_logic_vector(to_unsigned(s_driveDelay, 32));
         when 6  => io_rdata <= std_logic_vector(to_unsigned(lastReadSector, 32));
         when 7  => io_rdata <= "0000000000000" & trackInfoA_lba;
         when 8  => io_rdata <= totalMinutesBCD & totalSecondsBCD & minutesBCD & secondsBCD;
         when 9  => io_rdata <= s_ptr;
         when 10 => io_rdata <= header;
         when 11 => io_rdata <= subheader;
         when 12 => io_rdata <= nextSubdata(31 downto 0);
         when 13 => io_rdata <= nextSubdata(63 downto 32);
         when 14 => io_rdata <= nextSubdata(95 downto 64);
         when 15 => io_rdata <= std_logic_vector(peakvolumeR) & std_logic_vector(peakvolumeL);
         when others => null;
      end case;
   end process;

   process (clk)
   begin
      if rising_edge(clk) then

         -- hardware: the drive countdown goes on while the engine runs (paused, the drive would be slower than the
         -- original: XA and CD-DA sectors come exactly as fast as they play), and the engine's writes to it are
         -- reduced by the ce clocks the engine ran since its runs began back to back: the edge a write belongs to
         -- is that much later than in the original (e.g. a sector's countdown is loaded one edge after the
         -- previous one ended). Only the drive: the other delays have decisions at exact values and pause (a few
         -- us later answers, nothing adds up).
         if (not SIM_FREEZE and busy = '1') then
            if (ce = '1' and run_ce < 4095) then run_ce <= run_ce + 1; end if;
            if (ce = '1' and drive_dec = '1') then r_driveDelay <= r_driveDelay - 1; end if;
         end if;

         -- hardware: ackPendingIRQ -> ackPendingIRQNext is a one clock pipeline with cd_top's interrupt logic (which
         -- would raise the interrupt again and again otherwise): it runs in real time; the engine gets the event
         -- at its next edge (kept while it runs)
         if (not SIM_FREEZE) then
            if (ce = '1') then
               r_ackPendNext <= ackPendingIRQ;
            end if;
            if (busy = '0' and ce = '1') then
               k_ackPendNext <= '0';
            elsif (busy = '1' and r_ackPendNext = '1') then
               k_ackPendNext <= '1';
            end if;
         end if;

         if (busy = '0') then
            o_flags <= r_muted & r_driveBusy & r_working & r_cmd_busy & r_cmdPending;
         end if;

         -- hardware: interrupt hold (see quiet)
         if (busy = '0') then
            irq_run <= '0';                                   -- the run after the interrupt has finished
            if ((irq_evt = '1' or irq_wait = '1') and eng_start = '1') then
               irq_wait <= '0';
               irq_run  <= '1';
            elsif (irq_evt = '1') then
               irq_wait <= '1';
            end if;
         elsif (irq_evt = '1') then
            irq_wait <= '1';                                  -- raised while the engine runs
         end if;

         -- the rest of the core: parameter fifo fill level, pulses kept while the engine runs
         if (world = '1') then
            if (pA(3) = '1' and show = '1') then
               fifoParamCount <= (others => '0');
            elsif (FifoParam_Wr = '1') then
               fifoParamCount <= fifoParamCount + 1;
            elsif (pA(2) = '1' and show = '1') then
               fifoParamCount <= fifoParamCount - 1;
            end if;
            if (busy = '1') then
               if (beginCommand = '1') then k_begin <= '1'; k_newCmd <= newCmd; end if;
               if (ackRead_valid = '1') then k_ackRead <= '1'; end if;
               if (ackPendingIRQ = '1') then k_ackPend <= '1'; end if;
               if (ackPendingSector = '1') then k_ackSector <= '1'; end if;
               if (UpdateSubchannelDone = '1') then k_subDone <= '1'; end if;
               if (XA_reset = '1') then k_xaReset <= '1'; end if;
               if (XA_start = '1') then k_xaStart <= '1'; end if;
            end if;
         end if;

         if (busy = '0') then   -- an edge of this block (virtual time)

            if (ce = '1') then   -- kept pulses are used at this edge (they came at ce clocks)
               k_begin <= '0'; k_ackRead <= '0'; k_ackPend <= '0'; k_ackSector <= '0'; k_subDone <= '0';
               k_xaReset <= '0'; k_xaStart <= '0';
            end if;
            if (eng_start = '0') then
               run_ce <= 0;   -- runs back to back (the next edge needs the engine again) count on
            end if;

            s_ev        <= ev_now;
            s_flags     <= x"0" & r_internal_1 & '0' & inited & handleCommand & physWaitSub & eng_cont & longcmd & workIsGetID &
                           r_muted & r_driveBusy & r_working & r_cmd_busy & r_cmdPending & "000" & CDROM_IRQFLAG;
            s_cmdin     <= region & "000000" & pendingDriveResponse & FifoParam_Dout & e_newCmd;
            s_cmd_delay <= to_unsigned(cmd_delay mod 16, 4);
            s_workDelay <= workDelay;
            s_driveDelay<= r_driveDelay;
            s_ptr       <= x"00" & "000" & std_logic_vector(fifoParamCount) & trackcountBCD & '0' &
                           std_logic_vector(r_rsp) & '0' & std_logic_vector(r_wsp);

            -- command processing: strobes cleared at every edge
            pA <= (others => '0');

            if (SS_reset = '1') then
               startMotorReset <= '1';
            elsif (SS_wren = '1') then
               startMotorReset <= '0';
            end if;

            if (reset = '1') then

               pA(3)           <= '1';   -- FifoParam_reset
               pA(0)           <= '1';   -- FifoResponse_reset
               r_cmdPending    <= '0';
               r_cmd_busy      <= '0';
               cmd_delay       <= 0;
               r_working       <= '0';
               workDelay       <= 0;
               r_muted         <= '0';
               r_XaFile        <= x"00";
               r_XaChannel     <= x"00";

               r_driveBusy     <= '0';
               r_driveState    <= x"0";
               r_driveDelay    <= 0;
               r_internal      <= x"10";
               r_modeReg       <= x"20";
               r_currentLBA    <= 0;
               r_physicalLBA   <= 0;
               r_wsp           <= (others => '0');
               r_rsp           <= (others => '0');
               Xa_playing      <= '0';
               startMotorReset <= '0';

            else

               -- command processing (ce)
               if (ce = '1') then
                  handleCommand <= '0';
                  pB            <= (others => '0');

                  if (e_begin = '0' and r_cmd_busy = '1') then
                     if (CDROM_IRQFLAG /= "00000" or dma_read = '1') then
                        if (cmd_delay < 8192) then
                           cmd_delay <= 8192;
                        end if;
                     else
                        if (cmd_delay > 0) then
                           if (cmd_delay < 16 or ((r_driveBusy = '0' or r_driveDelay > 100) and (r_working = '0' or workDelay > 100))) then
                              cmd_delay <= cmd_delay - 1;
                           end if;
                        else
                           handleCommand <= '1';
                           r_cmd_busy    <= '0';
                        end if;
                     end if;
                  end if;

                  if (handleCommand = '0' and workDelay > 0) then
                     workDelay <= workDelay - 1;
                  end if;
                  if (workDelay = 11 and CDROM_IRQFLAG /= "00000") then
                     workDelay <= 11;
                  end if;

                  if (SIM_FREEZE) then
                     r_ackPendNext <= e_ackPend;
                  end if;
               end if;

               -- drive (ce, not during a soft reset)
               if (pB(4) = '0' and ce = '1') then
                  pC <= (others => '0');

                  r_internal_1 <= r_internal;

                  if (e_xaReset = '1') then
                     Xa_playing <= '0';
                  elsif (e_xaStart = '1') then
                     Xa_playing <= '1';
                  end if;

                  sectorBufferFilled <= '0';
                  if (r_driveState = x"4" and bufferAhead = '1') then   -- DRIVE_READING
                     sectorBufferFilled <= not Xa_playing;
                  end if;

                  if (drive_dec = '1') then
                     r_driveDelay <= r_driveDelay - 1;
                  end if;

                  if (e_ackRead = '1' or e_ackSector = '1') then
                     r_rsp <= r_rsp + 1;
                  end if;

                  r_newCD_1 <= newCD;
               end if;

            end if;

         end if;

         -- engine writes (while it runs, after the edge it handles)
         if (io_we = '1') then
            case (to_integer(io_waddr)) is
               when 0 =>   -- strobes: response fifo reset, write (+ data), param fifo read, reset, XA reset
                  pA <= pA or io_wdata(4 downto 0);
                  if (io_wdata(1) = '1') then r_RespDin <= io_wdata(15 downto 8); end if;
               when 1 =>   -- command pulses
                  pB <= pB or io_wdata(7 downto 0);
               when 2 =>   -- drive pulses
                  pC <= pC or io_wdata(12 downto 0);
               when 3 | 4 =>   -- flags: 3 sets, 4 clears the bits given
                  for i in 0 to 11 loop
                     if (io_wdata(i) = '1') then
                        case (i) is
                           when 0 => if (io_waddr = 3) then r_cmdPending  <= '1'; else r_cmdPending  <= '0'; end if;
                           when 1 => if (io_waddr = 3) then r_cmd_busy    <= '1'; else r_cmd_busy    <= '0'; end if;
                           when 2 => if (io_waddr = 3) then r_working     <= '1'; else r_working     <= '0'; end if;
                           when 3 => if (io_waddr = 3) then r_driveBusy   <= '1'; else r_driveBusy   <= '0'; end if;
                           when 4 => if (io_waddr = 3) then r_muted       <= '1'; else r_muted       <= '0'; end if;
                           when 5 => if (io_waddr = 3) then workIsGetID   <= '1'; else workIsGetID   <= '0'; end if;
                           when 6 => if (io_waddr = 3) then longcmd       <= '1'; else longcmd       <= '0'; end if;
                           when 7 => if (io_waddr = 3) then eng_cont      <= '1'; else eng_cont      <= '0'; end if;
                           when 8 => if (io_waddr = 3) then physWaitSub   <= '1'; else physWaitSub   <= '0'; end if;
                           when 9 => if (io_waddr = 3) then handleCommand <= '1'; else handleCommand <= '0'; end if;
                           when 10 => if (io_waddr = 3) then inited      <= '1'; else inited        <= '0'; end if;
                           when 11 => if (io_waddr = 3) then cdda_rep    <= '1'; else cdda_rep      <= '0'; end if;
                           when others => null;
                        end case;
                     end if;
                  end loop;
               when 5  => cmd_delay     <= to_integer(unsigned(io_wdata(16 downto 0)));
               when 6  => workDelay     <= to_integer(unsigned(io_wdata(24 downto 0)));
               when 7  =>   -- the value after the edge; in hardware less the ce clocks since (counted down meanwhile)
                  if (SIM_FREEZE) then
                     r_driveDelay <= to_integer(unsigned(io_wdata(26 downto 0)));
                  elsif (drive_wr(27) = '1') then   -- less than the clocks since: done
                     r_driveDelay <= 0;
                  else
                     r_driveDelay <= to_integer(drive_wr(26 downto 0));
                  end if;
               when 8  => r_driveState  <= io_wdata(3 downto 0);
               when 9  => r_internal    <= io_wdata(7 downto 0);
               when 10 => r_modeReg     <= io_wdata(7 downto 0);
               when 11 => r_XaFile      <= io_wdata(7 downto 0); r_XaChannel <= io_wdata(15 downto 8);
               when 12 => r_currentLBA  <= to_integer(unsigned(io_wdata(18 downto 0)));
               when 13 => r_physicalLBA <= to_integer(unsigned(io_wdata(18 downto 0)));
               when 14 => r_readLBA     <= to_integer(unsigned(io_wdata(18 downto 0)));
               when 15 => r_wsp         <= unsigned(io_wdata(2 downto 0));
               when 16 => r_rsp         <= unsigned(io_wdata(2 downto 0));
               when 17 => pA(1) <= '1'; r_RespDin <= io_wdata(7 downto 0);   -- response fifo write
               when others => null;
            end case;
         end if;

      end if;
   end process;

   ieng : entity work.cd_engine
   port map
   (
      clk      => clk,
      reset    => '0',
      start    => eng_start,
      busy     => busy,
      io_raddr => io_raddr,
      io_rdata => io_rdata,
      io_waddr => io_waddr,
      io_wdata => io_wdata,
      io_we    => io_we
   );

end architecture;
