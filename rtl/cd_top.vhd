library IEEE;
use IEEE.std_logic_1164.all;  
use IEEE.numeric_std.all; 
use STD.textio.all;

library mem;

entity cd_top is
   generic
   (
      SIM_FREEZE           : boolean := false   -- sim/cd_equiv: engine on clk_engine, eng_busy holds the testbench
   );
   port 
   (
      clk1x                : in  std_logic;
      clk_engine           : in  std_logic := '0';
      eng_busy             : out std_logic;
      ce                   : in  std_logic;
      reset                : in  std_logic;
      
      INSTANTSEEK          : in  std_logic;
      FORCECDSPEED         : in  std_logic_vector(2 downto 0);
      LIMITREADSPEED       : in  std_logic;
      hasCD                : in  std_logic;
      LIDopen              : in  std_logic;
      fastCD               : in  std_logic;
      testSeek             : in  std_logic;
      pauseOnCDSlow        : in  std_logic;
      region               : in  std_logic_vector(1 downto 0);
      region_out           : out std_logic_vector(1 downto 0);	  
      
      pauseCD              : out std_logic := '0';
      Pause_idle_cd        : out std_logic := '0';
      fullyIdle            : out std_logic;
      cdSlow               : out std_logic := '0';
      error                : out std_logic := '0';
      LBAdisplay           : out unsigned(19 downto 0);
      
      irqOut               : out std_logic := '0';
      
      spu_tick             : in  std_logic;
      cd_left              : out signed(15 downto 0) := (others => '0');
      cd_right             : out signed(15 downto 0) := (others => '0');
      
      mdec_idle            : in  std_logic;
      
      bus_addr             : in  unsigned(3 downto 0); 
      bus_dataWrite        : in  std_logic_vector(7 downto 0);
      bus_read             : in  std_logic;
      bus_write            : in  std_logic;
      bus_dataRead         : out std_logic_vector(7 downto 0) := (others => '0');
      
      dma_read             : in  std_logic;
      dma_readdata         : out std_logic_vector(7 downto 0);
      
      cd_hps_req           : out std_logic := '0';
      cd_hps_lba           : out std_logic_vector(31 downto 0);
      cd_hps_lba_sim       : out std_logic_vector(31 downto 0) := (others => '0');
      cd_hps_ack           : in  std_logic;
      cd_hps_write         : in  std_logic;
      cd_hps_data          : in  std_logic_vector(15 downto 0);
         
      trackinfo_data       : in  std_logic_vector(31 downto 0);
      trackinfo_addr       : in  std_logic_vector(8 downto 0);
      trackinfo_write      : in  std_logic;
      resetFromCD          : out std_logic := '0';
         
      SS_reset             : in  std_logic;
      SS_DataWrite         : in  std_logic_vector(31 downto 0);
      SS_Adr               : in  unsigned(13 downto 0);
      SS_wren              : in  std_logic;
      SS_rden              : in  std_logic;
      SS_DataRead          : out std_logic_vector(31 downto 0);
      SS_idle              : out std_logic
   );
end entity;

architecture arch of cd_top is
  
   constant RAW_SECTOR_SIZE         : integer := 2352;
   constant SECTOR_SYNC_SIZE        : integer := 12;
   constant RAW_SECTOR_OUTPUT_SIZE  : integer := RAW_SECTOR_SIZE - SECTOR_SYNC_SIZE;
   constant DATA_SECTOR_SIZE        : integer := 2048;
   constant PREGAPSIZE              : integer := 150;
   
   constant FRAMES_PER_SECOND       : integer := 75;
   constant FRAMES_PER_MINUTE       : integer := 75 * 60;
   constant LEAD_OUT_TRACK_NUMBER   : std_logic_vector(7 downto 0) := x"AA";
   
   constant READSPEED1X             : integer := 44100 * 16#300# / 75;
   constant READSPEED2X             : integer := 44100 * 16#300# / 150;
   constant READSPEED4X             : integer := 44100 * 16#300# / 300;
   constant READSPEED6X             : integer := 44100 * 16#300# / 450;
   constant READSPEED8X             : integer := 44100 * 16#300# / 600;
   
   -- data fifo
   signal FifoData_reset            : std_logic := '0';
   signal FifoData_Din              : std_logic_vector(7 downto 0) := (others => '0');
   signal FifoData_Wr               : std_logic := '0'; 
   signal FifoData_Dout             : std_logic_vector(7 downto 0);
   signal FifoData_Rd               : std_logic := '0';
   signal FifoData_Empty            : std_logic;
   
   -- cpu interface  
   signal CDROM_STATUS              : std_logic_vector(7 downto 0);
   signal CDROM_IRQENA              : std_logic_vector(4 downto 0);
   signal CDROM_IRQFLAG             : std_logic_vector(4 downto 0);
            
   signal beginCommand              : std_logic := '0';
   signal cmd_unpause               : std_logic := '0';
   signal newCmd                    : std_logic_vector(7 downto 0);
   signal nextCmd                   : std_logic_vector(7 downto 0);
   
   signal pendingDriveIRQ           : std_logic_vector(4 downto 0);
   signal pendingDriveResponse      : std_logic_vector(7 downto 0);
   signal ackPendingIRQ             : std_logic := '0';
   signal ackPendingSector          : std_logic := '0';
   signal ackRead_valid             : std_logic := '0';
   signal irqTimeout                : integer range 0 to 10000 := 0;
            
   signal FifoParam_reset           : std_logic := '0';
   signal FifoParam_Din             : std_logic_vector(7 downto 0) := (others => '0');
   signal FifoParam_Wr              : std_logic := '0'; 
   signal FifoParam_NearFull         : std_logic := '0'; 
   signal FifoParam_Dout            : std_logic_vector(7 downto 0);
   signal FifoParam_Rd              : std_logic := '0';
   signal FifoParam_Empty           : std_logic;
            
   -- command processing         
   signal cmd_busy                  : std_logic := '0';
   signal cmd_delay                 : integer range 0 to 120000;
   signal cmdPending                : std_logic := '0';
   signal handleCommand             : std_logic := '0';    
   signal paramCount                : integer range 0 to 6;
   signal fifoParamCount            : integer range 0 to 16;
   signal working                   : std_logic := '0';
   signal workCommand               : std_logic_vector(7 downto 0);
   signal workDelay                 : integer range 0 to 33554431;
   signal cmdAck                    : std_logic := '0';
   signal cmdIRQ                    : std_logic := '0';
   signal driveAck                  : std_logic := '0';
   signal getIDAck                  : std_logic := '0';
   signal startMotorCMD             : std_logic := '0';
   signal softReset                 : std_logic := '0';
   signal ackPendingIRQNext         : std_logic := '0';
   signal cmdResetXa                : std_logic := '0';
   signal updatePhysicalPosition    : std_logic := '0';
   signal muted                     : std_logic;
         
   signal setLocActive              : std_logic := '0';
   signal setLocReadStep            : integer range 0 to 5;
   signal setLocMinute              : unsigned(7 downto 0);
   signal setLocSecond              : unsigned(7 downto 0);
   signal setLocFrame               : unsigned(7 downto 0);         
   
   signal session                   : std_logic_vector(7 downto 0);
   signal fastForwardRate           : signed(7 downto 0);
   
   signal setFilterReadStep         : integer range 0 to 3;
   signal XaFilterFile              : std_logic_vector(7 downto 0);
   signal XaFilterChannel           : std_logic_vector(7 downto 0);
   signal ClearXACurrentSet         : std_logic;
   
   signal XaCurrentFile             : std_logic_vector(7 downto 0);
   signal XaCurrentChannel          : std_logic_vector(7 downto 0);
   signal XaCurrentSet              : std_logic;
   
   signal lastreportCDDA            : std_logic_vector(4 downto 0);
   signal CDDA_outcnt               : integer range 0 to 8;
      
   signal seekOnDiskCmd             : std_logic := '0';
   signal stop_afterseek            : std_logic := '0';
   signal setMode                   : std_logic := '0';
   signal newMode                   : std_logic_vector(7 downto 0);
   signal readSN                    : std_logic := '0';
   signal play                      : std_logic := '0';
   signal playTrack                 : std_logic := '0';
   signal setSession                : std_logic := '0';
   signal cmdStartMotor             : std_logic := '0';
   signal cmdStop                   : std_logic := '0';
   signal drive_stop                : std_logic := '0';
   signal shell_close               : std_logic := '0';
   
   signal errorResponseCmd_new      : std_logic := '0'; 
   signal errorResponseCmd_error    : std_logic_vector(7 downto 0);
   signal errorResponseCmd_reason   : std_logic_vector(7 downto 0);
   
   signal errorResponseNext_new     : std_logic := '0'; 
   signal errorResponseNext_reason  : std_logic_vector(7 downto 0);
    
   signal FifoResponse_reset        : std_logic := '0';
   signal FifoResponse_Din          : std_logic_vector(7 downto 0) := (others => '0');
   signal FifoResponse_Wr           : std_logic := '0'; 
   signal FifoResponse_Dout         : std_logic_vector(7 downto 0);
   signal FifoResponse_Rd           : std_logic := '0';
   signal FifoResponse_Empty        : std_logic;
    
   -- drive
   type tdrivestate is
   (
      DRIVE_IDLE,
      DRIVE_SEEKPHYSICAL,
      DRIVE_SEEKLOGICAL,
      DRIVE_SEEKIMPLICIT,
      DRIVE_READING,
      DRIVE_PLAYING,
      DRIVE_SPEEDCHANGEORTOCREAD,
      DRIVE_SPINNINGUP,
      DRIVE_CHANGESESSION,
      DRIVE_OPENING
	);
   signal driveState                : tdrivestate := DRIVE_IDLE;
         
   signal internalStatus            : std_logic_vector(7 downto 0);
   signal internalStatus_1          : std_logic_vector(7 downto 0);
   signal modeReg                   : std_logic_vector(7 downto 0);
         
   signal driveBusy                 : std_logic;
   signal driveDelay                : integer range 0 to 134217727;
   signal driveDelayNext            : integer range 0 to 134217727;
   signal driveREADSPEED            : integer range 0 to 134217727;
   
   signal allow_speedhack           : std_logic := '0';
   
   signal handleDrive               : std_logic := '0';
   signal startMotor                : std_logic := '0';
   signal startMotorReset           : std_logic := '0';
   signal ackDrive                  : std_logic := '0';
   signal ackDriveEnd               : std_logic := '0';
   signal seekOnDiskDrive           : std_logic := '0';
   signal seekOnDiskPlay            : std_logic := '0';
   signal ackRead                   : std_logic := '0';
   signal calcSeekTime              : std_logic := '0';
   signal addSeekTime               : std_logic := '0';
   signal limitDelayTime            : std_logic := '0';
         
   signal currentLBA                : integer range 0 to 524287;        
   signal physicalLBA               : integer range 0 to 524287;        
   signal seekLBA                   : integer range 0 to 524287; 
   signal playLBA                   : integer range 0 to 524287; 
   signal diffLBA                   : integer range 0 to 524287; 
   signal seekTimeMul               : integer range 0 to 137; 
   signal currentTrackBCD           : std_logic_vector(7 downto 0);
   signal nextTrack                 : std_logic_vector(7 downto 0);
   
   signal readAfterSeek             : std_logic := '0';
   signal playAfterSeek             : std_logic := '0';
   signal lastSectorHeaderValid     : std_logic := '0'; 
   
   signal errorResponseDrive_new    : std_logic := '0'; 
   signal errorResponseDrive_error  : std_logic_vector(7 downto 0);
   signal errorResponseDrive_reason : std_logic_vector(7 downto 0);

   -- exchange with data part
   signal readOnDisk                : std_logic := '0';   
   signal readOnDisk_buffer         : std_logic := '0';   
   signal readLBA                   : integer range 0 to 524287; 
   signal readLBA_buffer            : integer range 0 to 524287; 
   signal trackNumberBCD            : unsigned(7 downto 0) := x"00";
   signal trackNumber               : std_logic_vector(6 downto 0);
   
   signal copyData                  : std_logic := '0';  
   signal seekOK                    : std_logic := '1';
   signal afterSeek                 : std_logic := '0'; 
   signal startReading              : std_logic := '0';  
   signal startPlaying              : std_logic := '0';  
   signal processDataSector         : std_logic := '0';  
   signal processCDDASector         : std_logic := '0';  
   -- CD audio (CD-DA): playback, peak-volume tracking and CD-DA report packets. false removes them (XA-ADPCM is
   -- unaffected): SotN plays no audio track, only the BIOS CD player needs them (SotN's track 2 is the disc's
   -- CD-player warning message and music).
   constant CDDA_ON : boolean := true;
   -- Pocket: no LibCrypt copy-protection emulation (only some PAL games use it; the key for SotN USA is 0,
   -- so the subchannel check below can never trigger). Removes 32 sector-number comparators.
   constant LIBCRYPT_ON : boolean := false;
   signal processSeekHeader         : std_logic := '0';  
   signal clearSectorBuffers        : std_logic := '0';  
   signal writeSectorPointer        : unsigned(2 downto 0) := (others => '0');
   signal readSectorPointer         : unsigned(2 downto 0) := (others => '0');
   signal firstSectorPending        : std_logic := '0';
   
   type tphysicalUpdateState is
   (
      PHYSICALUPDATE_IDLE,
      PHYSICALUPDATE_START,
      PHYSICALUPDATE_CHECK,
      PHYSICALUPDATE_CALC1,
      PHYSICALUPDATE_CALC2,
      PHYSICALUPDATE_CALCDONE,
      PHYSICALUPDATE_READSUBCHANNEL
   );
   signal physicalUpdateState : tphysicalUpdateState := PHYSICALUPDATE_IDLE;
   
   signal phy_base                 : integer range 0 to 524287;
   signal phy_oldOffset            : integer range 0 to 31;
   signal phy_newOffset            : integer range 0 to 31;
   signal phy_spt                  : integer range 8 to 22 := 8;
   
   -- sector fetch
   type tsectorFetch is
   (
      SFETCH_IDLE,
      SFETCH_DELAY,
      SFETCH_START,
      SFETCH_HPSACK,
      SFETCH_HPSWORD,
      SFETCH_HPSDATA
   );
   signal sectorFetchState          : tsectorFetch := SFETCH_IDLE;
   
   signal sectorBuffer_addrA        : std_logic_vector(9 downto 0) := (others => '0');
   signal sectorBuffer_DataA        : std_logic_vector(31 downto 0) := (others => '0');
   signal sectorBuffer_wrenA        : std_logic;
   signal sectorBuffer_addrB        : std_logic_vector(9 downto 0);
   signal sectorBuffer_DataB        : std_logic_vector(31 downto 0);
      
   signal positionInIndex           : integer range -524288 to 524287; 
   signal lastReadSector            : integer range 0 to 524287; 
   signal subchannelSector          : integer range 0 to 524287; 
   signal fetchCount                : integer range 0 to 588;
   signal fetchDelay                : integer range 0 to 15;
      
   signal slow_timeout              : integer range 0 to 16777215 := 0;
   signal slow_block                : std_logic := '0';
   
   signal peakvolumeL               : signed(15 downto 0);
   signal peakvolumeR               : signed(15 downto 0);
      
   -- read subchannel
   type treadSubchannelState is
   (
      SSUB_IDLE,
      SSUB_START,
      SSUB_CALCPOS,
      SSUB_CALCSECTOR
   );
   signal readSubchannelState       : treadSubchannelState := SSUB_IDLE;
   
   signal readSubchannel            : std_logic := '0';
   signal triggerUpdateSubchannel   : std_logic := '0';
   signal UpdateSubchannel          : std_logic := '0';
   signal UpdateSubchannelDone      : std_logic := '0';
   signal subchannelLBAwork         : integer range 0 to 524287;  
   signal sub_SecondsHigh           : unsigned(3 downto 0); 
   signal sub_SecondsLow            : unsigned(3 downto 0); 
   signal sub_MinutesHigh           : unsigned(3 downto 0); 
   signal sub_MinutesLow            : unsigned(3 downto 0);
      
   -- sector process
   type tsectorProcess is
   (
      SPROC_IDLE,
      SPROC_SEEKREADHEADER,
      SPROC_SEEKREADSUBHEADER,
      SPROC_READHEADER,
      SPROC_READSUBHEADER,
      SPROC_START,
      SPROC_FIRST,
      SPROC_DATA,
      SPROC_XA_FIRST,
      SPROC_XA,
      SPROC_CDDAFIRST,
      SPROC_CDDA
   );
   signal sectorProcessState        : tsectorProcess := SPROC_IDLE;
   
   type tsectorBufferSizes is array(0 to 7) of integer range 0 to 588;
   signal sectorBufferSizes         : tsectorBufferSizes;
   
   signal sectorBufferFilled        : std_logic := '0';
   
   signal sectorBuffers_addrA       : std_logic_vector(12 downto 0) := (others => '0');
   signal sectorBuffers_DataA       : std_logic_vector(31 downto 0) := (others => '0');
   signal sectorBuffers_wrenA       : std_logic;
   signal sectorBuffers_addrB       : std_logic_vector(12 downto 0);
   signal sectorBuffers_DataB       : std_logic_vector(31 downto 0);
   
   signal procCount                 : integer range 0 to 588;
   signal procSize                  : integer range 0 to 588;
   signal procReadAddr              : integer range 0 to 588;
   signal header                    : std_logic_vector(31 downto 0);
   signal subheader                 : std_logic_vector(31 downto 0);
   signal headerIsData              : std_logic;
   signal headerDataCheck           : std_logic;
   signal headerDataSector          : std_logic;
   
   type tsubdata is array(0 to 11) of std_logic_vector(7 downto 0);
   signal subdata                   : tsubdata;
   signal nextSubdata               : tsubdata;
   
   signal XA_addr                   : integer range 0 to 587;
   signal XA_data                   : std_logic_vector(31 downto 0);
   signal XA_write                  : std_logic := '0';
   signal XA_start                  : std_logic := '0';
   signal XA_reset                  : std_logic := '0';
   signal XA_EOF                    : std_logic := '0';
   signal xa_muted                  : std_logic;
   signal Xa_playing                : std_logic;
   signal cdaudio_left              : signed(15 downto 0);
   signal cdaudio_right             : signed(15 downto 0);
   
   signal CDDA_write                : std_logic;
   signal CDDA_data                 : std_logic_vector(31 downto 0);
   
   -- copy data
   type tCopyState is
	(
		COPY_IDLE,
		COPY_FIRST,
		COPY_DATA
	);
   signal copyState                 : tCopyState := COPY_IDLE;

   signal copyCount                 : integer range 0 to 588;
   signal copyByteCnt               : integer range 0 to 3;
   signal copySize                  : integer range 0 to 588;    
   signal copyReadAddr              : integer range 0 to 588;
   
   signal copySectorPointer         : unsigned(2 downto 0) := (others => '0');
   signal ackRead_data              : std_logic := '0';
      
   -- cd audio volume
   signal cdvol_next00              : std_logic_vector(7 downto 0) := (others => '0');
   signal cdvol_next01              : std_logic_vector(7 downto 0) := (others => '0');
   signal cdvol_next10              : std_logic_vector(7 downto 0) := (others => '0');
   signal cdvol_next11              : std_logic_vector(7 downto 0) := (others => '0');
   signal cdvol_00                  : std_logic_vector(7 downto 0) := (others => '0');
   signal cdvol_01                  : std_logic_vector(7 downto 0) := (others => '0');
   signal cdvol_10                  : std_logic_vector(7 downto 0) := (others => '0');
   signal cdvol_11                  : std_logic_vector(7 downto 0) := (others => '0');
   
   signal cd_volume_step            : integer range 0 to 7;
   signal soundmulresult            : signed(16 downto 0) := (others => '0');
   signal soundmul1                 : signed(15 downto 0) := (others => '0');
   signal soundmul2                 : unsigned(7 downto 0) := (others => '0');
   signal soundsum                  : signed(17 downto 0) := (others => '0');
      
   -- track infos
   --signal totalLBAs                 : integer range 0 to 524287; 
   --signal trackcount                : std_logic_vector(7 downto 0);
   signal trackcountBCD             : std_logic_vector(7 downto 0);
   signal totalSecondsBCD           : std_logic_vector(7 downto 0);
   signal totalMinutesBCD           : std_logic_vector(7 downto 0);
   
   signal trackInfo_addrA           : std_logic_vector(6 downto 0) := (others => '0');
   signal trackInfo_DataA           : std_logic_vector(54 downto 0) := (others => '0');
   signal trackInfo_DataOutA        : std_logic_vector(54 downto 0) := (others => '0');
   signal trackInfo_wrenA           : std_logic;
   signal trackInfo_addrB           : std_logic_vector(6 downto 0) := (others => '0');
   signal trackInfo_DataOutB        : std_logic_vector(54 downto 0);
      
   signal startLBA                  : integer range 0 to 524287;
   signal endLBA                    : integer range 0 to 524287;
   signal minutesBCD                : std_logic_vector(7 downto 0);
   signal secondsBCD                : std_logic_vector(7 downto 0);
   signal isAudio                   : std_logic;
   
   signal isAudioCD                 : std_logic := '0';
   signal libcryptKey               : std_logic_vector(15 downto 0);
      
   type ttrackSearchState is
	(
		TRACKSEARCH_IDLE,
		TRACKSEARCH_READ,
      TRACKSEARCH_CHECK
	);
   signal trackSearchState          : ttrackSearchState := TRACKSEARCH_IDLE;

   signal newCDCounter              : unsigned(25 downto 0); -- ~2seconds
   signal newCD                     : std_logic := '0';
   signal newCD_1                   : std_logic := '0';

   -- savestates
   type t_ssarray is array(0 to 127) of std_logic_vector(31 downto 0);
   -- reset defaults that SS_reset used to load into ss_in (savestates removed)
   function ss_defaults return t_ssarray is
      variable v : t_ssarray := (others => (others => '0'));
   begin
      v(13) := x"00002010"; -- pendingDriveIRQ & nextCmd & modeReg & internalStatus
      v(21) := x"00000018"; -- CDROM_IRQFLAG & CDROM_IRQENA & CDROM_STATUS;
      v(23) := x"80000080"; --  CD Volume;
      v(24) := x"80000080"; --  CD Volume;
      v(25) := x"001F0000"; --  lastreportCDDA & trackNumber & currentTrackBCD;
      return v;
   end function;
   constant ss_in : t_ssarray := ss_defaults;
   signal ss_out  : t_ssarray := (others => (others => '0'));

   signal SS_rden_sectorbuffer   : std_logic;
   signal SS_rden_sectorbuffers  : std_logic;

   signal ss_idle_timeout        : integer range 0 to 7;
   
   signal ss_timeout             : unsigned(23 downto 0);

   -- PSX mech sectors-per-track table (based on DuckStation / rama measurements)
   function getSectorsPerTrack(lba : integer) return integer is
   begin
      -- minute = lba / FRAMES_PER_MINUTE compared against the table, written as
      -- compares on lba so no divider is built (lba is never negative)
      if    (lba <  1 * FRAMES_PER_MINUTE) then return 8;
      elsif (lba <  5 * FRAMES_PER_MINUTE) then return 9;
      elsif (lba <  8 * FRAMES_PER_MINUTE) then return 10;
      elsif (lba < 12 * FRAMES_PER_MINUTE) then return 11;
      elsif (lba < 17 * FRAMES_PER_MINUTE) then return 12;
      elsif (lba < 24 * FRAMES_PER_MINUTE) then return 13;
      elsif (lba < 28 * FRAMES_PER_MINUTE) then return 14;
      elsif (lba < 33 * FRAMES_PER_MINUTE) then return 15;
      elsif (lba < 40 * FRAMES_PER_MINUTE) then return 16;
      elsif (lba < 45 * FRAMES_PER_MINUTE) then return 17;
      elsif (lba < 53 * FRAMES_PER_MINUTE) then return 18;
      elsif (lba < 61 * FRAMES_PER_MINUTE) then return 19;
      elsif (lba < 68 * FRAMES_PER_MINUTE) then return 20;
      elsif (lba < 75 * FRAMES_PER_MINUTE) then return 21;
      else return 22;
      end if;
   end function;
   
   -- x mod m for x in 0..32 and m in 8..22, without a divider
   function modSmall(x : integer; m : integer) return integer is
      variable r : integer range 0 to 63;
   begin
      r := x;
      for i in 1 to 4 loop
         if (r >= m) then
            r := r - m;
         end if;
      end loop;
      return r;
   end function;

   -- frame number 0..127 to BCD (tens in 7..4, ones in 3..0), same as x/10 and x mod 10, without a divider
   function toBCD7(x : unsigned(6 downto 0)) return std_logic_vector is
      variable v : integer range 0 to 127;
      variable t : integer range 0 to 12;
   begin
      v := to_integer(x);
      t := 0;
      for k in 1 to 12 loop
         if (v >= k * 10) then
            t := k;
         end if;
      end loop;
      return std_logic_vector(to_unsigned(t, 4)) & std_logic_vector(to_unsigned(v - t * 10, 4));
   end function;

   -- cd_ctrl connections
   signal nextSubdata_v   : std_logic_vector(95 downto 0);
   signal ctrl_fetchIdle  : std_logic;
   signal ctrl_sfetchIdle : std_logic;
   signal ctrl_bufAhead   : std_logic;
   signal ctrl_driveState : std_logic_vector(3 downto 0);
   signal ctrl_cdda_out8  : std_logic;
   signal ctrl_quiet      : std_logic;
   signal irqPulse        : std_logic := '0';
   signal irqHeld         : std_logic := '0';
   signal irqFlagShown    : std_logic_vector(4 downto 0) := (others => '0');
   signal irqFlagCPU      : std_logic_vector(4 downto 0);

   -- debug
   -- synthesis translate_off
   type tsectorBuffer is array(0 to 587) of std_logic_vector(31 downto 0);
   type tsectorBuffers is array(0 to 7) of tsectorBuffer;
   signal sectorBuffers             : tsectorBuffers;
   -- synthesis translate_on   
      
begin 

   fullyIdle <= '1' when (cmd_busy = '0' and working = '0' and driveBusy = '0' and sectorFetchState = SFETCH_IDLE and sectorProcessState = SPROC_IDLE and copyState = COPY_IDLE) else '0';

   ififoData: entity mem.SyncFifoFallThrough
   generic map
   (
      SIZE             => 4096,
      DATAWIDTH        => 8,
      NEARFULLDISTANCE => 16
   )
   port map
   ( 
      clk      => clk1x,     
      reset    => FifoData_reset,   
                
      Din      => FifoData_Din,     
      Wr       => FifoData_Wr,      
      Full     => open,    
      NearFull => open,

      Dout     => FifoData_Dout,    
      Rd       => FifoData_Rd,      
      Empty    => FifoData_Empty   
   );
   
   FifoData_Rd <= ce when (FifoData_Empty = '0' and bus_read = '1' and bus_addr(3 downto 0) = "0010") else
                  ce when (FifoData_Empty = '0' and dma_read = '1') else 
                  '0';
   
   dma_readdata <= FifoData_Dout when (FifoData_Empty = '0') else (others => '0');
   
   LBAdisplay  <= to_unsigned(currentLBA, 20);
   
   ss_out(21)( 7 downto  0) <= CDROM_STATUS;
   ss_out(21)(12 downto  8) <= CDROM_IRQENA;
   ss_out(21)(20 downto 16) <= CDROM_IRQFLAG;
   ss_out(13)(28 downto 24) <= pendingDriveIRQ;
   ss_out(22)(17)           <= xa_muted;
   ss_out(23)( 7 downto  0) <= cdvol_next00;
   ss_out(23)(15 downto  8) <= cdvol_next01;
   ss_out(23)(23 downto 16) <= cdvol_next10;
   ss_out(23)(31 downto 24) <= cdvol_next11;
   ss_out(24)( 7 downto  0) <= cdvol_00;    
   ss_out(24)(15 downto  8) <= cdvol_01;    
   ss_out(24)(23 downto 16) <= cdvol_10;    
   ss_out(24)(31 downto 24) <= cdvol_11;    
   
   -- cpu interface
   process(clk1x)
   begin
      if (rising_edge(clk1x)) then
      
         FifoData_reset    <= '0';
         FifoResponse_Rd   <= '0';
         FifoParam_Wr      <= '0';
      
         if (reset = '1') then
            
            FifoData_reset  <= '1';
            
            irqTimeout      <= 0;
            
            CDROM_STATUS    <= ss_in(21)(7 downto 0); -- x"18";
            CDROM_IRQENA    <= ss_in(21)(12 downto 8); -- (others => '0');
            CDROM_IRQFLAG   <= ss_in(21)(20 downto 16); -- (others => '0');
            pendingDriveIRQ <= ss_in(13)(28 downto 24); -- (others => '0');
            xa_muted        <= ss_in(22)(17); -- '0';
            
            cdvol_next00    <= ss_in(23)( 7 downto  0); -- x"80"
            cdvol_next01    <= ss_in(23)(15 downto  8); -- x"00"
            cdvol_next10    <= ss_in(23)(23 downto 16); -- x"00"
            cdvol_next11    <= ss_in(23)(31 downto 24); -- x"80"
            cdvol_00        <= ss_in(24)( 7 downto  0); -- x"80"
            cdvol_01        <= ss_in(24)(15 downto  8); -- x"00"
            cdvol_10        <= ss_in(24)(23 downto 16); -- x"00"
            cdvol_11        <= ss_in(24)(31 downto 24); -- x"80"
            
         elsif (ce = '1') then
         
            beginCommand      <= '0';
            irqPulse          <= '0';
            ackRead_valid     <= '0';
            ackPendingIRQ     <= '0';
            ackPendingSector  <= '0';
            copyData          <= '0';

         
            CDROM_STATUS(2) <= '0';                      -- ADPBUSY XA-ADPCM fifo empty  (0=Empty) ;set when playing XA-ADPCM sound -> not used in duckstation
            CDROM_STATUS(3) <= FifoParam_Empty;          -- PRMEMPT Parameter fifo empty (1=Empty) ;triggered before writing 1st byte
            CDROM_STATUS(4) <= not FifoParam_NearFull;   -- PRMWRDY Parameter fifo full  (0=Full)  ;triggered after writing 16 bytes
            CDROM_STATUS(5) <= not FifoResponse_Empty;   -- RSLRRDY Response fifo empty  (0=Empty) ;triggered after reading LAST byte
            CDROM_STATUS(6) <= not FifoData_Empty;       -- DRQSTS  Data fifo empty      (0=Empty) ;triggered after reading LAST byte
            CDROM_STATUS(7) <= cmdPending;               -- BUSYSTS Command/parameter transmission busy  (1=Busy)  
         
            if (bus_write = '1') then
            
               if (bus_addr = x"0") then
                  CDROM_STATUS(1 downto 0) <= bus_dataWrite(1 downto 0);
               else
                  case (CDROM_STATUS(1 downto 0)) is
                     when "00" =>
                        case (bus_addr) is
                           when x"1" =>
                              beginCommand <= '1';
                              newCmd       <= bus_dataWrite;
                              
                           when x"2" =>
                              --todo: if (fifoParam.size() == 16) fifoParam.pop_front();
                              FifoParam_Wr  <= '1';
                              FifoParam_Din <= bus_dataWrite;
                           
                           when x"3" =>
                              if (bus_dataWrite(7) = '1') then
                                 if (FifoData_Empty = '1') then
                           
                                    if (firstSectorPending = '0') then
                                       copyData <= '1';
                                    end if;
                           
                                 end if;
                              else
                                 FifoData_reset <= '1';
                              end if;
                           when others => null;
                        end case;
                        
                     when "01" =>
                        case (bus_addr) is
                           when x"1" => -- sound map write -> do nothing
                           when x"2" =>
                              CDROM_IRQENA <= bus_dataWrite(4 downto 0);
                              
                           when x"3" =>
                              CDROM_IRQFLAG <= CDROM_IRQFLAG and (not bus_dataWrite(4 downto 0));
                              if (bus_dataWrite(6) = '1') then
                                 --todo: clear param fifo
                              end if;
                           
                           when others => null;
                        end case;
                     
                     when "10" =>
                        case (bus_addr) is
                           when x"1" => -- sound map coding info write -> do nothing
                           when x"2" => cdvol_next00 <= bus_dataWrite;
                           when x"3" => cdvol_next01 <= bus_dataWrite;
                           when others => null;
                        end case;
                        
                     when "11" =>
                        case (bus_addr) is
                           when x"1" => cdvol_next11 <= bus_dataWrite;
                           when x"2" => cdvol_next10 <= bus_dataWrite;
                           when x"3" => 
                              if (bus_dataWrite(5) = '1') then
                                 cdvol_00 <= cdvol_next00;
                                 cdvol_01 <= cdvol_next01;
                                 cdvol_10 <= cdvol_next10;
                                 cdvol_11 <= cdvol_next11;
                              end if;
                              xa_muted <= bus_dataWrite(0);
                           when others => null;
                        end case;
                     when others => null;
                  end case;
               end if;
            
            end if; -- end bus write
         
            if (bus_read = '1') then
               bus_dataRead <= (others => '0');
               case (bus_addr) is
                  when x"0" => 
                     bus_dataRead <= CDROM_STATUS;
                     
                  when x"1" =>
                     if (FifoResponse_Empty = '1') then
                        bus_dataRead    <= (others => '0');
                     else
                        bus_dataRead    <= FifoResponse_Dout;
                        FifoResponse_Rd <= '1';
                     end if;
                  
                  when x"2" =>
                     if (FifoData_Empty = '0') then
                        bus_dataRead <= FifoData_Dout;
                     else
                        bus_dataRead <= (others => '0');
                     end if;
                  
                  when x"3" =>
                     if (CDROM_STATUS(0) = '1') then
                        bus_dataRead <= "111" & irqFlagCPU;
                     else
                        bus_dataRead <= "111" & CDROM_IRQENA;
                     end if;
                  
                  when others => null;
               end case;
            end if;
            
            if (CDROM_IRQFLAG /= "00000") then
               irqTimeout <= 0;
            elsif (irqTimeout < 10000) then
               irqTimeout <= irqTimeout + 1;
            elsif (pendingDriveIRQ /= "00000" and ackPendingIRQ = '0' and ackPendingIRQNext = '0') then
               ackPendingIRQ   <= '1';
               if (pendingDriveIRQ = "00001") then
                  ackPendingSector <= '1';
               end if;
            end if;
            
            if (cmdAck = '1' or cmdIRQ = '1') then
               CDROM_IRQFLAG <= "00011";
               if (CDROM_IRQENA(1 downto 0) /= "00") then
                  irqPulse <= '1';
               end if;
            end if;            

            if (driveAck = '1' or ackDrive = '1') then
               if (CDROM_IRQFLAG /= "00000") then
                  pendingDriveIRQ      <= "00010";
                  pendingDriveResponse <= internalStatus;
               else
                  CDROM_IRQFLAG <= "00010";
                  if (CDROM_IRQENA(1) = '1') then
                     irqPulse <= '1';
                  end if;
               end if;
            end if;             

            if (ackDriveEnd = '1') then
               if (CDROM_IRQFLAG /= "00000") then
                  pendingDriveIRQ      <= "00100";
                  pendingDriveResponse <= internalStatus_1;
               else
                  CDROM_IRQFLAG <= "00100";
                  if (CDROM_IRQENA(2) = '1') then
                     irqPulse <= '1';
                  end if;
               end if;
            end if;
            
            -- INT1 (Data Ready)
            -- INT1 is always queued via pendingDriveIRQ to preserve hardware-like delay
            -- and avoid INT1/INT3 race conditions (Burger Burger, Jikkyō '95),
            -- while still guaranteeing delivery (Crime Crackers, Parodius).            
            if (ackRead = '1' or ackRead_data = '1') then            
               if (pendingDriveIRQ /= "00001") then
                  pendingDriveIRQ      <= "00001";
                  pendingDriveResponse <= internalStatus;
               end if;            
            end if;
            
            if (ackPendingIRQNext = '1') then
               CDROM_IRQFLAG   <= pendingDriveIRQ;
               pendingDriveIRQ <= (others => '0');
               if ((CDROM_IRQENA and pendingDriveIRQ) /= "00000") then
                  irqPulse <= '1';
               end if;
            end if;
            
            if (getIDAck = '1' and (hasCD = '0' or isAudioCD = '1')) then -- no async here, working will halt in case irq is still pending
               CDROM_IRQFLAG <= "00101";
               if (CDROM_IRQENA(0) = '1' or CDROM_IRQENA(2) = '1') then
                  irqPulse <= '1';
               end if;
            end if;
            if (getIDAck = '1' and (hasCD = '1' and isAudioCD = '0')) then -- no async here, working will halt in case irq is still pending
               CDROM_IRQFLAG <= "00010";
               if (CDROM_IRQENA(1) = '1') then
                  irqPulse <= '1';
               end if;
            end if;   
               
            if (errorResponseNext_new = '1') then
               CDROM_IRQFLAG   <= "00101";
               pendingDriveIRQ <= (others => '0');
               if (CDROM_IRQENA(2) = '1' or CDROM_IRQENA(0) = '1') then
                  irqPulse <= '1';
               end if;
            end if; 
            
            if (CDDA_outcnt = 8) then
               CDROM_IRQFLAG <= "00001";
               if (CDROM_IRQENA(0) = '1') then
                  irqPulse <= '1';
               end if;
            end if;
            
            if (softReset = '1') then
               FifoData_reset <= '1';
            end if;

         end if; -- ce
      end if;
   end process;
   
   ififoParam: entity mem.SyncFifoFallThrough
   generic map
   (
      SIZE             => 32,
      DATAWIDTH        => 8,
      NEARFULLDISTANCE => 16
   )
   port map
   ( 
      clk      => clk1x,     
      reset    => FifoParam_reset,   
                
      Din      => FifoParam_Din,     
      Wr       => FifoParam_Wr,      
      Full     => open,    
      NearFull => FifoParam_NearFull,

      Dout     => FifoParam_Dout,    
      Rd       => FifoParam_Rd,      
      Empty    => FifoParam_Empty   
   );
   
   
   ss_out(18)(1)             <= cmdPending;   
   ss_out(18)(0)             <= cmd_busy;     
   ss_out(12)(16 downto  0)  <= std_logic_vector(to_unsigned(cmd_delay, 17)); 
   ss_out(13)(23 downto 16)  <= nextCmd;
   ss_out(18)(2)             <= working;      
   ss_out( 0)(31 downto  0)  <= std_logic_vector(to_unsigned(workDelay, 32));    
   ss_out(14)(15 downto  8)  <= workCommand;  
   ss_out(18)(7)             <= muted;  
   ss_out(18)(3)             <= setLocActive; 
   ss_out(14)(23 downto 16)  <= std_logic_vector(setLocMinute); 
   ss_out(14)(31 downto 24)  <= std_logic_vector(setLocSecond); 
   ss_out(15)( 7 downto  0)  <= std_logic_vector(setLocFrame);
   ss_out(17)( 7 downto  0)  <= session; 
   ss_out(15)(23 downto 16)  <= std_logic_vector(fastForwardRate); 
   ss_out(16)(23 downto 16)  <= XaFilterFile;
   ss_out(16)(31 downto 24)  <= XaFilterChannel;
   ss_out(25)(20 downto 16)  <= lastreportCDDA;
   
   -- command processing
   -- command processing and drive: cd_ctrl (small engine + per-clock fast path, docs/cd_engine.md)
   nextSubdata_v <= nextSubdata(11) & nextSubdata(10) & nextSubdata(9) & nextSubdata(8) & nextSubdata(7) & nextSubdata(6) &
                    nextSubdata(5) & nextSubdata(4) & nextSubdata(3) & nextSubdata(2) & nextSubdata(1) & nextSubdata(0);
   ctrl_fetchIdle  <= '1' when (sectorFetchState = SFETCH_IDLE and readOnDisk_buffer = '0' and sectorProcessState = SPROC_IDLE and copyState = COPY_IDLE) else '0';
   ctrl_sfetchIdle <= '1' when (sectorFetchState = SFETCH_IDLE) else '0';
   ctrl_bufAhead   <= '1' when (sectorBufferSizes(to_integer(writeSectorPointer + 4)) /= 0) else '0';
   driveState      <= tdrivestate'val(to_integer(unsigned(ctrl_driveState)));
   CDDA_outcnt     <= 8 when ctrl_cdda_out8 = '1' else 0;

   -- hardware: in the original an interrupt and its response bytes come one clock apart; here the bytes can come
   -- an engine run (a few us) after the edge that raised the interrupt. So the CPU sees no new interrupt while
   -- the engine is not quiet: the pulse to the interrupt controller waits, and the flag register reads its value of
   -- the last quiet clock. In sim/cd_equiv (SIM_FREEZE) quiet is always '1': the original behaviour.
   irqOut     <= (irqPulse or irqHeld) and ctrl_quiet;
   irqFlagCPU <= CDROM_IRQFLAG when ctrl_quiet = '1' else irqFlagShown;
   process (clk1x)
   begin
      if (rising_edge(clk1x)) then
         if (ctrl_quiet = '1') then
            irqFlagShown <= CDROM_IRQFLAG;
         end if;
         if (reset = '1') then
            irqHeld <= '0';
         elsif (ce = '1') then
            irqHeld <= (irqPulse or irqHeld) and not ctrl_quiet;
         end if;
      end if;
   end process;

   -- the engine's clock is mapped straight onto the port (no signal in between: no delta cycle against clk1x)
   gctrl_sim : if SIM_FREEZE generate
      ictrl : entity work.cd_ctrl
      generic map (SIM_FREEZE => true)
      port map
      (
         clk => clk_engine, ce => ce, reset => reset, hasCD => hasCD, region => region, SS_reset => SS_reset, SS_wren => SS_wren,
         dma_read => dma_read,
         beginCommand => beginCommand, newCmd => newCmd, CDROM_IRQFLAG => CDROM_IRQFLAG, ackRead_valid => ackRead_valid,
         ackPendingIRQ => ackPendingIRQ, ackPendingSector => ackPendingSector, pendingDriveResponse => pendingDriveResponse,
         FifoParam_Dout => FifoParam_Dout, FifoParam_Empty => FifoParam_Empty, FifoParam_Wr => FifoParam_Wr,
         FifoResponse_Empty => FifoResponse_Empty,
         newCD => newCD, isAudioCD => isAudioCD, trackcountBCD => trackcountBCD, totalMinutesBCD => totalMinutesBCD,
         totalSecondsBCD => totalSecondsBCD, trackInfoA_lba => trackInfo_DataOutA(18 downto 0), minutesBCD => minutesBCD,
         secondsBCD => secondsBCD, isAudio => isAudio, lastReadSector => lastReadSector, header => header, subheader => subheader,
         headerIsData => headerIsData, headerDataSector => headerDataSector, nextSubdata => nextSubdata_v,
         peakvolumeL => peakvolumeL, peakvolumeR => peakvolumeR, UpdateSubchannelDone => UpdateSubchannelDone,
         fetchIdle => ctrl_fetchIdle, sectorFetchIdle => ctrl_sfetchIdle, bufferAhead => ctrl_bufAhead,
         XA_reset => XA_reset, XA_start => XA_start,
         FifoResponse_reset => FifoResponse_reset, FifoResponse_Wr => FifoResponse_Wr, FifoResponse_Din => FifoResponse_Din,
         FifoParam_Rd => FifoParam_Rd, FifoParam_reset => FifoParam_reset, cmdPending => cmdPending, cmdAck => cmdAck,
         cmdIRQ => cmdIRQ, driveAck => driveAck, getIDAck => getIDAck, softReset => softReset,
         errorResponseNext_new => errorResponseNext_new, ackPendingIRQNext => ackPendingIRQNext, cdda_out8 => ctrl_cdda_out8,
         ackDrive => ackDrive, ackDriveEnd => ackDriveEnd, ackRead => ackRead, internalStatus => internalStatus,
         internalStatus_1 => internalStatus_1,
         cmdResetXa => cmdResetXa, ClearXACurrentSet => ClearXACurrentSet, XaFilterFile => XaFilterFile,
         XaFilterChannel => XaFilterChannel, muted => muted, cmd_busy => cmd_busy, working => working, driveBusy => driveBusy,
         driveDelay => driveDelay, driveState => ctrl_driveState, modeReg => modeReg, currentLBA => currentLBA,
         physicalLBA => physicalLBA, readLBA => readLBA, writeSectorPointer => writeSectorPointer,
         readSectorPointer => readSectorPointer, readOnDisk => readOnDisk, seekOnDiskDrive => seekOnDiskDrive,
         startReading => startReading, startPlaying => startPlaying, processDataSector => processDataSector,
         processCDDASector => processCDDASector, processSeekHeader => processSeekHeader,
         clearSectorBuffers => clearSectorBuffers, triggerUpdateSubchannel => triggerUpdateSubchannel,
         handleDrive => handleDrive, newCD_1 => newCD_1,
         eng_busy => eng_busy, quiet => ctrl_quiet
      );
   end generate;
   gctrl : if not SIM_FREEZE generate
      ictrl : entity work.cd_ctrl
      generic map (SIM_FREEZE => false)
      port map
      (
         clk => clk1x, ce => ce, reset => reset, hasCD => hasCD, region => region, SS_reset => SS_reset, SS_wren => SS_wren,
         dma_read => dma_read,
         beginCommand => beginCommand, newCmd => newCmd, CDROM_IRQFLAG => CDROM_IRQFLAG, ackRead_valid => ackRead_valid,
         ackPendingIRQ => ackPendingIRQ, ackPendingSector => ackPendingSector, pendingDriveResponse => pendingDriveResponse,
         FifoParam_Dout => FifoParam_Dout, FifoParam_Empty => FifoParam_Empty, FifoParam_Wr => FifoParam_Wr,
         FifoResponse_Empty => FifoResponse_Empty,
         newCD => newCD, isAudioCD => isAudioCD, trackcountBCD => trackcountBCD, totalMinutesBCD => totalMinutesBCD,
         totalSecondsBCD => totalSecondsBCD, trackInfoA_lba => trackInfo_DataOutA(18 downto 0), minutesBCD => minutesBCD,
         secondsBCD => secondsBCD, isAudio => isAudio, lastReadSector => lastReadSector, header => header, subheader => subheader,
         headerIsData => headerIsData, headerDataSector => headerDataSector, nextSubdata => nextSubdata_v,
         peakvolumeL => peakvolumeL, peakvolumeR => peakvolumeR, UpdateSubchannelDone => UpdateSubchannelDone,
         fetchIdle => ctrl_fetchIdle, sectorFetchIdle => ctrl_sfetchIdle, bufferAhead => ctrl_bufAhead,
         XA_reset => XA_reset, XA_start => XA_start,
         FifoResponse_reset => FifoResponse_reset, FifoResponse_Wr => FifoResponse_Wr, FifoResponse_Din => FifoResponse_Din,
         FifoParam_Rd => FifoParam_Rd, FifoParam_reset => FifoParam_reset, cmdPending => cmdPending, cmdAck => cmdAck,
         cmdIRQ => cmdIRQ, driveAck => driveAck, getIDAck => getIDAck, softReset => softReset,
         errorResponseNext_new => errorResponseNext_new, ackPendingIRQNext => ackPendingIRQNext, cdda_out8 => ctrl_cdda_out8,
         ackDrive => ackDrive, ackDriveEnd => ackDriveEnd, ackRead => ackRead, internalStatus => internalStatus,
         internalStatus_1 => internalStatus_1,
         cmdResetXa => cmdResetXa, ClearXACurrentSet => ClearXACurrentSet, XaFilterFile => XaFilterFile,
         XaFilterChannel => XaFilterChannel, muted => muted, cmd_busy => cmd_busy, working => working, driveBusy => driveBusy,
         driveDelay => driveDelay, driveState => ctrl_driveState, modeReg => modeReg, currentLBA => currentLBA,
         physicalLBA => physicalLBA, readLBA => readLBA, writeSectorPointer => writeSectorPointer,
         readSectorPointer => readSectorPointer, readOnDisk => readOnDisk, seekOnDiskDrive => seekOnDiskDrive,
         startReading => startReading, startPlaying => startPlaying, processDataSector => processDataSector,
         processCDDASector => processCDDASector, processSeekHeader => processSeekHeader,
         clearSectorBuffers => clearSectorBuffers, triggerUpdateSubchannel => triggerUpdateSubchannel,
         handleDrive => handleDrive, newCD_1 => newCD_1,
         eng_busy => eng_busy, quiet => ctrl_quiet
      );
   end generate;

   
   ififoResponse: entity mem.SyncFifoFallThrough
   generic map
   (
      SIZE             => 32,
      DATAWIDTH        => 8,
      NEARFULLDISTANCE => 16
   )
   port map
   ( 
      clk      => clk1x,     
      reset    => FifoResponse_reset,   
                
      Din      => FifoResponse_Din,     
      Wr       => FifoResponse_Wr,      
      Full     => open,    
      NearFull => open,

      Dout     => FifoResponse_Dout,    
      Rd       => FifoResponse_Rd,      
      Empty    => FifoResponse_Empty   
   );
   
   ss_out(18)(4)            <= driveBusy;           
   ss_out(15)(27 downto 24) <= std_logic_vector(to_unsigned(tdrivestate'POS(driveState), 4));  
   ss_out( 4)(26 downto  0) <= std_logic_vector(to_unsigned(driveDelay, 27));           
   ss_out( 5)(26 downto  0) <= std_logic_vector(to_unsigned(driveDelayNext, 27));           
   ss_out(13)( 7 downto  0) <= internalStatus;       
   ss_out(13)(15 downto  8) <= modeReg;                   
   ss_out( 3)(19 downto  0) <= std_logic_vector(to_unsigned(currentLBA, 20));           
   ss_out( 7)(19 downto  0) <= std_logic_vector(to_unsigned(physicalLBA, 20));           
   ss_out(18)(5)            <= readAfterSeek;        
   ss_out(18)(6)            <= playAfterSeek;        
   ss_out(18)(8)            <= lastSectorHeaderValid;
   ss_out(16)( 2 downto  0) <= std_logic_vector(writeSectorPointer);   
   ss_out(16)(10 downto  8) <= std_logic_vector(readSectorPointer);    
   ss_out(25)( 7 downto  0) <= currentTrackBCD;    
   
   gSSsubdata: for i in 0 to 11 generate
   begin
      ss_out(i + 76)(7 downto 0) <= subdata(i);
   end generate;
   

   -- drive
   
   iramSectorBuffer: entity mem.dpram
   generic map 
   ( 
      addr_width => 10, 
      data_width => 32
   )
   port map
   (
      clock_a     => clk1x,
      address_a   => sectorBuffer_addrA,
      data_a      => sectorBuffer_DataA,
      wren_a      => sectorBuffer_wrenA,
                     
      clock_b     => clk1x,
      address_b   => sectorBuffer_addrB,
      data_b      => x"00000000",
      wren_b      => '0',
      q_b         => sectorBuffer_DataB
   );
   
   sectorBuffer_addrB <= std_logic_vector(SS_Adr(9 downto 0)) when (SS_rden = '1') else std_logic_vector(to_unsigned(procReadAddr, 10));
   
   sectorBuffers_addrB <= std_logic_vector(to_unsigned(to_integer(SS_Adr - 2048), 13)) when (SS_rden = '1' and SS_Adr >= 2048 and SS_Adr < 2048 + (1024 * 8)) else 
                          std_logic_vector(copySectorPointer & to_unsigned(copyReadAddr, 10));
   
   iramSectorBuffers: entity mem.dpram
   generic map 
   ( 
      addr_width => 13, 
      data_width => 32
   )
   port map
   (
      clock_a     => clk1x,
      address_a   => sectorBuffers_addrA,
      data_a      => sectorBuffers_DataA,
      wren_a      => sectorBuffers_wrenA,
      
      clock_b     => clk1x,
      address_b   => sectorBuffers_addrB,
      data_b      => x"00000000",
      wren_b      => '0',
      q_b         => sectorBuffers_DataB
   );
   
   ss_out(19)              <= header;
   ss_out(20)              <= subheader;
   ss_out(6)(19 downto 0)  <= std_logic_vector(to_unsigned(lastReadSector, 20));
   ss_out(11)(19 downto 0) <= std_logic_vector(to_unsigned(positionInIndex, 20));
   
   ss_out(22)( 7 downto 0) <= XaCurrentFile;
   ss_out(22)(15 downto 8) <= XaCurrentChannel;
   ss_out(22)(16)          <= XaCurrentSet;
   
   gSSnextsubdata: for i in 0 to 11 generate
   begin
      ss_out(i + 64)(7 downto 0) <= nextSubdata(i);
   end generate;
   
   gSSsectorBufferSizes: for i in 0 to 7 generate
   begin
      ss_out(i + 96)(11 downto 0) <= std_logic_vector(to_unsigned(sectorBufferSizes(i), 10)) & "00";
   end generate;
   
   -- data processing
   process(clk1x)
      variable checkData : std_logic_vector(31 downto 0);
      variable frameLeft : unsigned(6 downto 0);
   begin
      if (rising_edge(clk1x)) then

         FifoData_Wr  <= '0';
         
         sectorBuffer_wrenA  <= '0';
         sectorBuffers_wrenA <= '0';
         
         error               <= '0';
         
         pauseCD <= '0';
         if (pauseOnCDSlow = '1') then
            if ((sectorFetchState /= SFETCH_IDLE) and driveDelay < 32768) then
               pauseCD <= '1';
            end if;
         end if;
         
         if (testSeek = '0' and (driveState = DRIVE_SEEKLOGICAL or driveState = DRIVE_SEEKPHYSICAL or driveState = DRIVE_SEEKIMPLICIT)) then
            slow_block <= '1';
         elsif (handleDrive = '1' and (driveState = DRIVE_READING or driveState = DRIVE_PLAYING)) then
            slow_block <= '0';
         end if;
         
         if ((sectorFetchState /= SFETCH_IDLE) and driveDelay < 128 and slow_block = '0') then
            cdSlow       <= '1';
            slow_timeout <= 16777215;
         elsif (slow_timeout > 0) then
            slow_timeout <= slow_timeout - 1;
         else
            cdSlow <= '0';
         end if;
         
         XA_write   <= '0';
         XA_start   <= '0'; 
         CDDA_write <= '0'; 

         if (reset = '1') then
         
            cd_hps_req           <= '0';
            
            sectorFetchState     <= SFETCH_IDLE;
            sectorProcessState   <= SPROC_IDLE;
            copyState            <= COPY_IDLE;
            header               <= ss_in(19);
            subheader            <= ss_in(20);
            lastReadSector       <= to_integer(unsigned(ss_in(6)(19 downto 0))); -- 0
            
            XaCurrentFile        <= ss_in(22)( 7 downto 0);
            XaCurrentChannel     <= ss_in(22)(15 downto 8);
            XaCurrentSet         <= ss_in(22)(16);
            
            positionInIndex   <= to_integer(signed(ss_in(11)(19 downto 0))); -- 0
            
            for i in 0 to 11 loop
               nextSubdata(i) <= ss_in(i + 64)(7 downto 0);
            end loop;
            
            for i in 0 to 7 loop
               sectorBufferSizes(i) <= to_integer(unsigned(ss_in(i + 96)(11 downto 2)));
            end loop;
            
            cd_hps_lba <= (others => '1');
            
         elsif (SS_wren = '1') then
            
            if (SS_Adr >= 1024 and SS_Adr < 1024 + (RAW_SECTOR_SIZE / 4)) then
               sectorBuffer_addrA <= std_logic_vector(SS_Adr(9 downto 0));
               sectorBuffer_DataA <= SS_DataWrite;
               sectorBuffer_wrenA <= '1';
            end if;
            
            if (SS_Adr >= 2048 and SS_Adr < 2048 + (1024 * 8)) then 
               sectorBuffers_addrA <= std_logic_vector(to_unsigned(to_integer(SS_Adr - 2048), 13));
               sectorBuffers_DataA <= SS_DataWrite;
               sectorBuffers_wrenA <= '1';
               -- synthesis translate_off
               if ((to_integer(SS_Adr) mod 2048) < 588) then
                  sectorBuffers(((to_integer(SS_Adr) - 2048) / 1024))(to_integer(SS_Adr) mod 2048) <= SS_DataWrite;
               end if;
               -- synthesis translate_on
            end if;
            
            if (SS_Adr = 1024) then headerIsData <= '1'; headerDataSector <= '1'; headerDataCheck <= '1'; end if;
            if (SS_Adr = 1027 and SS_DataWrite(31 downto 24) /= x"02") then headerDataCheck <= '0'; headerDataSector <= '0'; end if;
            if (SS_Adr = 1028 and SS_DataWrite(22) = '1' and SS_DataWrite(18) = '1' and headerDataCheck = '1') then headerIsData <= '0'; end if;
            
         else
         
            if (ce = '1') then
               ackRead_data         <= '0';
               UpdateSubchannelDone <= '0';
            end if;
            
            readSubchannel <= '0';
   
            if (readOnDisk = '1') then
               readOnDisk_buffer <= '1';
               readLBA_buffer    <= readLBA;
            end if;
      
            case (sectorFetchState) is
            
               when SFETCH_IDLE =>
                  if (readOnDisk_buffer = '1' and ce = '1') then
                     lastReadSector    <= readLBA_buffer;
                     sectorFetchState  <= SFETCH_DELAY;
                     fetchCount        <= 0;
                     fetchDelay        <= 15;
                     readOnDisk_buffer <= '0';
                  end if;
                  
               when SFETCH_DELAY => -- delay to give processing a head start with copy and wait for HPS ack before new request
                  if (cd_hps_ack = '0') then
                     if (fetchDelay > 0) then
                        fetchDelay <= fetchDelay - 1;
                     elsif (trackSearchState = TRACKSEARCH_IDLE) then
                        sectorFetchState <= SFETCH_START;
                        if (trackNumberBCD = "01" and isAudio = '0') then
                           positionInIndex <= lastReadSector - startLBA - PREGAPSIZE;  
-- synthesis translate_off
                           positionInIndex <= lastReadSector - startLBA; -- pregap not present in simulation
-- synthesis translate_on
                        else
                           positionInIndex <= lastReadSector - startLBA;
                        end if;
                     end if;   
                  end if;
                  
               when SFETCH_START =>
                  if (cd_hps_lba /= std_logic_vector(to_unsigned(lastReadSector, 32))) then
                     sectorFetchState <= SFETCH_HPSACK;
                     cd_hps_req       <= '1';
                  else
                     sectorFetchState <= SFETCH_IDLE;
                  end if;
                     
                  cd_hps_lba       <= std_logic_vector(to_unsigned(lastReadSector, 32));
                  cd_hps_lba_sim   <= (others => '0'); 
                  -- synthesis translate_off
                  cd_hps_lba_sim(23 downto 0)  <= std_logic_vector(to_unsigned(lastReadSector - startLBA, 24));
                  cd_hps_lba_sim(31 downto 24) <= '0' & trackNumber;
                  -- synthesis translate_on
                  
                  readSubchannel <= '1';
															
                  if (LIBCRYPT_ON and (
                      (libcryptKey(15) = '1' and ((lastReadSector + 2) = 14105 or (lastReadSector + 2) = 14110)) or
                      (libcryptKey(14) = '1' and ((lastReadSector + 2) = 14231 or (lastReadSector + 2) = 14236)) or
                      (libcryptKey(13) = '1' and ((lastReadSector + 2) = 14485 or (lastReadSector + 2) = 14490)) or
                      (libcryptKey(12) = '1' and ((lastReadSector + 2) = 14579 or (lastReadSector + 2) = 14584)) or
                      (libcryptKey(11) = '1' and ((lastReadSector + 2) = 14649 or (lastReadSector + 2) = 14654)) or
                      (libcryptKey(10) = '1' and ((lastReadSector + 2) = 14899 or (lastReadSector + 2) = 14904)) or
                      (libcryptKey(9)  = '1' and ((lastReadSector + 2) = 15056 or (lastReadSector + 2) = 15061)) or
                      (libcryptKey(8)  = '1' and ((lastReadSector + 2) = 15130 or (lastReadSector + 2) = 15135)) or
                      (libcryptKey(7)  = '1' and ((lastReadSector + 2) = 15242 or (lastReadSector + 2) = 15247)) or
                      (libcryptKey(6)  = '1' and ((lastReadSector + 2) = 15312 or (lastReadSector + 2) = 15317)) or
                      (libcryptKey(5)  = '1' and ((lastReadSector + 2) = 15378 or (lastReadSector + 2) = 15383)) or
                      (libcryptKey(4)  = '1' and ((lastReadSector + 2) = 15628 or (lastReadSector + 2) = 15633)) or
                      (libcryptKey(3)  = '1' and ((lastReadSector + 2) = 15919 or (lastReadSector + 2) = 15924)) or
                      (libcryptKey(2)  = '1' and ((lastReadSector + 2) = 16031 or (lastReadSector + 2) = 16036)) or
                      (libcryptKey(1)  = '1' and ((lastReadSector + 2) = 16101 or (lastReadSector + 2) = 16106)) or
                      (libcryptKey(0)  = '1' and ((lastReadSector + 2) = 16167 or (lastReadSector + 2) = 16172))
                  )) then
                      readSubchannel <= '0';
                  end if;
                  
                  if (hasCD = '0') then
                     cd_hps_req       <= '0';
                     sectorFetchState <= SFETCH_IDLE;
                  end if;
                  
               when SFETCH_HPSACK => 
                  if (cd_hps_ack = '1') then
                     sectorFetchState <= SFETCH_HPSWORD;
                     cd_hps_req       <= '0';
                     peakvolumeL      <= (others => '0');
                     peakvolumeR      <= (others => '0');
                  end if;
               
               when SFETCH_HPSWORD =>
                  if (cd_hps_write = '1') then
                     sectorFetchState                <= SFETCH_HPSDATA;
                     if (positionInIndex >= 0) then
                        sectorBuffer_DataA(15 downto 0) <= cd_hps_data;
                        if (signed(cd_hps_data) > peakvolumeL) then
                           peakvolumeL <= signed(cd_hps_data);
                        end if;
                     else
                        sectorBuffer_DataA(15 downto 0) <= (others => '0');
                     end if; 
                  end if;
               
               when SFETCH_HPSDATA =>
                  if (cd_hps_write = '1') then
                     sectorBuffer_addrA <= std_logic_vector(to_unsigned(fetchCount, 10));
                     sectorBuffer_wrenA <= '1';
                     checkData := sectorBuffer_DataA;
                     if (positionInIndex >= 0) then
                        checkData(31 downto 16) := cd_hps_data;
                        if (signed(cd_hps_data) > peakvolumeR) then
                           peakvolumeR <= signed(cd_hps_data);
                        end if;
                     else
                        checkData(31 downto 16) := (others => '0');
                     end if;
                     sectorBuffer_DataA <= checkData;
                     
                     if (fetchCount = 587) then
                        sectorFetchState <= SFETCH_IDLE;
                     else
                        fetchCount  <= fetchCount + 1;
                        sectorFetchState <= SFETCH_HPSWORD;
                     end if;
                     
                     if (fetchCount = 0) then headerIsData <= '1'; headerDataSector <= '1'; headerDataCheck <= '1'; end if;
                     if (fetchcount = 3 and checkData(31 downto 24) /= x"02") then headerDataCheck <= '0'; headerDataSector <= '0'; end if;
                     if (fetchcount = 4 and checkData(22) = '1' and checkData(18) = '1' and headerDataCheck = '1') then headerIsData <= '0'; end if;
                  end if;
                  
            end case;
            
            -- reset cached sector on CD switch
            if (newCD = '1' and newCD_1 = '0') then
               cd_hps_lba <= (others => '1');
            end if;
            
            case (readSubchannelState) is
            
               when SSUB_IDLE =>
                  null;
                  
               when SSUB_START =>
                  nextSubdata(0)       <= '0' & (not isAudio) & "000000"; -- index control bits
                  nextSubdata(1)       <= std_logic_vector(trackNumberBCD); 

                  if ((subchannelSector - startLBA) >= PREGAPSIZE) then
                     nextSubdata(2)       <= x"01"; -- index number
                     if (isAudio = '0') then
                        subchannelLBAwork    <= subchannelSector - startLBA;
                     else
                        subchannelLBAwork    <= subchannelSector - startLBA - PREGAPSIZE;
                     end if;
                  else
                     nextSubdata(2)       <= x"00"; -- index number
                     subchannelLBAwork    <= PREGAPSIZE - (subchannelSector - startLBA) - 1;
                  end if;
                  readSubchannelState  <= SSUB_CALCPOS;
                  sub_SecondsHigh      <= (others => '0');
                  sub_SecondsLow       <= (others => '0');
                  sub_MinutesHigh      <= (others => '0');
                  sub_MinutesLow       <= (others => '0');
                  
               
               when SSUB_CALCPOS | SSUB_CALCSECTOR =>
                  if (subchannelLBAwork >= FRAMES_PER_SECOND) then
                     subchannelLBAwork <= subchannelLBAwork - FRAMES_PER_SECOND;
                  
                     if (sub_SecondsLow < 9) then
                        sub_SecondsLow <= sub_SecondsLow + 1;
                     else
                        sub_SecondsLow <= (others => '0');
                        if (sub_SecondsHigh < 5) then
                           sub_SecondsHigh <= sub_SecondsHigh + 1;
                        else
                           sub_SecondsHigh <= (others => '0');
                           if (sub_MinutesLow < 9) then
                              sub_MinutesLow <= sub_MinutesLow + 1;
                           else
                              sub_MinutesLow  <= (others => '0');
                              sub_MinutesHigh <= sub_MinutesHigh + 1;
                           end if;
                        end if;
                     end if;
                  else
                     frameLeft := to_unsigned(subchannelLBAwork, 7);
                     subchannelLBAwork <= 0;
                     if (readSubchannelState = SSUB_CALCPOS) then
                        readSubchannelState        <= SSUB_CALCSECTOR;
                        subchannelLBAwork          <= subchannelSector; 
                        nextSubdata(3)             <= std_logic_vector(sub_MinutesHigh & sub_MinutesLow);
                        nextSubdata(4)             <= std_logic_vector(sub_SecondsHigh & sub_SecondsLow);
                        nextSubdata(5)           <= toBCD7(frameLeft);
                        sub_SecondsHigh            <= (others => '0');
                        sub_SecondsLow             <= (others => '0');
                        sub_MinutesHigh            <= (others => '0');
                        sub_MinutesLow             <= (others => '0');
                     else
                        readSubchannelState        <= SSUB_IDLE;
                        UpdateSubchannelDone       <= UpdateSubchannel;
                        UpdateSubchannel           <= '0';
                        nextSubdata(7)             <= std_logic_vector(sub_MinutesHigh & sub_MinutesLow);
                        nextSubdata(8)             <= std_logic_vector(sub_SecondsHigh & sub_SecondsLow);
                        nextSubdata(9)           <= toBCD7(frameLeft);
                     end if;
                  end if;
               
            end case;
            
            if (readSubchannel = '1' or triggerUpdateSubchannel = '1') then
               readSubchannelState <= SSUB_START;
               if (triggerUpdateSubchannel = '1') then
                  subchannelSector <= physicalLBA;
                  UpdateSubchannel <= '1';
               else
                  subchannelSector <= lastReadSector +2;
               end if;
            end if;
            
            case (sectorProcessState) is
            
               when SPROC_IDLE =>
                  procReadAddr <= SECTOR_SYNC_SIZE / 4;
                  if (processDataSector = '1' and ce = '1') then
                     sectorProcessState <= SPROC_READHEADER;
                     procReadAddr       <= procReadAddr + 1;
                     procCount          <= 0;
                  elsif (processCDDASector = '1' and ce = '1') then
                     procReadAddr       <= 0;
                     procCount          <= 0;
                     sectorProcessState <= SPROC_CDDAFIRST;
                  elsif (processSeekHeader = '1' and ce = '1') then
                     sectorProcessState <= SPROC_SEEKREADHEADER;
                     procReadAddr       <= procReadAddr + 1;
                  end if;
                  
               when SPROC_SEEKREADHEADER    => sectorProcessState <= SPROC_SEEKREADSUBHEADER; header    <= sectorBuffer_DataB;
               when SPROC_SEEKREADSUBHEADER => sectorProcessState <= SPROC_IDLE;              subheader <= sectorBuffer_DataB;
                  
               when SPROC_READHEADER =>
                  sectorProcessState <= SPROC_READSUBHEADER;
                  header <= sectorBuffer_DataB;
                  
               when SPROC_READSUBHEADER => 
                  subheader <= sectorBuffer_DataB;
                  sectorProcessState <= SPROC_START;
                  if (modeReg(6) = '1') then -- xa_enable
                     if (header(31 downto 24) = x"02") then
                        if (sectorBuffer_DataB(22) = '1') then -- realtime
                           if (sectorBuffer_DataB(18) = '1') then -- audio
                              procReadAddr         <= 0;
                              sectorProcessState   <= SPROC_XA_FIRST;
                           end if;
                        end if;
                     end if;
                  end if;
                  
               when SPROC_START =>
                  sectorProcessState <= SPROC_FIRST;
                  if (modeReg(5) = '1') then -- raw sector read
                     procReadAddr <= SECTOR_SYNC_SIZE / 4;
                     procSize <= (RAW_SECTOR_OUTPUT_SIZE) / 4;
                  else
                     procReadAddr <= (SECTOR_SYNC_SIZE + 12) / 4;
                     procSize <= (DATA_SECTOR_SIZE) / 4;
                     if (header(31 downto 24) /= x"02") then
                        sectorProcessState   <= SPROC_IDLE;
                     end if;
                  end if;
               
               when SPROC_FIRST =>
                  sectorProcessState <= SPROC_DATA;
                  procReadAddr <= procReadAddr + 1;
                  if (sectorBufferSizes(to_integer(writeSectorPointer)) /= 0) then
                     --error <= '1';
                  end if;
                  sectorBufferSizes(to_integer(writeSectorPointer)) <= procSize;
				  firstSectorPending <= '0';
               
               when SPROC_DATA =>
                  procCount    <= procCount + 1;
                  if (procReadAddr < 587) then
                     procReadAddr <= procReadAddr + 1;
                  end if;
                  -- synthesis translate_off
                  sectorBuffers(to_integer(writeSectorPointer))(procCount) <= sectorBuffer_DataB;
                  -- synthesis translate_on
                  sectorBuffers_addrA <= std_logic_vector(writeSectorPointer & to_unsigned(procCount, 10));
                  sectorBuffers_DataA <= sectorBuffer_DataB;
                  sectorBuffers_wrenA <= '1';
                  if (procCount = (procSize - 1)) then
                     sectorProcessState  <= SPROC_IDLE;
                  end if;
                  
               when SPROC_XA_FIRST =>
                  sectorProcessState <= SPROC_XA;
                  procReadAddr       <= procReadAddr + 1;
                  if (modeReg(3) = '1' and ((XaFilterChannel /= subheader(15 downto 8)) or XaFilterFile /= subheader(7 downto 0))) then
                     sectorProcessState <= SPROC_IDLE;
                  else
                     if (XaCurrentSet = '0') then
                        if (subheader(15 downto 8) = x"FF" and (modeReg(3) = '0' or XaFilterChannel /= x"FF")) then
                           sectorProcessState <= SPROC_IDLE;   
                        else
                           XaCurrentFile    <= subheader(7 downto 0);
                           XaCurrentChannel <= subheader(15 downto 8);
                           XaCurrentSet     <= '1';   
                        end if;
                     elsif (XaCurrentSet = '1' and ((XaCurrentChannel /= subheader(15 downto 8)) or XaCurrentFile /= subheader(7 downto 0))) then
                        sectorProcessState <= SPROC_IDLE;
                     end if;
                  end if;
                  
               when SPROC_XA =>
                  procCount    <= procCount + 1;
                  if (procReadAddr < 587) then
                     procReadAddr <= procReadAddr + 1;
                  end if;
                  if (procCount = 587) then
                     sectorProcessState  <= SPROC_IDLE;
                     XA_start            <= '1';
                  end if;
                  XA_write <= '1';
                  XA_data  <= sectorBuffer_DataB;
                  XA_addr  <= procCount;
                  
               when SPROC_CDDAFIRST =>
                  sectorProcessState <= SPROC_CDDA;
                  procReadAddr       <= procReadAddr + 1;
               
               when SPROC_CDDA =>
                  procCount    <= procCount + 1;
                  if (procReadAddr < 587) then
                     procReadAddr <= procReadAddr + 1;
                  end if;
                  if (procCount = 587) then
                     sectorProcessState  <= SPROC_IDLE;
                  end if;
                  CDDA_write <= '1';
                  CDDA_data  <= sectorBuffer_DataB;
                  
            end case;

															
            case (copyState) is
            
               when COPY_IDLE =>
                  if (copyData = '1' and ce = '1') then
                     copyState         <= COPY_FIRST;
                     copyCount         <= 0;
                     copyReadAddr      <= 0;
                     copyByteCnt       <= 0;
                     copySectorPointer <= readSectorPointer;
                     sectorBufferSizes(to_integer(readSectorPointer)) <= 0;
                     if (sectorBufferSizes(to_integer(readSectorPointer)) = 0) then
                        copySize <= RAW_SECTOR_OUTPUT_SIZE / 4;
                     else
                        copySize <= sectorBufferSizes(to_integer(readSectorPointer));
                     end if;
                  end if;
               
               when COPY_FIRST =>
                  copyState     <= COPY_DATA;
                   if (sectorBufferSizes(to_integer(writeSectorPointer)) /= 0) then -- additional irq for missed sector
                     ackRead_data <= '1';
                  end if;
               
               when COPY_DATA =>
                  FifoData_Wr  <= '1';
                  case (copyByteCnt) is
                     when 0 => 
                        copyByteCnt <= 1; 
                        FifoData_Din <= sectorBuffers_DataB(7 downto 0);
                        
                     when 1 => 
                        copyByteCnt <= 2; 
                        FifoData_Din <= sectorBuffers_DataB(15 downto 8);
                        
                     when 2 => 
                        copyByteCnt  <= 3; 
                        FifoData_Din <= sectorBuffers_DataB(23 downto 16); 
                        copyReadAddr <= copyReadAddr + 1;
                        
                     when 3 => 
                        copyByteCnt  <= 0; 
                        FifoData_Din <= sectorBuffers_DataB(31 downto 24); 
                        copyCount    <= copyCount + 1;
                        if (copyCount = (copySize - 1)) then
                           copyState  <= COPY_IDLE;
                        end if;
                     when others => null;
                  end case;
                 
            end case;
            
            -- if data fifo is reset while copy is still ongoing, stop copy immidiatly so fifo stays empty
            if (FifoData_reset = '1') then            
               firstSectorPending <= '0';            
               if (copyState /= COPY_IDLE) then
                  copyState   <= COPY_IDLE;
                  FifoData_Wr <= '0';
               end if;            
            end if;
            
            if (clearSectorBuffers = '1') then
               sectorBufferSizes <= (others => 0);
			   firstSectorPending <= '1';
            end if;

         end if;
         
         if (XA_eof = '1') then
            XaCurrentFile    <= (others => '0');
            XaCurrentChannel <= (others => '0');
            XaCurrentSet     <= '0';         
         end if;
         
         if (ClearXACurrentSet = '1') then
            XaCurrentSet     <= '0';
         end if;
         
      end if;
   end process;
   
--##############################################################
--############################### Audio
--##############################################################
   
   XA_reset <= reset or softReset or startReading or startPlaying or cmdResetXa or seekOnDiskDrive;
   
   icd_xa : entity work.cd_xa
   port map
   (
      clk1x                => clk1x,    
      reset                => reset,

      spu_tick             => spu_tick,
      
      CDDA_write           => CDDA_write,
      CDDA_data            => CDDA_data, 
      
      XA_addr              => XA_addr,  
      XA_data              => XA_data,  
      XA_write             => XA_write, 
      XA_start             => XA_start,
      XA_reset             => XA_reset,
      
      XA_eof               => XA_eof,
      
      cdaudio_left         => cdaudio_left, 
      cdaudio_right        => cdaudio_right
   );
   
   process(clk1x)
   begin
      if (rising_edge(clk1x)) then
         
         if (spu_tick = '1') then
            if ((driveState = DRIVE_PLAYING or (driveState = DRIVE_READING and modeReg(0) = '1') or (modeReg(6) = '1' and xa_muted = '0')) and muted = '0') then
               cd_volume_step <= 0;
            else
               cd_left  <= (others => '0');
               cd_right <= (others => '0');
            end if;
         end if;
      
         case (cd_volume_step) is
            when 0 =>
               soundmul1 <= cdaudio_left;
               soundmul2 <= unsigned(cdvol_00);
               
            when 1 =>
               soundmul1 <= cdaudio_right;
               soundmul2 <= unsigned(cdvol_10);

            when 2 =>
               soundmul1 <= cdaudio_left;
               soundmul2 <= unsigned(cdvol_01);
               
               soundsum <= resize(soundmulresult, 18);
               
            when 3 =>
               soundmul1 <= cdaudio_right;
               soundmul2 <= unsigned(cdvol_11); 
               
               soundsum <= soundsum + resize(soundmulresult, 18);
            
            when 4 =>
               if (soundsum < -32768) then cd_left <= x"8000";
               elsif (soundsum > 32767) then cd_left <= x"7FFF";
               else cd_left <= soundsum(15 downto 0);
               end if;
            
               soundsum <= resize(soundmulresult, 18);
            
            when 5 =>
               soundsum <= soundsum + resize(soundmulresult, 18);
               
            when 6 =>
               if (soundsum < -32768) then cd_right <= x"8000";
               elsif (soundsum > 32767) then cd_right <= x"7FFF";
               else cd_right <= soundsum(15 downto 0);
               end if;
            
            when others => null;
         end case;
         if (cd_volume_step < 7) then
            cd_volume_step <= cd_volume_step + 1;
         end if; 
         
         soundmulresult <= resize(shift_right(soundmul1 * signed('0' & soundmul2), 7), 17);  
      
      end if;
   end process;
   
--##############################################################
--############################### track infos
--##############################################################

   ss_out(15)(15 downto 8) <= std_logic_vector(trackNumberBCD);
   ss_out(25)(14 downto 8) <= trackNumber;
   
   process(clk1x)
   begin
      if (rising_edge(clk1x)) then
      
         trackInfo_wrenA <= '0';
         
         if (reset = '1') then
         
            trackSearchState     <= TRACKSEARCH_IDLE;
         
            trackNumberBCD       <= unsigned(ss_in(15)(15 downto 8)); --  x"00";
            trackInfo_addrB      <= ss_in(25)(14 downto 8); -- x"00";
            trackNumber          <= ss_in(25)(14 downto 8); -- x"00";
            
            newCD                <= '0';
            newCDCounter         <= (others => '0');
            
         else  
         
            newCD <= '0';
            if (ce = '1') then
               if (newCDCounter > 0) then
                  newCDCounter <= newCDCounter - 1;
                  newCD        <= '1';
               end if;
            end if;
               
            case (trackSearchState) is
            
               when TRACKSEARCH_IDLE => 
                  if (sectorFetchState = SFETCH_IDLE and readOnDisk_buffer = '1' and ce = '1') then
                     trackSearchState <= TRACKSEARCH_READ;
                     trackInfo_addrB  <= std_logic_vector(to_unsigned(1, 7));
                     trackNumberBCD   <= x"01";
                  end if;
                  
               when TRACKSEARCH_READ =>
                  trackSearchState <= TRACKSEARCH_CHECK;
               
               when TRACKSEARCH_CHECK => 
                  if (lastReadSector >= startLBA and lastReadSector <= endLBA) then
                     trackSearchState <= TRACKSEARCH_IDLE;
                     trackNumber      <= trackInfo_addrB;
                  elsif (trackNumberBCD = x"99") then
                     trackSearchState <= TRACKSEARCH_IDLE;
                     trackNumberBCD   <= x"AA";
                     trackNumber      <= std_logic_vector(to_unsigned(0, 7));
                  else
                     trackSearchState <= TRACKSEARCH_READ;
                     trackInfo_addrB  <= std_logic_vector(unsigned(trackInfo_addrB) + 1);
                     if (trackNumberBCD(3 downto 0) = x"9") then
                        trackNumberBCD(3 downto 0) <= x"0";
                        trackNumberBCD(7 downto 4) <= trackNumberBCD(7 downto 4) + 1;
                     else
                        trackNumberBCD(3 downto 0) <= trackNumberBCD(3 downto 0) + 1;
                     end if;
                  end if;
            
            end case;
            
            if (trackinfo_write = '1') then
            
               newCDCounter <= (others => '1');
            
               -- header
               if (to_integer(unsigned(trackinfo_addr)) = 0) then
                  --trackcount    <= trackinfo_data(7 downto 0);
                  trackcountBCD <= trackinfo_data(15 downto 8);
               end if;
               
               --if (to_integer(unsigned(trackinfo_addr)) = 1) then
               --   totalLBAs <= to_integer(unsigned(trackinfo_data(18 downto 0)));
               --end if;
               
               if (to_integer(unsigned(trackinfo_addr)) = 2) then
                  totalSecondsBCD <= trackinfo_data(7 downto 0);
                  totalMinutesBCD <= trackinfo_data(15 downto 8);
               end if;               
               
               if (to_integer(unsigned(trackinfo_addr)) = 3) then
                  libcryptKey <= trackinfo_data(15 downto 0);
                  region_out  <= trackinfo_data(17 downto 16);
                  resetFromCD <= trackinfo_data(18) and (not LIDopen);
               end if;
            
               -- tracks
               if (trackinfo_addr(1 downto 0) = "00") then
                  trackInfo_DataA(18 downto 0)  <= trackinfo_data(18 downto 0); -- lbaStart
               end if;
               if (trackinfo_addr(1 downto 0) = "01") then
                  trackInfo_DataA(37 downto 19) <= trackinfo_data(18 downto 0); -- lbaEnd
               end if;
               if (trackinfo_addr(1 downto 0) = "10") then
                  trackInfo_DataA(45 downto 38) <= trackinfo_data( 7 downto 0); -- secondsBCD
                  trackInfo_DataA(53 downto 46) <= trackinfo_data(15 downto 8); -- minutesBCD
                  trackInfo_DataA(54)           <= trackinfo_data(16);          -- isAudio
                  if (to_integer(unsigned(trackinfo_addr)) = 6) then
                     isAudioCD <= trackinfo_data(16);
                  end if;
               end if;
               if (trackinfo_addr(1 downto 0) = "11") then
                  trackInfo_addrA <= trackinfo_addr(8 downto 2);
                  trackInfo_wrenA <= '1';
               end if;
               
            elsif (FifoParam_Empty = '0') then
               
               trackInfo_addrA <= std_logic_vector(to_unsigned(to_integer(unsigned(FifoParam_Dout(6 downto 4))) * 8 + 
                                                   to_integer(unsigned(FifoParam_Dout(6 downto 4))) * 2 + 
                                                   to_integer(unsigned(FifoParam_Dout(3 downto 0))), 7));
               
            --else trackInfo_addrA <= "0000100"; -- debug test!
            
            end if;
            
         end if;

      end if;
   end process;
   
   startLBA   <= to_integer(unsigned(trackInfo_DataOutB(18 downto 0)));
   endLBA     <= to_integer(unsigned(trackInfo_DataOutB(37 downto 19)));
   isAudio    <= trackInfo_DataOutB(54);    

   secondsBCD <= trackInfo_DataOutA(45 downto 38);
   minutesBCD <= trackInfo_DataOutA(53 downto 46);   
   
   iramTrackInfos: entity mem.dpram
   generic map 
   ( 
      addr_width => 7, 
      data_width => 55
   )
   port map
   (
      clock_a     => clk1x,
      address_a   => trackInfo_addrA,
      data_a      => trackInfo_DataA,
      wren_a      => trackInfo_wrenA,
      q_a         => trackInfo_DataOutA,
      
      clock_b     => clk1x,
      address_b   => trackInfo_addrB,
      data_b      => (54 downto 0 => '0'),
      wren_b      => '0',
      q_b         => trackInfo_DataOutB
   );
   
--##############################################################
--############################### savestates
--##############################################################

   process (clk1x)
   begin
      if (rising_edge(clk1x)) then
      
         if (SS_reset = '1') then
            
         elsif (SS_wren = '1' and SS_Adr < 128) then
         end if;
         
         if (cmd_busy = '1' or working = '1' or (driveBusy = '1' and driveDelay < 1024)) then
            ss_idle_timeout <= 7;
         elsif (ss_idle_timeout > 0) then
            ss_idle_timeout <= ss_idle_timeout - 1;
         end if;
         
         SS_idle <= '0';
         if ((FifoParam_Empty = '1' or ss_timeout(23) = '1') and (FifoResponse_Empty = '1' or ss_timeout(23) = '1') and cmd_busy = '0' and working = '0' and ss_idle_timeout = 0 and
           sectorFetchState = SFETCH_IDLE and readOnDisk = '0' and readOnDisk_buffer = '0' and readSubchannelState = SSUB_IDLE and sectorProcessState = SPROC_IDLE and copyState = COPY_IDLE) then
            SS_idle    <= '1';
            if (FifoParam_Empty = '1' and FifoResponse_Empty = '1') then
               ss_timeout <= (others => '0');
            end if;
         end if;
         
         Pause_idle_cd <= '1';
         
         if (ss_timeout(23) = '0') then
            ss_timeout <= ss_timeout + 1;
         end if;
         
         SS_rden_sectorbuffer <= '0';
         if (SS_rden = '1' and SS_Adr >= 1024 and SS_Adr < 1024 + (RAW_SECTOR_SIZE / 4)) then
            SS_rden_sectorbuffer <= '1';
         end if;
         
         SS_rden_sectorbuffers <= '0';
         if (SS_rden = '1' and SS_Adr >= 2048 and SS_Adr < 2048 + (1024 * 8)) then
            SS_rden_sectorbuffers <= '1';
         end if;
         
         if (SS_rden_sectorbuffer = '1') then
            SS_DataRead <= sectorBuffer_DataB;
         elsif (SS_rden_sectorbuffers = '1') then
            SS_DataRead <= sectorBuffers_DataB;
         elsif (SS_rden = '1' and SS_Adr < 128) then
            SS_DataRead <= ss_out(to_integer(SS_Adr));
         end if;
      
      end if;
   end process;

   -- synthesis translate_off

   goutput : if 1 = 1 generate
   signal outputCnt  : unsigned(31 downto 0) := (others => '0'); 
   signal clkCounter : unsigned(31 downto 0);
   
   begin
      process
         constant WRITETIME               : std_logic := '1';
            
         file outfile                     : text;
         variable f_status                : FILE_OPEN_STATUS;
         variable line_out                : line;
               
         variable bus_read_1              : std_logic;
         variable bus_addr_1              : unsigned(3 downto 0);
         variable cmdAck_1                : std_logic;
         variable driveAck_1              : std_logic;
         variable ackDrive_1              : std_logic;
         variable ackRead_valid_1         : std_logic;
         variable ackDriveEnd_1           : std_logic;
         variable ackPendingIRQNext_1     : std_logic;
         variable errorResponseNext_new_1 : std_logic;
         variable readOnDisk_1            : std_logic;
         variable readOnDisk_2            : std_logic;
         variable readOnDisk_3            : std_logic;
         variable fifoResponseSize        : unsigned(31 downto 0);
         variable newoutputCnt            : unsigned(31 downto 0); 
         variable fifoDataWrCnt           : unsigned(7 downto 0);
         variable datatemp                : unsigned(31 downto 0);
      begin
   
         file_open(f_status, outfile, "R:\\debug_cd_sim.txt", write_mode);
         file_close(outfile);
         file_open(f_status, outfile, "R:\\debug_cd_sim.txt", append_mode);
         
         while (true) loop
            
            wait until rising_edge(clk1x);
            
            if (FifoResponse_reset = '1') then fifoResponseSize := (others => '0'); end if;
            if (FifoResponse_Wr = '1') then fifoResponseSize := fifoResponseSize + 1; end if;
            if (FifoResponse_Rd = '1') then fifoResponseSize := fifoResponseSize - 1; end if;
            
            newoutputCnt := outputCnt;
            
            if (beginCommand = '1') then
               write(line_out, string'("CMD: "));
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter - 1));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, to_hstring(newCmd));
               write(line_out, string'(" 00000000")); 
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if; 
            
            if (cmdAck_1 = '1') then
               write(line_out, string'("RSPFIFO: "));
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter - 3));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, to_hstring(FifoResponse_Din));
               write(line_out, string'(" "));
               write(line_out, to_hstring(fifoResponseSize));               
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if; 
            cmdAck_1 := cmdAck or cmdIRQ;            
            
            if (driveAck_1 = '1' or ackDrive_1 = '1' or ackRead_valid_1 = '1' or ackDriveEnd_1 = '1' or ackPendingIRQNext_1 = '1') then
               write(line_out, string'("RSPFIFO2: "));
               if (WRITETIME = '1') then
                  if (driveAck_1 = '1') then write(line_out, to_hstring(clkCounter - 2));
                  elsif (ackDrive_1 = '1') then write(line_out, to_hstring(clkCounter - 3));
                  elsif (ackRead_valid_1 = '1') then write(line_out, to_hstring(clkCounter - 6));
                  elsif (ackDriveEnd_1 = '1') then write(line_out, to_hstring(clkCounter - 5));
                  elsif (ackPendingIRQNext_1 = '1') then write(line_out, to_hstring(clkCounter - 5));
                  end if;
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, to_hstring(FifoResponse_Din));
               write(line_out, string'(" "));
               write(line_out, to_hstring(fifoResponseSize));               
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if; 
            driveAck_1 := driveAck;
            ackDrive_1 := ackDrive;
            ackRead_valid_1 := ackRead_valid;
            ackDriveEnd_1 := ackDriveEnd;
            ackPendingIRQNext_1 := ackPendingIRQNext;
            
            if (getIDAck = '1') then
               write(line_out, string'("RSPFIFO2: "));
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter - 3));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, to_hstring(FifoResponse_Dout));
               write(line_out, string'(" "));
               write(line_out, to_hstring(fifoResponseSize));               
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if;
            
            if (errorResponseCmd_new = '1') then
               write(line_out, string'("RSPERROR: ")); 
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, string'("00 00")); 
               write(line_out, to_hstring(errorResponseCmd_reason));
               write(line_out, to_hstring(errorResponseCmd_error));
               write(line_out, to_hstring(internalStatus));
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if;      

            if (errorResponseNext_new_1 = '1') then
               write(line_out, string'("RSPFIFO2: "));
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter - 4));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, to_hstring(internalStatus or errorResponseCmd_error));
                write(line_out, string'(" ")); 
               write(line_out, to_hstring(fifoResponseSize));               
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if;
            errorResponseNext_new_1 := errorResponseNext_new; 
            
            if (errorResponseDrive_new = '1') then
               write(line_out, string'("RSPERROR: ")); 
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, string'("00 00")); 
               write(line_out, to_hstring(errorResponseDrive_reason));
               write(line_out, to_hstring(errorResponseDrive_error));
               write(line_out, to_hstring(internalStatus));
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if; 
            
            if (processDataSector = '1' and (modeReg(6) = '0' or headerIsData = '1') and (modeReg(5) = '1' or headerDataSector = '1')) then
                write(line_out, string'("WPTR: "));
                if (WRITETIME = '1') then
                   write(line_out, to_hstring(clkCounter - 4));
                   write(line_out, string'(" ")); 
                end if;
                if (modeReg(5) = '1') then
                   write(line_out, string'("24"));
                else
                   write(line_out, string'("00"));
                end if;
                write(line_out, string'(" 000000"));
                write(line_out, to_hstring("00000" & writeSectorPointer));               
                writeline(outfile, line_out);
                newoutputCnt := newoutputCnt + 1;
            end if; 
            
            if (copyState = COPY_FIRST) then
               for i in 0 to (copySize - 1) loop
                  write(line_out, string'("DATA: "));
                  datatemp := to_unsigned(i * 4, 32);
                  write(line_out, to_hstring(datatemp(7 downto 0)));
                  write(line_out, string'(" "));
                  datatemp := unsigned(sectorBuffers(to_integer(copySectorPointer))(i));
                  write(line_out, to_hstring(datatemp)); 
                  writeline(outfile, line_out);
                  newoutputCnt := newoutputCnt + 1;
               end loop;
            end if;
            
            if (readOnDisk_3 = '1') then
               write(line_out, string'("SECTORREAD: "));
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter - 7));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, string'("00 "));
               write(line_out, to_hstring(to_unsigned(lastReadSector, 32)));               
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if;
            readOnDisk_3 := readOnDisk_2;
            readOnDisk_2 := readOnDisk_1;
            readOnDisk_1 := readOnDisk;

            --if (copyState = COPY_DATA and copyByteCnt = 0) then
            --   write(line_out, string'("DATA: "));
            --   write(line_out, to_hstring(fifoDataWrCnt));
            --   write(line_out, string'(" "));
            --   write(line_out, to_hstring(sectorBuffers_DataB));               
            --   writeline(outfile, line_out);
            --   newoutputCnt := newoutputCnt + 1;
            --   fifoDataWrCnt := fifoDataWrCnt + 4;
            --elsif (copyState = COPY_IDLE) then
            --   fifoDataWrCnt := (others => '0');
            --end if;
            
            --if (dma_read = '1') then
            --   write(line_out, string'("DMAREAD: 00 000000"));
            --   write(line_out, to_hstring(dma_readdata));               
            --   writeline(outfile, line_out);
            --   newoutputCnt := newoutputCnt + 1;
            --end if; 
            
            if (bus_write = '1') then
               write(line_out, string'("CPUWRITE: ")); 
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, string'("0")); 
               write(line_out, to_hstring(bus_addr));
               write(line_out, string'(" 000000")); 
               write(line_out, to_hstring(bus_dataWrite));
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if; 
            
            if (bus_read_1 = '1') then
               write(line_out, string'("CPUREAD: "));
               if (WRITETIME = '1') then
                  write(line_out, to_hstring(clkCounter - 1));
                  write(line_out, string'(" ")); 
               end if;
               write(line_out, string'("0")); 
               write(line_out, to_hstring(bus_addr_1));
               write(line_out, string'(" 000000")); 
               write(line_out, to_hstring(bus_dataRead));
               writeline(outfile, line_out);
               newoutputCnt := newoutputCnt + 1;
            end if; 
            bus_read_1 := bus_read;
            bus_addr_1 := bus_addr;
            
            outputCnt <= newoutputCnt;
            clkCounter <= clkCounter + 1;
            
            if (reset = '1') then
               clkCounter <= x"00000000";
               file_close(outfile);
               file_open(f_status, outfile, "R:\\debug_cd_sim.txt", write_mode);
               file_close(outfile);
               file_open(f_status, outfile, "R:\\debug_cd_sim.txt", append_mode);
            end if;
           
         end loop;
         
      end process;
   
   end generate goutput;
   
   goutputcdda : if 1 = 1 generate
      signal outputCnt  : unsigned(23 downto 0) := (others => '0'); 
   begin
      process
         file outfile                  : text;
         variable f_status             : FILE_OPEN_STATUS;
         variable line_out             : line;
      begin
   
         file_open(f_status, outfile, "R:\\debug_cdda_sim.txt", write_mode);
         file_close(outfile);
         file_open(f_status, outfile, "R:\\debug_cdda_sim.txt", append_mode);
         
         while (true) loop
            
            wait until rising_edge(clk1x);
            
            if (CDDA_write = '1') then
               write(line_out, to_hstring(outputCnt));
               write(line_out, string'(" ")); 
               write(line_out, to_hstring(CDDA_data));
               writeline(outfile, line_out);
               outputCnt <= outputCnt + 1;
            end if; 
           
         end loop;
         
      end process;
   
   end generate goutputcdda;
   
   -- synthesis translate_on

end architecture;





