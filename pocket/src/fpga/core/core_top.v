//
// User core top-level: PlayStation (PSX_MiSTer core, Pocket build)
//
// Based on the Analogue openFPGA core template (core_top.v). PSX-specific parts: clocks (psx_plls.v),
// SDRAM (rtl/sdram.sv, MiSTer), the VHDL glue around the core (pocket_psx.vhd: PSRAM VRAM, BIOS loader,
// controller, video) and I2S audio (sound_i2s.sv, agg23, MIT).
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
//   [31:28] type
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [31:0]  cont1_key,
input   wire    [31:0]  cont2_key,
input   wire    [31:0]  cont3_key,
input   wire    [31:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 1;   // APF data arrives as little-endian 32-bit words (PSX byte order)

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
assign cart_tran_bank3 = 8'hzz;
assign cart_tran_bank3_dir = 1'b0;
assign cart_tran_bank2 = 8'hzz;
assign cart_tran_bank2_dir = 1'b0;
assign cart_tran_bank1 = 8'hzz;
assign cart_tran_bank1_dir = 1'b0;
assign cart_tran_bank0 = 4'hf;
assign cart_tran_bank0_dir = 1'b1;
assign cart_tran_pin30 = 1'b0;      // reset or cs2, we let the hw control it by itself
assign cart_tran_pin30_dir = 1'bz;
assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
assign cart_tran_pin31 = 1'bz;      // input
assign cart_tran_pin31_dir = 1'b0;  // input

// link port is unused, set to input only to be safe
// each bit may be bidirectional in some applications
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using

assign sram_a = 'h0;
assign sram_dq = {16{1'bZ}};
assign sram_oe_n  = 1;
assign sram_we_n  = 1;
assign sram_ub_n  = 1;
assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;


    wire    [31:0]  memcard_rd_data;            // memory card save slot read back (pocket_psx)

// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
    default: begin
        bridge_rd_data <= 0;
    end
    32'h2xxxxxxx: begin
        // memory card 1 save slot (APF reads it back on exit)
        bridge_rd_data <= memcard_rd_data;
    end
    32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
    end
    endcase
end


//

////////////////////////////////////////////////////////////////////////////////////////
// clocks

    wire    clk1x, clk2x, clk3x, clkvid, clkpix, clkpix_90;
    wire    pll_psx_locked, pll_vid_locked;
    wire    pll_core_locked = pll_psx_locked & pll_vid_locked;
    wire    pll_core_locked_s;
synch_3 s01(pll_core_locked, pll_core_locked_s, clk_74a);

psx_pll_core ipll_core (
    .refclk   ( clk_74a ),
    .rst      ( 0 ),
    .outclk_0 ( clk1x ),
    .outclk_1 ( clk2x ),
    .outclk_2 ( clk3x ),
    .locked   ( pll_psx_locked )
);

psx_pll_video ipll_video (
    .refclk   ( clk_74a ),
    .rst      ( 0 ),
    .outclk_0 ( clkvid ),
    .outclk_1 ( clkpix ),
    .outclk_2 ( clkpix_90 ),
    .locked   ( pll_vid_locked )
);

// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
    wire            status_boot_done = pll_core_locked_s; 
    wire            status_setup_done = pll_core_locked_s; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire    [31:0]  dataslot_requestwrite_size;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_update;
    wire    [15:0]  dataslot_update_id;
    wire    [31:0]  dataslot_update_size;
    
    wire            dataslot_allcomplete;

    wire     [31:0] rtc_epoch_seconds;
    wire     [31:0] rtc_date_bcd;
    wire     [31:0] rtc_time_bcd;
    wire            rtc_valid;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a

    wire            target_dataslot_read;       // CD sector reads (pocket_cd in pocket_psx)
    wire            target_dataslot_write = 1'b0;
    wire            target_dataslot_getfile = 1'b0;
    wire            target_dataslot_openfile = 1'b0;
    
    wire            target_dataslot_ack;        
    wire            target_dataslot_done;
    wire    [2:0]   target_dataslot_err;

    wire    [15:0]  target_dataslot_id;
    wire    [31:0]  target_dataslot_slotoffset;
    wire    [31:0]  target_dataslot_bridgeaddr;
    wire    [31:0]  target_dataslot_length;
    
    wire    [31:0]  target_buffer_param_struct = 32'h0; // to be mapped/implemented when using some Target commands
    wire    [31:0]  target_buffer_resp_struct = 32'h0;  // to be mapped/implemented when using some Target commands
    
// bridge data slot access
// synchronous to clk_74a

    wire    [9:0]   datatable_addr;             // read by pocket_cd (track file sizes)
    wire            datatable_wren;             // memory card save size (pocket_psx)
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;

core_bridge_cmd icb (

    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_size ( dataslot_requestwrite_size ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_update            ( dataslot_update ),
    .dataslot_update_id         ( dataslot_update_id ),
    .dataslot_update_size       ( dataslot_update_size ),
    
    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .rtc_epoch_seconds      ( rtc_epoch_seconds ),
    .rtc_date_bcd           ( rtc_date_bcd ),
    .rtc_time_bcd           ( rtc_time_bcd ),
    .rtc_valid              ( rtc_valid ),
    
    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .target_dataslot_read       ( target_dataslot_read ),
    .target_dataslot_write      ( target_dataslot_write ),
    .target_dataslot_getfile    ( target_dataslot_getfile ),
    .target_dataslot_openfile   ( target_dataslot_openfile ),
    
    .target_dataslot_ack        ( target_dataslot_ack ),
    .target_dataslot_done       ( target_dataslot_done ),
    .target_dataslot_err        ( target_dataslot_err ),

    .target_dataslot_id         ( target_dataslot_id ),
    .target_dataslot_slotoffset ( target_dataslot_slotoffset ),
    .target_dataslot_bridgeaddr ( target_dataslot_bridgeaddr ),
    .target_dataslot_length     ( target_dataslot_length ),

    .target_buffer_param_struct ( target_buffer_param_struct ),
    .target_buffer_resp_struct  ( target_buffer_resp_struct ),
    
    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q )

);

////////////////////////////////////////////////////////////////////////////////////////
// data slot loading (BIOS at bridge address 0): the core is held in reset while APF writes data slots

    reg     loading = 0;
always @(posedge clk_74a) begin
    if (dataslot_requestwrite) loading <= 1;
    else if (dataslot_allcomplete) loading <= 0;
end

////////////////////////////////////////////////////////////////////////////////////////
// SDRAM: MiSTer controller, 16-bit, clocked by clk3x (as on MiSTer); ch1 = core reads (incl. cache line
// and DMA reads), ch2 = core writes, ch3 = BIOS download, then SPU RAM

    wire            sdram_init;
    wire            ram_refresh;
    wire    [26:0]  ram_addr;
    wire    [31:0]  ram_din;
    wire    [3:0]   ram_be;
    wire            ram_rd_req, ram_wr_req, ram_dma, ram_cache;
    wire    [1:0]   ram_cntDMA;
    wire    [31:0]  ram_dout32;
    wire            ram_rd_ready, ram_wr_ready;
    wire    [3:0]   cache_wr;
    wire    [31:0]  cache_data;
    wire    [7:0]   cache_addr;
    wire            dma_wr, dma_reqprocessed;
    wire    [31:0]  dma_data;
    wire    [26:0]  dmafifo_adr;
    wire    [31:0]  dmafifo_data;
    wire            dmafifo_empty, dmafifo_read;
    wire    [26:0]  ch3_addr;
    wire    [31:0]  ch3_din, ch3_dout;
    wire    [3:0]   ch3_be;
    wire            ch3_req, ch3_rnw, ch3_ready;
    wire            sdram_ncs;

sdram #(.SINGLE_CHIP(1)) isdram (
    .SDRAM_DQ     ( dram_dq ),
    .SDRAM_A      ( dram_a ),
    .SDRAM_DQML   ( dram_dqm[0] ),
    .SDRAM_DQMH   ( dram_dqm[1] ),
    .SDRAM_BA     ( dram_ba ),
    .SDRAM_nCS    ( sdram_ncs ),     // chip select is tied on the Pocket board
    .SDRAM_nWE    ( dram_we_n ),
    .SDRAM_nRAS   ( dram_ras_n ),
    .SDRAM_nCAS   ( dram_cas_n ),
    .SDRAM_CKE    ( dram_cke ),
    .SDRAM_CLK    ( dram_clk ),
    .SDRAM_EN     ( 1'b1 ),

    .init         ( sdram_init ),
    .clk          ( clk3x ),
    .clk_base     ( clk1x ),
    .refreshForce ( ram_refresh ),
    .ram_idle     ( ),

    .ch1_addr     ( ram_addr ),
    .ch1_din      ( ),
    .ch1_dout     ( ),
    .ch1_dout32   ( ram_dout32 ),
    .ch1_req      ( ram_rd_req ),
    .ch1_rnw      ( 1'b1 ),
    .ch1_dma      ( ram_dma ),
    .ch1_cntDMA   ( ram_cntDMA ),
    .ch1_cache    ( ram_cache ),
    .ch1_ready    ( ram_rd_ready ),
    .cache_wr     ( cache_wr ),
    .cache_data   ( cache_data ),
    .cache_addr   ( cache_addr ),
    .dma_wr       ( dma_wr ),
    .dma_reqprocessed ( dma_reqprocessed ),
    .dma_data     ( dma_data ),

    .ch2_addr     ( ram_addr ),
    .ch2_din      ( ram_din ),
    .ch2_dout     ( ),
    .ch2_req      ( ram_wr_req ),
    .ch2_rnw      ( 1'b0 ),
    .ch2_be       ( ram_be ),
    .ch2_ready    ( ram_wr_ready ),

    .ch3_addr     ( ch3_addr ),
    .ch3_din      ( ch3_din ),
    .ch3_dout     ( ch3_dout ),
    .ch3_req      ( ch3_req ),
    .ch3_rnw      ( ch3_rnw ),
    .ch3_be       ( ch3_be ),
    .ch3_ready    ( ch3_ready ),

    .dmafifo_adr  ( dmafifo_adr ),
    .dmafifo_data ( dmafifo_data ),
    .dmafifo_empty( dmafifo_empty ),
    .dmafifo_read ( dmafifo_read )
);

////////////////////////////////////////////////////////////////////////////////////////
// PSX core and its Pocket glue (VHDL)

    wire    [15:0]  cram0_dq_out, cram0_dq_in, cram1_dq_out, cram1_dq_in;
    wire            cram0_dq_oe, cram1_dq_oe;
    wire    [15:0]  sound_left, sound_right;
    wire            loader_overflow;

assign cram0_dq    = cram0_dq_oe ? cram0_dq_out : 16'hZZZZ;
assign cram0_dq_in = cram0_dq;
assign cram1_dq    = cram1_dq_oe ? cram1_dq_out : 16'hZZZZ;
assign cram1_dq_in = cram1_dq;

pocket_psx ipsx (
    .clk_74a          ( clk_74a ),
    .clk1x            ( clk1x ),
    .clk2x            ( clk2x ),
    .clk3x            ( clk3x ),
    .clkvid           ( clkvid ),
    .clkpix           ( clkpix ),
    .reset_n_74a      ( reset_n ),
    .pll_locked       ( pll_core_locked ),
    .loading_74a      ( loading ),
    .bridge_addr      ( bridge_addr ),
    .bridge_wr        ( bridge_wr ),
    .bridge_wr_data   ( bridge_wr_data ),
    .bridge_rd        ( bridge_rd ),
    .memcard_rd_data  ( memcard_rd_data ),
    .datatable_addr   ( datatable_addr ),
    .datatable_q      ( datatable_q ),
    .datatable_wren   ( datatable_wren ),
    .datatable_data   ( datatable_data ),
    .dsr_read         ( target_dataslot_read ),
    .dsr_id           ( target_dataslot_id ),
    .dsr_offset       ( target_dataslot_slotoffset ),
    .dsr_bridgeaddr   ( target_dataslot_bridgeaddr ),
    .dsr_length       ( target_dataslot_length ),
    .dsr_done         ( target_dataslot_done ),
    .cont1_key        ( cont1_key ),
    .loader_overflow  ( loader_overflow ),

    .sdram_init       ( sdram_init ),
    .ram_refresh      ( ram_refresh ),
    .ram_addr         ( ram_addr ),
    .ram_din          ( ram_din ),
    .ram_be           ( ram_be ),
    .ram_rd_req       ( ram_rd_req ),
    .ram_wr_req       ( ram_wr_req ),
    .ram_dma          ( ram_dma ),
    .ram_cntDMA       ( ram_cntDMA ),
    .ram_cache        ( ram_cache ),
    .ram_dout32       ( ram_dout32 ),
    .ram_rd_ready     ( ram_rd_ready ),
    .ram_wr_ready     ( ram_wr_ready ),
    .cache_wr         ( cache_wr ),
    .cache_data       ( cache_data ),
    .cache_addr       ( cache_addr ),
    .dma_wr           ( dma_wr ),
    .dma_reqprocessed ( dma_reqprocessed ),
    .dma_data         ( dma_data ),
    .dmafifo_adr      ( dmafifo_adr ),
    .dmafifo_data     ( dmafifo_data ),
    .dmafifo_empty    ( dmafifo_empty ),
    .dmafifo_read     ( dmafifo_read ),
    .ch3_addr         ( ch3_addr ),
    .ch3_din          ( ch3_din ),
    .ch3_be           ( ch3_be ),
    .ch3_req          ( ch3_req ),
    .ch3_rnw          ( ch3_rnw ),
    .ch3_dout         ( ch3_dout ),
    .ch3_ready        ( ch3_ready ),

    .cram0_a          ( cram0_a ),
    .cram0_dq_out     ( cram0_dq_out ),
    .cram0_dq_oe      ( cram0_dq_oe ),
    .cram0_dq_in      ( cram0_dq_in ),
    .cram0_clk        ( cram0_clk ),
    .cram0_adv_n      ( cram0_adv_n ),
    .cram0_cre        ( cram0_cre ),
    .cram0_ce0_n      ( cram0_ce0_n ),
    .cram0_ce1_n      ( cram0_ce1_n ),
    .cram0_oe_n       ( cram0_oe_n ),
    .cram0_we_n       ( cram0_we_n ),
    .cram0_ub_n       ( cram0_ub_n ),
    .cram0_lb_n       ( cram0_lb_n ),
    .cram1_a          ( cram1_a ),
    .cram1_dq_out     ( cram1_dq_out ),
    .cram1_dq_oe      ( cram1_dq_oe ),
    .cram1_dq_in      ( cram1_dq_in ),
    .cram1_clk        ( cram1_clk ),
    .cram1_adv_n      ( cram1_adv_n ),
    .cram1_cre        ( cram1_cre ),
    .cram1_ce0_n      ( cram1_ce0_n ),
    .cram1_ce1_n      ( cram1_ce1_n ),
    .cram1_oe_n       ( cram1_oe_n ),
    .cram1_we_n       ( cram1_we_n ),
    .cram1_ub_n       ( cram1_ub_n ),
    .cram1_lb_n       ( cram1_lb_n ),

    .video_rgb        ( video_rgb ),
    .video_de         ( video_de ),
    .video_skip       ( video_skip ),
    .video_vs         ( video_vs ),
    .video_hs         ( video_hs ),

    .audio_lrck       ( audio_lrck ),
    .sound_left       ( sound_left ),
    .sound_right      ( sound_right )
);

assign video_rgb_clock    = clkpix;
assign video_rgb_clock_90 = clkpix_90;

////////////////////////////////////////////////////////////////////////////////////////
// audio: 48 kHz samples (signed 16 bit, clk1x; pocket_audio48 in pocket_psx interpolates the PSX's 44.1 kHz
// output at the frames of this word clock) to the Pocket's I2S

sound_i2s #(
    .CHANNEL_WIDTH ( 16 ),
    .SIGNED_INPUT  ( 1 )
) isound (
    .clk_74a    ( clk_74a ),
    .clk_audio  ( clk1x ),
    .audio_l    ( sound_left ),
    .audio_r    ( sound_right ),
    .audio_mclk ( audio_mclk ),
    .audio_lrck ( audio_lrck ),
    .audio_dac  ( audio_dac )
);

endmodule
