`timescale 1ns / 1ps
//=============================================================================
// Module  : UART_RPI_CNN_TOP
// Purpose : Top-level module for RPi, CNN, and SHT40 Sensor Integration.
//=============================================================================
module UART_RPI_CNN_TOP (
    input  wire clk,              // 100 MHz
    input  wire rst,              // Active-High Reset

    output wire uart_tx_pin,
    input  wire uart_rx_pin,

    input  wire rpi_signal,       // RPi GPIO (1-sec toggle on crisis)
    input  wire cnn_signal,       // Z7-20에서 오는 단일 선

    // SHT40 I2C Interface
    inout  wire sht_i2c_sda,
    output wire sht_i2c_scl,

    output wire led_tx_active,
    output wire led_rpi_active,
    output wire led_cnn_active
);

    // 내부 신호 선언 (모두 서브모듈 출력으로 구동 → wire)
    wire        baud_tick;
    wire [7:0]  tx_data;
    wire        tx_start, tx_busy, tx_done;
    wire [7:0]  rx_data;
    wire        rx_done;

    wire        rpi_event, rpi_active;
    wire [7:0]  cnn_data;
    wire        cnn_event;

    wire        sht_event;
    wire [31:0] sht_data;

    wire [7:0]  cmd_out, cmd_payload;
    wire        cmd_valid;

    // Baud Rate Generator (115200)
    baud_gen #(.CLK_FREQ(100_000_000), .BAUD_RATE(115_200)) u_baud_gen (
        .clk(clk), .rst(rst), .tick(baud_tick)
    );

    // UART Modules
    uart_tx u_uart_tx (
        .clk(clk), .rst(rst), .baud_tick(baud_tick),
        .start(tx_start), .data(tx_data), .tx(uart_tx_pin),
        .busy(tx_busy), .done(tx_done)
    );

    uart_rx #(.CLK_FREQ(100_000_000), .BAUD_RATE(115_200)) u_uart_rx (
        .clk(clk), .rst(rst), .rx(uart_rx_pin),
        .rx_data(rx_data), .rx_done(rx_done), .rx_error()
    );

    // RPi Detector (posedge only)
    rpi_det u_rpi_det (
        .clk(clk), .rst(rst), .rpi_signal(rpi_signal),
        .rpi_event(rpi_event), .rpi_active(rpi_active)
    );

    // CNN Latch
    cnn_latch u_cnn_latch (
        .clk(clk), .rst(rst), .cnn_signal(cnn_signal),
        .cnn_data(cnn_data), .cnn_event(cnn_event)
    );

    // SHT40 I2C Controller
    sht40_i2c #(.CLK_FREQ(100_000_000)) u_sht40_ctrl (
        .clk(clk), .rst(rst),
        .i2c_sda(sht_i2c_sda), .i2c_scl(sht_i2c_scl),
        .sht_data(sht_data), .sht_valid(sht_event)
    );

    // Packet Builder
    packet_builder u_pkt_builder (
        .clk(clk), .rst(rst),
        .rpi_event(rpi_event), .rpi_active(rpi_active),
        .cnn_event(cnn_event), .cnn_data(cnn_data),
        .sht_event(sht_event), .sht_data(sht_data),
        .tx_busy(tx_busy), .tx_done(tx_done),
        .tx_start(tx_start), .tx_byte(tx_data)
    );

    // Command Decoder
    cmd_decoder u_cmd_decoder (
        .clk(clk), .rst(rst),
        .rx_data(rx_data), .rx_done(rx_done),
        .cmd_out(cmd_out), .cmd_payload(cmd_payload), .cmd_valid(cmd_valid)
    );

    // Status LEDs
    assign led_tx_active  = tx_busy;
    assign led_rpi_active = rpi_active;
    assign led_cnn_active = cnn_event;

endmodule