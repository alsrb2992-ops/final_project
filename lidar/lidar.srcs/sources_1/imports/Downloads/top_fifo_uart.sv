`timescale 1ns / 1ps

module top_fifo_uart (
    input  clk,
    input  rst,
    input  lidar_rx,
    output pc_tx

);

    logic [7:0] w_rx_data, w_rx_rdata, w_tx_rdata;
    logic w_rx_done, w_tx_full, w_rx_empty, w_tx_empty, w_tx_busy;

    assign rx_data = w_rx_rdata;
    assign empty   = w_rx_empty;



    uart_rx_my U_UART_RX (
        .clk(clk),
        .reset(rst),
        .rx(lidar_rx),
        .rx_data(w_rx_data),
        .rx_done(w_rx_done)
    );

    top_fifo U_fifo_rx (
        .clk(clk),
        .reset(rst),
        .wdata(w_rx_data),
        .rd(~w_tx_full),
        .wr(w_rx_done),
        .rdata(w_rx_rdata),
        .full(),
        .empty(w_rx_empty)
    );

    top_fifo U_fifo_tx (
        .clk(clk),
        .reset(rst),
        .wdata(w_rx_rdata),
        .rd(~w_tx_busy),
        .wr(~w_rx_empty),
        .rdata(w_tx_rdata),
        .full(w_tx_full),
        .empty(w_tx_empty)
    );

    uart_tx_my U_UART_TX (
        .clk(clk),
        .reset(rst),
        .tx_data(w_tx_rdata),
        .tx_start(~w_tx_empty),
        .tx(pc_tx),
        .tx_busy(w_tx_busy),
        .tx_done(tx_done)
    );


endmodule
