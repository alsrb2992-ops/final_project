`timescale 1ns / 1ps

module i2c_slave(
    input clk,
    input reset,
    input [7:0] tx_data,
    input scl,
    output logic [7:0] rx_data,
    output logic rx_done,
    inout sda
);

    logic clear;
    logic [$clog2(10000)-1:0] counter;
    logic scl_c;
    logic sda_c; // i2c 입력 sync
    logic pos_scl;
    logic neg_scl;
    logic [7:0] rx_data_next;

    logic [6:0] slave_address = 7'b1101110; //  8'hdc ->

    typedef enum 
    {   IDLE,
        START, ADDRESS1, ADDRESS2,
        AACK1, AACK2, AACK3, AACK4,
        WRITE1, WRITE2, WRITE3, WRITE4,
        WACK1, WACK2, WACK3, WACK4,
        READ1, READ2, READ3, READ4,
        RACK1, RACK2, RACK3, RACK4,
        STOP1, STOP2 } state_t;

    state_t state;
    state_t state_next;

    logic sda_r;
    logic sda_next;
    logic sel;
    logic sel_next;
    logic [3:0] bit_count;
    logic [3:0] bit_count_next;

    assign sda = sel ? sda_r : 1'bz;

    clk_counter_slv U_CLK_COUNTER(
        .clk(clk),
        .reset(reset),
        .clear(clear),
        .counter(counter)
    );

    synchronizer U_SCL_SYNC(
        .clk(clk),
        .reset(reset),
        .in_sig(scl),
        .sig_c(scl_c)
    );

    synchronizer U_SDA_SYNC(
        .clk(clk),
        .reset(reset),
        .in_sig(sda),
        .sig_c(sda_c)
    );

    edge_detector U_SCL_EDGEDETECTOR(
        .clk(clk),
        .reset(reset),
        .in_sig(scl_c),
        .pos_sig(pos_scl),
        .neg_sig(neg_scl)
    );

    always_ff @(posedge clk or posedge reset ) begin
        if(reset) begin
            state <= IDLE;
            sda_r <= 1;
            sel <= 1;
            bit_count <= 0;
            rx_data <= 0;

        end
        else begin
            state <= state_next;
            sda_r <= sda_next;
            sel <= sel_next;
            bit_count <= bit_count_next;
            rx_data <= rx_data_next;
        end
    end

    always_comb begin
        state_next = state;
        sda_next = sda_r;
        sel_next = sel;
        clear = 0;
        bit_count_next = bit_count;
        rx_data_next = rx_data;
        rx_done = 0;
        case(state)
            IDLE : begin
                sel_next = 0;
                clear = 1;
                bit_count_next = 0;
                if(!sda_c) begin
                    state_next = START;
                end
            end
            START : begin
                if(!sda_c & neg_scl) begin
                    state_next = ADDRESS1;
                end
                else if(counter == 9999) begin
                    state_next = IDLE;
                end
            end
            ADDRESS1 : begin
                sel_next = 0;
                if(pos_scl) begin
                    state_next = ADDRESS2;
                    clear = 1;
                end
                else if(counter == 9999) begin
                    state_next = IDLE;
                end
            end
            ADDRESS2 : begin
                sel_next = 0;
                if(counter == 249) begin
                    rx_data_next[7-bit_count] = sda_c;
                    clear = 1;
                    if(bit_count == 7) begin
                        state_next = AACK1;
                        bit_count_next = 0;
                    end
                    else begin
                        state_next = ADDRESS1;
                        bit_count_next = bit_count + 1;
                    end

                end
            end
            AACK1 : begin
                sel_next = 0;
                if(neg_scl) begin
                    clear = 1;
                    state_next = AACK2;
                end
            end
            AACK2 : begin
                sel_next = 0;
                if(counter == 299) begin // sel이 둘다 1인 순간을 피하기 위해 delay
                    clear = 1;
                    state_next = AACK3;
                end
            end
            AACK3 : begin
                sel_next = 1;
                sda_next = (rx_data[7:1] == slave_address) ? 0 : 1;
                if(neg_scl) begin
                    clear = 1;
                    state_next = AACK4;
                end
            end
            AACK4 : begin
                sda_next = (rx_data[7:1] == slave_address) ? 0 : 1;
                sel_next = 1;
                if(counter == 199) begin // sel이 둘다 1인 순간을 피하기 위해 앞당겨서 z
                    clear = 1;
                    if(sda) begin
                        state_next = IDLE;
                    end
                    else begin
                        if(rx_data[0])
                            state_next = WRITE1;    // MASTER : READ
                        else
                            state_next = READ1;     // MASTER : WRITE
                    end

                end
            end
            WRITE1 : begin
                sda_next = tx_data[7-bit_count];
                sel_next = 1;
                if(neg_scl) begin
                    clear = 1;
                    state_next = WRITE2;
                end
            end
            WRITE2 : begin
                sda_next = tx_data[7-bit_count];
                sel_next = 1;
                if(counter == 199) begin
                    clear = 1;
                    if(bit_count == 7) begin
                        bit_count_next = 0;
                        state_next = WACK1;
                    end
                    else begin
                        bit_count_next = bit_count + 1;
                        state_next = WRITE1;
                    end
                end  
            end
            WACK1 : begin
                sel_next = 0;
                if(pos_scl) begin
                    clear = 1;
                    state_next = WACK2;
                end
            end
            WACK2 : begin
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = WACK3;
                end
            end
            WACK3 : begin
                sel_next = 0;
                if(neg_scl) begin
                    clear = 1;
                    state_next = WACK4;
                end
            end
            WACK4 : begin
                sel_next = 0;
                if(counter == 199) begin
                    clear = 1;
                    state_next = STOP1;
                end
            end
            READ1 : begin
                sel_next = 0;
                if(pos_scl) begin
                    clear = 1;
                    state_next = READ2;
                end
            end
            READ2 : begin
                sel_next = 0;
                if(counter == 249) begin
                    rx_data_next[7-bit_count] = sda;
                    clear = 1;
                    if(bit_count == 7) begin
                        bit_count_next = 0;
                        state_next = RACK1;
                    end
                    else begin
                        bit_count_next = bit_count + 1;
                        state_next = READ1;
                    end
                end
            end
            RACK1 : begin
                sel_next = 0;
                if(neg_scl) begin
                    clear = 1;
                    state_next = RACK2;
                end
            end
            RACK2 : begin
                sel_next = 0;
                if(counter == 299) begin // sel이 둘다 1인 순간을 피하기 위해 delay
                    clear = 1;
                    state_next = RACK3;
                end
            end
            RACK3 : begin
                sel_next = 1;
                sda_next = 0;
                if(neg_scl) begin
                    clear = 1;
                    state_next = RACK4;
                end
            end
            RACK4 : begin
                sel_next = 1;
                sda_next = 0;
                if(counter == 199) begin // sel이 둘다 1인 순간을 피하기 위해 앞당겨서 z
                    clear = 1;
                    state_next = STOP1;
                    rx_done = 1;
                end
            end
            STOP1 : begin
                sel_next = 0;
                if(pos_scl) begin
                    clear = 1;
                    state_next = STOP2;
                end
            end
            STOP2 : begin
                sel_next = 0;
                if(!sda_c) begin
                    clear = 1;
                    state_next = IDLE;
                end
            end
        endcase
    end


endmodule

module clk_counter_slv(
    input clk,
    input reset,
    input clear,
    output logic [$clog2(10000)-1:0] counter
);

    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            counter <= 0;
        end
        else begin
            if(clear) begin
                counter <= 0;
            end
            else begin
                counter <= counter + 1;
            end
        end
    end


    
endmodule

module synchronizer (
    input clk,
    input reset,
    input in_sig,
    output sig_c
);
    logic reg1, reg2;

    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            reg1 <= 1;
            reg2 <= 1;
        end
        else begin
            reg1 <= in_sig;
            reg2 <= reg1;
        end
    end

    assign sig_c = reg2;
    
endmodule

module edge_detector (
    input clk,
    input reset,
    input in_sig,
    output logic pos_sig,
    output logic neg_sig
);

    logic sig_delay;

    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            sig_delay <= 0;
        end
        else begin
            sig_delay <= in_sig;
        end
    end

    assign pos_sig = in_sig & !sig_delay;
    assign neg_sig = !in_sig & sig_delay;
    
endmodule